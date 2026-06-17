#=
sexs_ge_tutorial.jl — implementing a new AVR model (SEXS_GE) in PowerSimulationsDynamics.

This script is the spine of the "adding a new dynamic model" tutorial. It:
  1. loads a hand-written PowerSystems component struct (SEXSGE.jl),
  2. defines the three PSID methods that give the struct its dynamics
     (mass matrix, ODE, initialization),
  3. reuses the SEXS validation system, hot-swaps the exciter for SEXS_GE, and
  4. runs a transient (line trip) with the IDA solver.

WHERE THIS REALLY GOES:
  * The struct (SEXSGE.jl): PowerSystems.jl, generated from a JSON descriptor.
  * mass_matrix_avr_entries! / mdl_avr_ode!:
      PowerSimulationsDynamics.jl/src/models/generator_models/avr_models.jl
  * initialize_avr!:
      PowerSimulationsDynamics.jl/src/initialization/generator_components/init_avr.jl

Run it from the test environment so PSID's deps (Sundials/IDA, NLsolve) are available:
    julia --project=test docs/src/code_base_developer_guide/sexs_ge_tutorial/sexs_ge_tutorial.jl
=#

import PowerSimulationsDynamics as PSID
import PowerSystems as PSY
import Sundials
import NLsolve

# The struct PowerSystems would normally generate. We include it so the whole model is
# visible to the methods below. (In production it lives in PowerSystems.jl.)
include(joinpath(@__DIR__, "SEXSGE.jl"))

#######################################################################################
# 1. Mass matrix entries
#######################################################################################
# Each DIFFERENTIAL state with a time constant T contributes T on the mass-matrix diagonal,
# turning `dx = f(x)` into `T dx = f(x)`. The PI state `Vi` gets NO entry (default 1): its
# time constant Tc rides inside the integral gain ki = Kc/Tc, not on the integrator.
function PSID.mass_matrix_avr_entries!(
    mass_matrix,
    avr::SEXSGE,
    global_index::Base.ImmutableDict{Symbol, Int64},
)
    mass_matrix[global_index[:Vm], global_index[:Vm]] = get_Tr(avr)
    mass_matrix[global_index[:Vr], global_index[:Vr]] = get_Tb(avr)
    mass_matrix[global_index[:Vf], global_index[:Vf]] = get_Te(avr)
    return
end

#######################################################################################
# 2. ODE (right-hand side)
#######################################################################################
function PSID.mdl_avr_ode!(
    device_states::AbstractArray{<:PSID.ACCEPTED_REAL_TYPES},
    output_ode::AbstractArray{<:PSID.ACCEPTED_REAL_TYPES},
    inner_vars::AbstractArray{<:PSID.ACCEPTED_REAL_TYPES},
    dynamic_device::PSID.DynamicWrapper{PSY.DynamicGenerator{M, S, SEXSGE, TG, P}},
    h,
    t,
) where {M <: PSY.Machine, S <: PSY.Shaft, TG <: PSY.TurbineGov, P <: PSY.PSS}
    # Reference and local state indices
    V0_ref = PSID.get_V_ref(dynamic_device)
    local_ix = PSID.get_local_state_ix(dynamic_device, SEXSGE)

    internal_states = @view device_states[local_ix]
    Vm = internal_states[1]
    Vr = internal_states[2]
    Vi = internal_states[3]
    Vf = internal_states[4]

    # External signals from the rest of the device
    V_th = sqrt(inner_vars[PSID.VR_gen_var]^2 + inner_vars[PSID.VI_gen_var]^2)  # Ec
    Vs = inner_vars[PSID.V_pss_var]                                            # PSS output

    # Parameters
    avr = PSY.get_avr(dynamic_device)
    Ta_Tb = get_Ta_Tb(avr)
    Tb = get_Tb(avr)
    Ta = Tb * Ta_Tb
    Te = get_Te(avr)
    Tr = get_Tr(avr)
    K = get_K(avr)
    Kc = get_Kc(avr)
    Tc = get_Tc(avr)
    Emin, Emax = get_V_lim(avr)
    Efd_min, Efd_max = get_Efd_lim(avr)

    # Block 1 — voltage transducer:  Vm = Ec / (1 + sTr)
    _, dVm = PSID.low_pass_mass_matrix(V_th, Vm, 1.0, Tr)
    # Summing junction
    e = V0_ref - Vm + Vs
    # Block 2 — lead-lag:  (1 + sTa) / (1 + sTb)
    V_LL, dVr = PSID.lead_lag_mass_matrix(e, Vr, 1.0, Ta, Tb)
    # Block 3 — PI:  Kc (1 + sTc) / (sTc) = Kc + (Kc/Tc)(1/s)  -> kp = Kc, ki = Kc/Tc
    u_pi, dVi = PSID.pi_block(V_LL, Vi, Kc, Kc / Tc)
    # Block 4 — forward lag with non-windup limits:  K / (1 + sTe), clamped to [Emin, Emax]
    Vf_out, dVf = PSID.low_pass_nonwindup_mass_matrix(u_pi, Vf, K, Te, Emin, Emax)
    # Output clamp on the field voltage
    Vf_sat = clamp(Vf_out, Efd_min, Efd_max)

    # Write derivatives back (order matches [:Vm, :Vr, :Vi, :Vf])
    output_ode[local_ix[1]] = dVm
    output_ode[local_ix[2]] = dVr
    output_ode[local_ix[3]] = dVi
    output_ode[local_ix[4]] = dVf

    # Field voltage handed to the machine
    inner_vars[PSID.Vf_var] = Vf_sat
    return
end

#######################################################################################
# 3. Initialization
#######################################################################################
# The integrator forces its own input (V_LL) to zero at equilibrium, which pins
# V_ref = Vm = Ec (terminal voltage). The remaining states then close in form; we still
# solve with NLsolve to mirror the house style used by the other AVRs in init_avr.jl.
function PSID.initialize_avr!(
    device_states,
    static::PSY.StaticInjection,
    dynamic_device::PSID.DynamicWrapper{PSY.DynamicGenerator{M, S, SEXSGE, TG, P}},
    inner_vars::AbstractVector,
) where {M <: PSY.Machine, S <: PSY.Shaft, TG <: PSY.TurbineGov, P <: PSY.PSS}
    # Field voltage solved by the machine, and measured terminal voltage
    Vf0 = inner_vars[PSID.Vf_var]
    Vm = sqrt(inner_vars[PSID.VR_gen_var]^2 + inner_vars[PSID.VI_gen_var]^2)

    avr = PSY.get_avr(dynamic_device)
    Ta_Tb = get_Ta_Tb(avr)
    Tb = get_Tb(avr)
    Ta = Tb * Ta_Tb
    K = get_K(avr)
    Kc = get_Kc(avr)
    Tc = get_Tc(avr)
    Emin, Emax = get_V_lim(avr)

    # Unknowns: V_ref and the states Vr, Vi, Vf  (Vm is fixed at the terminal voltage)
    function f!(out, x)
        V_ref = x[1]
        Vr = x[2]
        Vi = x[3]
        Vf = x[4]
        e = V_ref - Vm                      # Vs = 0 at initialization
        V_LL = Vr + (Ta / Tb) * e
        u_pi = Kc * V_LL + (Kc / Tc) * Vi
        out[1] = (1.0 - Ta / Tb) * e - Vr   # lead-lag steady state
        out[2] = V_LL                       # PI integrator input must vanish
        out[3] = K * u_pi - Vf              # forward lag steady state
        out[4] = Vf - Vf0                   # field voltage matches the machine
    end
    x0 = [1.0, 0.0, Vf0 * Tc / (K * Kc), Vf0]
    sol = NLsolve.nlsolve(f!, x0; ftol = PSID.STRICT_NLSOLVE_F_TOLERANCE)
    if !NLsolve.converged(sol)
        @warn("Initialization of SEXSGE AVR in $(PSY.get_name(static)) failed")
    else
        sol_x0 = sol.zero
        V_ref, Vr, Vi, Vf = sol_x0[1], sol_x0[2], sol_x0[3], sol_x0[4]
        if (Vf > Emax + PSID.BOUNDS_TOLERANCE) || (Vf < Emin - PSID.BOUNDS_TOLERANCE)
            @error(
                "Field voltage Vf = $Vf outside regulator limits [$Emin, $Emax]. Consider updating the operating point."
            )
        end
        PSY.set_V_ref!(avr, V_ref)
        PSID.set_V_ref(dynamic_device, V_ref)
        avr_ix = PSID.get_local_state_ix(dynamic_device, SEXSGE)
        avr_states = @view device_states[avr_ix]
        avr_states[1] = Vm
        avr_states[2] = Vr
        avr_states[3] = Vi
        avr_states[4] = Vf
    end
    return
end

#######################################################################################
# 4. Build the SEXS validation system and hot-swap the exciter for SEXS_GE
#######################################################################################
const REPO = normpath(joinpath(@__DIR__, "..", "..", "..", ".."))
const SEXS_DIR = joinpath(REPO, "test", "benchmarks", "psse", "SEXS")

sys = PSY.System(joinpath(SEXS_DIR, "ThreeBusMulti.raw"), joinpath(SEXS_DIR, "ThreeBus_SEXS.dyr"))
for l in PSY.get_components(PSY.StandardLoad, sys)
    PSID.transform_load_to_constant_impedance(l)
end

# Reach the static generator that carries the SEXS dynamic generator
static = first(
    s for s in PSY.get_components(PSY.StaticInjection, sys) if
    PSY.get_dynamic_injector(s) isa PSY.DynamicGenerator
)
old_dyn = PSY.get_dynamic_injector(static)
old_avr = PSY.get_avr(old_dyn)

# Build SEXS_GE, copying the shared SEXS parameters and adding the GE-specific ones
new_avr = SEXSGE(;
    Ta_Tb = PSY.get_Ta_Tb(old_avr),
    Tb = PSY.get_Tb(old_avr),
    K = PSY.get_K(old_avr),
    Te = PSY.get_Te(old_avr),
    Tr = 0.02,
    Kc = 1.0,
    Tc = 10.0,
    V_lim = PSY.get_V_lim(old_avr),
    Efd_lim = (min = -50.0, max = 50.0),
    V_ref = PSY.get_V_ref(old_avr),
)

# Rebuild the dynamic generator with everything identical EXCEPT the exciter
new_dyn = PSY.DynamicGenerator(;
    name = PSY.get_name(old_dyn),
    ω_ref = PSY.get_ω_ref(old_dyn),
    machine = PSY.get_machine(old_dyn),
    shaft = PSY.get_shaft(old_dyn),
    avr = new_avr,
    prime_mover = PSY.get_prime_mover(old_dyn),
    pss = PSY.get_pss(old_dyn),
    base_power = PSY.get_base_power(old_dyn),
)

# Swap the exciter in place: removes the old dynamic generator and attaches the new one
# (the new injector must share the static injector's name, which it does).
PSY.replace_dynamic_injector!(sys, static, new_dyn)

#######################################################################################
# 5. Simulate a line trip and verify
#######################################################################################
sim_dir = mktempdir()
sim = PSID.Simulation!(
    PSID.ResidualModel,
    sys,
    sim_dir,
    (0.0, 20.0),
    PSID.BranchTrip(1.0, PSY.Line, "BUS 1-BUS 2-i_1"),
)

# Initialization checkpoint — inspect the AVR's initialized states before the transient
@info "Initialized states:"
PSID.show_states_initial_value(sim)

# Small-signal sanity check
small_sig = PSID.small_signal_analysis(sim)
@info "Small-signal stable: $(small_sig.stable)"

# Run with IDA
status = PSID.execute!(sim, Sundials.IDA(); dtmax = 0.005, saveat = 0.005)
@info "Simulation status: $status"
@assert status == PSID.SIMULATION_FINALIZED

results = PSID.read_results(sim)
t_vf, Vf = PSID.get_field_voltage_series(results, PSY.get_name(new_dyn))
t_v, V = PSID.get_voltage_magnitude_series(results, 102)
@info "Field voltage Vf: initial=$(round(Vf[1]; digits=4)), final=$(round(Vf[end]; digits=4))"
@info "Bus 102 |V|: initial=$(round(V[1]; digits=4)), min=$(round(minimum(V); digits=4))"
@info "SEXS_GE tutorial ran successfully."
