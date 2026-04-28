#=
Inner variables are used for keeping track of internal variables
of different dynamic devices. These are not states, since can be computed
algebraically using states and parameters, and are only used internally in each
component. In such case, it is possible to avoid creating dummy states that
must be added to the vector of states, by handling those as inner variables.
In some cases, depending on the complexity of the model, some inner vars are
also defined as states, but for the sake of generality are also added as inner
variables, since some models may not treat such variables as states.
=#

"""
Generator Inner Vars:
*  `τe_var` :: Electric torque
*  `τm_var` :: Mechanical torque
*  `Vf_var` :: Field voltage
*  `V_pss_var` :: Additional PSS voltage
*  `VR_gen_var` :: Real part of the terminal voltage
*  `VI_gen_var` :: Imaginary part of the terminal voltage
*  `ψd_var` :: Stator Flux (if defined) in the d-axis
*  `ψq_var` :: Stator Flux (if defined) in the q-axis
"""
@enum generator_inner_vars begin
    τe_var = 1
    τm_var = 2
    Vf_var = 3
    V_pss_var = 4
    VR_gen_var = 5
    VI_gen_var = 6
    ψd_var = 7
    ψq_var = 8
    Xad_Ifd_var = 9
end

Base.to_index(ix::generator_inner_vars) = Int(ix)

"""
Inverter Inner Vars:
* `md_var` :: Modulation signal on the d-component
* `mq_var` :: Modulation signal on the q-component
* `Vdc_var` :: DC voltage supplied by the DC source
* `Vr_filter_var` :: Voltage seen in the capacitor of the filter in the R-component
* `Vi_filter_var` :: Voltage seen in the capacitor of the filter in the I-component
* `θ_freq_estimator_var` :: Angle estimated by the frequency estimator.
* `ω_freq_estimator_var` :: Frequency estimated by the frequency estimator.
* `V_oc_var` :: Control voltage reference in the d-axis supplied from the outer loop control to the inner loop (for Voltage Mode Control)
* `Id_oc_var` :: Control current reference in the d-axis supplied from the outer loop control to the inner loop (for Current Mode Control)
* `Iq_oc_var` :: Control current reference in the q-axis supplied from the outer loop control to the inner loop (for Current Mode Control)
* `Id_ic_var` :: Control current reference in the d-axis supplied from the inner loop control to the converter (for Generic Models)
* `Iq_ic_var` :: Control current reference in the q-axis supplied from the inner loop control to the converter (for Generic Models)
* `Ir_cnv_var` :: Control current reference in the R-axis supplied from the converter to the filter (for Generic Models)
* `Ii_cnv_var` :: Control current reference in the I-axis supplied from the converter to the filter (for Generic Models)
* `ω_oc_var` :: Control frequency supplied from the outer loop control the inner loop
* `θ_oc_var` :: Variation of the angle (PLL or VSM) of the inverter
* `Vr_inv_var` :: Real terminal voltage on the inverter
* `Vi_inv_var` :: Imaginary terminal voltage on the inverter
* `Vr_cnv_var` :: Voltage supplied from the converter in the R-component
* `Vi_cnv_var` :: Voltage supplied from the converter in the I-component
* `P_ES_var` :: Power supplied from the Energy Source side
"""
@enum inverter_inner_vars begin
    md_var = 1
    mq_var = 2
    Vdc_var = 3
    Vr_filter_var = 4
    Vi_filter_var = 5
    θ_freq_estimator_var = 6
    ω_freq_estimator_var = 7
    V_oc_var = 8
    Id_oc_var = 9
    Iq_oc_var = 10
    Id_ic_var = 11
    Iq_ic_var = 12
    Ir_cnv_var = 13
    Ii_cnv_var = 14
    Ir_filter_var = 15
    Ii_filter_var = 16
    Ir_inv_var = 17
    Ii_inv_var = 18
    ω_oc_var = 19
    θ_oc_var = 20
    Vr_inv_var = 21
    Vi_inv_var = 22
    Vr_cnv_var = 23
    Vi_cnv_var = 24
    P_ES_var = 25
end

Base.to_index(ix::inverter_inner_vars) = Int(ix)

@enum dq_ref begin
    d = 1
    q = 2
end

@enum RI_ref begin
    R = 1
    I = 2
end

Base.to_index(ix::dq_ref) = Int(ix)
Base.to_index(ix::RI_ref) = Int(ix)

const MAPPING_DICT = Dict{String, OrderedDict{Symbol, Int}}
const DEVICE_INTERNAL_MAPPING = Base.ImmutableDict{Int, Vector{Int}}

const GEN_INNER_VARS_SIZE = 9
const INV_INNER_VARS_SIZE = 25
const PVS_INNER_VARS_SIZE = 0
const VOLTAGE_DIVISION_LOWER_BOUND = 0.01

const SIMULATION_ACCEPTED_KWARGS = [
    :initialize_simulation,
    :initial_conditions,
    :system_to_file,
    :file_level,
    :console_level,
    :frequency_reference,
    :all_lines_dynamic,
    :all_branches_dynamic,
    :disable_timer_outputs,
]
# Location of the global vars in the Caches
const GLOBAL_VAR_SYS_FREQ_INDEX = 1
get_vars_ix() = Dict{Int, Int}(GLOBAL_VAR_SYS_FREQ_INDEX => -1)

const RELAXED_NLSOLVE_F_TOLERANCE = :1e-6
const STRICT_NLSOLVE_F_TOLERANCE = :1e-9
const NLSOLVE_X_TOLERANCE = :1e-9
const MINIMAL_ACCEPTABLE_NLSOLVE_F_TOLERANCE = :1e-4
const MAX_INIT_RESIDUAL = 10.0
const MAX_NLSOLVE_INTERATIONS = 10
const BOUNDS_TOLERANCE = 1e-6

const SIMULATION_LOG_FILENAME = "power-simulations-dynamics.log"

const ACCEPTED_CONTROL_REFS = [:V_ref, :ω_ref, :P_ref, :Q_ref]

const ACCEPTED_LOADCHANGE_REFS = [
    :P_ref,
    :Q_ref,
    :P_ref_impedance,
    :Q_ref_impedance,
    :P_ref_power,
    :Q_ref_power,
    :P_ref_current,
    :Q_ref_current,
]

const DIFFEQ_SOLVE_KWARGS = [
    :dense,
    :saveat,
    :save_idxs,
    :tstops,
    :d_discontinuities,
    :save_everystep,
    :save_on,
    :save_start,
    :save_end,
    :initialize_save,
    :adaptive,
    :abstol,
    :reltol,
    :dt,
    :dtmax,
    :dtmin,
    :force_dtmin,
    :internalnorm,
    :controller,
    :failfactor,
    :calck,
    :alias,
    :maxiters,
    :callback,
    :isoutofdomain,
    :unstable_check,
    :verbose,
    :merge_callbacks,
    :progress,
    :progress_steps,
    :progress_name,
    :progress_message,
    :timeseries_errors,
    :dense_errors,
    :weak_timeseries_errors,
    :weak_dense_errors,
    :calculate_errors,
    :initializealg,
    :alg,
    :save_noise,
    :delta,
    :seed,
    :alg_hints,
    :kwargshandle,
    :trajectories,
    :batch_size,
    :sensealg,
    :advance_to_tstop,
    :stop_at_next_tstop,
    :default_set,
    :second_time,
    :prob_choice,
]
"""
Defines the status of the simulation object
"""
@enum BUILD_STATUS begin
    BUILT = 0
    BUILD_IN_PROGRESS = 1
    BUILD_INCOMPLETE = 2
    BUILD_FAILED = 3
    SIMULATION_INITIALIZED = 4
    SIMULATION_STARTED = 5
    SIMULATION_FINALIZED = 6
    SIMULATION_FAILED = 7
end

const BUILD_TIMER = TimerOutputs.TimerOutput()

const ACCEPTED_REAL_TYPES = Union{Float64, ForwardDiff.Dual}
