# # Tutorial Active Constant Power Load model
#
# ## Introduction
#
# This tutorial will introduce you to the functionality of `PowerSimulationsDynamics` and
# [`PowerSystems.jl`](https://sienna-platform.github.io/PowerSystems.jl/stable/) to explore active load components and a small-signal analysis.
#
# This tutorial presents a simulation of a two-bus system with a GFM inverter at bus 1, and
# a load on bus 2. We will change the model from a constant power load model, to a constant
# impedance model and then to a 12-state active constant power load model.
#
# ## Dependencies

using PowerSimulationsDynamics;
PSID = PowerSimulationsDynamics
using PowerSystemCaseBuilder
using PowerSystems
const PSY = PowerSystems;

# !!! note
#     `PowerSystemCaseBuilder.jl` is a helper library that makes it easier to reproduce
#     examples in the documentation and tutorials. Normally you would pass your local files
#     to create the system data instead of calling the function [`PowerSystemCaseBuilder.build_system`](@extref).
#
# [`PowerSystems.jl`](https://sienna-platform.github.io/PowerSystems.jl/stable/) (abbreviated with `PSY`) is used to properly define the data structure and
# establish an equilibrium point initial condition with a power flow routine using `PowerFlows`.
#
# ## Load the system
#
# We load the system using `PowerSystemCaseBuilder.jl`. This system has an inverter located
# at bus 1.

sys = build_system(PSIDSystems, "2 Bus Load Tutorial Droop")

#

first(get_components(DynamicInverter, sys))

# The load is an exponential load modeled as a constant power load since the coefficients
# are set to zero.

first(get_components(PSY.ExponentialLoad, sys))

# ## Run a small-signal analysis
#
# We set up the [`Simulation`](@ref). Since the droop model does not have a frequency state, we use a
# constant frequency reference frame for the network.

sim = Simulation(ResidualModel,
    sys,
    mktempdir(),
    (0.0, 1.0);
    frequency_reference = ConstantFrequency())

# The following provides a summary of eigenvalues for this droop system with a constant
# power load:

sm = small_signal_analysis(sim);
df = summary_eigenvalues(sm);
show(df; allrows = true, allcols = true)

# In this inverter model, the filter is modeled using differential equations, and as
# described in the literature, interfacing a RL filter against an algebraic constant power
# load usually results in unstable behavior as observed with the positive real part eigenvalue.
#
# ## Change to a constant impedance load model
#
# Since the load is an exponential load model we can change the exponent coefficients to 2.0
# to behave as a constant impedance model:

## Update load coefficients to 2.0
load = first(get_components(PSY.ExponentialLoad, sys));
PSY.set_α!(load, 2.0);
PSY.set_β!(load, 2.0);

# We then re-run the small-signal analysis:

sim = Simulation(ResidualModel,
    sys,
    mktempdir(),
    (0.0, 1.0);
    frequency_reference = ConstantFrequency())

#

sm = small_signal_analysis(sim);
df = summary_eigenvalues(sm);
show(df; allrows = true, allcols = true)

# Observe that now the system is small-signal stable (since there is only one device the
# angle of the inverter is used as a reference, and hence is zero).
#
# ## Adding a dynamic active load model
#
# To consider a dynamic model in the load it is only required to attach a dynamic component
# to the static load model. When a dynamic load model is attached, the active and reactive
# power of the static model are used to define reference parameters to ensure that the
# dynamic load model matches the static load output power.
#
# Note that when a dynamic model is attached to a static model, the static model does not
# participate in the dynamic system equations, i.e. the only model interfacing to the network
# equations is the dynamic model and not the static model (the exponential load).
#
# We define a function to create a active load model with the specific parameters:

## Parameters taken from active load model from N. Bottrell Masters
## Thesis "Small-Signal Analysis of Active Loads and Large-signal Analysis
## of Faults in Inverter Interfaced Microgrid Applications", 2014.

## The parameters are then per-unitized to be scalable to represent an aggregation
## of multiple active loads

## Base AC Voltage: Vb = 380 V
## Base Power (AC and DC): Pb = 10000 VA
## Base AC Current: Ib = 10000 / 380 = 26.32 A
## Base AC Impedance: Zb = 380 / 26.32 =  14.44 Ω
## Base AC Inductance: Lb = Zb / Ωb = 14.44 / 377 = 0.3831 H
## Base AC Capacitance: Cb = 1 / (Zb * Ωb) = 0.000183697 F
## Base DC Voltage: Vb_dc = (√8/√3) Vb = 620.54 V
## Base DC Current: Ib_dc = Pb / V_dc = 10000/620.54 = 16.12 A
## Base DC Impedance: Zb_dc = Vb_dc / Ib_dc = 38.50 Ω
## Base DC Capacitance: Cb_dc = 1 / (Zb_dc * Ωb) = 6.8886315e-5 F

Ωb = 2 * pi * 60;
Vb = 380;
Pb = 10000;
Ib = Pb / Vb;
Zb = Vb / Ib;
Lb = Zb / Ωb;
Cb = 1 / (Zb * Ωb);
Vb_dc = sqrt(8) / sqrt(3) * Vb;
Ib_dc = Pb / Vb_dc;
Zb_dc = Vb_dc / Ib_dc;
Cb_dc = 1 / (Zb_dc * Ωb);

function active_cpl(load)
    return PSY.ActiveConstantPowerLoad(;
        name = get_name(load),
        r_load = 70.0 / Zb_dc,
        c_dc = 2040e-6 / Cb_dc,
        rf = 0.1 / Zb,
        lf = 2.3e-3 / Lb,
        cf = 8.8e-6 / Cb,
        rg = 0.03 / Zb,
        lg = 0.93e-3 / Lb,
        kp_pll = 0.4,
        ki_pll = 4.69,
        kpv = 0.5 * (Vb_dc / Ib_dc),
        kiv = 150.0 * (Vb_dc / Ib_dc),
        kpc = 15.0 * (Ib / Vb),
        kic = 30000.0 * (Ib / Vb),
        base_power = 100.0,
    )
end

# We then attach the model to the system:

load = first(get_components(PSY.ExponentialLoad, sys));
dyn_load = active_cpl(load)
add_component!(sys, dyn_load, load)

# Finally, we set up the simulation:

sim = Simulation(ResidualModel,
    sys,
    mktempdir(),
    (0.0, 1.0);
    frequency_reference = ConstantFrequency())

#

sm = small_signal_analysis(sim);
df = summary_eigenvalues(sm);
show(df; allrows = true, allcols = true)

# Observe the new states of the active load model and that the system is small-signal stable.
