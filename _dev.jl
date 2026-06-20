using Revise
using PowerSimulationsDynamics
const PSID = PowerSimulationsDynamics

using Sundials
using PowerSystems
const PSY = PowerSystems
using LinearAlgebra
# using Plots
using Logging
# unicodeplots()
#using OrdinaryDiffEq

sys1 = System("/Users/jdlara/.julia/dev/PowerSimulationsDynamics/sys_1invs.json")
sys3 = System("/Users/jdlara/.julia/dev/PowerSimulationsDynamics/sys_3invs.json")
sys4 = System("/Users/jdlara/.julia/dev/PowerSimulationsDynamics/sys_3invs.json")
remove_components!(sys4, PeriodicVariableSource)

pf_res1 = solve_powerflow(sys1)
pf_res3 = solve_powerflow(sys3)

sim1 = Simulation(
    ResidualModel,
    sys1, #system
    mktempdir(),
    (0.0, 10.0), #time span
    ;
    console_level = Logging.Warn,
) #Type of Fault

sim3 = Simulation(
    ResidualModel,
    sys3, #system
    mktempdir(),
    (0.0, 10.0), #time span
    ;
    console_level = Logging.Warn,
) #Type of Fault

sim4 = Simulation(
    ResidualModel,
    sys4, #system
    mktempdir(),
    (0.0, 10.0), #time span
    ;
    console_level = Logging.Warn,
) #Type of Fault

sim = Simulation(
    MassMatrixModel,
    omib_sys, #system
    mktempdir(),
    tspan, #time span
    ;
    console_level = Logging.Debug,
) #Type of Fault
