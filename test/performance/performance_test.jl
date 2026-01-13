precompile = @timed using PowerSimulationsDynamics

using PowerSimulationsDynamics
import PowerSimulationsDynamics as PSID
using Sundials
using PowerSystems
import PowerSystems as PSY
using OrdinaryDiffEq
using Logging

open("precompile_time.txt", "a") do io
    write(io, "| $(ARGS[1]) | $(precompile.time) |\n")
end
include("../utils/get_results.jl")
sys = System("test/data_tests/WECC_240_dynamic.json"; runchecks = false)
for l in get_components(PSY.StandardLoad, sys)
    transform_load_to_constant_impedance(l)
end

# First runs
try
    for m in [ResidualModel, MassMatrixModel]
        Simulation(
            m,
            sys,
            pwd(),
            (0.0, 20.0), #time span
            BranchTrip(1.0, Line, "CORONADO    -1101-PALOVRDE    -1401-i_10");
            console_level = Logging.Error,
        )
    end
catch e
    @error exception = (e, catch_backtrace())
end

try
    sim_ida, time_build_ida, _, _ = @timed Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0), #time span
        BranchTrip(1.0, Line, "CORONADO    -1101-PALOVRDE    -1401-i_10");
        console_level = Logging.Error,
    )
    status = execute!(sim_ida, IDA(); dtmax = 0.01, enable_progress_bar = false)
    if status == PSID.SIMULATION_FINALIZED
        res_ida = read_results(sim_ida)
        solve_time = res_ida.time_log[:timed_solve_time]
        open("execute_time.txt", "a") do io
            write(io, "| $(ARGS[1])-Build ResidualModel | $(time_build_ida) |\n")
            write(io, "| $(ARGS[1])-Execute ResidualModel | $(solve_time) |\n")
        end
    else
        error("FAILED TO SOLVE")
    end
catch e
    @error exception = (e, catch_backtrace())
    open("execute_time.txt", "a") do io
        write(io, "| $(ARGS[1])- ResidualModel | FAILED TO TEST |\n")
    end
end

try
    sim_rodas, time_build_rodas, _, _ = @timed Simulation(
        MassMatrixModel,
        sys, #system
        pwd(),
        (0.0, 20.0), #time span
        BranchTrip(1.0, Line, "CORONADO    -1101-PALOVRDE    -1401-i_10"),
        #console_level = Logging.Error,
    ) #Type of Fault
    status = execute!(sim_rodas, Rodas4())
    if status == PSID.SIMULATION_FINALIZED
        res_rodas = read_results(sim_rodas)
        solve_time = res_rodas.time_log[:timed_solve_time]
        open("execute_time.txt", "a") do io
            write(io, "| $(ARGS[1])-Build MassMatrixModel | $(time_build_rodas) |\n")
            write(io, "| $(ARGS[1])-Execute MassMatrixModel | $(solve_time) |\n")
        end
    else
        error("FAILED TO SOLVE")
    end
catch e
    @error exception = (e, catch_backtrace())
    open("execute_time.txt", "a") do io
        write(io, "| $(ARGS[1])-MassMatrixModel | FAILED TO TEST |\n")
    end
end
