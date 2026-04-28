using PowerSimulationsDynamics
using PowerSystems
using Test
using NLsolve
using SciMLBase
using Sundials
using OrdinaryDiffEq
using OrdinaryDiffEqRosenbrock
using ADTypes
using DelayDiffEq
using DelimitedFiles
using DataFrames
using InfrastructureSystems
using PowerSystemCaseBuilder
using PowerFlows
using PowerNetworkMatrices
import LinearAlgebra
using Logging

import Aqua
Aqua.test_unbound_args(PowerSimulationsDynamics)
Aqua.test_undefined_exports(PowerSimulationsDynamics)
Aqua.test_ambiguities(PowerSimulationsDynamics)
Aqua.test_stale_deps(PowerSimulationsDynamics)
Aqua.test_deps_compat(PowerSimulationsDynamics)

const IS = InfrastructureSystems
const PSY = PowerSystems
const PSID = PowerSimulationsDynamics
const PNM = PowerNetworkMatrices
const PSB = PowerSystemCaseBuilder

const DISABLED_TEST_FILES = [
    "test_case49_csvgn1.jl", # This test is currently disabled due to convergence issues with the PowerFlow static case.
]
test_file_dir = isempty(dirname(@__FILE__)) ? "test" : dirname(@__FILE__)
const TEST_FILES_DIR = test_file_dir

ENV["RUNNING_PSID_TESTS"] = "true"

"""
Copied @includetests from https://github.com/ssfrr/TestSetExtensions.jl.
Ideally, we could import and use TestSetExtensions.  Its functionality was broken by changes
in Julia v0.7.  Refer to https://github.com/ssfrr/TestSetExtensions.jl/pull/7.
"""

"""
Includes the given test files, given as a list without their ".jl" extensions.
If none are given it will scan the directory of the calling file and include all
the julia files.
"""
macro includetests(testarg...)
    if length(testarg) == 0
        tests = []
    elseif length(testarg) == 1
        tests = testarg[1]
    else
        error("@includetests takes zero or one argument")
    end

    quote
        tests = $tests
        rootfile = @__FILE__
        if length(tests) == 0
            tests = readdir(dirname(rootfile))
            tests = filter(
                f ->
                    startswith(f, "test_") && endswith(f, ".jl") && f != basename(rootfile),
                tests,
            )
        else
            tests = map(f -> string(f, ".jl"), tests)
        end
        println()
        for test in tests
            test ∈ DISABLED_TEST_FILES && continue
            print(splitext(test)[1], ": ")
            include(test)
            println()
        end
    end
end

const LOG_FILE = "psid-test.log"

LOG_LEVELS = Dict(
    "Debug" => Logging.Debug,
    "Info" => Logging.Info,
    "Warn" => Logging.Warn,
    "Error" => Logging.Error,
)

function get_logging_level(env_name::String, default)
    level = get(ENV, env_name, default)
    log_level = get(LOG_LEVELS, level, nothing)
    if isnothing(log_level)
        error("Invalid log level $level: Supported levels: $(values(LOG_LEVELS))")
    end

    return log_level
end

function run_tests()
    console_level = get_logging_level("SYS_CONSOLE_LOG_LEVEL", "Error")
    console_logger = ConsoleLogger(stderr, console_level)
    file_level = get_logging_level("SYS_LOG_LEVEL", "Info")

    include("utils/get_results.jl")
    include("utils/mock_structs.jl")
    include("utils/data_utils.jl")
    include("./data_tests/dynamic_test_data.jl")
    include("./results/results_initial_conditions.jl")
    include("./results/results_eigenvalues.jl")

    IS.open_file_logger(LOG_FILE, file_level) do file_logger
        multi_logger = IS.MultiLogger(
            [console_logger, file_logger],
            IS.LogEventTracker((Logging.Info, Logging.Warn, Logging.Error)),
        )
        global_logger(multi_logger)

        # Testing Topological components of the schema
        @time @testset "Begin PowerSimulationsDynamics tests" begin
            @includetests ARGS
        end

        @test length(IS.get_log_events(multi_logger.tracker, Logging.Error)) == 0
        @info IS.report_log_summary(multi_logger)
    end
end

logger = global_logger()

try
    run_tests()
finally
    # Guarantee that the global logger is reset.
    global_logger(logger)
    nothing
end
