"""
Case 27:
This case study a three bus system with 1 machine (One d- One q-: 4th order model), a VSM of 19 states and an infinite source.
The test changes botht he voltage magnitude and phase angle of the source bus.
"""

##################################################
############### LOAD DATA ########################
##################################################

# Use the sme test data as Test 09
threebus_sys = build_system(PSIDTestSystems, "psid_test_threebus_machine_vsm")

##################################################
############### SOLVE PROBLEM ####################
##################################################

####### Changing magnitude of votlage at source bus #########

# time span
tspan = (0.0, 20.0);
case_source = collect(PSY.get_components(PSY.Source, threebus_sys))[1]

# Define Fault using Callbacks
V_source_change = SourceBusVoltageChange(1.0, case_source, :V_ref, 1.02)

@testset "Test 27 Source Bus Voltage Magnitude Perturbation ResidualModel" begin
    path = mktempdir()
    try
        # Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            threebus_sys, # system
            path,
            tspan,
            V_source_change,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test09_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end
        @test (diff_val[1] < 1e-3)

        # Solve problem
        execute!(sim, IDA(); abstol = 1e-4, reltol = 1e-4)
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-103-1", :θ_oc))
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 27 Source Bus Voltage Magnitude Perturbation MassMatrixModel" begin
    path = mktempdir()
    try
        # Define Simulation Problem
        sim = Simulation(
            MassMatrixModel,
            threebus_sys, # system
            path,
            tspan,
            V_source_change,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test09_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end
        @test (diff_val[1] < 1e-3)

        # Solve problem
        execute!(sim, Rodas4(); abstol = 1e-4, reltol = 1e-4)
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-103-1", :θ_oc))
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

####### Changing angle of voltage at source bus #########

#time span
tspan = (0.0, 20.0);
case_source = collect(PSY.get_components(PSY.Source, threebus_sys))[1]

#Define Fault using Callbacks
V_source_change = SourceBusVoltageChange(1.0, case_source, :θ_ref, 0.1)

@testset "Test 27 Source Bus Voltage Angle Perturbation ResidualModel" begin
    path = (joinpath(pwd(), "test-27"))
    !isdir(path) && mkdir(path)
    try
        # Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            threebus_sys, # system
            path,
            tspan,
            V_source_change,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test09_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end
        @test (diff_val[1] < 1e-3)

        # Solve problem
        execute!(sim, IDA(); abstol = 1e-4, reltol = 1e-4)
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-103-1", :θ_oc))
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 27 Source Bus Voltage Angle Perturbation MassMatrixModel" begin
    path = (joinpath(pwd(), "test-27"))
    !isdir(path) && mkdir(path)
    try
        sim = Simulation(
            MassMatrixModel,
            threebus_sys, # system
            path,
            tspan,
            V_source_change,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test09_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end
        @test (diff_val[1] < 1e-3)

        # Solve problem
        execute!(sim, Rodas4(); abstol = 1e-4, reltol = 1e-4)
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-103-1", :θ_oc))
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
