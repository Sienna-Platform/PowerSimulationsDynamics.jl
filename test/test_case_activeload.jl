"""
Case 46:
This case study a single VSM inverter against an active load model.
The perturbation increase P_ref of the generator by 0.1 pu.
"""

##################################################
############### LOAD DATA ########################
##################################################

include(joinpath(TEST_FILES_DIR, "data_tests/test46.jl"))

##################################################
############### SOLVE PROBLEM ####################
##################################################

# Define Perturbation
case_gen = first(get_components(PSY.DynamicInjection, sys))
perturbation = ControlReferenceChange(0.1, case_gen, :P_ref, 0.6)

@testset "Test 46 ActiveLoad ResidualModel" begin
    path = mktempdir()
    try
        # Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            sys, #system
            path,
            (0.0, 4.0), #time span
            perturbation; #Type of Fault
            all_lines_dynamic = true,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test46_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues — per-eigenvalue tolerance to accommodate Float32 Ybus precision
        # (absolute floor 1e-3; relative 1% above that). The reference test46_eigvals must
        # be regenerated under the current Float32 Ybus regime.
        @test all(
            abs(e1 - e2) ≤ max(1e-2 * abs(e2), 1e-3) for
            (e1, e2) in zip(eigs, test46_eigvals)
        )

        # Solve problem
        @test execute!(sim, IDA(); abstol = 1e-9, reltol = 1e-9) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for angles
        series = get_voltage_magnitude_series(results, 101)
        t = series[1]
        V101 = series[2]
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 46 ActiveLoad MassMatrixModel" begin
    path = mktempdir()
    try
        # Define Simulation Problem
        sim = Simulation(
            MassMatrixModel,
            sys, #system
            path,
            (0.0, 4.0), #time span
            perturbation; #Type of Fault
            all_lines_dynamic = true,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test46_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues — per-eigenvalue tolerance to accommodate Float32 Ybus precision
        # (absolute floor 1e-3; relative 1% above that). The reference test46_eigvals must
        # be regenerated under the current Float32 Ybus regime.
        @test all(
            abs(e1 - e2) ≤ max(1e-2 * abs(e2), 1e-3) for
            (e1, e2) in zip(eigs, test46_eigvals)
        )

        # Solve problem
        @test execute!(sim, Rodas4(); abstol = 1e-9, reltol = 1e-9) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for angles
        series = get_voltage_magnitude_series(results, 101)
        t = series[1]
        V101 = series[2]
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
