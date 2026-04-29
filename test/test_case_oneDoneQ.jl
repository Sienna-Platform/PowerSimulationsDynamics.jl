"""
Case 2:
This case study a three bus system with 2 machines (One d- One q-: 4th order model) and an infinite source.
The fault drop the connection between buses 1 and 3, eliminating the direct connection between the infinite source
and the generator located in bus 3.
"""

##################################################
############### LOAD DATA ########################
##################################################

threebus_sys = build_system(PSIDTestSystems, "psid_test_threebus_oneDoneQ")
pf = ACPowerFlow()
solve_and_store_power_flow!(pf, threebus_sys)
Ybus_fault = get_ybus_fault_threebus_sys(threebus_sys)

##################################################
############### SOLVE PROBLEM ####################
##################################################

@testset "Test 02 OneDoneQ ResidualModel" begin
    path = mktempdir()
    try
        Ybus_change = NetworkSwitch(
            1.0, #change at t = 1.0
            Ybus_fault,
        )
        # Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            threebus_sys, #system
            path,
            (0.0, 20.0), #time span
            Ybus_change, #Type of Fault
        ) #initial guess

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test02_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - test02_eigvals) < 1e-3
        @test LinearAlgebra.norm(eigs - test02_eigvals_psat, Inf) < 5.0

        # Solve problem
        @test execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-102-1", :δ))
        t = series[1]
        δ = series[2]

        # Test Mechanical Torque
        _, τm = get_mechanical_torque_series(results, "generator-102-1")
        setpoints = get_setpoints(results)
        P_ref = setpoints["generator-102-1"]["P_ref"]
        @test P_ref == τm[1] == τm[end]

        # Obtain PSAT benchmark data
        psat_csv = joinpath(TEST_FILES_DIR, "benchmarks/psat/Test02/Test02_delta.csv")
        t_psat, δ_psat = get_csv_delta(psat_csv)

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(t - t_psat) == 0.0
        @test LinearAlgebra.norm(δ - δ_psat, Inf) <= 1e-3

        power = PSID.get_activepower_series(results, "generator-102-1")
        rpower = PSID.get_reactivepower_series(results, "generator-102-1")
        @test isa(power, Tuple{Vector{Float64}, Vector{Float64}})
        @test isa(rpower, Tuple{Vector{Float64}, Vector{Float64}})
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 02 OneDoneQ MassMatrixModel" begin
    path = mktempdir()
    try
        Ybus_change = NetworkSwitch(
            1.0, #change at t = 1.0
            Ybus_fault,
        )
        # Define Simulation Problem
        sim = Simulation(
            MassMatrixModel,
            threebus_sys, #system
            path,
            (0.0, 20.0), #time span
            Ybus_change, #Type of Fault
        ) #initial guess

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test02_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - test02_eigvals) < 1e-3
        @test LinearAlgebra.norm(eigs - test02_eigvals_psat, Inf) < 5.0

        # Solve problem
        @test execute!(sim, Rodas4(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-102-1", :δ))
        t = series[1]
        δ = series[2]

        # Test Mechanical Torque
        _, τm = get_mechanical_torque_series(results, "generator-102-1")
        setpoints = get_setpoints(results)
        P_ref = setpoints["generator-102-1"]["P_ref"]
        @test P_ref == τm[1] == τm[end]

        # Obtain PSAT benchmark data
        psat_csv = joinpath(TEST_FILES_DIR, "benchmarks/psat/Test02/Test02_delta.csv")
        t_psat, δ_psat = get_csv_delta(psat_csv)

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(t - t_psat) == 0.0
        @test LinearAlgebra.norm(δ - δ_psat, Inf) <= 1e-3

        power = PSID.get_activepower_series(results, "generator-102-1")
        rpower = PSID.get_reactivepower_series(results, "generator-102-1")
        @test isa(power, Tuple{Vector{Float64}, Vector{Float64}})
        @test isa(rpower, Tuple{Vector{Float64}, Vector{Float64}})
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
