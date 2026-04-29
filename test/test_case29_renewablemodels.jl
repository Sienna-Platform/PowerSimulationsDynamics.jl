"""
Case 29:
This case study a three bus system with 1 Generic Renewable Model (REPCA, REECB, REGCA) and an infinite source.
The fault drop the connection between buses 1 and 3, eliminating the direct connection between the load at bus 1
and the generator located in bus 3. The infinite generator is located at bus 2.
"""

##################################################
############### LOAD DATA ########################
##################################################

include(joinpath(TEST_FILES_DIR, "data_tests/test29.jl"))

##################################################
############### SOLVE PROBLEM ####################
##################################################

names = ["RENA: No Flags", "RENA: Freq Flag"]
F_flags = [0, 1]

csv_files = (
    joinpath(TEST_FILES_DIR, "benchmarks/psse/RENA/TEST_RENA_DEFAULT_FLAGS.csv"),
    joinpath(TEST_FILES_DIR, "benchmarks/psse/RENA/TEST_RENA_FREQ_FLAG.csv"),
)

init_conditions = [test29_x0_init, test29_x0_Fflag_init]

eigs_values = [test29_eigvals, test29_eigvals_fflag]

# time span
tspan = (0.0, 5.0);

function test_renA_implicit(csv_file, init_cond, eigs_value, F_Flag)
    path = (joinpath(pwd(), "test-psse-renA"))
    !isdir(path) && mkdir(path)
    try
        sys = System(threebus_file_dir)
        add_source_to_ref(sys)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        for g in get_components(Generator, sys)
            case_gen = inv_generic_renewable(g, F_Flag)
            add_component!(sys, case_gen, g)
        end

        Ybus_change = BranchTrip(1.0, Line, "BUS 1-BUS 3-i_1")

        sim = Simulation(ResidualModel, sys, path, tspan, Ybus_change)

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in init_cond
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - eigs_value) < 1e-2

        # Solve problem
        @test execute!(
            sim,
            IDA();
            saveat = 0.005,
            abstol = 1e-4,
            reltol = 1e-4,
        ) == PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for generator
        t, voltage = get_voltage_magnitude_series(results, 103)
        _, power = get_activepower_series(results, "generator-103-1")
        _, rpower = get_reactivepower_series(results, "generator-103-1")

        M = get_csv_data(csv_file)
        M_t = M[:, 1]
        M_p = M[:, 2]
        M_q = M[:, 3]
        M_v = M[:, 4]
        t_psse, p_psse = clean_extra_timestep!(M_t, M_p)
        _, q_psse = clean_extra_timestep!(M_t, M_q)
        _, v_psse = clean_extra_timestep!(M_t, M_v)

        # Test Transient Simulation Results
        # PSSE results are in Degrees
        @test LinearAlgebra.norm(voltage - v_psse, 2) <= 1e-2
        @test LinearAlgebra.norm(power - p_psse, 2) <= 1e-2
        @test LinearAlgebra.norm(rpower - q_psse, 2) <= 1e-2
        @test LinearAlgebra.norm(t - round.(t_psse, digits = 3)) == 0.0
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

function test_renA_mass_matrix(csv_file, init_cond, eigs_value, F_Flag)
    path = (joinpath(pwd(), "test-psse-renA"))
    !isdir(path) && mkdir(path)
    try
        sys = System(threebus_file_dir)
        add_source_to_ref(sys)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        for g in get_components(Generator, sys)
            case_gen = inv_generic_renewable(g, F_Flag)
            add_component!(sys, case_gen, g)
        end

        Ybus_change = BranchTrip(1.0, Line, "BUS 1-BUS 3-i_1")

        sim = Simulation(MassMatrixModel, sys, path, tspan, Ybus_change)

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in init_cond
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - eigs_value) < 1e-2

        # Solve problem
        @test execute!(
            sim,
            Rodas4();
            dtmax = 0.005,
            saveat = 0.005,
            abstol = 1e-6,
            reltol = 1e-6,
        ) == PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for generator
        t, voltage = get_voltage_magnitude_series(results, 103)
        _, power = get_activepower_series(results, "generator-103-1")
        _, rpower = get_reactivepower_series(results, "generator-103-1")

        M = get_csv_data(csv_file)
        M_t = M[:, 1]
        M_p = M[:, 2]
        M_q = M[:, 3]
        M_v = M[:, 4]
        t_psse, p_psse = clean_extra_timestep!(M_t, M_p)
        _, q_psse = clean_extra_timestep!(M_t, M_q)
        _, v_psse = clean_extra_timestep!(M_t, M_v)

        # Test Transient Simulation Results
        # PSSE results are in Degrees
        @test LinearAlgebra.norm(voltage - v_psse, 2) <= 1e-2
        @test LinearAlgebra.norm(power - p_psse, 2) <= 1e-2
        @test LinearAlgebra.norm(rpower - q_psse, 2) <= 1e-2
        @test LinearAlgebra.norm(t - round.(t_psse, digits = 3)) == 0.0
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 29 RENA ResidualModel" begin
    for (ix, name) in enumerate(names)
        @testset "$(name)" begin
            #dyr_file = dyr_files[ix]
            csv_file = csv_files[ix]
            F_flag = F_flags[ix]
            init_cond = init_conditions[ix]
            eigs_value = eigs_values[ix]
            test_renA_implicit(csv_file, init_cond, eigs_value, F_flag)
        end
    end
end

@testset "Test 29 RENA MassMatrixModel" begin
    for (ix, name) in enumerate(names)
        @testset "$(name)" begin
            #dyr_file = dyr_files[ix]
            csv_file = csv_files[ix]
            F_flag = F_flags[ix]
            init_cond = init_conditions[ix]
            eigs_value = eigs_values[ix]
            test_renA_mass_matrix(csv_file, init_cond, eigs_value, F_flag)
        end
    end
end

"""
Case 29: Test with dyr files
This case study a three bus system with 1 Generic Renewable Model (REPCA, REECB, REGCA) and an infinite source.
The fault drop the connection between buses 1 and 3, eliminating the direct connection between the load at bus 1
and the generator located in bus 3. The infinite generator is located at bus 2.
"""

raw_file_dir = joinpath(TEST_FILES_DIR, "benchmarks/psse/RENA/ThreeBusRenewable.raw")

names_dyr = [
    "RENA: No F_Flag, Ref_Flag = 1, V_Flag = 1, Q_Flag = 0",
    "RENA: No F_Flag, Ref_Flag = 1, V_Flag = 0, Q_Flag = 0",
    "RENA: No F_Flag, Ref_Flag = 1, V_Flag = 1, Q_Flag = 1",
]

csv_files_dyr = [
    joinpath(TEST_FILES_DIR, "benchmarks/psse/RENA/TEST_RENA_NOFREQ_REF_FLAG.csv"),
    joinpath(TEST_FILES_DIR, "benchmarks/psse/RENA/TEST_RENA_NOFREQ_REF_FLAG.csv"),
    joinpath(TEST_FILES_DIR, "benchmarks/psse/RENA/TEST_RENA_NOFREQ_REF_FLAG_Q_FLAG.csv"),
]

dyr_files = [
    joinpath(
        TEST_FILES_DIR,
        "benchmarks/psse/RENA/ThreeBus_REN_A_NOFREQFLAG_with_REF_FLAG.dyr",
    ),
    joinpath(
        TEST_FILES_DIR,
        "benchmarks/psse/RENA/ThreeBus_REN_A_NOFREQFLAG_with_REF_FLAG_no_V_FLAG.dyr",
    ),
    joinpath(
        TEST_FILES_DIR,
        "benchmarks/psse/RENA/ThreeBus_REN_A_NOFREQFLAG_with_REF_FLAG_Q_FLAG.dyr",
    ),
]

init_conditions_dyr =
    [test29_x0_init_Rflag, test29_x0_init_Rflag_no_Vflag, test29_x0_init_Rflag_Qflag]

eigs_values_dyr =
    [test29_eigvals_Rflag, test29_eigvals_Rflag_no_Vflag, test29_eigvals_Rflag_Qflag]

tspans = [(0.0, 10.0), (0.0, 10.0), (0.0, 20.0)]

function test_renA_implicit_dyr(dyr_file, csv_file, init_cond, eigs_value, tspan)
    path = (joinpath(pwd(), "test-psse-renA_dyr"))
    !isdir(path) && mkdir(path)
    try
        sys = System(raw_file_dir, dyr_file)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        # Define Simulation Problem
        sim = Simulation!(
            ResidualModel,
            sys, #system
            path,
            tspan, #time span
            BranchTrip(1.0, Line, "BUS 1-BUS 3-i_1"), #Type of Fault
        ) #Type of Fault

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in init_cond
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        #Test Eigenvalues
        @test LinearAlgebra.norm(eigs - eigs_value) < 1e-2

        # Solve problem
        @test execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for generator
        t, voltage = get_voltage_magnitude_series(results, 103)
        _, angl = get_voltage_angle_series(results, 103)
        _, angl_ref = get_voltage_angle_series(results, 102)

        # Obtain data from PSS/E
        M, _ = readdlm(csv_file, ','; header = true)

        M_t = M[:, 1]
        M_θ = M[:, 2]
        M_v = M[:, 3]

        t_psse, v_psse = clean_extra_timestep!(M_t, M_v)
        _, θ_psse = clean_extra_timestep!(M_t, M_θ)

        # Test Transient Simulation Results
        # PSSE results are in Degrees
        @test LinearAlgebra.norm(voltage - v_psse, 2) <= 1e-2
        @test LinearAlgebra.norm((angl - angl_ref) - θ_psse .* pi / 180, 2) <= 1e-2
        @test LinearAlgebra.norm(t - round.(t_psse, digits = 3)) == 0.0
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

function test_renA_massmatrix_dyr(dyr_file, csv_file, init_cond, eigs_value, tspan)
    path = (joinpath(pwd(), "test-psse-renA_dyr"))
    !isdir(path) && mkdir(path)
    try
        sys = System(raw_file_dir, dyr_file)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        # Define Simulation Problem
        sim = Simulation!(
            MassMatrixModel,
            sys, #system
            path,
            tspan, #time span
            BranchTrip(1.0, Line, "BUS 1-BUS 3-i_1"), #Type of Fault
        ) #Type of Fault

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in init_cond
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        #Test Eigenvalues
        @test LinearAlgebra.norm(eigs - eigs_value) < 1e-2

        # Solve problem
        @test execute!(sim, Rodas4(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for generator
        t, voltage = get_voltage_magnitude_series(results, 103)
        _, angl = get_voltage_angle_series(results, 103)
        _, angl_ref = get_voltage_angle_series(results, 102)

        # Obtain data from PSS/E
        M, _ = readdlm(csv_file, ','; header = true)

        M_t = M[:, 1]
        M_θ = M[:, 2]
        M_v = M[:, 3]

        t_psse, v_psse = clean_extra_timestep!(M_t, M_v)
        _, θ_psse = clean_extra_timestep!(M_t, M_θ)

        # Test Transient Simulation Results
        # PSSE results are in Degrees
        @test LinearAlgebra.norm(voltage - v_psse, 2) <= 1e-2
        @test LinearAlgebra.norm((angl - angl_ref) - θ_psse .* pi / 180, 2) <= 1e-2
        @test LinearAlgebra.norm(t - round.(t_psse, digits = 3)) == 0.0
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 29 RENA ResidualModel with dyr" begin
    for (ix, name) in enumerate(names_dyr)
        @testset "$(name)" begin
            dyr_file = dyr_files[ix]
            csv_file = csv_files_dyr[ix]
            init_cond = init_conditions_dyr[ix]
            eigs_value = eigs_values_dyr[ix]
            tspan = tspans[ix]
            test_renA_implicit_dyr(dyr_file, csv_file, init_cond, eigs_value, tspan)
        end
    end
end

@testset "Test 29 RENA MassMatrixModel with dyr" begin
    for (ix, name) in enumerate(names_dyr)
        @testset "$(name)" begin
            dyr_file = dyr_files[ix]
            csv_file = csv_files_dyr[ix]
            init_cond = init_conditions_dyr[ix]
            eigs_value = eigs_values_dyr[ix]
            tspan = tspans[ix]
            test_renA_massmatrix_dyr(dyr_file, csv_file, init_cond, eigs_value, tspan)
        end
    end
end
