"""
Validation PSSE/DEGOV:
This case study defines a three bus system with an infinite bus, ClassicMachine+DEGOV and a load.
The fault drop the line connecting the infinite bus and generator
"""

##################################################
############### SOLVE PROBLEM ####################
##################################################
raw_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/DEGOV/ThreeBusMulti.raw")
dyr_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/DEGOV/ThreeBus_GAST_simple.dyr") #manually replace with DEGOV

csv_file_degov_nodelay_speed =
    joinpath(TEST_FILES_DIR, "benchmarks/psse/DEGOV/degov_nodelay_speed_final.csv")
csv_file_degov_delay_speed =
    joinpath(TEST_FILES_DIR, "benchmarks/psse/DEGOV/degov_delay_speed_final.csv")

@testset "Test 57 DEGOV MassMatrixModel" begin
    path = mktempdir()
    try
        sys = System(raw_file, dyr_file)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end
        gen = get_component(ThermalStandard, sys, "generator-102-1")
        dyn_gen = get_component(DynamicGenerator, sys, "generator-102-1")

        new_gov = PSY.DEGOV(;
            T1 = 0.0,
            T2 = 0.0,
            T3 = 0.0,
            K = 18.0,
            T4 = 12.0,
            T5 = 5.0,
            T6 = 0.2,
            Td = 0.0,
            P_ref = 0.0,
        )
        dyn_gen_new = DynamicGenerator(;
            name = get_name(dyn_gen),
            ω_ref = get_ω_ref(dyn_gen),
            machine = get_machine(dyn_gen),
            shaft = get_shaft(dyn_gen),
            avr = get_avr(dyn_gen),
            prime_mover = new_gov,
            pss = get_pss(dyn_gen),
            base_power = get_base_power(dyn_gen),
        )
        remove_component!(sys, dyn_gen)
        add_component!(sys, dyn_gen_new, gen)

        # Define Simulation Problem
        sim = Simulation!(
            MassMatrixModel,
            sys,
            path,
            (0.0, 5.0),
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1");
            frequency_reference = ReferenceBus(),
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test_57_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end
        @test (diff_val[1] < 1e-5)

        @test_throws ErrorException small_signal_analysis(sim)

        # Solve problem
        @test execute!(
            sim,
            MethodOfSteps(Rodas4(; autodiff = AutoFiniteDiff()));
            dtmax = (1 / 240),
            saveat = (1 / 240),
        ) ==
              PSID.SIMULATION_FINALIZED
        results_nodelay = read_results(sim)

        t_psid, ω_psid_nodelay = get_frequency_series(results_nodelay, "generator-102-1")
        t_pw, ω_pw_nodelay = get_csv_delta(csv_file_degov_nodelay_speed)
        @test LinearAlgebra.norm(ω_pw_nodelay - ω_psid_nodelay, Inf) <= 4.7e-5

        # Add delay 
        set_Td!(get_prime_mover(dyn_gen_new), 1.0)
        sim = Simulation!(
            MassMatrixModel,
            sys,
            path,
            (0.0, 5.0),
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1");
            frequency_reference = ReferenceBus(),
        )

        @test execute!(
            sim,
            MethodOfSteps(Rodas4(; autodiff = AutoFiniteDiff()));
            saveat = (1 / 240),
        ) ==
              PSID.SIMULATION_FINALIZED
        results_delay = read_results(sim)

        t_psid, ω_psid_delay = get_frequency_series(results_delay, "generator-102-1")
        t_pw, ω_pw_delay = get_csv_delta(csv_file_degov_delay_speed)
        @test LinearAlgebra.norm(ω_pw_delay - ω_psid_delay, Inf) <= 3.1e-5

        #Plotting for debug (add PlotlyJS):
        #=         t1 = PlotlyJS.scatter(; x = t_pw, y = ω_pw_nodelay, name = "speed-pw -- no delay")
                t2 = PlotlyJS.scatter(;
                    x = t_psid,
                    y = ω_psid_nodelay,
                    name = "speed-psid -- no delay",
                )
                t3 = PlotlyJS.scatter(; x = t_pw, y = ω_pw_delay, name = "speed-pw -- 1s delay")
                t4 = PlotlyJS.scatter(;
                    x = t_psid,
                    y = ω_psid_delay,
                    name = "speed-psid -- 1s delay",
                )
                display(PlotlyJS.plot([t1, t2, t3, t4])) =#

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 57 DEGOV ResidualModel" begin
    path = mktempdir()
    try
        sys = System(raw_file, dyr_file)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end
        gen = get_component(ThermalStandard, sys, "generator-102-1")
        dyn_gen = get_component(DynamicGenerator, sys, "generator-102-1")

        new_gov = PSY.DEGOV(;
            T1 = 0.0,
            T2 = 0.0,
            T3 = 0.0,
            K = 18.0,
            T4 = 12.0,
            T5 = 5.0,
            T6 = 0.2,
            Td = 0.0,
            P_ref = 0.0,
        )
        dyn_gen_new = DynamicGenerator(;
            name = get_name(dyn_gen),
            ω_ref = get_ω_ref(dyn_gen),
            machine = get_machine(dyn_gen),
            shaft = get_shaft(dyn_gen),
            avr = get_avr(dyn_gen),
            prime_mover = new_gov,
            pss = get_pss(dyn_gen),
            base_power = get_base_power(dyn_gen),
        )
        remove_component!(sys, dyn_gen)
        add_component!(sys, dyn_gen_new, gen)

        # Define Simulation Problem
        sim = Simulation!(
            ResidualModel,
            sys,
            path,
            (0.0, 5.0),
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1");
            frequency_reference = ReferenceBus(),
        )
        @test sim.status == PSID.BUILD_FAILED

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
