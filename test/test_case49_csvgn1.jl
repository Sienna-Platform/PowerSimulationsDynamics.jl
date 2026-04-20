"""
Test for Dynamic Injector model : CSVGN1 available in PSS/e
This case study defines a three bus system with an infinite bus in 1,
a constant impedance load in bus 2 a a constant impedance load in bus 3
and a CSVGn1 in bus 3
The disturbance is the change in the impedance load in bus 2 from 455 MW
to 400 MW
"""
##################################################
############### LOAD DATA ########################
##################################################

raw_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/CSVGN1/3_BUS_System.raw")
dyr_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/CSVGN1/3_BUS_System.dyr")
csv_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/CSVGN1/results_PSSe.csv")

function csvgn1_1(source)
    return PSY.CSVGN1(;
        name = get_name(source),
        K = 20.0,
        T1 = 0.0,
        T2 = 1.0,
        T3 = 0.154833,
        T4 = 1.0,
        T5 = 0.005167,
        Rmin = 0.0,
        Vmax = 1.0,
        Vmin = 0.0,
        CBase = 60.0,
        base_power = 500.0,
    )
end

function get_load_by_name(system, name)
    for l in get_components(StandardLoad, system)
        if get_name(l) == name
            return l
        end
    end
end

function get_bus_by_number(system, number)
    for b in get_components(ACBus, system)
        if get_number(b) == number
            return b
        end
    end
end

@testset "Test 49 CSVGN1 ResidualModel" begin
    path = joinpath(pwd(), "test-psse-csvgn1")
    !isdir(path) && mkdir(path)
    try
        # Define system
        sys = System(raw_file, dyr_file)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        # Define Source and Attach CSVGN1
        bus_3 = get_bus_by_number(sys, 3)

        inf_source() = Source(;
            name = "CSVGN1", #name
            available = true, #availability
            active_power = 0.0,
            reactive_power = 0.0,
            bus = bus_3, #bus
            R_th = 0.0,
            X_th = 0.0, #Xth
            active_power_limits = (min = -1e6, max = 1e6),
            reactive_power_limits = (min = -1e6, max = 1e6),
        )

        add_component!(sys, inf_source())

        set_bustype!(bus_3, 2)

        for s in get_components(Source, sys)
            #Find the source at bus 3
            if get_number(get_bus(s)) == 3
                #Attach the dynamic injection to the source in the system
                set_dynamic_injector!(s, csvgn1_1(s))
            end
        end

        # Define Perturbation
        load = get_load_by_name(sys, "load21")

        perturbation_load =
            LoadChange(0.005, load, :P_ref_impedance, 400 / get_base_power(sys))

        # Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            sys, #system
            path,
            (0.0, 0.07), #time span
            perturbation_load, #Type of Fault
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test49_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test diff_val[1] < 1e-3

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Solve problem
        @test execute!(sim, IDA(); saveat = 0.0001) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain results

        t_psid, v1_psid = get_voltage_magnitude_series(results, 1)
        _, v2_psid = get_voltage_magnitude_series(results, 2)
        _, v3_psid = get_voltage_magnitude_series(results, 3)

        # Obtain PSSE results
        M = get_csv_data(csv_file)
        t_psse = M[:, 1]
        v1_psse = M[:, 2]
        v2_psse = M[:, 3]
        v3_psse = M[:, 4]

        # Test Transient Simulation Results

        @test LinearAlgebra.norm(t_psid - round.(t_psse, digits = 4)) == 0.0
        @test LinearAlgebra.norm(v1_psid - v1_psse, Inf) <= 1e-3
        @test LinearAlgebra.norm(v2_psid - v2_psse, Inf) <= 1e-3
        @test LinearAlgebra.norm(v3_psid - v3_psse, Inf) <= 1e-3

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
