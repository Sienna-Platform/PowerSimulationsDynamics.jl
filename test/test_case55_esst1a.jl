"""
Test for AVR model : ESST1A available in PSS/e
This case study defines a four bus system with an infinite bus in 1,
a GENSAL in bus 2 and a constant impedance load in bus 3
The GENSAL machine has the ESST1A and a HYGOV.
The disturbance is the outage of one line between buses 1 and 4
"""
##################################################
############### LOAD DATA ########################
##################################################

raw_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/ESST1A/TVC_System_32.raw")
dyr_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/ESST1A/TVC_System.dyr")
csv_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/ESST1A/results_PSSe.csv")

function get_gen_by_number(system, number)
    for gen in get_components(Generator, system)
        if get_number(get_bus(gen)) == number
            return gen
        end
    end
end

# ESST1A

esst1a_avr() = ESST1A(;
    UEL_flags = 1,
    PSS_flags = 1,
    Tr = 0.01,
    Vi_lim = (-5.0, 5.0),
    Tc = 14.1421356237310,
    Tb = 20.0,
    Tc1 = 14.1421356237310,
    Tb1 = 20.0,
    Ka = 200.0,
    Ta = 0.1,
    Va_lim = (min = 2.1, max = 6.3),
    Vr_lim = (min = 0.0, max = 6.0),
    Kc = 0.5,
    Kf = 0.01,
    Tf = 0.01,
    K_lr = 0.01,
    I_lr = 2.8,
)

@testset "Test 53 ESST1A ResidualModel" begin
    path = joinpath(pwd(), "test-psse-esst1a")
    !isdir(path) && mkdir(path)
    try
        # Define system
        sys = System(raw_file, dyr_file)
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        # Get gen at bus 2
        g = get_gen_by_number(sys, 2)

        # Initially the dynamic injector has an EXST1 AVR.
        # So it is completely removed and replaced.

        # Get the initial dynamic injector for the gen
        # at bus 2
        dynamic_injector = get_dynamic_injector(g)

        # Remove the initial dynamic injector
        remove_component!(sys, dynamic_injector)

        # Define the new dynamic injector
        dyn_gen = DynamicGenerator(;
            name = get_name(g),
            machine = deepcopy(get_machine(dynamic_injector)),
            shaft = deepcopy(get_shaft(dynamic_injector)),
            avr = esst1a_avr(),
            prime_mover = deepcopy(get_prime_mover(dynamic_injector)),
            ω_ref = 1.0,
            pss = deepcopy(get_pss(dynamic_injector)),
        )

        # Attach the new dynamic injector to the gen
        # at bus 2
        add_component!(sys, dyn_gen, g)

        # Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            sys, #system
            path,
            (0.0, 20.0), #time span
            BranchTrip(1.0, Line, "BUS1-BUS4-i_1"), #Type of Fault
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test55_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test diff_val[1] < 5e-5

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Solve problem
        @test execute!(sim, IDA(); saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain results
        t_psid, v2_psid = get_voltage_magnitude_series(results, 2)
        _, v3_psid = get_voltage_magnitude_series(results, 3)
        _, ω_psid = get_state_series(results, ("generator-2-1", :ω))
        _, efd_psid = get_field_voltage_series(results, "generator-2-1")

        # Obtain PSSE results
        M = get_csv_data(csv_file)
        t_psse = M[:, 1]
        v2_psse = M[:, 2]
        v3_psse = M[:, 3]
        ω_psse = M[:, 4]
        efd_psse = M[:, 5]

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(t_psid - round.(t_psse, digits = 3)) == 0.0
        @test LinearAlgebra.norm(v2_psid - v2_psse, Inf) <= 1e-3
        @test LinearAlgebra.norm(v3_psid - v3_psse, Inf) <= 1e-3
        @test LinearAlgebra.norm(ω_psid - ω_psse, Inf) <= 1e-3
        @test LinearAlgebra.norm(efd_psid - efd_psse, Inf) <= 5e-2

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
