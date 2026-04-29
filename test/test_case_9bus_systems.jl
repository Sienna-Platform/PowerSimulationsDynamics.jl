@testset "Test 32 9-Bus Machine Only System" begin
    path = mktempdir()
    try
        sys = build_system(PSIDTestSystems, "psid_test_ieee_9bus")
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
        end

        gen_static = get_component(ThermalStandard, sys, "generator-2-1")
        gen_dynamic = get_dynamic_injector(gen_static)
        sim = Simulation(
            ResidualModel,
            sys,
            path,
            (0.0, 2.0),
            ControlReferenceChange(1.0, gen_dynamic, :P_ref, 0.78),
        )
        small_sig = small_signal_analysis(sim)
        @test execute!(sim, IDA()) == PSID.SIMULATION_FINALIZED

        sim = Simulation(
            MassMatrixModel,
            sys,
            path,
            (0.0, 2.0),
            ControlReferenceChange(1.0, gen_dynamic, :P_ref, 0.78),
        )
        small_sig = small_signal_analysis(sim)
        @test execute!(sim, Rodas4(); abstol = 1e-4, reltol = 1e-4) ==
              PSID.SIMULATION_FINALIZED

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end
