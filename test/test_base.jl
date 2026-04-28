##################################################
############### LOAD DATA ########################
##################################################

omib_sys = build_system(PSIDTestSystems, "psid_test_omib")
omib_sys_file = build_system(PSIDTestSystems, "psid_test_omib"; force_build = true)

##################################################
############### SOLVE PROBLEM ####################
##################################################
#Define Fault: Change of YBus

@testset "Prog meter enabling" begin
    @test !PSID._prog_meter_enabled()
end

@testset "Make Simulation" begin
    path1 = mktempdir()
    try
        #Compute Y_bus after fault
        fault_branch = deepcopy(collect(get_components(Branch, omib_sys))[1])
        fault_branch.r = 0.00
        fault_branch.x = 0.1
        Ybus_fault = PSID.build_ybus_from_branches(
            [fault_branch],
            collect(get_components(ACBus, omib_sys));
            base_power = PSY.get_base_power(omib_sys),
        )[:, :]

        Ybus_change = NetworkSwitch(
            1.0, #change at t = 1.0
            Ybus_fault,
        ) #New YBus
        #Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            omib_sys, #system
            path1,
            (0.0, 30.0), #time span
            Ybus_change;
            system_to_file = true,
            console_level = Logging.Error,
        )
        @test sim.status == PSID.BUILT
        # Show methods must not throw under PrettyTables v3 (regression: backend = :auto was invalid).
        @test (show(IOBuffer(), MIME"text/plain"(), sim); true)
        @test (show(IOBuffer(), MIME"text/html"(), sim); true)
        # Test accessor functions
        dyn_wrapper = PowerSimulationsDynamics.get_dynamic_wrapper(sim, "generator-102-1")
        @test isa(dyn_wrapper, PowerSimulationsDynamics.DynamicWrapper)
        dic_init_conds = read_initial_conditions(sim)
        @test isa(dic_init_conds, Dict)
        dic_control_refs = get_setpoints(sim)
        @test isa(dic_control_refs, Dict)

        o_system = System(joinpath(path1, "input_system.json"))
        for b in get_components(ACBus, o_system)
            b_sys = get_component(Bus, omib_sys, get_name(b))
            b_file = get_component(Bus, omib_sys_file, get_name(b))
            @test get_angle(b) == get_angle(b_sys)
            @test get_angle(b) == get_angle(b_file)
        end

    finally
        @info("removing test files")
        rm(path1; force = true, recursive = true)
    end

    path2 = mktempdir()
    try
        #Compute Y_bus after fault
        fault_branch = deepcopy(collect(get_components(Branch, omib_sys))[1])
        fault_branch.r = 0.00
        fault_branch.x = 0.1
        Ybus_fault = PSID.build_ybus_from_branches(
            [fault_branch],
            collect(get_components(ACBus, omib_sys));
            base_power = PSY.get_base_power(omib_sys),
        )[:, :]

        Ybus_change = NetworkSwitch(
            1.0, #change at t = 1.0
            Ybus_fault,
        ) #New YBus
        #Define Simulation Problem
        sim = Simulation!(
            ResidualModel,
            omib_sys, #system
            path2,
            (0.0, 30.0), #time span
            Ybus_change;
            system_to_file = true,
            console_level = Logging.Error,
        )
        @test sim.status == PSID.BUILT
        m_system = System(joinpath(path2, "initialized_system.json"))
        for b in get_components(ACBus, m_system)
            b_sys = get_component(Bus, omib_sys, get_name(b))
            b_file = get_component(Bus, omib_sys_file, get_name(b))
            @test get_angle(b) == get_angle(b_sys)
            if get_bustype(b) == PSY.ACBusTypes.REF
                @test get_angle(b) == get_angle(b_file)
            else
                @test get_angle(b) != get_angle(b_file)
            end
        end

    finally
        @info("removing test files")
        rm(path2; force = true, recursive = true)
    end
end

@testset "Solve Twice Simulation" begin
    path1 = mktempdir()
    try
        #Compute Y_bus after fault
        fault_branch = deepcopy(collect(get_components(Branch, omib_sys))[1])
        fault_branch.r = 0.00
        fault_branch.x = 0.1
        Ybus_fault = PSID.build_ybus_from_branches(
            [fault_branch],
            collect(get_components(ACBus, omib_sys));
            base_power = PSY.get_base_power(omib_sys),
        )[:, :]

        Ybus_change = NetworkSwitch(
            1.0, #change at t = 1.0
            Ybus_fault,
        ) #New YBus
        #Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            omib_sys, #system
            path1,
            (0.0, 30.0), #time span
            Ybus_change;
            system_to_file = true,
            console_level = Logging.Error,
        )
        @test sim.status == PSID.BUILT

        # First run
        @test execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        # Second run
        @test execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
    finally
        @info("removing test files")
        rm(path1; force = true, recursive = true)
    end
end

@testset "Simulation kwargs: disable_timer_outputs" begin
    path = mktempdir()
    try
        #Define Simulation Problem
        sim = Simulation(
            ResidualModel,
            omib_sys, #system
            path,
            (0.0, 30.0); #time span
            system_to_file = true,
            console_level = Logging.Error,
            disable_timer_outputs = true,
        )
        @test sim.status == PSID.BUILT
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Hybrid Line Indexing" begin
    ## Create threebus system with more dyn lines ##
    three_bus_file_dir = joinpath(TEST_FILES_DIR, "data_tests/ThreeBusInverter.raw")
    threebus_sys_dyns = System(three_bus_file_dir; runchecks = false)
    for l in get_components(PSY.StandardLoad, threebus_sys_dyns)
        transform_load_to_constant_impedance(l)
    end
    add_source_to_ref(threebus_sys_dyns)

    dyn_branch12 =
        DynamicBranch(get_component(Branch, threebus_sys_dyns, "BUS 1-BUS 2-i_1"))
    dyn_branch23 =
        DynamicBranch(get_component(Branch, threebus_sys_dyns, "BUS 2-BUS 3-i_1"))

    add_component!(threebus_sys_dyns, dyn_branch12)
    add_component!(threebus_sys_dyns, dyn_branch23)

    # Attach dyn devices
    for g in get_components(Generator, threebus_sys_dyns)
        if get_number(get_bus(g)) == 102
            case_gen = dyn_gen_second_order(g)
            add_component!(threebus_sys_dyns, case_gen, g)
        elseif get_number(get_bus(g)) == 103
            case_inv = inv_case78(g)
            add_component!(threebus_sys_dyns, case_inv, g)
        end
    end

    # Tests for all Dynamic Lines
    sim = Simulation(
        ResidualModel,
        threebus_sys_dyns,
        mktempdir(),
        (0.0, 10.0);
        console_level = Logging.Error,
    )
    @test sim.status == PSID.BUILT
    sim_inputs = sim.inputs
    DAE_vector = PSID.get_DAE_vector(sim_inputs)
    @test all(DAE_vector)
    @test all(LinearAlgebra.diag(sim_inputs.mass_matrix) .> 0)
    total_shunts = PSID.get_total_shunts(sim_inputs)
    # Total shunts matrix follows same pattern as the rectangular Ybus
    for v in LinearAlgebra.diag(total_shunts[1:3, 4:end])
        @test v > 0
    end

    for v in LinearAlgebra.diag(total_shunts[4:end, 1:3])
        @test v < 0
    end

    for entry in LinearAlgebra.diag(sim_inputs.mass_matrix)
        @test entry > 0
    end
    voltage_buses_ix = PSID.get_voltage_buses_ix(sim_inputs)
    @test length(voltage_buses_ix) == 3
    @test isempty(PSID.get_current_buses_ix(sim_inputs))

    # Tests for dynamic lines with b = 0
    set_b!(dyn_branch12, (from = 0.0, to = 0.0))
    set_b!(dyn_branch23, (from = 0.0, to = 0.0))
    sim = Simulation(
        ResidualModel,
        threebus_sys_dyns,
        pwd(),
        (0.0, 10.0);
        console_level = Logging.Error,
    )
    @test sim.status == PSID.BUILT
    sim_inputs = sim.inputs
    DAE_vector = PSID.get_DAE_vector(sim_inputs)
    @test sum(.!DAE_vector) == 6
    for (ix, entry) in enumerate(DAE_vector)
        if !entry
            @test LinearAlgebra.diag(sim_inputs.mass_matrix)[ix] == 0
        elseif entry
            @test LinearAlgebra.diag(sim_inputs.mass_matrix)[ix] > 0
        else
            @test false
        end
    end

    total_shunts = PSID.get_total_shunts(sim_inputs)
    @test sum(LinearAlgebra.diag(total_shunts)) == 0.0
    voltage_buses_ix = PSID.get_voltage_buses_ix(sim_inputs)
    @test isempty(voltage_buses_ix)
end

@testset "Test initial conditions externally" begin
    #Create system with 2 GENROU generators
    threebus_file_dir =
        joinpath(TEST_FILES_DIR, "benchmarks/psse/GENROU/ThreeBusMulti_LessLoad.raw")
    dyr_file_dir =
        joinpath(TEST_FILES_DIR, "benchmarks/psse/GENROU/ThreeBus_GENROU_SEXS.dyr")
    sys = System(threebus_file_dir, dyr_file_dir)
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_impedance(l)
    end
    x0_test = zeros(23)
    x0_test[1:6] .= 1.0
    #Initialize System from given initial condition
    sim = Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0),
        BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1");
        initialize_simulation = false,
        initial_conditions = x0_test,
        console_level = Logging.Error,
    )
    @test LinearAlgebra.norm(sim.x0_init - x0_test) <= 1e-6
    #Initialize System normally
    sim_normal = Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0),
        BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1");
        console_level = Logging.Error,
    )
    #Save states without generator at bus 2
    x0 = sim_normal.x0_init
    x0_no_gen = vcat(x0[1:6], x0[13:end])
    #Make Generator 2 unavailable and transform bus into PQ bus
    gen = PSY.get_component(ThermalStandard, sys, "generator-102-1")
    PSY.set_available!(gen, false)
    b = PSY.get_component(Bus, sys, "BUS 2")
    PSY.set_bustype!(b, PSY.ACBusTypes.PQ)
    #Create Simulation without Gen 2 starting from steady-state with Gen 2
    sim_trip_gen = Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0);
        initialize_simulation = false,
        initial_conditions = x0_no_gen,
        console_level = Logging.Error,
    )
    @test LinearAlgebra.norm(sim_trip_gen.x0_init - x0_no_gen) <= 1e-6
    #Create Simulation without Gen 2 at steady state
    sim_normal_no_gen = Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0),
        BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1");
        console_level = Logging.Error,
    )
    @test length(sim_normal_no_gen.x0_init) == 17
    #Ignore Initial Conditions without passing initialize_simulation = false
    sim_ignore_init = Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0);
        initial_conditions = x0_no_gen,
        console_level = Logging.Error,
    )
    @test LinearAlgebra.norm(sim_ignore_init.x0_init - sim_normal_no_gen.x0_init) <= 1e-6
    #Pass wrong vector size
    x0_wrong = zeros(20)
    # @test_logs (:error, "Build failed") match_mode = :any Simulation(
    #     ResidualModel,
    #     sys,
    #     pwd(),
    #     (0.0, 20.0);
    #     initialize_simulation = false,
    #     initial_conditions = x0_wrong;
    #    console_level = Logging.Error
    # )
    #Flat start initialization
    sim_flat = Simulation(
        ResidualModel,
        sys,
        pwd(),
        (0.0, 20.0);
        console_level = Logging.Error,
        initialize_simulation = false,
    )
    x0_flat = zeros(17)
    x0_flat[1:3] .= 1.0
    @test LinearAlgebra.norm(sim_flat.x0_init - x0_flat) <= 1e-6
end

@testset "Test Network Kirchoff Calculation" begin
    voltages = [
        1.05
        1.019861574381897
        0.0
        -0.016803841801155062
    ]
    V_r = voltages[1:2]
    V_i = voltages[3:end]
    ybus_ = PNM.Ybus(omib_sys; include_constant_impedance_loads = false).data
    I_balance_ybus = -1 * ybus_ * (V_r + V_i .* 1im)
    inputs = PSID.SimulationInputs(ResidualModel, omib_sys, ConstantFrequency())
    I_balance_sim = zeros(4)
    PSID.network_model(inputs, I_balance_sim, voltages)
    for i in 1:2
        @test isapprox(real(I_balance_ybus[i]), I_balance_sim[i])
        @test isapprox(imag(I_balance_ybus[i]), I_balance_sim[i + 2])
    end
end

@testset "Test Network Modification Callback Affects" begin
    three_bus_file_dir = joinpath(TEST_FILES_DIR, "data_tests/ThreeBusInverter.raw")
    threebus_sys = System(three_bus_file_dir; runchecks = false)
    add_source_to_ref(threebus_sys)
    for l in get_components(PSY.StandardLoad, threebus_sys)
        transform_load_to_constant_impedance(l)
    end
    # Attach dyn devices
    for g in get_components(Generator, threebus_sys)
        if get_number(get_bus(g)) == 102
            case_gen = dyn_gen_second_order(g)
            add_component!(threebus_sys, case_gen, g)
        elseif get_number(get_bus(g)) == 103
            case_inv = inv_case78(g)
            add_component!(threebus_sys, case_inv, g)
        end
    end

    ybus_original = PNM.Ybus(threebus_sys; include_constant_impedance_loads = false)

    inputs = PSID.SimulationInputs(ResidualModel, threebus_sys, ConstantFrequency())

    for i in 1:3, j in 1:3
        complex_ybus = ybus_original.data[i, j]
        @test inputs.ybus_rectangular[i, j] == real(complex_ybus)
        @test inputs.ybus_rectangular[i + 3, j + 3] == real(complex_ybus)
        @test inputs.ybus_rectangular[i + 3, j] == -imag(complex_ybus)
        @test inputs.ybus_rectangular[i, j + 3] == imag(complex_ybus)
    end

    br = get_component(Line, threebus_sys, "BUS 1-BUS 3-i_1")
    PSID.ybus_update!(inputs, br, -1.0)

    remove_component!(threebus_sys, br)
    ybus_line_trip = PNM.Ybus(threebus_sys; include_constant_impedance_loads = false)

    # Use is approx because the inversion of complex might be different in
    # floating point than the inversion of a single float
    for i in 1:3, j in 1:3
        complex_ybus = ybus_line_trip.data[i, j]
        @test isapprox(inputs.ybus_rectangular[i, j], real(complex_ybus), atol = 1e-6)
        @test isapprox(
            inputs.ybus_rectangular[i + 3, j + 3],
            real(complex_ybus),
            atol = 1e-6,
        )
        @test isapprox(inputs.ybus_rectangular[i + 3, j], -imag(complex_ybus), atol = 1e-6)
        @test isapprox(inputs.ybus_rectangular[i, j + 3], imag(complex_ybus), atol = 1e-6)
    end

    threebus_sys = System(three_bus_file_dir; runchecks = false)
    ybus_original = PNM.Ybus(threebus_sys; include_constant_impedance_loads = false)
    cb1 = NetworkSwitch(1.0, ybus_original)

    @test all(
        cb1.ybus_rectangular .== NetworkSwitch(1.0, ybus_original.data).ybus_rectangular,
    )

    for i in 1:3, j in 1:3
        complex_ybus = ybus_original.data[i, j]
        @test cb1.ybus_rectangular[i, j] == real(complex_ybus)
        @test cb1.ybus_rectangular[i + 3, j + 3] == real(complex_ybus)
        @test cb1.ybus_rectangular[i + 3, j] == -imag(complex_ybus)
        @test cb1.ybus_rectangular[i, j + 3] == imag(complex_ybus)
    end
end

@testset "Test Generation perturbations callback affects" begin
    three_bus_file_dir = joinpath(TEST_FILES_DIR, "data_tests/ThreeBusInverter.raw")
    threebus_sys = System(three_bus_file_dir; runchecks = false)
    for l in get_components(PSY.StandardLoad, threebus_sys)
        transform_load_to_constant_impedance(l)
    end
    add_source_to_ref(threebus_sys)
    # Attach dyn devices
    for g in get_components(Generator, threebus_sys)
        if get_number(get_bus(g)) == 102
            case_gen = dyn_gen_second_order(g)
            add_component!(threebus_sys, case_gen, g)
        elseif get_number(get_bus(g)) == 103
            case_inv = inv_case78(g)
            add_component!(threebus_sys, case_inv, g)
        end
    end

    mach = get_component(DynamicGenerator, threebus_sys, "generator-102-1")
    inv = get_component(DynamicInverter, threebus_sys, "generator-103-1")

    cref = ControlReferenceChange(1.0, mach, :P_ref, 10.0)
    ωref = ControlReferenceChange(1.0, inv, :ω_ref, 0.9)

    inputs = PSID.SimulationInputs(ResidualModel, threebus_sys, ConstantFrequency())
    integrator_for_test = MockIntegrator(inputs)

    cref_affect_f = PSID.get_affect(inputs, threebus_sys, cref)
    ωref_affect_f = PSID.get_affect(inputs, threebus_sys, ωref)

    cref_affect_f(integrator_for_test)
    ωref_affect_f(integrator_for_test)

    @test PSID.get_P_ref(inputs.dynamic_injectors[1]) == 10.0
    @test PSID.get_ω_ref(inputs.dynamic_injectors[2]) == 0.9

    inputs = PSID.SimulationInputs(ResidualModel, threebus_sys, ConstantFrequency())
    integrator_for_test = MockIntegrator(inputs)

    mach_trip = PSID.GeneratorTrip(1.0, mach)
    inv_trip = PSID.GeneratorTrip(1.0, inv)

    mtrip_affect_f = PSID.get_affect(inputs, threebus_sys, mach_trip)
    itrip_affect_f = PSID.get_affect(inputs, threebus_sys, inv_trip)

    mtrip_affect_f(integrator_for_test)
    itrip_affect_f(integrator_for_test)

    @test PSID.get_connection_status(inputs.dynamic_injectors[1]) == 0.0
    @test PSID.get_connection_status(inputs.dynamic_injectors[2]) == 0.0
end

@testset "Test Load perturbations callback affects" begin
    three_bus_file_dir = joinpath(TEST_FILES_DIR, "data_tests/ThreeBusInverter.raw")
    threebus_sys = System(three_bus_file_dir; runchecks = false)
    for l in get_components(PSY.StandardLoad, threebus_sys)
        transform_load_to_constant_impedance(l)
    end
    add_source_to_ref(threebus_sys)
    # Attach dyn devices
    for g in get_components(Generator, threebus_sys)
        if get_number(get_bus(g)) == 102
            case_gen = dyn_gen_second_order(g)
            add_component!(threebus_sys, case_gen, g)
        elseif get_number(get_bus(g)) == 103
            case_inv = inv_case78(g)
            add_component!(threebus_sys, case_inv, g)
        end
    end

    load_1 = get_component(ElectricLoad, threebus_sys, "load1011")
    load_2 = get_component(ElectricLoad, threebus_sys, "load1021")

    load_val = LoadChange(1.0, load_1, :P_ref, 10.0)
    load_trip = LoadTrip(1.0, load_2)

    inputs = PSID.SimulationInputs(ResidualModel, threebus_sys, ConstantFrequency())
    integrator_for_test = MockIntegrator(inputs)

    lref_affect_f = PSID.get_affect(inputs, threebus_sys, load_val)
    ltrip_affect_f = PSID.get_affect(inputs, threebus_sys, load_trip)

    lref_affect_f(integrator_for_test)
    zip_load1 = first(filter(x -> PSY.get_name(x) == "BUS 1", inputs.static_loads))
    @test PSID.get_P_impedance(zip_load1) == 10.0
    ltrip_affect_f(integrator_for_test)
    zip_load2 = first(filter(x -> PSY.get_name(x) == "BUS 2", inputs.static_loads))
    @test PSID.get_P_impedance(zip_load2) == 0.0
    @test PSID.get_Q_impedance(zip_load2) == 0.0
end

@testset "Global Index" begin
    three_bus_file_dir = joinpath(TEST_FILES_DIR, "data_tests/ThreeBusInverter.raw")
    threebus_sys_dyns = System(three_bus_file_dir; runchecks = false)
    for l in get_components(PSY.StandardLoad, threebus_sys_dyns)
        transform_load_to_constant_impedance(l)
    end
    add_source_to_ref(threebus_sys_dyns)

    dyn_branch12 =
        DynamicBranch(get_component(Branch, threebus_sys_dyns, "BUS 1-BUS 2-i_1"))
    dyn_branch23 =
        DynamicBranch(get_component(Branch, threebus_sys_dyns, "BUS 2-BUS 3-i_1"))

    add_component!(threebus_sys_dyns, dyn_branch12)
    add_component!(threebus_sys_dyns, dyn_branch23)

    # Attach dyn devices
    for g in get_components(Generator, threebus_sys_dyns)
        if get_number(get_bus(g)) == 102
            case_gen = dyn_gen_second_order(g)
            add_component!(threebus_sys_dyns, case_gen, g)
        elseif get_number(get_bus(g)) == 103
            case_inv = inv_case78(g)
            add_component!(threebus_sys_dyns, case_inv, g)
        end
    end

    # Tests for all Dynamic Lines
    sim = Simulation(
        ResidualModel,
        threebus_sys_dyns,
        mktempdir(),
        (0.0, 10.0);
        console_level = Logging.Error,
    )
    global_index = PSID.make_global_state_map(sim.inputs)
    @test_throws ErrorException PSID.get_state_from_ix(global_index, 40)
    @test ("V_2", :R) == PSID.get_state_from_ix(global_index, 2)
    @test ("BUS 2-BUS 3-i_1", :Il_I) == PSID.get_state_from_ix(global_index, 10)
    @test ("generator-102-1", :Vr1) == PSID.get_state_from_ix(global_index, 16)
    @test ("generator-103-1", :ε_pll) == PSID.get_state_from_ix(global_index, 30)
end

@testset "Frequency Reference" begin
    sys = System(
        joinpath(TEST_FILES_DIR, "data_tests/240busWECC_2018_PSS32_fixed_shunts.raw"),
        joinpath(TEST_FILES_DIR, "data_tests/240busWECC_2018_PSS.dyr");
        bus_name_formatter = x -> string(strip(x["name"])) * "-" * string(x["index"]),
    )
    for l in get_components(PSY.StandardLoad, sys)
        transform_load_to_constant_impedance(l)
    end
    sim = Simulation(
        ResidualModel,
        sys, #system
        mktempdir(),
        (0.0, 20.0); #time span
        # Not initialized to speed up the test
        initialize_simulation = false,
        console_level = Logging.Error,
    )

    @test PSID.get_global_vars_update_pointers(sim.inputs)[1] != 0

    sim = Simulation(
        ResidualModel,
        sys, #system
        mktempdir(),
        (0.0, 20.0);
        # Not initialized to speed up the test
        initialize_simulation = false,
        frequency_reference = ConstantFrequency(),
        console_level = Logging.Error,
    )

    @test PSID.get_global_vars_update_pointers(sim.inputs)[1] == 0

    ref_bus = get_component(Bus, sys, "TESLA-3933")
    devs_in_ref = get_components(x -> get_bus(x) == ref_bus, StaticInjection, sys)
    for d in devs_in_ref
        remove_component!(sys, get_dynamic_injector(d))
        remove_component!(sys, d)
    end

    # @test_logs (:error, "Build failed") match_mode = :any Simulation(
    #     ResidualModel,
    #     sys, #system
    #     mktempdir(),
    #     (0.0, 20.0),
    #     # Not initialized to speed up the test
    #     initialize_simulation = false,
    #     frequency_reference = ConstantFrequency(), #time span
    # )
end

@testset "Line and Branch transformation" begin
    #Define Simulation Problem
    omib_sys_copy = deepcopy(omib_sys)
    sim = Simulation(
        ResidualModel,
        omib_sys_copy, #system
        mktempdir(),
        (0.0, 30.0); #time span
        all_lines_dynamic = true,
        console_level = Logging.Error,
    )
    @test sim.status == PSID.BUILT
    inputs = PSID.get_simulation_inputs(sim)
    @test !isempty(PSID.get_dynamic_branches(inputs))
    @test sum(PSID.get_ybus(inputs)) == 0.0

    sim = Simulation(
        ResidualModel,
        omib_sys_copy, #system
        mktempdir(),
        (0.0, 30.0); #time span
        all_branches_dynamic = true,
        console_level = Logging.Error,
    )
    @test sim.status == PSID.BUILT
    inputs = PSID.get_simulation_inputs(sim)
    @test !isempty(PSID.get_dynamic_branches(inputs))
    @test sum(PSID.get_ybus(inputs)) == 0.0

    sim = Simulation!(
        ResidualModel,
        omib_sys_copy, #system
        mktempdir(),
        (0.0, 30.0); #time span
        all_lines_dynamic = true,
        console_level = Logging.Error,
    )
    @test sim.status == PSID.BUILT
    inputs = PSID.get_simulation_inputs(sim)
    @test !isempty(PSID.get_dynamic_branches(inputs))
    @test sum(PSID.get_ybus(inputs)) == 0.0
    @test !isempty(PSY.get_components(PSY.DynamicBranch, omib_sys_copy))
    @test isempty(PSY.get_components(PSY.Line, omib_sys_copy))
end

@testset "Jacobian" begin
    for model_type in [ResidualModel, MassMatrixModel]
        omib_sys_copy = deepcopy(omib_sys)
        @test_throws IS.ConflictingInputsError get_jacobian(model_type, omib_sys_copy, -1)
        jac = get_jacobian(model_type, omib_sys_copy)
        @test isa(jac, Function)
        @test isa(jac.Jv, PSID.SparseArrays.SparseMatrixCSC{Float64, Int64})

        # ODE Jacobian
        test_x = deepcopy(jac.x)
        test_JM = similar(jac.Jv)
        jac(test_JM, test_x, 0, 0)
        @test test_JM == jac.Jv

        # DAE Jacobian
        test_x = deepcopy(jac.x)
        test_dx = similar(jac.x)
        test_JM = similar(jac.Jv)
        jac(test_JM, test_dx, test_x, 0, 0.0, 0)
        @test test_JM == jac.Jv

        jac_dense = get_jacobian(model_type, omib_sys_copy, 0)
        @test isa(jac_dense, Function)
        @test !isa(jac_dense.Jv, PSID.SparseArrays.SparseMatrixCSC{Float64, Int64})
        @test isa(jac_dense.Jv, Matrix{Float64})
    end
end

@testset "Jacobian with Delays" begin
    for model_type in [MassMatrixModel]
        model_type = MassMatrixModel
        omib_sys_copy = deepcopy(omib_sys)
        add_degov_to_omib!(omib_sys_copy)
        @test_throws IS.ConflictingInputsError get_jacobian(model_type, omib_sys_copy, -1)

        jac = get_jacobian(model_type, omib_sys_copy)
        @test isa(jac, Function)
        @test isa(jac.Jv, PSID.SparseArrays.SparseMatrixCSC{Float64, Int64})

        jac_dense = get_jacobian(model_type, omib_sys_copy, 0)
        @test isa(jac_dense, Function)
        @test !isa(jac_dense.Jv, PSID.SparseArrays.SparseMatrixCSC{Float64, Int64})
        @test isa(jac_dense.Jv, Matrix{Float64})
    end
end

@testset "Test 01 OMIB ResidualModel" begin
    path = mktempdir()
    try
        #Compute Y_bus after fault
        fault_branch = deepcopy(collect(get_components(Branch, omib_sys))[1])
        fault_branch.r = 0.00
        fault_branch.x = 0.1
        Ybus_fault = PSID.build_ybus_from_branches(
            [fault_branch],
            collect(get_components(ACBus, omib_sys));
            base_power = PSY.get_base_power(omib_sys),
        )[:, :]

        Ybus_change = NetworkSwitch(
            1.0, #change at t = 1.0
            Ybus_fault,
        ) #New YBus
        # Define Simulation Problem
        sim = Simulation!(
            ResidualModel,
            omib_sys, #system
            path,
            (0.0, 20.0), #time span
            Ybus_change;
            console_level = Logging.Error,
        )

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in test01_x0_init
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - test01_eigvals) < 1e-3
        @test LinearAlgebra.norm(eigs - test01_eigvals_psat, Inf) < 5.0

        # Solve problem
        @test execute!(sim, IDA(); dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for angles
        series = get_state_series(results, ("generator-102-1", :δ); dt = 0.01)
        t = series[1]
        δ = series[2]
        @test typeof(t) == Vector{Float64}
        @test typeof(δ) == Vector{Float64}
        @test t[2] - t[1] == 0.01
        series = get_state_series(results, ("generator-102-1", :δ); dt = [0.02, 0.05])
        t = series[1]
        δ = series[2]
        @test typeof(t) == Vector{Float64}
        @test typeof(δ) == Vector{Float64}
        @test t[1] == 0.02
    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 01 OMIB AVR Error" begin
    # Define Classic OMIB with different AVR
    omib_sys_error = build_system(PSIDTestSystems, "psid_test_omib"; force_build = true)
    ############### Data Dynamic devices ########################
    function dyn_gen_error(generator)
        return DynamicGenerator(;
            name = get_name(generator),
            ω_ref = 1.0,
            machine = machine_classic(),
            shaft = shaft_damping(),
            avr = avr_type1(),
            prime_mover = tg_none(),
            pss = pss_none(),
        )
    end
    #Attach dynamic generator. Currently use PSS/e format based on bus #.
    gen = [g for g in get_components(Generator, omib_sys_error)][1]
    # Not recommended way of handling DynamicInjectors in PSY but useful for this test.
    gen.dynamic_injector = nothing
    case_gen = dyn_gen_error(gen)
    add_component!(omib_sys_error, case_gen, gen)

    @test_throws ErrorException PSID.DynamicWrapper(
        gen,
        case_gen,
        1,
        1:6,
        1:6,
        1:9,
        100.0,
        60.0,
    )
end
