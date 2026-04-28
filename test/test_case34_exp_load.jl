"""
Validation Exponential Load
This case study defines a three bus system with an infinite bus, GENROU and a load.
The fault drop the line connecting the infinite bus and GENROU. The test validates
that the Exponential Model with α = β = 0 is equivalent to a constant power model.
"""

##################################################
############### SOLVE PROBLEM ####################
##################################################

raw_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/LOAD/ThreeBusMulti.raw")
dyr_file = joinpath(TEST_FILES_DIR, "benchmarks/psse/LOAD/ThreeBus_GENROU.dyr")

# Create Constant Power load system
sys_power = System(raw_file, dyr_file)

# Create Exponential load system
sys_exp = System(raw_file, dyr_file)
tspan = (0.0, 20.0)
# Replace Constant Power loads for Exponential loads
for l in collect(get_components(PSY.StandardLoad, sys_exp))
    exp_load = PSY.ExponentialLoad(;
        name = PSY.get_name(l),
        available = PSY.get_available(l),
        bus = PSY.get_bus(l),
        active_power = PSY.get_constant_active_power(l),
        reactive_power = PSY.get_constant_reactive_power(l),
        α = 0.0, # Constant Power
        β = 0.0, # Constant Power
        base_power = PSY.get_base_power(l),
        max_active_power = PSY.get_max_constant_active_power(l),
        max_reactive_power = PSY.get_max_constant_reactive_power(l),
    )
    PSY.remove_component!(sys_exp, l)
    PSY.add_component!(sys_exp, exp_load)
end

@testset "Test 34 ExponentialLoad ResidualModel" begin
    path = (joinpath(pwd(), "test-34"))
    !isdir(path) && mkdir(path)
    try
        # Instantiate Simulations
        sim_power = Simulation(
            ResidualModel,
            sys_power,
            path,
            tspan,
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"), #Type of Fault
        )
        sim_exp = Simulation(
            ResidualModel,
            sys_exp,
            path,
            tspan,
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"), #Type of Fault
        )

        # Test Initial Conditions
        @test LinearAlgebra.norm(sim_power.x0_init - sim_exp.x0_init) < 1e-4

        # Test Small Signal
        ss_power = small_signal_analysis(sim_power)
        @test ss_power.stable
        ss_exp = small_signal_analysis(sim_exp)
        @test ss_exp.stable
        # Compare Eigenvalues
        @test LinearAlgebra.norm(ss_power.eigenvalues - ss_exp.eigenvalues) < 1e-4

        # Solve Problems
        @test execute!(sim_power, IDA(); abstol = 1e-9, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results_power = read_results(sim_power)
        @test execute!(sim_exp, IDA(); abstol = 1e-9, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results_exp = read_results(sim_exp)

        # Store results
        _, v102_power = get_voltage_magnitude_series(results_power, 102)
        _, v102_exp = get_voltage_magnitude_series(results_exp, 102)
        _, v103_power = get_voltage_magnitude_series(results_power, 103)
        _, v103_exp = get_voltage_magnitude_series(results_exp, 103)

        #TODO: Test for LoadPower
        p = get_activepower_series(results_exp, "load1031")

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(v102_power - v102_exp, Inf) <= 1e-2
        @test LinearAlgebra.norm(v103_power - v103_exp, Inf) <= 1e-2

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 34 ExponentialLoad MassMatrixModel" begin
    path = (joinpath(pwd(), "test-34"))
    !isdir(path) && mkdir(path)
    try
        # Instantiate Simulations
        sim_power = Simulation(
            MassMatrixModel,
            sys_power,
            path,
            tspan,
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"), #Type of Fault
        )
        sim_exp = Simulation(
            MassMatrixModel,
            sys_exp,
            path,
            tspan,
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"), #Type of Fault
        )

        # Test Initial Conditions
        @test LinearAlgebra.norm(sim_power.x0_init - sim_exp.x0_init) < 1e-4

        # Test Small Signal
        ss_power = small_signal_analysis(sim_power)
        @test ss_power.stable
        ss_exp = small_signal_analysis(sim_exp)
        @test ss_exp.stable
        # Compare Eigenvalues
        @test LinearAlgebra.norm(ss_power.eigenvalues - ss_exp.eigenvalues) < 1e-4

        # Solve Problems
        @test execute!(sim_power, Rodas5P(); abstol = 1e-9, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results_power = read_results(sim_power)
        @test execute!(sim_exp, Rodas5P(); abstol = 1e-9, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results_exp = read_results(sim_exp)

        # Store results
        _, v102_power = get_voltage_magnitude_series(results_power, 102)
        _, v102_exp = get_voltage_magnitude_series(results_exp, 102)
        _, v103_power = get_voltage_magnitude_series(results_power, 103)
        _, v103_exp = get_voltage_magnitude_series(results_exp, 103)

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(v102_power - v102_exp, Inf) <= 1e-2
        @test LinearAlgebra.norm(v103_power - v103_exp, Inf) <= 1e-2

    finally
        @info("removing test files")
        rm(path; force = true, recursive = true)
    end
end

@testset "Test 34 ExponentialLoad LoadChange / LoadTrip callback affects" begin
    # Regression: LoadChange and LoadTrip on ExponentialLoad previously
    # MethodError'd in _find_zip_load_ix and UndefVarError'd on P_change/Q_change.
    exp_ld = first(get_components(PSY.ExponentialLoad, sys_exp))
    P0 = PSY.get_active_power(exp_ld)
    Q0 = PSY.get_reactive_power(exp_ld)
    base_power_conv = PSY.get_base_power(exp_ld) / PSY.get_base_power(sys_exp)

    inputs = PSID.SimulationInputs(ResidualModel, sys_exp, ConstantFrequency())
    integrator_for_test = MockIntegrator(inputs)

    new_P_ref = P0 + 0.1
    pert_change = LoadChange(1.0, exp_ld, :P_ref, new_P_ref)
    affect_change = PSID.get_affect(inputs, sys_exp, pert_change)
    affect_change(integrator_for_test)

    wrapped =
        first(
            filter(x -> haskey(PSID.get_exp_names(x), PSY.get_name(exp_ld)),
                inputs.static_loads),
        )
    tuple_ix = PSID.get_exp_names(wrapped)[PSY.get_name(exp_ld)]
    exp_params = PSID.get_exp_params(wrapped)[tuple_ix]
    @test isapprox(exp_params.P_exp, P0 + (new_P_ref - P0) * base_power_conv; atol = 1e-12)

    # Trip the same exponential load and verify it is removed from the wrapper.
    pert_trip = LoadTrip(2.0, exp_ld)
    affect_trip = PSID.get_affect(inputs, sys_exp, pert_trip)
    affect_trip(integrator_for_test)
    @test !haskey(PSID.get_exp_names(wrapped), PSY.get_name(exp_ld))
end
