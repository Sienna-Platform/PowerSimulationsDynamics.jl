function get_flat_start(inputs::SimulationInputs)
    bus_count = get_bus_count(inputs)
    var_count = get_variable_count(inputs)
    initial_conditions = zeros(var_count)
    initial_conditions[1:bus_count] .= 1.0
    return initial_conditions
end

function power_flow_solution!(
    initial_guess::Vector{Float64},
    sys::PSY.System,
    inputs::SimulationInputs,
)
    pf = PF.ACPowerFlow{PF.TrustRegionACPowerFlow}()
    res = solve_and_save_powerflow!(pf, sys, true)
    if !res
        @error("PowerFlow failed to solve")
        return BUILD_FAILED
    end
    bus_size = length(PSY.get_bus_numbers(sys))
    @debug "Updating bus voltage magnitude and angle to match power flow result"
    for bus in PSY.get_components(PSY.Bus, sys)
        bus_n = PSY.get_number(bus)
        bus_ix = get_lookup(inputs)[bus_n]
        initial_guess[bus_ix] = PSY.get_magnitude(bus) * cos(PSY.get_angle(bus))
        initial_guess[bus_ix + bus_size] = PSY.get_magnitude(bus) * sin(PSY.get_angle(bus))
        @debug "$(PSY.get_name(bus)) V_r = $(initial_guess[bus_ix]), V_i = $(initial_guess[bus_ix + bus_size])"
    end
    @debug "After PowerFlow returning to device base"
    PSY.set_units_base_system!(sys, "DEVICE_BASE")
    return BUILD_INCOMPLETE
end

function initialize_static_injection!(inputs::SimulationInputs)
    @debug "Updating Source internal voltage magnitude and angle"
    static_injection_devices = get_static_injectors(inputs)
    if !isempty(static_injection_devices)
        try
            for s in static_injection_devices
                initialize_static_device!(s)
            end
        catch e
            bt = catch_backtrace()
            @error "Static Injection Failed to Initialize" exception = e, bt
            return BUILD_FAILED
        end
    end
    return BUILD_INCOMPLETE
end

function _initialization_debug(dynamic_device, static, x0_device::Vector{Float64})
    residual = similar(x0_device)
    Vm = PSY.get_magnitude(PSY.get_bus(static))
    θ = PSY.get_angle(PSY.get_bus(static))
    device!(
        x0_device,
        residual,
        Vm * cos(θ),
        Vm * sin(θ),
        zeros(10),
        zeros(10),
        [1.0],
        zeros(100),
        dynamic_device,
        0,
    )
    for (ix, state) in enumerate(PSY.get_states(dynamic_device))
        @debug state residual[ix]
    end
    return
end

function initialize_dynamic_injection!(
    initial_guess::Vector{Float64},
    inputs::SimulationInputs,
    system::PSY.System,
)
    @debug "Updating Dynamic Injection Component Initial Guess"
    initial_inner_vars = zeros(get_inner_vars_count(inputs))
    try
        for dynamic_device in get_dynamic_injectors(inputs)
            static = PSY.get_component(
                dynamic_device.static_type,
                system,
                PSY.get_name(dynamic_device),
            )
            @debug "Initializing $(PSY.get_name(dynamic_device)) - $(typeof(dynamic_device.device))"
            n_states = PSY.get_n_states(dynamic_device)
            _inner_vars = @view initial_inner_vars[get_inner_vars_index(dynamic_device)]
            x0_device = initialize_dynamic_device!(dynamic_device, static, _inner_vars)
            @assert length(x0_device) == n_states
            ix_range = get_ix_range(dynamic_device)
            initial_guess[ix_range] = x0_device
            @debug _initialization_debug(dynamic_device, static, x0_device)
        end
    catch e
        bt = catch_backtrace()
        @error "Dynamic Injection Failed to Initialize" exception = e, bt
        return BUILD_FAILED
    end
    return BUILD_INCOMPLETE
end

function initialize_dynamic_branches!(
    initial_guess::Vector{Float64},
    inputs::SimulationInputs,
)
    try
        @debug "Initializing Dynamic Branches"
        for br in get_dynamic_branches(inputs)
            @debug "$(PSY.get_name(br)) -  $(typeof(br))"
            n_states = PSY.get_n_states(br)
            x0_branch = initialize_dynamic_device!(br)
            IS.@assert_op length(x0_branch) == n_states
            ix_range = get_ix_range(br)
            initial_guess[ix_range] = x0_branch
        end
    catch e
        bt = catch_backtrace()
        @error "Dynamic Branches Failed to Initialize" exception = e, bt
        return BUILD_FAILED
    end
    return BUILD_INCOMPLETE
end

function check_valid_values(initial_guess::Vector{Float64}, inputs::SimulationInputs)
    invalid_initial_guess = String[]
    for i in get_bus_range(inputs)
        if initial_guess[i] > 1.3 || initial_guess[i] < -1.3
            push!(invalid_initial_guess, "Voltage entry $i")
        end
    end

    for device in get_dynamic_injectors(inputs)
        device_index = get_global_index(device)
        if haskey(device_index, :ω)
            dev_freq = initial_guess[device_index[:ω]]
            if dev_freq > 1.2 || dev_freq < 0.8
                push!(invalid_initial_guess, "$(PSY.get_name(device)) - :ω")
            end
        end
        all(isfinite, initial_guess) && continue
        for state in get_global_index(device)
            if state.second ∈ i
                push!(invalid_initial_guess, "$device - $(state.first)")
            end
        end
    end

    if !isempty(invalid_initial_guess)
        @error("Invalid initial condition values $invalid_initial_guess")
        return BUILD_FAILED
    end

    return BUILD_IN_PROGRESS
end

# Default implementation for both models. This implementation is to future proof if there is
# a divergence between the required build methods
function _calculate_initial_guess!(x0_init::Vector{Float64}, sim::Simulation)
    inputs = get_simulation_inputs(sim)
    @assert sim.status == BUILD_INCOMPLETE
    while sim.status == BUILD_INCOMPLETE
        @debug "Start state intialization routine"
        TimerOutputs.@timeit BUILD_TIMER "Power Flow solution" begin
            sim.status = power_flow_solution!(sim.x0_init, get_system(sim), inputs)
        end
        TimerOutputs.@timeit BUILD_TIMER "Initialize Static Injectors" begin
            sim.status = initialize_static_injection!(inputs)
        end
        TimerOutputs.@timeit BUILD_TIMER "Initialize Dynamic Injectors" begin
            sim.status = initialize_dynamic_injection!(sim.x0_init, inputs, get_system(sim))
        end
        if has_dyn_lines(inputs)
            TimerOutputs.@timeit BUILD_TIMER "Initialize Dynamic Branches" begin
                sim.status = initialize_dynamic_branches!(sim.x0_init, inputs)
            end
        else
            @debug "No Dynamic Branches in the system"
        end
        sim.status = check_valid_values(sim.x0_init, inputs)
    end
    return
end

function precalculate_initial_conditions!(x0_init::Vector{Float64}, sim::Simulation)
    _calculate_initial_guess!(x0_init, sim)
    return sim.status != BUILD_FAILED
end

"""
Returns a Dictionary with the resulting initial conditions of the simulation
"""
function read_initial_conditions(sim::Simulation)
    system = get_system(sim)
    simulation_inputs = get_simulation_inputs(sim)
    bus_size = get_bus_count(simulation_inputs)
    V_R = Dict{Int, Float64}()
    V_I = Dict{Int, Float64}()
    Vm = Dict{Int, Float64}()
    θ = Dict{Int, Float64}()
    for bus in PSY.get_components(PSY.Bus, system)
        bus_n = PSY.get_number(bus)
        bus_ix = get_lookup(simulation_inputs)[bus_n]
        V_R[bus_n] = get_initial_conditions(sim)[bus_ix]
        V_I[bus_n] = get_initial_conditions(sim)[bus_ix + bus_size]
        Vm[bus_n] = sqrt(
            get_initial_conditions(sim)[bus_ix]^2 +
            get_initial_conditions(sim)[bus_ix + bus_size]^2,
        )
        θ[bus_n] = atan(
            get_initial_conditions(sim)[bus_ix + bus_size],
            get_initial_conditions(sim)[bus_ix],
        )
    end
    results = Dict{String, Any}("V_R" => V_R, "V_I" => V_I, "Vm" => Vm, "θ" => θ)
    for device in get_dynamic_injectors(simulation_inputs)
        states = PSY.get_states(device)
        name = PSY.get_name(device)
        global_index = get_global_index(device)
        x0_device = Dict{Symbol, Float64}()
        for s in states
            x0_device[s] = get_initial_conditions(sim)[global_index[s]]
        end
        results[name] = x0_device
    end
    dyn_branches = get_dynamic_branches(simulation_inputs)
    if !isempty(dyn_branches)
        for br in dyn_branches
            states = PSY.get_states(br)
            name = PSY.get_name(br)
            global_index = get_global_index(br)
            x0_br = Dict{Symbol, Float64}()
            for s in states
                x0_br[s] = get_initial_conditions(sim)[global_index[s]]
            end
            printed_name = "Line " * name
            results[printed_name] = x0_br
        end
    end
    return results
end

function set_operating_point!(
    x0_init::Vector{Float64},
    inputs::SimulationInputs,
    system::PSY.System,
)
    status = BUILD_INCOMPLETE
    while status == BUILD_INCOMPLETE
        status = power_flow_solution!(x0_init, system, inputs)
        status = initialize_static_injection!(inputs)
        status = initialize_dynamic_injection!(x0_init, inputs, system)
        status = initialize_dynamic_branches!(x0_init, inputs)
        status = SIMULATION_INITIALIZED
    end
    if status != SIMULATION_INITIALIZED
        error("Failed to find operating point")
    end
    return
end
