"""
Container for the output of a dynamic simulation run.

`SimulationResults` stores the solved trajectory together with lookup structures
used by post-processing utilities (state maps, bus lookup, setpoints, and
auxiliary pointers).
"""
struct SimulationResults
    global_index::MAPPING_DICT
    bus_lookup::Dict{Int, Int}
    system::PSY.System
    time_log::Dict{Symbol, Any}
    solution::SciMLBase.AbstractODESolution
    setpoints::Dict{String, Dict{String, Float64}}
    global_vars_update_pointers::Dict{Int, Int}
    function SimulationResults(
        inputs::SimulationInputs,
        system::PSY.System,
        time_log,
        solution,
    )
        new(
            make_global_state_map(inputs),
            get_lookup(inputs),
            system,
            time_log,
            solution,
            get_setpoints(inputs),
            get_global_vars_update_pointers(inputs),
        )
    end
end

get_global_index(res::SimulationResults) = res.global_index
get_bus_count(res::SimulationResults) = get_n_buses(res.system)
get_bus_lookup(res::SimulationResults) = res.bus_lookup
get_system(res::SimulationResults) = res.system
get_solution(res::SimulationResults) = res.solution
get_setpoints(res::SimulationResults) = res.setpoints
get_global_vars_update_pointers(res::SimulationResults) = res.global_vars_update_pointers
"""
Internal function to obtain as a Vector of Float64 of a specific state. It receives the solution and the
global index for a state.

"""
function _post_proc_state_series(solution, ix::Int, dt::Nothing, unique_timestamps::Bool)
    if unique_timestamps
        ix_t = unique(i -> solution.t[i], eachindex(solution.t))
    else
        ix_t = eachindex(solution.t)
    end
    ts = solution.t[ix_t]
    state = solution[ix, ix_t]
    return ts, state
end

function _post_proc_state_series(solution, ix::Int, dt::Float64, ::Bool)
    ts = collect(range(0; stop = solution.t[end], step = dt))
    state = solution(ts; idxs = ix)
    return ts, state.u
end

function _post_proc_state_series(solution, ix::Int, dt::Vector{Float64}, ::Bool)
    state = solution(dt; idxs = ix)
    return dt, state.u
end

"""
Function to obtain the state time series of a specific state. It receives the simulation, and a tuple
containing the name of the Dynamic Device and the symbol of the state.
"""
function post_proc_state_series(
    res::SimulationResults,
    ref::Tuple{String, Symbol},
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    global_state_index = get_global_index(res)
    if !haskey(global_state_index, ref[1])
        available_devices = join(keys(global_state_index), ", ")
        error("Device $(ref[1]) not found. Available devices: $available_devices")
    end

    state_dict = global_state_index[ref[1]]
    if !haskey(state_dict, ref[2])
        available_states = join(keys(state_dict), ", ")
        error(
            "State $(ref[2]) not found in device $(ref[1]). Available states: $available_states",
        )
    end

    ix = state_dict[ref[2]]
    return _post_proc_state_series(get_solution(res), ix, dt, unique_timestamps)
end

"""
Function to obtain voltage and output currents for a dynamic device. It receives the simulation, and the name
of the Dynamic Device.
"""
function post_proc_voltage_current_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)::NTuple{5, Vector{Float64}}
    #Note: Type annotation since get_dynamic_injector is type unstable and solution is Union{Nothing, DAESol}
    system = get_system(res)
    bus_lookup = get_bus_lookup(res)
    n_buses = length(bus_lookup)
    solution = res.solution
    device = PSY.get_component(PSY.StaticInjection, system, name)
    if isnothing(device)
        error("Device $(name) not found in the system")
    end
    bus_ix = get(bus_lookup, PSY.get_number(PSY.get_bus(device)), -1)
    ts, V_R, V_I =
        post_proc_voltage_series(solution, bus_ix, n_buses, dt, unique_timestamps)
    dyn_device = PSY.get_dynamic_injector(device)
    if isnothing(dyn_device)
        _, I_R, I_I = compute_output_current(res, device, V_R, V_I, dt, unique_timestamps)
    else
        _, I_R, I_I =
            compute_output_current(res, dyn_device, V_R, V_I, dt, unique_timestamps)
    end
    return ts, V_R, V_I, I_R, I_I
end

"""
Function to obtain voltage using the bus index (and not the bus number). It receives the solution, the bus index and
the total number of buses.

"""
function post_proc_voltage_series(
    solution,
    bus_ix::Int,
    n_buses::Int,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    bus_ix < 0 && error("Bus number $(bus_number) not found.")
    ts, V_R = _post_proc_state_series(solution, bus_ix, dt, unique_timestamps)
    _, V_I = _post_proc_state_series(solution, bus_ix + n_buses, dt, unique_timestamps)
    return collect(ts), collect(V_R), collect(V_I)
end

"""
Function to compute the real current output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_real_current_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    ts, _, _, I_R, _ = post_proc_voltage_current_series(res, name, dt, unique_timestamps)
    return ts, I_R
end
"""
Function to compute the imaginary current output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_imaginary_current_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    ts, _, _, _, I_I = post_proc_voltage_current_series(res, name, dt, unique_timestamps)
    return ts, I_I
end

"""
Function to compute the active power output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_activepower_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    ts, V_R, V_I, I_R, I_I =
        post_proc_voltage_current_series(res, name, dt, unique_timestamps)
    return ts, V_R .* I_R + V_I .* I_I
end

"""
Function to compute the active power output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_reactivepower_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    ts, V_R, V_I, I_R, I_I =
        post_proc_voltage_current_series(res, name, dt, unique_timestamps)
    return ts, V_I .* I_R - V_R .* I_I
end

"""
Function to compute the field current output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_field_current_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    system = get_system(res)
    bus_lookup = get_bus_lookup(res)
    n_buses = length(bus_lookup)
    solution = res.solution
    device = PSY.get_component(PSY.StaticInjection, system, name)
    bus_ix = get(bus_lookup, PSY.get_number(PSY.get_bus(device)), -1)
    ts, V_R, V_I =
        post_proc_voltage_series(solution, bus_ix, n_buses, dt, unique_timestamps)
    dyn_device = PSY.get_dynamic_injector(device)
    _, I_fd = compute_field_current(res, dyn_device, V_R, V_I, dt, unique_timestamps)
    return ts, I_fd
end

"""
Function to compute the field voltage output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_field_voltage_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    system = get_system(res)
    device = PSY.get_component(PSY.StaticInjection, system, name)
    dyn_device = PSY.get_dynamic_injector(device)
    ts, Vf = compute_field_voltage(res, dyn_device, dt, unique_timestamps)
    return ts, Vf
end

"""
Function to compute the pss output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the string
name of the Dynamic Injection device.

"""
function post_proc_pss_output_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    system = get_system(res)
    device = PSY.get_component(PSY.StaticInjection, system, name)
    dyn_device = PSY.get_dynamic_injector(device)
    ts, Vs = compute_pss_output(res, dyn_device, dt, unique_timestamps)
    return ts, Vs
end

"""
Function to compute the mechanical torque output time series of a Dynamic Injection series out of the DAE Solution. It receives the solution and the
string name of the Dynamic Injection device.

"""
function post_proc_mechanical_torque_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    system = get_system(res)
    device = PSY.get_component(PSY.StaticInjection, system, name)
    dyn_device = PSY.get_dynamic_injector(device)
    ts, τm = compute_mechanical_torque(res, dyn_device, dt, unique_timestamps)
    return ts, τm
end

"""
Function to compute the current flowing through an AC branch through their series element.
The current is computed through the `from` bus into the `to` bus.
"""
function post_proc_branch_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    system = get_system(res)
    bus_lookup = get_bus_lookup(res)
    n_buses = length(bus_lookup)
    solution = res.solution
    branch = PSY.get_component(PSY.ACBranch, system, name)
    if isnothing(branch)
        error("Branch $(name) not found in the system")
    end
    bus_from_number = PSY.get_number(PSY.get_from(PSY.get_arc(branch)))
    bus_to_number = PSY.get_number(PSY.get_to(PSY.get_arc(branch)))
    bus_from_ix = get(bus_lookup, bus_from_number, -1)
    bus_to_ix = get(bus_lookup, bus_to_number, -1)
    ts, V_R_from, V_I_from =
        post_proc_voltage_series(solution, bus_from_ix, n_buses, dt, unique_timestamps)
    _, V_R_to, V_I_to =
        post_proc_voltage_series(solution, bus_to_ix, n_buses, dt, unique_timestamps)
    r = PSY.get_r(branch)
    x = PSY.get_x(branch)
    I_flow = ((V_R_from + V_I_from * 1im) - (V_R_to + V_I_to * 1im)) ./ (r + x * 1im)
    return ts, V_R_from, V_I_from, V_R_to, V_I_to, real.(I_flow), imag.(I_flow)
end

"""
Function to compute the frequency of a Dynamic Injection component.
"""
function post_proc_frequency_series(
    res::SimulationResults,
    name::String,
    dt::Union{Nothing, Float64, Vector{Float64}},
    unique_timestamps::Bool,
)
    system = get_system(res)
    device = PSY.get_component(PSY.StaticInjection, system, name)
    dyn_device = PSY.get_dynamic_injector(device)
    if isnothing(dyn_device)
        error("Dynamic Injection $(name) not found in the system")
    end
    ts, ω = compute_frequency(res, dyn_device, dt, unique_timestamps)
end

"""
    get_state_series(
        res::SimulationResults,
        ref::Tuple{String, Symbol};
        dt::Union{Nothing, Float64, Vector{Float64}} = nothing,
        unique_timestamps::Bool = true,
    )
    end

Function to obtain series of states out of DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `ref:Tuple{String, Symbol}` : Tuple used to identify the dynamic device, via its name, as a `String`, and the associated state as a `Symbol`.
"""
function get_state_series(
    res::SimulationResults,
    ref::Tuple{String, Symbol};
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_state_series(res, ref, dt, unique_timestamps)
end

"""
    get_voltage_magnitude_series(
        res::SimulationResults,
        bus_number::Int
    )

Function to obtain the voltage magnitude series out of the DAE Solution.

# Arguments:

- `res::SimulationResults` : Simulation Results object that contains the solution
- `bus_number::Int` : Bus number identifier
"""
function get_voltage_magnitude_series(
    res::SimulationResults,
    bus_number::Int;
    dt = nothing,
    unique_timestamps = true,
)
    n_buses = get_bus_count(res)
    bus_ix = get(get_bus_lookup(res), bus_number, 0)
    ts, V_R, V_I =
        post_proc_voltage_series(res.solution, bus_ix, n_buses, dt, unique_timestamps)
    return ts, sqrt.(V_R .^ 2 .+ V_I .^ 2)
end

"""
    get_voltage_angle_series(
        res::SimulationResults,
        bus_number::Int
    )

Function to obtain the voltage angle series out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `bus_number::Int` : Bus number identifier
"""
function get_voltage_angle_series(
    res::SimulationResults,
    bus_number::Int;
    dt = nothing,
    unique_timestamps = true,
)
    n_buses = get_bus_count(res)
    bus_ix = get(get_bus_lookup(res), bus_number, 0)
    ts, V_R, V_I =
        post_proc_voltage_series(res.solution, bus_ix, n_buses, dt, unique_timestamps)
    return ts, atan.(V_I, V_R)
end

"""
    get_real_current_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the real current time series of a Dynamic Injection series out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_real_current_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_real_current_series(res, name, dt, unique_timestamps)
end

"""
    get_imaginary_current_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the imaginary current time series of a Dynamic Injection series out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_imaginary_current_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_imaginary_current_series(res, name, dt, unique_timestamps)
end

"""
    get_activepower_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the active power output time series of a Dynamic Injection series out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_activepower_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_activepower_series(res, name, dt, unique_timestamps)
end

"""
    get_reactivepower_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the reactive power output time series of a Dynamic Injection series out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_reactivepower_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_reactivepower_series(res, name, dt, unique_timestamps)
end

"""
    get_field_current_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the field current time series of a Dynamic Generator out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_field_current_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_field_current_series(res, name, dt, unique_timestamps)
end

"""
    get_field_voltage_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the field voltage time series of a Dynamic Generator out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_field_voltage_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_field_voltage_series(res, name, dt, unique_timestamps)
end

"""
    get_pss_output_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the pss output time series of a Dynamic Generator out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_pss_output_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_pss_output_series(res, name, dt, unique_timestamps)
end

"""
    get_mechanical_torque_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the mechanical torque time series of the mechanical torque out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_mechanical_torque_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_mechanical_torque_series(res, name, dt, unique_timestamps)
end

"""
    get_real_current_branch_flow(
            res::SimulationResults,
            name::String,
    )
Function to obtain the real current flowing through the series element of a Branch

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified line
"""
function get_real_current_branch_flow(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    ts, _, _, _, _, Ir, _ = post_proc_branch_series(res, name, dt, unique_timestamps)
    return ts, Ir
end

"""
    get_imaginary_current_branch_flow(
            res::SimulationResults,
            name::String,
    )
Function to obtain the imaginary current flowing through the series element of a Branch

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified line
"""
function get_imaginary_current_branch_flow(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    ts, _, _, _, _, _, Ii = post_proc_branch_series(res, name, dt, unique_timestamps)
    return ts, Ii
end

"""
    get_activepower_branch_flow(
            res::SimulationResults,
            name::String,
            location::Symbol,
    )
Function to obtain the active power flowing through the series element of a Branch.
The user must specified is the power should be computed in the :from or to :bus, by
specifying a symbol.

If :from is specified, the power is computed flowing outwards the :from bus.
If :to is specified, the power is computed flowing into the :to bus.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified line
- `location::Symbol` : :from or :to to specify a bus
"""
function get_activepower_branch_flow(
    res::SimulationResults,
    name::String,
    location::Symbol;
    dt = nothing,
    unique_timestamps = true,
)
    ts, V_R_from, V_I_from, V_R_to, V_I_to, Ir, Ii =
        post_proc_branch_series(res, name, dt, unique_timestamps)
    if location == :from
        return ts, V_R_from .* Ir + V_I_from .* Ii
    elseif location == :to
        return ts, -(V_R_to .* Ir + V_I_to .* Ii)
    else
        error("The location symbol $(:location) must be :from or :to to specify the bus.")
    end
    return
end

"""
    get_reactivepower_branch_flow(
            res::SimulationResults,
            name::String,
            location::Symbol,
    )
Function to obtain the reactive power flowing through the series element of a Branch.
The user must specified is the power should be computed in the :from or to :bus, by
specifying a symbol.

If :from is specified, the power is computed flowing outwards the :from bus.
If :to is specified, the power is computed flowing into the :to bus.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified line
- `location::Symbol` : :from or :to to specify a bus
"""
function get_reactivepower_branch_flow(
    res::SimulationResults,
    name::String,
    location::Symbol;
    dt = nothing,
    unique_timestamps = true,
)
    ts, V_R_from, V_I_from, V_R_to, V_I_to, Ir, Ii =
        post_proc_branch_series(res, name, dt, unique_timestamps)
    if location == :from
        return ts, V_I_from .* Ir - V_R_from .* Ii
    elseif location == :to
        return ts, -(V_I_to .* Ir - V_R_to .* Ii)
    else
        error("The location symbol $(:location) must be :from or :to to specify the bus.")
    end
    return
end

"""
    get_frequency_series(
            res::SimulationResults,
            name::String,
    )

Function to obtain the frequency time series of a Dynamic Injection out of the DAE Solution.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
- `name::String` : Name to identify the specified device
"""
function get_frequency_series(
    res::SimulationResults,
    name::String;
    dt = nothing,
    unique_timestamps = true,
)
    return post_proc_frequency_series(res, name, dt, unique_timestamps)
end

"""
    show_states_initial_value(res::SimulationResults)

Function to print initial states.

# Arguments

- `res::SimulationResults` : Simulation Results object that contains the solution
"""
function show_states_initial_value(res::SimulationResults)
    bus_size = get_bus_count(res)
    system = get_system(res)
    x0_init = res.solution.u[1]
    println("Voltage Variables")
    println("====================")
    buses_sorted =
        sort(collect(PSY.get_components(PSY.Bus, system)); by = x -> PSY.get_number(x))
    for bus in buses_sorted
        name = PSY.get_name(bus)
        println(name)
        println("====================")
        bus_n = PSY.get_number(bus)
        bus_ix = get_bus_lookup(res)[bus_n]
        V_R = x0_init[bus_ix]
        V_I = x0_init[bus_ix + bus_size]
        Vm = sqrt(V_R^2 + V_I^2)
        θ = angle(V_R + V_I * 1im)
        print("Vm ", round(Vm; digits = 4), "\n")
        print("θ ", round(θ; digits = 4), "\n")
        println("====================")
    end
    println("====================")
    for device in PSY.get_components(PSY.DynamicInjection, system)
        states = PSY.get_states(device)
        name = PSY.get_name(device)
        println("Differential States")
        println(name)
        println("====================")
        global_index = get_global_index(res)[name]
        for s in states
            print(s, " ", round(x0_init[global_index[s]]; digits = 4), "\n")
        end
        println("====================")
    end
    dyn_branches = PSY.get_components(PSY.DynamicBranch, system)
    if !isempty(dyn_branches)
        println("====================")
        println("Line Current States")
        for br in dyn_branches
            states = PSY.get_states(br)
            name = PSY.get_name(br)
            printed_name = "Line " * name
            println("====================")
            println(printed_name)
            global_index = get_global_index(res)[name]
            for (i, s) in enumerate(states)
                print(s, " ", round(x0_init[global_index[s]]; digits = 5), "\n")
            end
            println("====================")
        end
    end
    return
end
