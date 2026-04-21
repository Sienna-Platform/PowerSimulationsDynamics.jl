mutable struct Simulation{T <: SimulationModel}
    status::BUILD_STATUS
    problem::Union{Nothing, SciMLBase.DEProblem}
    tspan::NTuple{2, Float64}
    sys::PSY.System
    perturbations::Vector{<:Perturbation}
    x0_init::Vector{Float64}
    initialized::Bool
    tstops::Vector{Float64}
    callbacks::Vector
    simulation_folder::String
    inputs::Union{Nothing, SimulationInputs}
    results::Union{Nothing, SimulationResults}
    console_level::Base.CoreLogging.LogLevel
    file_level::Base.CoreLogging.LogLevel
    multimachine::Bool
    frequency_reference::Union{ConstantFrequency, ReferenceBus}
end

get_system(sim::Simulation) = sim.sys
get_simulation_inputs(sim::Simulation) = sim.inputs
get_initial_conditions(sim::Simulation) = sim.x0_init
get_tspan(sim::Simulation) = sim.tspan

function Simulation(
    ::Type{T},
    sys::PSY.System;
    tspan,
    initial_conditions,
    initialize_simulation,
    perturbations,
    simulation_folder,
    console_level,
    file_level,
    frequency_reference,
) where {T <: SimulationModel}
    PSY.set_units_base_system!(sys, "DEVICE_BASE")

    return Simulation{T}(
        BUILD_INCOMPLETE,
        nothing,
        tspan,
        sys,
        perturbations,
        initial_conditions,
        !initialize_simulation,
        Vector{Float64}(),
        Vector{SciMLBase.AbstractDiscreteCallback}(),
        simulation_folder,
        nothing,
        nothing,
        console_level,
        file_level,
        false,
        frequency_reference,
    )
end

function Simulation!(
    ::Type{T},
    system::PSY.System,
    simulation_folder::String,
    tspan::NTuple{2, Float64},
    perturbation::Perturbation;
    kwargs...,
) where {T <: SimulationModel}
    return Simulation!(T, system, simulation_folder, tspan, [perturbation]; kwargs...)
end

function Simulation(
    ::Type{T},
    system::PSY.System,
    simulation_folder::String,
    tspan::NTuple{2, Float64},
    perturbation::Perturbation;
    kwargs...,
) where {T <: SimulationModel}
    return Simulation(T, system, simulation_folder, tspan, [perturbation]; kwargs...)
end

"""
    function Simulation!
        ::SimulationModel
        system::PowerSystems.System
        simulation_folder::String
        tspan::NTuple{2, Float64},
        perturbations::Vector{<:Perturbation} = Vector{Perturbation}();
        kwargs...,
    end

Builds the simulation object and conducts the indexing process. The initial conditions are stored in the system.

# Arguments:
- `::SimulationModel` : Type of Simulation Model. `ResidualModel` or `MassMatrixModel`. See [Models Section](https://sienna-platform.github.io/PowerSimulationsDynamics.jl/stable/models/) for more details
- `system::PowerSystems.System` : System data
- `simulation_folder::String` : Folder directory
- `tspan::NTuple{2, Float64}` : Time span for simulation
- `perturbations::Vector{<:Perturbation}` : Vector of Perturbations for the Simulation. Default: No Perturbations
- `initialize_simulation::Bool` : Runs the initialization routine. If false, simulation runs based on the operation point stored in System
- `initial_conditions::Vector{Float64}` : Allows the user to pass a vector with the initial condition values desired in the simulation. If initialize_simulation = true, these values are used as a first guess and overwritten.
- `frequency_reference` : Default `ReferenceBus`. Determines which frequency model is used for the network. Currently there are two options available:
    - `ConstantFrequency` assumes that the network frequency is 1.0 per unit at all times.
    - `ReferenceBus` will use the frequency state of a Dynamic Generator (rotor speed) or Dynamic Inverter (virtual speed) connected to the Reference Bus (defined in the Power Flow data) as the network frequency. If multiple devices are connected to such bus, the device with larger base power will be used as a reference. If a Voltage Source is connected to the Reference Bus, then a `ConstantFrequency` model will be used.
- `system_to_file::Bool` : Default `false`. Serializes the initialized system
- `console_level::Logging` : Default `Logging.Warn`. Sets the level of logging output to the console. Can be set to `Logging.Error`, `Logging.Warn`, `Logging.Info` or `Logging.Debug`
- `file_level::Logging` : Default `Logging.Info`. Sets the level of logging output to file. Can be set to `Logging.Error`, `Logging.Warn`, `Logging.Info` or `Logging.Debug`
- `disable_timer_outputs::Bool` : Default `false`. Allows the user to display timer information about the construction and initilization of the Simulation.
"""
function Simulation!(
    ::Type{T},
    system::PSY.System,
    simulation_folder::String,
    tspan::NTuple{2, Float64},
    perturbations::Vector{<:Perturbation} = Vector{Perturbation}();
    kwargs...,
) where {T <: SimulationModel}
    # Instantiates the Simulation object
    sim = Simulation(
        T,
        system;
        tspan = tspan,
        initial_conditions = get(kwargs, :initial_conditions, Vector{Float64}()),
        initialize_simulation = get(kwargs, :initialize_simulation, true),
        simulation_folder = simulation_folder,
        perturbations = perturbations,
        console_level = get(kwargs, :console_level, Logging.Warn),
        file_level = get(kwargs, :file_level, Logging.Info),
        frequency_reference = get(kwargs, :frequency_reference, ReferenceBus()),
    )

    build!(sim; kwargs...)
    if get(kwargs, :system_to_file, false)
        PSY.to_json(system, joinpath(simulation_folder, "initialized_system.json"))
    end
    return sim
end

"""
    function Simulation
        ::SimulationModel
        system::PowerSystems.System
        simulation_folder::String
        tspan::NTuple{2, Float64},
        perturbations::Vector{<:Perturbation} = Vector{Perturbation}();
        kwargs...,
    end

Builds the simulation object and conducts the indexing process. The original system is not modified and a copy its created and stored in the Simulation.

# Arguments:
- `::SimulationModel` : Type of Simulation Model. `ResidualModel` or `MassMatrixModel`. See [Models Section](https://sienna-platform.github.io/PowerSimulationsDynamics.jl/stable/models/) for more details
- `system::PowerSystems.System` : System data
- `simulation_folder::String` : Folder directory
- `tspan::NTuple{2, Float64}` : Time span for simulation
- `perturbations::Vector{<:Perturbation}` : Vector of Perturbations for the Simulation. Default: No Perturbations
- `initialize_simulation::Bool` : Runs the initialization routine. If false, simulation runs based on the operation point stored in System
- `initial_conditions::Vector{Float64}` : Allows the user to pass a vector with the initial condition values desired in the simulation. If initialize_simulation = true, these values are used as a first guess and overwritten.
- `frequency_reference` : Default `ReferenceBus`. Determines which frequency model is used for the network. Currently there are two options available:
    - `ConstantFrequency` assumes that the network frequency is 1.0 per unit at all times.
    - `ReferenceBus` will use the frequency state of a Dynamic Generator (rotor speed) or Dynamic Inverter (virtual speed) connected to the Reference Bus (defined in the Power Flow data) as the network frequency. If multiple devices are connected to such bus, the device with larger base power will be used as a reference. If a Voltage Source is connected to the Reference Bus, then a `ConstantFrequency` model will be used.
- `system_to_file::Bool` : Default `false`. Serializes the initialized system
- `console_level::Logging` : Default `Logging.Warn`. Sets the level of logging output to the console. Can be set to `Logging.Error`, `Logging.Warn`, `Logging.Info` or `Logging.Debug`
- `file_level::Logging` : Default `Logging.Info`. Sets the level of logging output to file. Can be set to `Logging.Error`, `Logging.Warn`, `Logging.Info` or `Logging.Debug`
- `disable_timer_outputs::Bool` : Default `false`. Allows the user to display timer information about the construction and initilization of the Simulation.
"""
function Simulation(
    ::Type{T},
    system::PSY.System,
    simulation_folder::String,
    tspan::NTuple{2, Float64},
    perturbations::Vector{<:Perturbation} = Vector{Perturbation}();
    kwargs...,
) where {T <: SimulationModel}
    simulation_system = deepcopy(system)
    # Instantiates the Simulation object
    sim = Simulation(
        T,
        simulation_system;
        tspan = tspan,
        initial_conditions = get(kwargs, :initial_conditions, Vector{Float64}()),
        initialize_simulation = get(kwargs, :initialize_simulation, true),
        simulation_folder = simulation_folder,
        perturbations = perturbations,
        console_level = get(kwargs, :console_level, Logging.Warn),
        file_level = get(kwargs, :file_level, Logging.Info),
        frequency_reference = get(kwargs, :frequency_reference, ReferenceBus()),
    )
    build!(sim; kwargs...)
    if get(kwargs, :system_to_file, false)
        PSY.to_json(system, joinpath(simulation_folder, "input_system.json"))
    end
    return sim
end

function reset!(sim::Simulation{T}) where {T <: SimulationModel}
    @info "Rebuilding the simulation after reset"
    sim.inputs = SimulationInputs(T, get_system(sim), sim.frequency_reference)
    sim.status = BUILD_INCOMPLETE
    sim.initialized = false
    build!(sim)
    @info "Simulation reset to status $(sim.status)"
    return
end

function configure_logging(sim::Simulation, file_mode; kwargs...)
    return IS.configure_logging(;
        console = true,
        console_stream = stderr,
        console_level = get(kwargs, :console_level, sim.console_level),
        file = true,
        filename = joinpath(sim.simulation_folder, SIMULATION_LOG_FILENAME),
        file_level = get(kwargs, :file_level, sim.file_level),
        file_mode = file_mode,
        tracker = nothing,
        set_global = false,
        progress = _prog_meter_enabled(),
    )
end

function _build_inputs!(sim::Simulation{T}) where {T <: SimulationModel}
    simulation_system = get_system(sim)
    sim.inputs = SimulationInputs(T, simulation_system, sim.frequency_reference)
    @debug "Simulation Inputs Created"
    return
end

function _get_flat_start(inputs::SimulationInputs)
    bus_count = get_bus_count(inputs)
    var_count = get_variable_count(inputs)
    initial_conditions = zeros(var_count)
    initial_conditions[1:bus_count] .= 1.0
    return initial_conditions
end

function _initialize_state_space(sim::Simulation{T}) where {T <: SimulationModel}
    simulation_inputs = get_simulation_inputs(sim)
    x0_init = sim.x0_init
    if isempty(x0_init) && sim.initialized
        @warn "Initial Conditions set to flat start"
        sim.x0_init = _get_flat_start(simulation_inputs)
    elseif isempty(x0_init) && !sim.initialized
        sim.x0_init = _get_flat_start(simulation_inputs)
    elseif !isempty(x0_init) && sim.initialized
        if length(sim.x0_init) != get_variable_count(simulation_inputs)
            throw(
                IS.ConflictingInputsError(
                    "The size of the provided initial state space does not match the model's state space.",
                ),
            )
        end
    elseif !isempty(x0_init) && !sim.initialized
        @warn "initial_conditions were provided with initialize_simulation. User's initial_conditions will be overwritten."
        if length(sim.x0_init) != get_variable_count(simulation_inputs)
            sim.x0_init = _get_flat_start(simulation_inputs)
        end
    else
        @assert false
    end
    return
end

function _pre_initialize_simulation!(sim::Simulation)
    _initialize_state_space(sim)
    if sim.initialized != true
        @info("Pre-Initializing Simulation States")
        sim.initialized = precalculate_initial_conditions!(sim.x0_init, sim)
        if !sim.initialized
            error(
                "The simulation failed to find an adequate initial guess for the initialization. Check the intialization routine.",
            )
        end
    else
        @warn("Using existing initial conditions value for simulation initialization")
        sim.status = SIMULATION_INITIALIZED
    end
    return
end

function _get_jacobian(sim::Simulation{ResidualModel})
    inputs = get_simulation_inputs(sim)
    x0_init = get_initial_conditions(sim)
    return JacobianFunctionWrapper(
        ResidualModel(inputs, x0_init, JacobianCache),
        x0_init,
        # sparse_retrieve_loop = 0,
    )
end

function _get_jacobian(sim::Simulation{MassMatrixModel})
    inputs = get_simulation_inputs(sim)
    x0_init = get_initial_conditions(sim)
    return JacobianFunctionWrapper(
        MassMatrixModel(inputs, x0_init, JacobianCache),
        x0_init,
    )
end

function _build_perturbations!(sim::Simulation)
    @info "Attaching Perturbations"
    if isempty(sim.perturbations)
        @debug "The simulation has no perturbations"
        return SciMLBase.CallbackSet(), [0.0]
    end
    inputs = get_simulation_inputs(sim)
    perturbations = sim.perturbations
    perturbations_count = length(perturbations)
    callback_vector = Vector{SciMLBase.DiscreteCallback}(undef, perturbations_count)
    tstops = Float64[]
    for (ix, pert) in enumerate(perturbations)
        _add_callback!(tstops, callback_vector, ix, pert, sim, inputs)
    end
    sim.tstops = tstops
    sim.callbacks = callback_vector
    return
end

function _add_callback!(
    tstops::Vector{Float64},
    callback_vector::Vector{SciMLBase.DiscreteCallback},
    ix::Int,
    pert::T,
    sim::Simulation,
    inputs::SimulationInputs,
) where {T <: Perturbation}
    @debug pert
    condition = (x, t, integrator) -> t in [pert.time]
    affect = get_affect(inputs, get_system(sim), pert)
    callback_vector[ix] = SciMLBase.DiscreteCallback(condition, affect)
    push!(tstops, pert.time)
    return
end

function _get_diffeq_problem(
    sim::Simulation,
    model::SystemModel{ResidualModel, NoDelays},
    jacobian::JacobianFunctionWrapper,
)
    x0 = get_initial_conditions(sim)
    dx0 = zeros(length(x0))
    simulation_inputs = get_simulation_inputs(sim)
    sim.problem = SciMLBase.DAEProblem(
        SciMLBase.DAEFunction{true}(
            model;
            jac = jacobian,
            tgrad = (dT, u, p, t) -> dT .= false,
            jac_prototype = jacobian.Jv,
        ),
        dx0,
        x0,
        get_tspan(sim),
        simulation_inputs;
        differential_vars = get_DAE_vector(simulation_inputs),
    )
    sim.status = BUILT
    return
end

function _get_diffeq_problem(
    sim::Simulation,
    model::SystemModel{MassMatrixModel, NoDelays},
    jacobian::JacobianFunctionWrapper,
)
    simulation_inputs = get_simulation_inputs(sim)
    sim.problem = SciMLBase.ODEProblem(
        SciMLBase.ODEFunction{true}(
            model;
            mass_matrix = get_mass_matrix(simulation_inputs),
            jac = jacobian,
            jac_prototype = jacobian.Jv,
            # Necessary to avoid unnecessary calculations in Rosenbrock methods
            tgrad = (dT, u, p, t) -> dT .= false,
        ),
        sim.x0_init,
        get_tspan(sim),
        simulation_inputs,
    )
    sim.status = BUILT
    return
end

function get_history_function(simulation_inputs::Simulation{MassMatrixModel})
    x0 = get_initial_conditions(simulation_inputs)
    h(p, t; idxs = nothing) = typeof(idxs) <: Number ? x0[idxs] : x0
    return h
end

function _get_diffeq_problem(
    sim::Simulation,
    model::SystemModel{MassMatrixModel, HasDelays},
    jacobian::JacobianFunctionWrapper,
)
    simulation_inputs = get_simulation_inputs(sim)
    h = get_history_function(sim)
    sim.problem = SciMLBase.DDEProblem(
        SciMLBase.DDEFunction{true}(
            model;
            mass_matrix = get_mass_matrix(simulation_inputs),
            jac = jacobian,
            jac_prototype = jacobian.Jv,
        ),
        sim.x0_init,
        h,
        get_tspan(sim),
        simulation_inputs;
        constant_lags = filter!(x -> x != 0, unique(simulation_inputs.delays)),
    )
    sim.status = BUILT

    return
end

function _build!(sim::Simulation{T}; kwargs...) where {T <: SimulationModel}
    check_kwargs(kwargs, SIMULATION_ACCEPTED_KWARGS, "Simulation")
    # Branches are a super set of Lines. Passing both kwargs will
    # be redundant.
    if get(kwargs, :disable_timer_outputs, false)
        TimerOutputs.disable_timer!(BUILD_TIMER)
    else
        TimerOutputs.enable_timer!(BUILD_TIMER)
        TimerOutputs.reset_timer!(BUILD_TIMER)
    end

    TimerOutputs.@timeit BUILD_TIMER "Build Simulation" begin
        try
            if get(kwargs, :all_branches_dynamic, false)
                TimerOutputs.@timeit BUILD_TIMER "AC Branch Transform to Dynamic" begin
                    sys = get_system(sim)
                    transform_branches_to_dynamic(sys, PSY.ACBranch)
                end
            elseif get(kwargs, :all_lines_dynamic, false)
                TimerOutputs.@timeit BUILD_TIMER "Line Transform to Dynamic" begin
                    sys = get_system(sim)
                    transform_branches_to_dynamic(sys, PSY.Line)
                end
            end
            TimerOutputs.@timeit BUILD_TIMER "Build Simulation Inputs" begin
                _build_inputs!(sim)
                sim.multimachine =
                    get_global_vars_update_pointers(sim.inputs)[GLOBAL_VAR_SYS_FREQ_INDEX] !=
                    0
            end
            TimerOutputs.@timeit BUILD_TIMER "Pre-initialization" begin
                _pre_initialize_simulation!(sim)
            end
            if sim.status != BUILD_FAILED
                simulation_inputs = get_simulation_inputs(sim)
                TimerOutputs.@timeit BUILD_TIMER "Calculate Jacobian" begin
                    jacobian = _get_jacobian(sim)
                end
                TimerOutputs.@timeit BUILD_TIMER "Make Model Function" begin
                    model = T(simulation_inputs, get_initial_conditions(sim), SimCache)
                end
                TimerOutputs.@timeit BUILD_TIMER "Initial Condition NLsolve refinement" begin
                    refine_initial_condition!(sim, model, jacobian)
                end
                TimerOutputs.@timeit BUILD_TIMER "Build Perturbations" begin
                    _build_perturbations!(sim)
                end
                TimerOutputs.@timeit BUILD_TIMER "Make DiffEq Problem" begin
                    _get_diffeq_problem(sim, model, jacobian)
                end
                @info "Simulations status = $(sim.status)"
            else
                @error "The simulation couldn't be initialized correctly. Simulations status = $(sim.status)"
            end
        catch e
            bt = catch_backtrace()
            @error "$T failed to build" exception = e, bt
            sim.status = BUILD_FAILED
        end
    end
    return
end

function build!(sim; kwargs...)
    logger = configure_logging(sim, "w")
    Logging.with_logger(logger) do
        _build!(sim; kwargs...)
        #if sim.status == BUILT
        string_buffer = IOBuffer()
        TimerOutputs.print_timer(
            string_buffer,
            BUILD_TIMER;
            sortby = :firstexec,
            compact = true,
        )
        @info "\n$(String(take!(string_buffer)))\n"
        #end
    end
    close(logger)
    return sim.status
end

function simulation_pre_step!(sim::Simulation)
    if sim.status == BUILD_FAILED
        error(
            "The Simulation status is $(sim.status). Can not continue, correct your inputs and build the simulation again.",
        )
    elseif sim.status == BUILT
        @debug "Simulation status is $(sim.status)."
    elseif sim.status == SIMULATION_FINALIZED
        reset!(sim)
        @info "The Simulation status is $(sim.status). Resetting the simulation"
    else
        error("Simulation status is $(sim.status). Can't continue.")
    end
    return
end

function _prog_meter_enabled()
    return isa(stderr, Base.TTY) &&
           (get(ENV, "CI", nothing) != "true") &&
           (get(ENV, "RUNNING_PSID_TESTS", nothing) != "true")
end

function _filter_kwargs(kwargs)
    return Dict(k => v for (k, v) in kwargs if in(k, DIFFEQ_SOLVE_KWARGS))
end

function _execute!(sim::Simulation, solver; kwargs...)
    @debug "status before execute" sim.status
    simulation_pre_step!(sim)
    sim.status = SIMULATION_STARTED
    time_log = Dict{Symbol, Any}()
    if get(kwargs, :auto_abstol, false)
        cb = AutoAbstol(true, get(kwargs, :abstol, 1e-9))
        callbacks = SciMLBase.CallbackSet((), tuple(push!(sim.callbacks, cb)...))
    else
        callbacks = SciMLBase.CallbackSet((), tuple(sim.callbacks...))
    end

    solution,
    time_log[:timed_solve_time],
    time_log[:solve_bytes_alloc],
    time_log[:sec_in_gc] = @timed SciMLBase.solve(
        sim.problem,
        solver;
        callback = callbacks,
        tstops = !isempty(sim.tstops) ? [sim.tstops[1] ÷ 2, sim.tstops...] : [],
        progress = get(kwargs, :enable_progress_bar, _prog_meter_enabled()),
        progress_steps = 1,
        advance_to_tstop = !isempty(sim.tstops),
        initializealg = SciMLBase.NoInit(),
        _filter_kwargs(kwargs)...,
    )
    sim.results = SimulationResults(
        get_simulation_inputs(sim),
        get_system(sim),
        time_log,
        solution,
    )
    if SciMLBase.successful_retcode(solution)
        sim.status = SIMULATION_FINALIZED
    else
        @error("The simulation failed with return code $(solution.retcode)")
        sim.status = SIMULATION_FAILED
    end
end

"""
    execute!(
        sim::Simulation,
        solver;
        kwargs...
    )

Solves the time-domain dynamic simulation model.

# Arguments
- `sim::Simulation` : Initialized simulation object
- `solver` : Solver used for numerical integration. Must be passed correctly depending on the Type of Simulation Model
- `enable_progress_bar::Bool` : Default: `true`. Enables progress bar for the integration routine.
- Additional solver keyword arguments can be included. See [Common Solver Options](https://diffeq.sciml.ai/stable/basics/common_solver_opts/) in the `DifferentialEquations.jl` documentation for more details.
"""
function execute!(sim::Simulation, solver; kwargs...)
    logger = configure_logging(sim, "a"; kwargs...)
    Logging.with_logger(logger) do
        try
            _execute!(sim, solver; kwargs...)
        catch e
            @error "Execution failed" exception = (e, catch_backtrace())
            sim.status = SIMULATION_FAILED
        end
    end
    close(logger)
    return sim.status
end

function read_results(sim::Simulation)
    return sim.results
end

function get_dynamic_wrapper(sim::Simulation, name::AbstractString)
    return get_dynamic_wrapper(get_simulation_inputs(sim), name)
end

"""
    get_setpoints(sim::Simulation)

Function that returns the reference setpoints for all the dynamic devices.

# Arguments

- `sim::Simulation` : Simulation object that contains the initial condition and setpoints.
"""
function get_setpoints(sim::Simulation)
    return get_setpoints(get_simulation_inputs(sim))
end
