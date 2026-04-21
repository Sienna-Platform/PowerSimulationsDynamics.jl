get_inner_vars_count(::PSY.DynamicGenerator) = GEN_INNER_VARS_SIZE
get_inner_vars_count(::PSY.DynamicInverter) = INV_INNER_VARS_SIZE
get_inner_vars_count(::PSY.PeriodicVariableSource) = 0
get_inner_vars_count(::PSY.SingleCageInductionMachine) = 0
get_inner_vars_count(::PSY.SimplifiedSingleCageInductionMachine) = 0
get_inner_vars_count(::PSY.CSVGN1) = 0
get_inner_vars_count(::PSY.AggregateDistributedGenerationA) = 0
get_inner_vars_count(::PSY.ActiveConstantPowerLoad) = 0

index(::Type{<:PSY.TurbineGov}) = 1
index(::Type{<:PSY.PSS}) = 2
index(::Type{<:PSY.Machine}) = 3
index(::Type{<:PSY.Shaft}) = 4
index(::Type{<:PSY.AVR}) = 5

index(::Type{<:PSY.DCSource}) = 1
index(::Type{<:PSY.FrequencyEstimator}) = 2
index(::Type{<:PSY.OuterControl}) = 3
index(::Type{<:PSY.InnerControl}) = 4
index(::Type{<:PSY.Converter}) = 5
index(::Type{<:PSY.Filter}) = 6

get_delays(::PSY.DynamicInjection) = nothing
get_delays(
    dynamic_injector::PSY.DynamicGenerator{M, S, A, PSY.DEGOV, P},
) where {M <: PSY.Machine, S <: PSY.Shaft, A <: PSY.AVR, P <: PSY.PSS} =
    Float64[PSY.get_Td(PSY.get_prime_mover(dynamic_injector))]

"""
Wraps DynamicInjection devices from PowerSystems to handle changes in controls and connection
status, and allocate the required indexes of the state space.
"""
struct DynamicWrapper{T <: PSY.DynamicInjection}
    device::T
    system_base_power::Float64
    system_base_frequency::Float64
    static_type::Type{<:PSY.StaticInjection}
    bus_category::Type{<:BusCategory}
    connection_status::Base.RefValue{Float64}
    V_ref::Base.RefValue{Float64}
    ω_ref::Base.RefValue{Float64}
    P_ref::Base.RefValue{Float64}
    Q_ref::Base.RefValue{Float64}
    inner_vars_index::Vector{Int}
    ix_range::Vector{Int}
    ode_range::Vector{Int}
    bus_ix::Int
    global_index::Base.ImmutableDict{Symbol, Int}
    component_state_mapping::Base.ImmutableDict{Int, Vector{Int}}
    input_port_mapping::Base.ImmutableDict{Int, Vector{Int}}
    ext::Dict{String, Any}

    function DynamicWrapper(
        device::T,
        system_base_power::Float64,
        system_base_frequency::Float64,
        static_type::Type{<:PSY.StaticInjection},
        bus_category::Type{<:BusCategory},
        connection_status::Base.RefValue{Float64},
        V_ref::Base.RefValue{Float64},
        ω_ref::Base.RefValue{Float64},
        P_ref::Base.RefValue{Float64},
        Q_ref::Base.RefValue{Float64},
        inner_vars_index,
        ix_range,
        ode_range,
        bus_ix::Int,
        global_index::Base.ImmutableDict{Symbol, Int},
        component_state_mapping::Base.ImmutableDict{Int, Vector{Int}},
        input_port_mapping::Base.ImmutableDict{Int, Vector{Int}},
        ext::Dict{String, Any},
    ) where {T <: PSY.DynamicInjection}
        is_valid(device)

        new{T}(
            device,
            system_base_power,
            system_base_frequency,
            static_type,
            bus_category,
            connection_status,
            V_ref,
            ω_ref,
            P_ref,
            Q_ref,
            Vector{Int}(inner_vars_index),
            Vector{Int}(ix_range),
            Vector{Int}(ode_range),
            bus_ix,
            global_index,
            component_state_mapping,
            input_port_mapping,
            ext,
        )
    end
end

function state_port_mappings(
    dynamic_device::T,
    device_states,
) where {T <: Union{PSY.DynamicGenerator, PSY.DynamicInverter}}
    component_state_mapping = Dict{Int, Vector{Int}}()
    input_port_mapping = Dict{Int, Vector{Int}}()
    for c in PSY.get_dynamic_components(dynamic_device)
        if c isa PSY.OutputCurrentLimiter
            continue
        end
        ix = index(typeof(c))
        component_state_mapping[ix] = _index_local_states(c, device_states)
        input_port_mapping[ix] = _index_port_mapping!(c, device_states)
    end
    return (component_state_mapping, input_port_mapping)
end

function state_port_mappings(dynamic_device::PSY.DynamicInjection, device_states)
    component_state_mapping = Dict{Int, Vector{Int}}()
    input_port_mapping = Dict{Int, Vector{Int}}()
    return (component_state_mapping, input_port_mapping)
end

function DynamicWrapper(
    device::T,
    dynamic_device::D,
    bus_ix::Int,
    ix_range,
    ode_range,
    inner_var_range,
    sys_base_power,
    sys_base_freq,
) where {T <: PSY.StaticInjection, D <: PSY.DynamicInjection}
    device_states = PSY.get_states(dynamic_device)

    component_state_mapping, input_port_mapping =
        state_port_mappings(dynamic_device, device_states)

    # Consider the special case when the static device is StandardLoad
    if isa(device, PSY.StandardLoad)
        reactive_power = get_total_q(device)
    else
        reactive_power = PSY.get_reactive_power(device)
    end

    return DynamicWrapper(
        dynamic_device,
        sys_base_power,
        sys_base_freq,
        T,
        BUS_MAP[PSY.get_bustype(PSY.get_bus(device))],
        Base.Ref(1.0),
        Base.Ref(PSY.get_V_ref(dynamic_device)),
        Base.Ref(PSY.get_ω_ref(dynamic_device)),
        Base.Ref(PSY.get_P_ref(dynamic_device)),
        Base.Ref(reactive_power),
        inner_var_range,
        ix_range,
        ode_range,
        bus_ix,
        Base.ImmutableDict(
            sort!(device_states .=> ix_range; by = x -> x.second, rev = true)...,
        ),
        if isempty(component_state_mapping)
            Base.ImmutableDict{Int, Vector{Int}}()
        else
            Base.ImmutableDict(component_state_mapping...)
        end,
        if isempty(input_port_mapping)
            Base.ImmutableDict{Int, Vector{Int}}()
        else
            Base.ImmutableDict(input_port_mapping...)
        end,
        Dict{String, Any}(),
    )
end

function DynamicWrapper(
    device::PSY.ThermalStandard,
    dynamic_device::PSY.AggregateDistributedGenerationA,
    bus_ix::Int,
    ix_range,
    ode_range,
    inner_var_range,
    sys_base_power,
    sys_base_freq,
)
    device_states = PSY.get_states(dynamic_device)

    component_state_mapping, input_port_mapping =
        state_port_mappings(dynamic_device, device_states)
    @warn "Under/over voltage tripping and under/over frequency tripping are not yet implemented for AggregateDistributedGenerationA!"

    return DynamicWrapper(
        dynamic_device,
        sys_base_power,
        sys_base_freq,
        PSY.ThermalStandard,
        BUS_MAP[PSY.get_bustype(PSY.get_bus(device))],
        Base.Ref(1.0),
        Base.Ref(PSY.get_V_ref(dynamic_device)),
        Base.Ref(PSY.get_ω_ref(dynamic_device)),
        Base.Ref(PSY.get_P_ref(dynamic_device)),
        Base.Ref(PSY.get_reactive_power(device)),
        inner_var_range,
        ix_range,
        ode_range,
        bus_ix,
        Base.ImmutableDict(
            sort!(device_states .=> ix_range; by = x -> x.second, rev = true)...,
        ),
        if isempty(component_state_mapping)
            Base.ImmutableDict{Int, Vector{Int}}()
        else
            Base.ImmutableDict(component_state_mapping...)
        end,
        if isempty(input_port_mapping)
            Base.ImmutableDict{Int, Vector{Int}}()
        else
            Base.ImmutableDict(input_port_mapping...)
        end,
        Dict{String, Any}(),
    )
end

function DynamicWrapper(
    device::PSY.Source,
    dynamic_device::D,
    bus_ix::Int,
    ix_range,
    ode_range,
    inner_var_range,
    sys_base_power,
    sys_base_freq,
) where {D <: PSY.DynamicInjection}
    device_states = PSY.get_states(dynamic_device)
    IS.@assert_op PSY.get_X_th(dynamic_device) == PSY.get_X_th(device)
    IS.@assert_op PSY.get_R_th(dynamic_device) == PSY.get_R_th(device)

    return DynamicWrapper(
        dynamic_device,
        sys_base_power,
        sys_base_freq,
        PSY.Source,
        BUS_MAP[PSY.get_bustype(PSY.get_bus(device))],
        Base.Ref(1.0),
        Base.Ref(0.0),
        Base.Ref(0.0),
        Base.Ref(0.0),
        Base.Ref(0.0),
        collect(inner_var_range),
        collect(ix_range),
        collect(ode_range),
        bus_ix,
        Base.ImmutableDict(Dict(device_states .=> ix_range)...),
        Base.ImmutableDict{Int, Vector{Int}}(),
        Base.ImmutableDict{Int, Vector{Int}}(),
        Dict{String, Any}(),
    )
end

function _index_local_states(component::PSY.DynamicComponent, device_states::Vector{Symbol})
    component_state_index = Vector{Int}(undef, PSY.get_n_states(component))
    component_states = PSY.get_states(component)
    for (ix, s) in enumerate(component_states)
        component_state_index[ix] = findfirst(x -> x == s, device_states)
    end
    return component_state_index
end

function _index_port_mapping!(
    component::PSY.DynamicComponent,
    device_states::Vector{Symbol},
)
    ports = Ports(component)
    index_component_inputs = Vector{Int}()
    for i in ports[:states]
        tmp = [(ix, var) for (ix, var) in enumerate(device_states) if var == i]
        isempty(tmp) && continue
        push!(index_component_inputs, tmp[1][1])
    end
    return index_component_inputs
end

get_device(wrapper::DynamicWrapper) = wrapper.device
get_device_type(::DynamicWrapper{T}) where {T <: PSY.DynamicInjection} = T
get_bus_category(wrapper::DynamicWrapper) = wrapper.bus_category
get_inner_vars_index(wrapper::DynamicWrapper) = wrapper.inner_vars_index
get_ix_range(wrapper::DynamicWrapper) = wrapper.ix_range
get_ode_ouput_range(wrapper::DynamicWrapper) = wrapper.ode_range
get_bus_ix(wrapper::DynamicWrapper) = wrapper.bus_ix
get_global_index(wrapper::DynamicWrapper) = wrapper.global_index
get_component_state_mapping(wrapper::DynamicWrapper) = wrapper.component_state_mapping
get_input_port_mapping(wrapper::DynamicWrapper) = wrapper.input_port_mapping
get_ext(wrapper::DynamicWrapper) = wrapper.ext

get_system_base_power(wrapper::DynamicWrapper) = wrapper.system_base_power
get_system_base_frequency(wrapper::DynamicWrapper) = wrapper.system_base_frequency

get_P_ref(wrapper::DynamicWrapper) = wrapper.P_ref[]
get_Q_ref(wrapper::DynamicWrapper) = wrapper.Q_ref[]
get_V_ref(wrapper::DynamicWrapper) = wrapper.V_ref[]
get_ω_ref(wrapper::DynamicWrapper) = wrapper.ω_ref[]

set_P_ref(wrapper::DynamicWrapper, val::Float64) = wrapper.P_ref[] = val
set_Q_ref(wrapper::DynamicWrapper, val::Float64) = wrapper.Q_ref[] = val
set_V_ref(wrapper::DynamicWrapper, val::Float64) = wrapper.V_ref[] = val
set_ω_ref(wrapper::DynamicWrapper, val::Float64) = wrapper.ω_ref[] = val

# PSY overloads for the wrapper
PSY.get_name(wrapper::DynamicWrapper) = PSY.get_name(wrapper.device)
PSY.get_ext(wrapper::DynamicWrapper) = PSY.get_ext(wrapper.device)
PSY.get_states(wrapper::DynamicWrapper) = PSY.get_states(wrapper.device)
PSY.get_n_states(wrapper::DynamicWrapper) = PSY.get_n_states(wrapper.device)
PSY.get_base_power(wrapper::DynamicWrapper) = PSY.get_base_power(wrapper.device)

PSY.get_machine(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicGenerator} =
    wrapper.device.machine
PSY.get_shaft(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicGenerator} =
    wrapper.device.shaft
PSY.get_avr(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicGenerator} =
    wrapper.device.avr
PSY.get_prime_mover(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicGenerator} =
    wrapper.device.prime_mover
PSY.get_pss(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicGenerator} =
    wrapper.device.pss

PSY.get_converter(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    wrapper.device.converter
PSY.get_outer_control(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    wrapper.device.outer_control
PSY.get_inner_control(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    wrapper.device.inner_control
PSY.get_dc_source(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    wrapper.device.dc_source
PSY.get_freq_estimator(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    wrapper.device.freq_estimator
PSY.get_filter(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    wrapper.device.filter
PSY.get_limiter(wrapper::DynamicWrapper{T}) where {T <: PSY.DynamicInverter} =
    PSY.get_limiter(wrapper.device)

# PSY overloads of specific Dynamic Injectors

PSY.get_V_ref(wrapper::PSY.SingleCageInductionMachine) = 1.0
PSY.get_V_ref(wrapper::PSY.SimplifiedSingleCageInductionMachine) = 1.0
PSY.get_P_ref(wrapper::PSY.SingleCageInductionMachine) = PSY.get_τ_ref(wrapper)
PSY.get_P_ref(wrapper::PSY.SimplifiedSingleCageInductionMachine) = PSY.get_τ_ref(wrapper)
PSY.get_ω_ref(wrapper::PSY.SingleCageInductionMachine) = 1.0
PSY.get_ω_ref(wrapper::PSY.SimplifiedSingleCageInductionMachine) = 1.0

function get_local_state_ix(
    wrapper::DynamicWrapper,
    ::Type{T},
) where {T <: PSY.DynamicComponent}
    return wrapper.component_state_mapping[index(T)]
end

function get_input_port_ix(
    wrapper::DynamicWrapper,
    ::Type{T},
) where {T <: PSY.DynamicComponent}
    return wrapper.input_port_mapping[index(T)]
end

function get_local_state_ix(wrapper::DynamicWrapper, ::T) where {T <: PSY.DynamicComponent}
    return get_local_state_ix(wrapper, T)
end

function get_input_port_ix(wrapper::DynamicWrapper, ::T) where {T <: PSY.DynamicComponent}
    return get_input_port_ix(wrapper, T)
end

struct StaticWrapper{T <: PSY.StaticInjection, V}
    device::T
    connection_status::Base.RefValue{Float64}
    V_ref::Base.RefValue{Float64}
    θ_ref::Base.RefValue{Float64}
    P_ref::Base.RefValue{Float64}
    Q_ref::Base.RefValue{Float64}
    bus_ix::Int
    ext::Dict{String, Any}
end

function DynamicWrapper(device::T, bus_ix::Int) where {T <: PSY.Device}
    bus = PSY.get_bus(device)
    StaticWrapper{T, BUS_MAP[PSY.get_bustype(bus)]}(
        device,
        Base.Ref(1.0),
        Base.Ref(PSY.get_magnitude(bus)),
        Base.Ref(PSY.get_angle(bus)),
        Base.Ref(PSY.get_active_power(device)),
        Base.Ref(PSY.get_reactive_power(device)),
        bus_ix,
        Dict{String, Any}(),
    )
end

function StaticWrapper(device::T, bus_ix::Int) where {T <: PSY.Source}
    bus = PSY.get_bus(device)
    return StaticWrapper{T, BUS_MAP[PSY.get_bustype(bus)]}(
        device,
        Base.Ref(1.0),
        Base.Ref(PSY.get_internal_voltage(device)),
        Base.Ref(PSY.get_internal_angle(device)),
        Base.Ref(PSY.get_active_power(device)),
        Base.Ref(PSY.get_reactive_power(device)),
        bus_ix,
        Dict{String, Any}(),
    )
end

# get_bustype is already exported in PSY. So this is named this way to avoid name collisions
get_bus_category(::StaticWrapper{<:PSY.StaticInjection, U}) where {U} = U

# TODO: something smart to forward fields
get_device(wrapper::StaticWrapper) = wrapper.device
get_bus_ix(wrapper::StaticWrapper) = wrapper.bus_ix
get_ext(wrapper::StaticWrapper) = wrapper.ext

get_P_ref(wrapper::StaticWrapper) = wrapper.P_ref[]
get_Q_ref(wrapper::StaticWrapper) = wrapper.Q_ref[]
get_V_ref(wrapper::StaticWrapper) = wrapper.V_ref[]
get_θ_ref(wrapper::StaticWrapper) = wrapper.θ_ref[]

set_P_ref(wrapper::StaticWrapper, val::Float64) = wrapper.P_ref[] = val
set_Q_ref(wrapper::StaticWrapper, val::Float64) = wrapper.Q_ref[] = val
set_V_ref(wrapper::StaticWrapper, val::Float64) = wrapper.V_ref[] = val
set_θ_ref(wrapper::StaticWrapper, val::Float64) = wrapper.θ_ref[] = val

PSY.get_bus(wrapper::StaticWrapper) = PSY.get_bus(wrapper.device)
PSY.get_active_power(wrapper::StaticWrapper) = PSY.get_active_power(wrapper.device)
PSY.get_reactive_power(wrapper::StaticWrapper) = PSY.get_reactive_power(wrapper.device)
PSY.get_name(wrapper::StaticWrapper) = PSY.get_name(wrapper.device)
PSY.get_ext(wrapper::StaticWrapper) = PSY.get_ext(wrapper.device)

mutable struct ExpLoadParams
    P_exp::Float64
    P_coeff::Float64
    Q_exp::Float64
    Q_coeff::Float64
end

mutable struct StaticLoadWrapper
    bus::PSY.Bus
    V_ref::Float64
    θ_ref::Float64
    P_power::Float64
    P_current::Float64
    P_impedance::Float64
    Q_power::Float64
    Q_current::Float64
    Q_impedance::Float64
    exp_params::Vector{ExpLoadParams}
    exp_names::Dict{String, Int}
    bus_ix::Int
end

function _zip_load_terms(::PSY.ElectricLoad, _sys)
    return (
        P_power = 0.0,
        P_current = 0.0,
        P_impedance = 0.0,
        Q_power = 0.0,
        Q_current = 0.0,
        Q_impedance = 0.0,
    )
end

function _zip_load_terms(ld::PSY.PowerLoad, sys)
    return (
        P_power = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_active_power(ld)
        end,
        P_current = 0.0,
        P_impedance = 0.0,
        Q_power = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_reactive_power(ld)
        end,
        Q_current = 0.0,
        Q_impedance = 0.0,
    )
end

function _zip_load_terms(ld::PSY.StandardLoad, sys)
    return (
        P_power = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_constant_active_power(ld)
        end,
        P_current = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_current_active_power(ld)
        end,
        P_impedance = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_impedance_active_power(ld)
        end,
        Q_power = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_constant_reactive_power(ld)
        end,
        Q_current = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_current_reactive_power(ld)
        end,
        Q_impedance = PSY.with_units_base(sys, PSY.UnitSystem.SYSTEM_BASE) do
            PSY.get_impedance_reactive_power(ld)
        end,
    )
end

function StaticLoadWrapper(
    bus::PSY.Bus,
    loads::Vector{PSY.ElectricLoad},
    bus_ix::Int,
    sys_base_power::Float64,
    sys::PSY.System,
)
    P_power = 0.0
    P_current = 0.0
    P_impedance = 0.0
    Q_power = 0.0
    Q_current = 0.0
    Q_impedance = 0.0

    # Add ZIP Loads
    for ld in loads
        terms = _zip_load_terms(ld, sys)
        P_power += terms.P_power
        P_current += terms.P_current
        P_impedance += terms.P_impedance
        Q_power += terms.Q_power
        Q_current += terms.Q_current
        Q_impedance += terms.Q_impedance
    end

    # Add Exponential Loads
    exp_loads = filter(x -> isa(x, PSY.ExponentialLoad) && PSY.get_available(x), loads)
    exp_params = Vector{ExpLoadParams}(undef, length(exp_loads))
    dict_names = Dict{String, Int}()
    if !isempty(exp_loads)
        for (ix, ld) in enumerate(exp_loads)
            base_power_conversion = PSY.get_base_power(ld) / sys_base_power
            dict_names[PSY.get_name(ld)] = ix
            exp_params[ix] = ExpLoadParams(
                PSY.get_active_power(ld) * base_power_conversion,
                PSY.get_α(ld),
                PSY.get_reactive_power(ld) * base_power_conversion,
                PSY.get_β(ld),
            )
        end
    end

    return StaticLoadWrapper(
        bus,
        PSY.get_magnitude(bus),
        PSY.get_angle(bus),
        P_power,
        P_current,
        P_impedance,
        Q_power,
        Q_current,
        Q_impedance,
        exp_params,
        dict_names,
        bus_ix,
    )
end

PSY.get_bus(wrapper::StaticLoadWrapper) = wrapper.bus
PSY.get_name(wrapper::StaticLoadWrapper) = PSY.get_name(wrapper.bus)

get_V_ref(wrapper::StaticLoadWrapper) = wrapper.V_ref
get_θ_ref(wrapper::StaticLoadWrapper) = wrapper.θ_ref
get_P_power(wrapper::StaticLoadWrapper) = wrapper.P_power
get_P_current(wrapper::StaticLoadWrapper) = wrapper.P_current
get_P_impedance(wrapper::StaticLoadWrapper) = wrapper.P_impedance
get_Q_power(wrapper::StaticLoadWrapper) = wrapper.Q_power
get_Q_current(wrapper::StaticLoadWrapper) = wrapper.Q_current
get_Q_impedance(wrapper::StaticLoadWrapper) = wrapper.Q_impedance
get_exp_params(wrapper::StaticLoadWrapper) = wrapper.exp_params
get_exp_names(wrapper::StaticLoadWrapper) = wrapper.exp_names
get_bus_ix(wrapper::StaticLoadWrapper) = wrapper.bus_ix

set_V_ref!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.V_ref = val
set_θ_ref!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.θ_ref = val
set_P_power!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.P_power = val
set_P_current!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.P_current = val
set_P_impedance!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.P_impedance = val
set_Q_power!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.Q_power = val
set_Q_current!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.Q_current = val
set_Q_impedance!(wrapper::StaticLoadWrapper, val::Float64) = wrapper.Q_impedance = val

function set_connection_status(wrapper::Union{StaticWrapper, DynamicWrapper}, val::Int)
    if val == 0
        @debug "Generator $(PSY.get_name(wrapper)) status set to off"
    elseif val == 1
        @debug "Generator $(PSY.get_name(wrapper)) status set to on"
    else
        error("Invalid status $val. It can only take values 1 or 0")
    end
    wrapper.connection_status[] = Float64(val)
end

get_connection_status(wrapper::Union{StaticWrapper, DynamicWrapper}) =
    wrapper.connection_status[]
