"""
Frequency-reference mode that keeps system frequency fixed at 1.0 p.u.

Use this mode when the network frequency should not track a dynamic device
state during simulation.
"""
struct ConstantFrequency end

"""
Frequency-reference mode that tracks the frequency state at the reference bus.

If a dynamic injector at the slack bus provides a compatible frequency state,
that state is used as the network frequency reference.
"""
struct ReferenceBus end

function _get_frequency_state(d::DynamicWrapper{T}) where {T <: PSY.DynamicGenerator}
    return get_global_index(d)[:ω]
end

function _get_frequency_state(d::DynamicWrapper{T}) where {T <: PSY.DynamicInverter}
    return get_global_index(d)[:ω_oc]
end

function _get_frequency_state(d::DynamicWrapper{PSY.PeriodicVariableSource})
    return 0
end

function get_frequency_reference!(
    ::Type{ConstantFrequency},
    dynamic_injection_data::Vector,
    static_injection_data::Vector,
)
    ref_devices = filter(x -> get_bus_category(x) == SLACKBus, static_injection_data)
    if length(ref_devices) < 1
        dyn_devices = filter(x -> get_bus_category(x) == SLACKBus, dynamic_injection_data)
        if length(dyn_devices) < 1
            throw(
                IS.ConflictingInputsError(
                    "ConstantFrequency model requires at least one generation unit at the Bus ACBusTypes.REF",
                ),
            )
        end
    end
    return 0
end

function get_frequency_reference!(
    ::Type{ReferenceBus},
    dynamic_injection_data::Vector,
    static_injection_data::Vector,
)
    reference = -1
    static_ref_devices = filter(x -> get_bus_category(x) == SLACKBus, static_injection_data)
    if isempty(static_ref_devices)
        ref_devices = filter(x -> get_bus_category(x) == SLACKBus, dynamic_injection_data)
        if length(ref_devices) > 1
            ref_devices = filter(
                x -> (
                    haskey(get_global_index(x), :ω_oc) || haskey(get_global_index(x), :ω)
                ),
                ref_devices,
            )
            if isempty(ref_devices)
                throw(
                    IS.ConflictingInputsError(
                        "ReferenceBus model requires at least one generator with frequency controls.",
                    ),
                )
            end
            ref_devices = sort(ref_devices; by = x -> PSY.get_base_power(x))
        elseif length(ref_devices) < 1
            throw(
                IS.ConflictingInputsError(
                    "ReferenceBus model requires at least one bus of type ACBusTypes.REF with a DynamicInjection or Source device connected to it",
                ),
            )
        elseif length(ref_devices) != 1
            @assert false
        end
        reference = _get_frequency_state(first(ref_devices))
    else
        @info "The reference Bus has a Source connected to it. The frequency reference model will change to ConstantFrequency"
        reference = 0
    end
    return reference
end
