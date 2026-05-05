abstract type Perturbation end

"""
    mutable struct BranchTrip <: Perturbation
        time::Float64
        branch_type::Type{<:[`PowerSystems.ACBranch`](@extref)}
        branch_name::String
    end

A BranchTrip completely disconnects a branch from the system. Currently there is only support for static branches disconnection, [`PowerSystems.Line`](@extref) and [`PowerSystems.Transformer2W`](@extref).
Future releases will provide support for a Dynamic Line disconnection.
**Note:** Islanding is currently not supported in `PowerSimulationsDynamics.jl`. If a `BranchTrip` isolates a generation unit, the system may diverge due to the isolated generator.

# Arguments:

- `time::Float64` : Defines when the Branch Trip will happen. This time should be inside the time span considered in the Simulation
- `branch_tipe::Type{<:PowerSystems.ACBranch}` : Branch type (subtype of [`PowerSystems.ACBranch`](@extref))
- `branch_name::String` : User defined name for identifying the branch
"""
mutable struct BranchTrip <: Perturbation
    time::Float64
    branch_type::Type{<:PSY.ACBranch}
    branch_name::String
end

"""
    mutable struct BranchImpedanceChange <: Perturbation
        time::Float64
        branch_type::Type{<:PSY.ACBranch}
        branch_name::String
        multiplier::Float64
    end

A BranchImpedanceChange change the impedance of a branch by a user defined multiplier. Currently there is only support for static branches disconnection, [`PowerSystems.Line`](@extref) and [`PowerSystems.Transformer2W`](@extref).
Future releases will provide support for a Dynamic Line disconnection.

# Arguments:

- `time::Float64` : Defines when the Branch Impedance Change will happen. This time should be inside the time span considered in the Simulation
- `branch_tipe::Type{<:PowerSystems.ACBranch}` : Branch type (subtype of [`PowerSystems.ACBranch`](@extref))
- `branch_name::String` : User defined name for identifying the branch
- `multiplier::Float64` : User defined value for impedance multiplier.
"""
mutable struct BranchImpedanceChange <: Perturbation
    time::Float64
    branch_type::Type{<:PSY.ACBranch}
    branch_name::String
    multiplier::Float64
end

function _get_branch_for_perturbation(
    sys::PSY.System,
    perturbation::T,
) where {T <: Perturbation}
    if perturbation.branch_type == PSY.DynamicBranch
        error("DynamicBranch is not supported currently with perturbation $T")
    end

    if length(PSY.get_components(perturbation.branch_type, sys)) < 1
        error(
            "The system does not contain branches type $(perturbation.branch_type). \\
      If used all_branches_dynamic or all_lines_dynamic make sure the branch $(perturbation.branch_name) was not converted",
        )
    end

    branch = PSY.get_component(perturbation.branch_type, sys, perturbation.branch_name)
    if branch === nothing || !PSY.get_available(branch)
        throw(
            IS.ConflictingInputsError(
                "Branch $(perturbation.branch_type) $(perturbation.branch_name) not existent or unavailable in the system. Use PSY.show_components(system, $(perturbation.branch_type)) to check valid component names",
            ),
        )
    end
    return branch
end

function get_affect(::SimulationInputs, sys::PSY.System, pert::BranchImpedanceChange)
    branch = _get_branch_for_perturbation(sys, pert)
    mult = 0.0
    if pert.multiplier < 0.0
        throw(
            IS.ConflictingInputsError(
                "Negative multipliers are not allowed for BranchImpedanceChange perturbations",
            ),
        )
    elseif pert.multiplier < 1.0
        mult = 1.0 - pert.multiplier
    elseif pert.multiplier > 1.0
        mult = pert.multiplier - 1.0
    else
        @assert pert.multiplier == 1.0
    end

    return (integrator) -> begin
        @debug "Changing impedance line $(PSY.get_name(branch)) by a factor of $(pert.multiplier)"
        ybus_update!(integrator.p, branch, mult)
    end

    return
end

function get_affect(::SimulationInputs, sys::PSY.System, pert::BranchTrip)
    branch = _get_branch_for_perturbation(sys, pert)
    return (integrator) -> begin
        @debug "Tripping line $(PSY.get_name(branch))"
        ybus_update!(integrator.p, branch, -1.0)
    end
    return
end

function _record_change!(
    ybus::SparseArrays.SparseMatrixCSC{Float32, Int},
    bus_from_no::Int,
    bus_to_no::Int,
    n_buses::Int,
    Y11_real::Float64,
    Y11_imag::Float64,
    Y22_real::Float64,
    Y22_imag::Float64,
    Y12_real::Float64,
    Y21_real::Float64,
    Y12_imag::Float64,
    Y21_imag::Float64,
)

    # First Quadrant Real Part Changes
    ybus[bus_from_no, bus_from_no] += Y11_real
    ybus[bus_from_no, bus_to_no] += Y12_real
    ybus[bus_to_no, bus_from_no] += Y21_real
    ybus[bus_to_no, bus_to_no] += Y22_real

    # Second Quadrant Imag Part changes
    ybus[bus_from_no, bus_from_no + n_buses] += Y11_imag
    ybus[bus_from_no, bus_to_no + n_buses] += Y12_imag
    ybus[bus_to_no, bus_from_no + n_buses] += Y21_imag
    ybus[bus_to_no, bus_to_no + n_buses] += Y22_imag

    # Third Quadrant -1*Imag Part changes
    ybus[bus_from_no + n_buses, bus_from_no] -= Y11_imag
    ybus[bus_from_no + n_buses, bus_to_no] -= Y12_imag
    ybus[bus_to_no + n_buses, bus_from_no] -= Y21_imag
    ybus[bus_to_no + n_buses, bus_to_no] -= Y22_imag

    # Fourth Quadrant Real Part Changes
    ybus[bus_from_no + n_buses, bus_from_no + n_buses] += Y11_real
    ybus[bus_from_no + n_buses, bus_to_no + n_buses] += Y12_real
    ybus[bus_to_no + n_buses, bus_from_no + n_buses] += Y21_real
    ybus[bus_to_no + n_buses, bus_to_no + n_buses] += Y22_real
    return
end

function ybus_update!(
    ybus::SparseArrays.SparseMatrixCSC{Float32, Int},
    b::PSY.Line,
    num_bus::Dict{Int, Int},
    mult::Float64,
)
    arc = PSY.get_arc(b)
    bus_from_no = num_bus[PSY.get_number(arc.from)]
    bus_to_no = num_bus[arc.to.number]
    n_buses = length(num_bus)

    Y_l = (1 / (PSY.get_r(b) + PSY.get_x(b) * 1im))

    Y11_real = mult * real(Y_l)
    Y11_imag = mult * (imag(Y_l) + PSY.get_b(b).from)
    Y22_real = mult * real(Y_l)
    Y22_imag = mult * (imag(Y_l) + PSY.get_b(b).to)
    Y12_real = Y21_real = -mult * real(Y_l)
    Y12_imag = Y21_imag = -mult * imag(Y_l)

    _record_change!(
        ybus,
        bus_from_no,
        bus_to_no,
        n_buses,
        Y11_real,
        Y11_imag,
        Y22_real,
        Y22_imag,
        Y12_real,
        Y21_real,
        Y12_imag,
        Y21_imag,
    )
    return
end

function ybus_update!(
    ybus::SparseArrays.SparseMatrixCSC{Float32, Int},
    b::PSY.Transformer2W,
    num_bus::Dict{Int, Int},
    mult::Float64,
)
    arc = PSY.get_arc(b)
    bus_from_no = num_bus[PSY.get_number(arc.from)]
    bus_to_no = num_bus[arc.to.number]
    n_buses = length(num_bus)

    Y_l = (1 / (PSY.get_r(b) + PSY.get_x(b) * 1im))

    Y11_real = mult * real(Y_l)
    Y11_imag = mult * imag(Y_l) - PSY.get_primary_shunt(b)
    Y22_real = mult * real(Y_l)
    Y22_imag = mult * imag(Y_l)
    Y12_real = Y21_real = -mult * real(Y_l)
    Y12_imag = Y21_imag = -mult * imag(Y_l)

    _record_change!(
        ybus,
        bus_from_no,
        bus_to_no,
        n_buses,
        Y11_real,
        Y11_imag,
        Y22_real,
        Y22_imag,
        Y12_real,
        Y21_real,
        Y12_imag,
        Y21_imag,
    )
    return
end

function ybus_update!(
    ybus::SparseArrays.SparseMatrixCSC{Float32, Int},
    b::PSY.TapTransformer,
    num_bus::Dict{Int, Int},
    mult::Float64,
)
    arc = PSY.get_arc(b)
    bus_from_no = num_bus[PSY.get_number(arc.from)]
    bus_to_no = num_bus[arc.to.number]
    n_buses = length(num_bus)

    Y_t = 1 / (PSY.get_r(b) + PSY.get_x(b) * 1im)
    c = 1 / PSY.get_tap(b)
    b = PSY.get_primary_shunt(b)

    Y11 = (Y_t * c^2)
    Y21 = Y12 = (-Y_t * c)
    Y22 = Y_t

    Y11_real = mult * real(Y11)
    Y11_imag = mult * imag(Y12) - b
    Y22_real = mult * real(Y22)
    Y22_imag = mult * imag(Y22)
    Y12_real = -mult * real(Y12)
    Y21_real = -mult * real(Y21)
    Y12_imag = -mult * imag(Y12)
    Y21_imag = -mult * imag(Y21)

    _record_change!(
        ybus,
        bus_from_no,
        bus_to_no,
        n_buses,
        Y11_real,
        Y11_imag,
        Y22_real,
        Y22_imag,
        Y12_real,
        Y21_real,
        Y12_imag,
        Y21_imag,
    )
    return
end

function ybus_update!(
    ybus::SparseArrays.SparseMatrixCSC{Float32, Int},
    b::PSY.PhaseShiftingTransformer,
    num_bus::Dict{Int, Int},
    mult::Float64,
)
    arc = PSY.get_arc(b)
    bus_from_no = num_bus[PSY.get_number(arc.from)]
    bus_to_no = num_bus[arc.to.number]
    n_buses = length(num_bus)

    Y_t = 1 / (PSY.get_r(b) + PSY.get_x(b) * 1im)
    tap = (PSY.get_tap(b) * exp(PSY.get_α(b) * 1im))
    c_tap = (PSY.get_tap(b) * exp(-1 * PSY.get_α(b) * 1im))
    b = PSY.get_primary_shunt(b)

    Y11 = (Y_t / abs(tap)^2)
    Y12 = (-Y_t / c_tap)
    Y21 = (-Y_t / tap)
    Y22 = Y_t

    Y11_real = mult * real(Y11)
    Y11_imag = mult * imag(Y12) - b
    Y22_real = mult * real(Y22)
    Y22_imag = mult * imag(Y22)
    Y12_real = -mult * real(Y12)
    Y21_real = -mult * real(Y21)
    Y12_imag = -mult * imag(Y12)
    Y21_imag = -mult * imag(Y21)

    _record_change!(
        ybus,
        bus_from_no,
        bus_to_no,
        n_buses,
        Y11_real,
        Y11_imag,
        Y22_real,
        Y22_imag,
        Y12_real,
        Y21_real,
        Y12_imag,
        Y21_imag,
    )

    return
end

# NOTE: A FixedAdmittance ybus_update! method intentionally does not exist.
# FixedAdmittance is folded into the static Ybus by PNM.Ybus(sys) at build time
# (see _wrap_static_injectors / _wrap_loads in simulation_inputs.jl) and is
# never updated dynamically. If a FixedAdmittance perturbation is added in the
# future, the new ybus_update! method must mirror the rectangular four-entry
# pattern used for ACBranch above, since ybus_rectangular is real-valued Float32.

function ybus_update!(integrator_params, branch::PSY.ACBranch, mult::Float64)
    ybus_update!(integrator_params.ybus_rectangular, branch, integrator_params.lookup, mult)
    return
end

"""
    function NetworkSwitch(
        time::Float64,
        ybus::SparseArrays.SparseMatrixCSC{Complex{Float64}, Int},
    )

Allows to modify directly the admittance matrix, Ybus, used in the Simulation.
This allows the user to perform branch modifications, three phase faults (with impedance larger than zero) or branch trips, as long as the new Ybus provided captures that perturbation.

# Arguments:

- `time::Float64` : Defines when the Network Switch will happen. This time should be inside the time span considered in the Simulation
- `ybus::SparseArrays.SparseMatrixCSC{Complex, Int}` : Complex admittance matrix
"""
mutable struct NetworkSwitch <: Perturbation
    time::Float64
    ybus_rectangular::SparseArrays.SparseMatrixCSC{Float32, Int}
    function NetworkSwitch(
        time::Float64,
        ybus::SparseArrays.SparseMatrixCSC{Complex{Float32}, Int},
    )
        n_bus = size(ybus)[1]
        sub_nets = PNM.find_subnetworks(ybus, collect(1:n_bus))
        length(sub_nets) > 1 && throw(IS.DataFormatError("Network not connected"))
        # TODO: Improve performance here
        ybus_rect = transform_ybus_to_rectangular(ybus)
        new(time, ybus_rect)
    end
end

function NetworkSwitch(time::Float64, ybus::PNM.Ybus)
    return NetworkSwitch(time, ybus.data)
end

function get_affect(::SimulationInputs, ::PSY.System, pert::NetworkSwitch)
    return (integrator) -> begin
        # TODO: This code can be more performant using SparseMatrix methods
        for (i, v) in enumerate(pert.ybus_rectangular)
            @debug "Changing Ybus network"
            integrator.p.ybus_rectangular[i] = v
        end
        return
    end
end

"""
    mutable struct ControlReferenceChange <: Perturbation
        time::Float64
        device::[`PowerSystems.DynamicInjection`](@extref)
        signal::Symbol
        ref_value::Float64
    end

A ControlReferenceChange allows to change the reference setpoint provided by a generator/inverter.

# Arguments:

- `time::Float64` : Defines when the Control Reference Change will happen. This time should be inside the time span considered in the Simulation
- `device::Type{<:PowerSystems.DynamicInjection}` : Dynamic device ([`PowerSystems.DynamicInjection`](@extref))
- `signal::Symbol` : determines which reference setpoint will be modified. The accepted signals are:
    - `:P_ref`: Modifies the active power reference setpoint.
    - `:V_ref`: Modifies the voltage magnitude reference setpoint (if used).
    - `:Q_ref`: Modifies the reactive power reference setpoint (if used).
    - `:ω_ref`: Modifies the frequency setpoint.
- `ref_value::Float64` : User defined value for setpoint reference.
"""
mutable struct ControlReferenceChange <: Perturbation
    time::Float64
    device::PSY.DynamicInjection
    signal::Symbol
    ref_value::Float64

    function ControlReferenceChange(
        time::Float64,
        device::PSY.DynamicInjection,
        signal::Symbol,
        ref_value::Float64,
    )
        if signal ∉ ACCEPTED_CONTROL_REFS
            error("Signal $signal not accepted as a control reference change")
        end
        new(time, device, signal, ref_value)
    end
end

function _is_same_device(
    device1::DynamicWrapper{T},
    device2::T,
) where {T <: PSY.DynamicInjection}
    return PSY.get_name(device1) == PSY.get_name(device2)
end

function _is_same_device(
    ::DynamicWrapper{T},
    ::U,
) where {T <: PSY.DynamicInjection, U <: PSY.DynamicInjection}
    return false
end

function _find_device_index(inputs::SimulationInputs, device::PSY.DynamicInjection)
    wrapped_devices = get_dynamic_injectors(inputs)
    wrapped_device_ixs = findall(x -> _is_same_device(x, device), wrapped_devices)
    if isempty(wrapped_device_ixs)
        error(
            "Device $(typeof(device))-$(PSY.get_name(device)) not found in the simulation inputs",
        )
    end
    return wrapped_device_ixs[1]
end

function _is_same_device(
    device1::StaticWrapper{T},
    device2::U,
) where {T <: PSY.StaticInjection, U <: PSY.StaticInjection}
    if T != U
        return false
    end
    if PSY.get_name(device1) == PSY.get_name(device2)
        return true
    elseif PSY.get_name(device1) != PSY.get_name(device2)
        return false
    else
        error("comparison failed for $device1 and $device2")
    end
end

function _find_device_index(inputs::SimulationInputs, device::PSY.StaticInjection)
    wrapped_devices = get_static_injectors(inputs)
    wrapped_device_ixs = findall(x -> _is_same_device(x, device), wrapped_devices)
    if isempty(wrapped_device_ixs)
        error(
            "Device $(typeof(device))-$(PSY.get_name(device)) not found in the simulation inputs",
        )
    end
    return wrapped_device_ixs[1]
end

function get_affect(inputs::SimulationInputs, ::PSY.System, pert::ControlReferenceChange)
    wrapped_device_ix = _find_device_index(inputs, pert.device)
    return (integrator) -> begin
        wrapped_device = get_dynamic_injectors(integrator.p)[wrapped_device_ix]
        @debug "Changing $(PSY.get_name(wrapped_device)) $(pert.signal) to $(pert.ref_value)"
        getfield(wrapped_device, pert.signal)[] = pert.ref_value
        return
    end
end

"""
    mutable struct SourceBusVoltageChange <: Perturbation
        time::Float64
        device::PSY.Source
        signal::Symbol
        ref_value::Float64
    end

A `SourceBusVoltageChange` allows to change the reference setpoint provided by a voltage source.

# Arguments:

- `time::Float64` : Defines when the Control Reference Change will happen. This time should be inside the time span considered in the Simulation
- `device::Type{<:PowerSystems.Source}` : Voltage source device ([`PowerSystems.Source`](@extref))
- `signal::Symbol` : determines which reference setpoint will be modified. The accepted signals are:
    - :V_ref Modifies the internal voltage magnitude reference setpoint.
    - :θ_ref  Modifies the internal voltage angle reference setpoint.
- `ref_value::Float64` : User defined value for setpoint reference.
"""
mutable struct SourceBusVoltageChange <: Perturbation
    time::Float64
    device::PSY.Source
    signal::Symbol
    ref_value::Float64
end

function get_affect(inputs::SimulationInputs, ::PSY.System, pert::SourceBusVoltageChange)
    wrapped_device_ix = _find_device_index(inputs, pert.device)
    return (integrator) -> begin
        wrapped_device = get_static_injectors(integrator.p)[wrapped_device_ix]
        if pert.signal == :V_ref
            set_V_ref(wrapped_device, pert.ref_value)
        elseif pert.signal == :θ_ref
            set_θ_ref(wrapped_device, pert.ref_value)
        else
            error("Signal $signal not accepted as a control reference change in SourceBus")
        end
        return
    end
end

"""
    mutable struct GeneratorTrip <: Perturbation
        time::Float64
        device::[`PowerSystems.DynamicInjection`](@extref)
    end

A `GeneratorTrip` allows to disconnect a Dynamic Generation unit from the system at a specified time.

# Arguments:

- `time::Float64` : Defines when the Generator Trip will happen. This time should be inside the time span considered in the Simulation
- `device::Type{<:PowerSystems.DynamicInjection}` : Device to disconnect ([`PowerSystems.DynamicInjection`](@extref))
"""
mutable struct GeneratorTrip <: Perturbation
    time::Float64
    device::PSY.DynamicInjection
end

function get_affect(inputs::SimulationInputs, ::PSY.System, pert::GeneratorTrip)
    wrapped_device_ix = _find_device_index(inputs, pert.device)
    return (integrator) -> begin
        wrapped_device = get_dynamic_injectors(integrator.p)[wrapped_device_ix]
        ix_range = get_ix_range(wrapped_device)
        @debug "Changing connection status $(PSY.get_name(wrapped_device)), setting states $ix_range to 0.0"
        if integrator.du !== nothing
            @debug "setting du $ix_range to 0.0"
            integrator.du[ix_range] .= 0.0
        end
        integrator.u[ix_range] .= 0.0
        set_connection_status(wrapped_device, 0)
    end
end

"""
    mutable struct LoadChange <: Perturbation
        time::Float64
        device::[`PowerSystems.ElectricLoad`](@extref)
        signal::Symbol
        ref_value::Float64
    end

A LoadChange allows to change the active or reactive power setpoint from a load.

# Arguments:

- `time::Float64` : Defines when the Load Change will happen. This time should be inside the time span considered in the Simulation
- `device::Type{<:PowerSystems.ElectricLoad}` : Load device ([`PowerSystems.ElectricLoad`](@extref))
- `signal::Symbol` : determines which reference setpoint will be modified. The accepted signals are:
    - `:P_ref`: Modifies the active power reference setpoint.
    - `:Q_ref`: Modifies the reactive power reference setpoint.
- `ref_value::Float64` : User defined value for setpoint reference.
"""
mutable struct LoadChange <: Perturbation
    time::Float64
    device::PSY.ElectricLoad
    signal::Symbol
    ref_value::Float64

    function LoadChange(
        time::Float64,
        device::PSY.ElectricLoad,
        signal::Symbol,
        ref_value::Float64,
    )
        # Currently I'm assumming P_ref and Q_ref are constant impedance to
        if signal ∈ [:P_ref, :Q_ref]
            @warn(
                "P_ref and Q_ref signals will be deprecated. It will be assumed as a change in constant impedance for StandardLoads and a change in constant power for PowerLoads. Allowed signals are $(ACCEPTED_LOADCHANGE_REFS)"
            )
        end
        if signal ∉ ACCEPTED_LOADCHANGE_REFS
            error("Signal $signal not accepted as a control reference change in Loads")
        end
        new(time, device, signal, ref_value)
    end
end

function _find_device_index(inputs::SimulationInputs, device::PSY.ElectricLoad)
    wrapped_devices = get_static_loads(inputs)
    wrapped_device_ixs = findall(x -> _is_same_device(x, device), wrapped_devices)
    if isempty(wrapped_device_ixs)
        error(
            "Device $(typeof(device))-$(PSY.get_name(device)) not found in the simulation inputs",
        )
    end
    return wrapped_device_ixs[1]
end

function get_affect(inputs::SimulationInputs, sys::PSY.System, pert::LoadChange)
    sys_base_power = PSY.get_base_power(sys)
    wrapped_device_ix = _find_zip_load_ix(inputs, pert.device)
    ld = pert.device
    if !PSY.get_available(ld)
        @error("Load $(PSY.get_name(ld)) is unavailable. Perturbation ignored")
        return
    end
    ref_value = pert.ref_value
    signal = pert.signal
    if isa(ld, PSY.PowerLoad)
        return (integrator) -> begin
            base_power_conversion = PSY.get_base_power(ld) / sys_base_power
            P_old = PSY.get_active_power(ld)
            Q_old = PSY.get_reactive_power(ld)
            P_change = 0.0
            Q_change = 0.0
            if signal ∈ [:P_ref, :P_ref_power]
                P_change = (ref_value - P_old) * base_power_conversion
            elseif signal ∈ [:Q_ref, :Q_ref_power]
                Q_change = ref_value - Q_old * base_power_conversion
            else
                error(
                    "Signal is not accepted for Constant PowerLoad. Please specify the correct signal type.",
                )
            end
            wrapped_zip = get_static_loads(integrator.p)[wrapped_device_ix]
            P_power = get_P_power(wrapped_zip)
            Q_power = get_Q_power(wrapped_zip)
            set_P_power!(wrapped_zip, P_power + P_change)
            set_Q_power!(wrapped_zip, Q_power + Q_change)
            @debug "Changing load at bus $(PSY.get_name(wrapped_zip)) $(pert.signal) to $(pert.ref_value)"
            return
        end
    elseif isa(ld, PSY.StandardLoad)
        return (integrator) -> begin
            base_power_conversion = PSY.get_base_power(ld) / sys_base_power
            wrapped_zip = get_static_loads(integrator.p)[wrapped_device_ix]
            # List all cases for StandardLoad changes
            if signal ∈ [:P_ref, :P_ref_impedance]
                P_old = PSY.get_impedance_active_power(ld)
                P_change = (ref_value - P_old) * base_power_conversion
                P_impedance = get_P_impedance(wrapped_zip)
                set_P_impedance!(wrapped_zip, P_impedance + P_change)
            elseif signal ∈ [:Q_ref, :Q_ref_impedance]
                Q_old = PSY.get_impedance_reactive_power(ld)
                Q_change = (ref_value - Q_old) * base_power_conversion
                Q_impedance = get_Q_impedance(wrapped_zip)
                set_Q_impedance!(wrapped_zip, Q_impedance + Q_change)
            elseif signal == :P_ref_power
                P_old = PSY.get_constant_active_power(ld)
                P_change = (ref_value - P_old) * base_power_conversion
                P_power = get_P_power(wrapped_zip)
                set_P_power!(wrapped_zip, P_power + P_change)
            elseif signal == :Q_ref_power
                Q_old = PSY.get_constant_reactive_power(ld)
                Q_change = (ref_value - Q_old) * base_power_conversion
                Q_power = get_Q_power(wrapped_zip)
                set_Q_power!(wrapped_zip, Q_power + Q_change)
            elseif signal == :P_ref_current
                P_old = PSY.get_current_active_power(ld)
                P_change = (ref_value - P_old) * base_power_conversion
                P_current = get_P_current(wrapped_zip)
                set_P_current!(wrapped_zip, P_current + P_change)
            elseif signal == :Q_ref_current
                Q_old = PSY.get_current_reactive_power(ld)
                Q_change = (ref_value - Q_old) * base_power_conversion
                Q_current = get_Q_current(wrapped_zip)
                set_Q_current!(wrapped_zip, Q_current + Q_change)
            else
                error("It should never be here. Should have failed in the constructor.")
            end
        end
    elseif isa(ld, PSY.ExponentialLoad)
        return (integrator) -> begin
            base_power_conversion = PSY.get_base_power(ld) / sys_base_power
            P_old = PSY.get_active_power(ld)
            Q_old = PSY.get_reactive_power(ld)
            P_change = 0.0
            Q_change = 0.0
            if signal ∈ [:P_ref, :P_ref_power]
                P_change = (ref_value - P_old) * base_power_conversion
            elseif signal ∈ [:Q_ref, :Q_ref_power]
                Q_change = (ref_value - Q_old) * base_power_conversion
            else
                error(
                    "Signal $(signal) is not accepted for ExponentialLoad. Use :P_ref/:P_ref_power or :Q_ref/:Q_ref_power.",
                )
            end
            ld_name = PSY.get_name(ld)
            wrapped_zip = get_static_loads(integrator.p)[wrapped_device_ix]
            exp_names = get_exp_names(wrapped_zip)
            exp_vects = get_exp_params(wrapped_zip)
            tuple_ix = exp_names[ld_name]
            exp_params = exp_vects[tuple_ix]
            exp_params.P_exp += P_change
            exp_params.Q_exp += Q_change
            @debug "Changing exponential load $(ld_name) at wrapper $(PSY.get_name(wrapped_zip)) $(signal) to $(ref_value)"
            return
        end
    else
        error(
            "The load type of load $(PSY.get_name(ld)) is not supported for a LoadChange perturbation",
        )
    end
    return
end

"""
    mutable struct LoadTrip <: Perturbation
        time::Float64
        device::[`PowerSystems.ElectricLoad`](@extref)
    end

A `LoadTrip` allows the user to disconnect a load from the system.

# Arguments:

- `time::Float64` : Defines when the Generator Trip will happen. This time should be inside the time span considered in the Simulation
- `device::Type{<:PowerSystems.ElectricLoad}` : Load to disconnect ([`PowerSystems.ElectricLoad`](@extref))
"""
mutable struct LoadTrip <: Perturbation
    time::Float64
    device::PSY.ElectricLoad
end

function get_affect(inputs::SimulationInputs, sys::PSY.System, pert::LoadTrip)
    sys_base_power = PSY.get_base_power(sys)
    wrapped_device_ix = _find_zip_load_ix(inputs, pert.device)
    ld = pert.device
    if !PSY.get_available(ld)
        @error("Load $(PSY.get_name(ld)) is unavailable. Perturbation ignored")
        return
    end
    if isa(ld, PSY.PowerLoad)
        base_power_conversion = PSY.get_base_power(ld) / sys_base_power
        P_trip = PSY.get_active_power(ld) * base_power_conversion
        Q_trip = PSY.get_reactive_power(ld) * base_power_conversion
        return (integrator) -> begin
            PSY.set_available!(ld, false)
            wrapped_zip = get_static_loads(integrator.p)[wrapped_device_ix]
            P_power = get_P_power(wrapped_zip)
            Q_power = get_Q_power(wrapped_zip)
            set_P_power!(wrapped_zip, P_power - P_trip)
            set_Q_power!(wrapped_zip, Q_power - Q_trip)
            @debug "Removing load power values from ZIP load at $(PSY.get_name(wrapped_zip))"
            return
        end
    elseif isa(ld, PSY.StandardLoad)
        base_power_conversion = PSY.get_base_power(ld) / sys_base_power
        P_power_trip = PSY.get_constant_active_power(ld) * base_power_conversion
        Q_power_trip = PSY.get_constant_reactive_power(ld) * base_power_conversion
        P_current_trip = PSY.get_current_active_power(ld) * base_power_conversion
        Q_current_trip = PSY.get_current_reactive_power(ld) * base_power_conversion
        P_impedance_trip = PSY.get_impedance_active_power(ld) * base_power_conversion
        Q_impedance_trip = PSY.get_impedance_reactive_power(ld) * base_power_conversion
        return (integrator) -> begin
            PSY.set_available!(ld, false)
            wrapped_zip = get_static_loads(integrator.p)[wrapped_device_ix]
            # Update Constant Power
            P_power = get_P_power(wrapped_zip)
            Q_power = get_Q_power(wrapped_zip)
            set_P_power!(wrapped_zip, P_power - P_power_trip)
            set_Q_power!(wrapped_zip, Q_power - Q_power_trip)
            # Update Constant Current
            P_current = get_P_current(wrapped_zip)
            Q_current = get_Q_current(wrapped_zip)
            set_P_current!(wrapped_zip, P_current - P_current_trip)
            set_Q_current!(wrapped_zip, Q_current - Q_current_trip)
            # Update Constant Impedance
            P_impedance = get_P_impedance(wrapped_zip)
            Q_impedance = get_Q_impedance(wrapped_zip)
            set_P_impedance!(wrapped_zip, P_impedance - P_impedance_trip)
            set_Q_impedance!(wrapped_zip, Q_impedance - Q_impedance_trip)
            @debug "Removing load power values from ZIP load at $(PSY.get_name(wrapped_zip))"
            return
        end
    elseif isa(ld, PSY.ExponentialLoad)
        return (integrator) -> begin
            PSY.set_available!(ld, false)
            ld_name = PSY.get_name(ld)
            wrapped_zip = get_static_loads(integrator.p)[wrapped_device_ix]
            exp_names = get_exp_names(wrapped_zip)
            exp_params = get_exp_params(wrapped_zip)
            tuple_ix = exp_names[ld_name]
            deleteat!(exp_params, tuple_ix)
            delete!(exp_names, ld_name)
            @debug "Removing exponential load entry $(ld_name) at wrapper $(PSY.get_name(wrapped_zip))"
            return
        end
    end
    return
end

function _find_zip_load_ix(
    inputs::SimulationInputs,
    device::U,
) where {U <: Union{PSY.PowerLoad, PSY.StandardLoad, PSY.ExponentialLoad}}
    wrapped_devices = get_static_loads(inputs)
    bus_affected = PSY.get_bus(device)
    wrapped_device_ixs =
        findall(x -> PSY.get_name(x) == PSY.get_name(bus_affected), wrapped_devices)
    if isempty(wrapped_device_ixs)
        error(
            "Device $(typeof(device))-$(PSY.get_name(device)) not found in the simulation inputs",
        )
    else
        IS.@assert_op length(wrapped_device_ixs) == 1
    end
    return wrapped_device_ixs[1]
end

"""
    function PerturbState(
        time::Float64,
        index::Int,
        value::Float64,
    )
Allows the user to modify the state `index` by adding `value`. The user should modify dynamic states only, since algebraic state may require to do a reinitialization.
# Arguments:
- `time::Float64` : Defines when the modification of the state will happen. This time should be inside the time span considered in the Simulation.
- `index::Int` : Defines which state index you want to modify
- `value::Float64` : Defines how much the state will increase in value
"""
mutable struct PerturbState <: Perturbation
    time::Float64
    index::Int
    value::Float64
end

function get_affect(::SimulationInputs, ::PSY.System, pert::PerturbState)
    return (integrator) -> begin
        @debug "Modifying state"
        integrator.u[pert.index] += pert.value
        return
    end
end
