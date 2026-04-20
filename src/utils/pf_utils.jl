function _transform_load_data_to_constant_power!(data::PF.PowerFlowData)
    total_active_power =
        data.bus_active_power_withdrawals .+
        data.bus_active_power_constant_current_withdrawals .+
        data.bus_active_power_constant_impedance_withdrawals
    total_reactive_power =
        data.bus_reactive_power_withdrawals .+
        data.bus_reactive_power_constant_current_withdrawals .+
        data.bus_reactive_power_constant_impedance_withdrawals
    data.bus_active_power_withdrawals .= total_active_power
    data.bus_reactive_power_withdrawals .= total_reactive_power
    data.bus_active_power_constant_current_withdrawals .= 0.0
    data.bus_active_power_constant_impedance_withdrawals .= 0.0
    data.bus_reactive_power_constant_current_withdrawals .= 0.0
    data.bus_reactive_power_constant_impedance_withdrawals .= 0.0
    return
end

function solve_and_save_powerflow!(pf, system, use_constant_power_loads_in_pf; kwargs...)
    if !use_constant_power_loads_in_pf
        converged = PF.solve_and_store_power_flow!(pf, system)
    else
        converged = false
        PSY.with_units_base(system, PSY.UnitSystem.SYSTEM_BASE) do
            data = PF.PowerFlowData(pf, system)
            _transform_load_data_to_constant_power!(data)

            converged = PF.solve_power_flow!(data)

            if converged
                PF.write_power_flow_solution!(
                    system,
                    pf,
                    data,
                    get(kwargs, :maxIterations, PF.DEFAULT_NR_MAX_ITER),
                )
                @info(
                    "PowerFlow solve converged, the results have been stored in the system"
                )
            else
                @error("The power flow solver returned convergence = $converged")
            end
        end
    end
    return converged
end

function get_total_q(l::PSY.StandardLoad)
    return PSY.get_constant_reactive_power(l) + PSY.get_current_reactive_power(l) +
           PSY.get_impedance_reactive_power(l)
end

function get_total_p(l::PSY.StandardLoad)
    return PSY.get_constant_active_power(l) + PSY.get_current_active_power(l) +
           PSY.get_impedance_active_power(l)
end
