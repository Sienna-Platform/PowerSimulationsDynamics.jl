function add_source_to_ref(sys::PSY.System, X_th::Float64)
    for g in PSY.get_components(StaticInjection, sys)
        isa(g, ElectricLoad) && continue
        g.bus.bustype == ACBusTypes.REF &&
            error("A device is already attached to the REF bus")
    end

    slack_bus =
        [b for b in PSY.get_components(ACBus, sys) if b.bustype == ACBusTypes.REF][1]
    inf_source = Source(;
        name = "InfBus", #name
        available = true, #availability
        active_power = 0.0,
        reactive_power = 0.0,
        bus = slack_bus, #bus
        R_th = 0.0,
        X_th = X_th, #Xth
        active_power_limits = (min = -1e6, max = 1e6),
        reactive_power_limits = (min = -1e6, max = 1e6),
    )
    PSY.add_component!(sys, inf_source)
    return
end

function add_source_to_ref(sys::PSY.System)
    for g in PSY.get_components(StaticInjection, sys)
        isa(g, ElectricLoad) && continue
        g.bus.bustype == ACBusTypes.REF &&
            error("A device is already attached to the REF bus")
    end

    slack_bus =
        [b for b in PSY.get_components(ACBus, sys) if b.bustype == ACBusTypes.REF][1]
    inf_source = Source(;
        name = "InfBus", #name
        available = true, #availability
        active_power = 0.0,
        reactive_power = 0.0,
        bus = slack_bus, #bus
        R_th = 0.0,
        X_th = 5e-6, #Xth
        active_power_limits = (min = -1e6, max = 1e6),
        reactive_power_limits = (min = -1e6, max = 1e6),
    )
    PSY.add_component!(sys, inf_source)
    return
end

function get_ybus_fault_threebus_sys(sys)
    fault_branch =
        filter!(x -> get_name(x) != "BUS 1-BUS 3-i_1", collect(get_components(Branch, sys)))
    sorted_buses =
        sort!(collect(get_components(ACBus, threebus_sys)); by = x -> get_number(x))
    Ybus_fault = PNM.Ybus(fault_branch, sorted_buses)[:, :]
    return Ybus_fault
end

function add_degov_to_omib!(omib_sys)
    gen = get_component(ThermalStandard, omib_sys, "generator-102-1")
    dyn_gen = get_component(DynamicGenerator, omib_sys, "generator-102-1")
    new_gov = PSY.DEGOV(;
        T1 = 0.0,
        T2 = 0.0,
        T3 = 0.0,
        K = 18.0,
        T4 = 12.0,
        T5 = 5.0,
        T6 = 0.2,
        Td = 0.5,
        P_ref = 0.0,
    )
    dyn_gen_new = DynamicGenerator(;
        name = get_name(dyn_gen),
        ω_ref = get_ω_ref(dyn_gen),
        machine = get_machine(dyn_gen),
        shaft = get_shaft(dyn_gen),
        avr = get_avr(dyn_gen),
        prime_mover = new_gov,
        pss = get_pss(dyn_gen),
        base_power = get_base_power(dyn_gen),
    )
    remove_component!(omib_sys, dyn_gen)
    add_component!(omib_sys, dyn_gen_new, gen)
end
