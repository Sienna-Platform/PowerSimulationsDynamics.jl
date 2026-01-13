import PowerSystems as PSY
using NLsolve

############### Data Network ########################
include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))

############### Data Network ########################
threebus_file_dir = joinpath(dirname(@__FILE__), "ThreeBusMulti.raw")
threebus_sys = System(threebus_file_dir; runchecks = false)

function inv_case78(static_device)
    return DynamicInverter(;
        name = get_name(static_device),
        ω_ref = 1.0, # ω_ref,
        converter = converter_high_power(), #converter
        outer_control = outer_control(), #outer control
        inner_control = inner_control(), #inner control voltage source
        dc_source = dc_source_lv(), #dc source
        freq_estimator = pll(), #pll
        filter = filt(), #filter
    )
end

function dyn_gen_multi_tg(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator),
        ω_ref = 1.0, # ω_ref,
        machine = machine_classic(), #machine
        shaft = shaft_damping(), #shaft
        avr = avr_none(), #avr
        prime_mover = tg_type2(), #tg
        pss = pss_none(),
    )
end

# Add dynamic generators to the system (each gen is linked through a static one)
for g in get_components(Generator, threebus_sys)
    if get_number(get_bus(g)) == 101
        case_gen = inv_case78(g)
        add_component!(threebus_sys, case_gen, g)
    elseif get_number(get_bus(g)) == 102
        case_gen = dyn_gen_multi_tg(g)
        add_component!(threebus_sys, case_gen, g)
    end
end

for l in get_components(PSY.StandardLoad, threebus_sys)
    transform_load_to_constant_impedance(l)
end

#Create Ybus_Fault
fault_branches = deepcopy(collect(get_components(Branch, threebus_sys)))
for br in fault_branches
    if get_name(br) == "BUS 1-BUS 3-i_1"
        br.r = 4 * br.r
        br.x = 4 * br.x
    end
end
Ybus_fault = PNM.Ybus(fault_branches, collect(get_components(ACBus, threebus_sys)))[:, :];
