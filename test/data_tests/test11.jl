import PowerSystems as PSY

############### Data Network ########################
include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))
############### Data Network ########################
threebus_file_dir = joinpath(dirname(@__FILE__), "ThreeBusInverter.raw")
threebus_sys = System(threebus_file_dir; runchecks = false)
add_source_to_ref(threebus_sys)
dyn_branch = DynamicBranch(get_component(Branch, threebus_sys, "BUS 2-BUS 3-i_1"))
add_component!(threebus_sys, dyn_branch)

############### Data devices ########################

function dyn_gen_second_order(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator),
        ω_ref = 1.0, # ω_ref,
        machine = machine_oneDoneQ(), #machine
        shaft = shaft_no_damping(), #shaft
        avr = avr_type1(), #avr
        prime_mover = tg_none(), #tg
        pss = pss_none(), #pss
    )
end

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

for g in get_components(Generator, threebus_sys)
    if get_number(get_bus(g)) == 102
        case_gen = dyn_gen_second_order(g)
        add_component!(threebus_sys, case_gen, g)
    elseif get_number(get_bus(g)) == 103
        case_inv = inv_case78(g)
        add_component!(threebus_sys, case_inv, g)
    end
end

#Create Ybus_Fault
sys3 = System(threebus_file_dir; runchecks = false)
add_source_to_ref(sys3)
remove_component!(Line, sys3, "BUS 2-BUS 3-i_1")
#Create Ybus_Fault
fault_branches2 = get_components(Line, sys3)
for br in fault_branches2
    if get_name(br) == "BUS 1-BUS 2-i_1"
        br.r = 3 * br.r
        br.x = 3 * br.x
        b_new = (from = br.b.from / 3, to = br.b.to / 3)
        br.b = b_new
    end
end
Ybus_fault = PNM.Ybus(sys3).data;

for l in get_components(PSY.StandardLoad, threebus_sys)
    transform_load_to_constant_impedance(l)
end
