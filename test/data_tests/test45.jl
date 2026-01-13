import PowerSystems as PSY
using NLsolve

############### Data Network ########################
include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))

############### Data Network ########################
threebus_file_dir = joinpath(dirname(@__FILE__), "ThreeBusPSCAD.raw")
threebus_sys = System(threebus_file_dir; runchecks = false)

function dyn_gen_sauerpai(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator), #static generator
        ω_ref = 1.0, # ω_ref
        machine = machine_sauerpai(), #machine
        shaft = shaft_no_damping(), #shaft
        avr = avr_type1(), #avr
        prime_mover = tg_none(), #tg
        pss = pss_none(),
    ) #pss
end
#=
function inv_darco(static_device)
    return PSY.DynamicInverter(
        get_name(static_device),
        1.0, #ω_ref
        converter_low_power(), #converte
        outer_control(), #outercontrol
        inner_control(), #inner_control
        dc_source_lv(),
        pll(),
        filt(),
    ) #pss
end =#

function inv_darco_droop(static_device)
    return PSY.DynamicInverter(
        get_name(static_device),
        1.0, #ω_ref
        converter_low_power(), #converter
        outer_control_droop(), #outercontrol
        inner_control(), #inner_control
        dc_source_lv(),
        no_pll(),
        filt(),
    ) #pss
end

for l in get_components(PSY.StandardLoad, threebus_sys)
    transform_load_to_constant_impedance(l)
end

for g in get_components(Generator, threebus_sys)
    if get_number(get_bus(g)) == 101
        case_gen = dyn_gen_sauerpai(g)
        add_component!(threebus_sys, case_gen, g)
    elseif get_number(get_bus(g)) == 102
        case_gen = inv_darco_droop(g)
        add_component!(threebus_sys, case_gen, g)
    end
end

for b in get_components(Line, threebus_sys)
    if get_name(b) != "BUS 1-BUS 2-i_1"
        dyn_branch = PowerSystems.DynamicBranch(b)
        add_component!(threebus_sys, dyn_branch)
    end
end
