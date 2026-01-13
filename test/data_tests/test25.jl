import PowerSystems as PSY
using NLsolve

include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))

############### Data Network ########################
sys_dir = joinpath(dirname(@__FILE__), "ThreeBusMultiLoad.raw")
sys = System(sys_dir; runchecks = false)

############### Data Dynamic devices ########################
function dyn_gen_marconato(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator),
        ω_ref = 1.0, # ω_ref,
        machine = machine_marconato(), #machine
        shaft = shaft_no_damping(), #shaft
        avr = AVRSimple(1.0), #avr
        prime_mover = tg_none(), #tg
        pss = pss_none(),
    )
end

function dyn_gen_marconato_tg(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator),
        ω_ref = 1.0, # ω_ref,
        machine = machine_marconato(), #machine
        shaft = shaft_no_damping(), #shaft
        avr = AVRSimple(1.0), #avr
        prime_mover = tg_type2(), #tg
        pss = pss_none(),
    )
end

# Add dynamic generators to the system (each gen is linked through a static one)
for g in get_components(Generator, sys)
    if get_number(get_bus(g)) == 101
        case_gen = dyn_gen_marconato_tg(g)
        add_component!(sys, case_gen, g)
    elseif get_number(get_bus(g)) == 102
        case_gen = dyn_gen_marconato(g)
        add_component!(sys, case_gen, g)
    end
end

# Transform all lines into dynamic lines
# Not recommended approach since it will remove static lines while going through the loop
for line in get_components(Line, sys)
    dyn_line = DynamicBranch(line)
    add_component!(sys, dyn_line)
end

for l in get_components(PSY.StandardLoad, sys)
    transform_load_to_constant_impedance(l)
end
