using PowerSystems
using NLsolve
const PSY = PowerSystems

############### Data Network ########################
include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))

############### Data Network ########################
threebus_file_dir = joinpath(dirname(@__FILE__), "ThreeBusNetwork.raw")
threebus_sys = System(threebus_file_dir; runchecks = false)
add_source_to_ref(threebus_sys)

function dyn_gen_avr_type2(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator), #static generator
        ω_ref = 1.0, # ω_ref
        machine = machine_oneDoneQ(), #machine
        shaft = shaft_no_damping(), #shaft
        avr = avr_type2(), #avr
        prime_mover = tg_type1(), #tg
        pss = pss_none(),
    ) #pss
end

function dyn_gen_simple_avr(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator), #static generator
        ω_ref = 1.0, # ω_ref
        machine = machine_oneDoneQ(), #machine
        shaft = shaft_no_damping(), #shaft
        avr = avr_propr(), #avr
        prime_mover = tg_none(), #tg
        pss = pss_none(),
    ) #pss
end

for l in get_components(PSY.StandardLoad, threebus_sys)
    transform_load_to_constant_impedance(l)
end

# Add dynamic generators to the system (each gen is linked through a static one)
for g in get_components(Generator, threebus_sys)
    if get_number(get_bus(g)) == 102
        case_gen = dyn_gen_avr_type2(g)
        add_component!(threebus_sys, case_gen, g)
    elseif get_number(get_bus(g)) == 103
        case_gen = dyn_gen_simple_avr(g)
        add_component!(threebus_sys, case_gen, g)
    end
end

#Compute Y_bus after fault
fault_branches = deepcopy(collect(get_components(Branch, threebus_sys))[2:end])
Ybus_fault = PSID.build_ybus_from_branches(
    fault_branches,
    collect(get_components(ACBus, threebus_sys));
    base_power = PSY.get_base_power(threebus_sys),
)[:, :]
