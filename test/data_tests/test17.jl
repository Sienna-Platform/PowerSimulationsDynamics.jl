import PowerSystems as PSY
using NLsolve

include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))

############### Data Network ########################

raw_file_dir = joinpath(dirname(@__FILE__), "../benchmarks/psse/GENROU/ThreeBusMulti.raw")
sys = System(raw_file_dir);

#Replace Gen101 by Source
remove_component!(ThermalStandard, sys, "generator-101-1");
add_source_to_ref(sys)

function dyn_gen_genrou(generator)
    return PSY.DynamicGenerator(;
        name = get_name(generator),
        ω_ref = 1.0, #ω_ref
        machine = machine_genrou(), #machine
        shaft = shaft_genrou(), #shaft
        avr = avr_type1(), #avr
        prime_mover = tg_none(), #tg
        pss = pss_none(),
    ) #pss
end

for l in get_components(PSY.StandardLoad, sys)
    transform_load_to_constant_impedance(l)
end

#Add GENROU to System
g = get_component(ThermalStandard, sys, "generator-102-1")
dyn_gen = dyn_gen_genrou(g)
add_component!(sys, dyn_gen, g);
