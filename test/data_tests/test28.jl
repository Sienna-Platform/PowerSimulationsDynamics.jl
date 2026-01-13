import PowerSystems as PSY
using NLsolve

include(joinpath(dirname(@__FILE__), "dynamic_test_data.jl"))

############### Data Network ########################
sys_dir = joinpath(dirname(@__FILE__), "OMIB.raw")
sys = System(sys_dir; runchecks = false)
add_source_to_ref(sys)
############### Data Dynamic devices ########################

function pvs_simple(source)
    return PeriodicVariableSource(;
        name = get_name(source),
        R_th = get_R_th(source),
        X_th = get_X_th(source),
        internal_voltage_bias = 1.0,
        internal_voltage_frequencies = [2 * pi],
        internal_voltage_coefficients = [(1.0, 0.0)],
        internal_angle_bias = 0.0,
        internal_angle_frequencies = [2 * pi],
        internal_angle_coefficients = [(0.0, 1.0)],
    )
end

#Attach dynamic generator
gen = [g for g in get_components(Generator, sys)][1]
case_gen = dyn_gen_second_order(gen)
add_component!(sys, case_gen, gen)

#Attach periodic variable source
source = [s for s in get_components(Source, sys)][1]
pvs = pvs_simple(source)
add_component!(sys, pvs, source)

for l in get_components(PSY.StandardLoad, sys)
    transform_load_to_constant_impedance(l)
end
