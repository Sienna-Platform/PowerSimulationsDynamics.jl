mutable struct AutoAbstolAffect{T}
    current_max::T
end

function (p::AutoAbstolAffect)(integrator)
    p.current_max = max(p.current_max, maximum(integrator.u))
    integrator.opts.abstol = p.current_max * integrator.opts.reltol
    SciMLBase.derivative_discontinuity!(integrator, false)
end

function AutoAbstol(save = true, init_current_max = 1e-9)
    affect! = AutoAbstolAffect(init_current_max)
    condtion = (u, t, integrator) -> true
    save_positions = (save, false)
    return SciMLBase.DiscreteCallback(condtion, affect!; save_positions = save_positions)
end
