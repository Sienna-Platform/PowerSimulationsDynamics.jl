# Delays

PowerSimulationsDynamics supports models with constant delays in a mass matrix formulation:

```math
\begin{align}
M\frac{dx(t)}{dt} = f(x(t), x(t-\tau_1), ... , x(t-\tau_N))   
\end{align}
```

For more information on solving such models, refer to the documentation for [DelayDiffEq.jl](https://github.com/SciML/DelayDiffEq.jl) package.

The following models include time delays:

  - `DEGOV`

There is currently limited support for including models with time delays. The following limitations apply:

  - Only constant delays are supported (state dependent delays are not).
  - System models with delays must use `MassMatrixModel`  formulation (`ResidualModel` is not currently compatible).
  - System models with delays are not compatible with small signal analysis tools.
  - The system formulation with delays is not compatible with automatic differentiation for calculating the gradient with respect to time. The setting `autodiff=false` should be set when passing the solver (e.g. `MethodofSteps(Rodas5(autodiff=false))`).
