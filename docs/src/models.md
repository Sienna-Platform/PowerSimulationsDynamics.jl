
# Models

## Simulation Models

PowerSimulations dynamics supports two formulations for the simulation model and define different methods for each simulation model. You can pass `ResidualModel` or `MassMatrixModel` to a call to Simulation to define the preferred formulation.

In this way, we provide a common set of development requirements for contributors of new models that maintains the same flexibility in choosing the solving algorithm.

- *MassMatrixModel*: Defines models that can be solved using [Mass-Matrix Solvers](https://diffeq.sciml.ai/stable/solvers/dae_solve/#OrdinaryDiffEq.jl-(Mass-Matrix)). The model is formulated as follows:

```math
\begin{align}
M\frac{dx(t)}{dt} = f(x(t))
\end{align}
```

At this stage we have not conducted extensive tests with all the solvers in [DifferentialEquations](https://diffeq.sciml.ai/) most of our tests use `Rodas5()`.


- *ResidualModel*: Define models that can be solved using [Implicit ODE solvers](https://diffeq.sciml.ai/stable/solvers/dae_solve/#OrdinaryDiffEq.jl-(Implicit-ODE)) and also the solver IDA from [Sundials](https://diffeq.sciml.ai/stable/solvers/dae_solve/#Sundials.jl). The model is formulated to solved the following problem:

```math
\begin{align}
r(t) = \frac{dx(t)}{dt} - f(x(t))
\end{align}
```

At this stage we have not conducted extensive tests with all the solvers in [DifferentialEquations](https://diffeq.sciml.ai/) if you are solving a larger system use `IDA()`.

### The dynamic system model in PowerSimulationsDynamics

In order to support both formulations, the default implementation of the ResidualModel solves the following problem:

```math
\begin{align}
r(t) = M\frac{dx(t)}{dt} - f(x(t))
\end{align}
```

## Solution approaches

`PowerSimulationsDynamics.jl` construct the entire function that is passed to [DifferentialEquations](https://diffeq.sciml.ai/) to solve it using different solvers. This is called the Simultaneous-solution approach to numerically integrate over time. It gives the user the flexibility to model devices with a combination of differential and algebraic states. In addition, it gives the flexibility to model the network using an algebraic or differential model.

On the other hand, industrial tools such as PSS/E uses a Partitioned-solution approach, on which the network must be modeled using an algebraic approach, and the differential equations and algebraic equations are solved sequentially. This method is usually faster if the heuristics for convergence when solving sequentially are properly tuned. However, boundary techniques must be considered when the connection with the devices and network is not converging when applying the partitioned-solution approach.

The difference in solution methods can complicate the validation and comparison between software tools. In addition it can affect the computational properties due to reliance of heuristics. These solution aspects are important to consider when using different tools for simulating power systems dynamics.

For more details, check Brian Stott paper ["Power system dynamic response calculations"](https://ieeexplore.ieee.org/document/1455502).

## Generator Models

Here we discuss the structure and models used to model generators in `PowerSimulationsDynamics.jl`. See [`PowerSystems.jl` dynamic devices](https://sienna-platform.github.io/PowerSystems.jl/stable/modeler_guide/example_dynamic_data/)
for details.

Each generator is a data structure composed of the following components defined in `PowerSystems.jl`:

- [`Machine`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_Machine/#Machine): That defines the stator electro-magnetic dynamics.
- [`Shaft`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_Shaft/#Shaft): That describes the rotor electro-mechanical dynamics.
- [`Automatic Voltage Regulator`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_AVR/#AVR): Electromotive dynamics to model an AVR controller.
- [`Power System Stabilizer`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_PSS/#PSS): Control dynamics to define an stabilization signal for the AVR.
- [`Prime Mover and Turbine Governor`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_TurbineGov/#TurbineGov): Thermo-mechanical dynamics and associated controllers.

The implementation of Synchronous generators as components uses the following structure to
share values across components.

```@raw html
<img src="https://raw.githubusercontent.com/Sienna-Platform/PowerSystems.jl/main/docs/src/assets/gen_metamodel.png" width="75%">
```

## Inverter Models

Here we discuss the structure and models used to model inverters in `PowerSimulationsDynamics.jl`. See [`PowerSystems.jl` dynamic devices](https://sienna-platform.github.io/PowerSystems.jl/stable/modeler_guide/example_dynamic_data/)
for details. One of the key contributions in this software package is a separation of the
components in a way that resembles current practices for synchronoues machine modeling.

- [`DC Source`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_DCSource/#DCSource): Defines the dynamics of the DC side of the converter.
- [`Frequency Estimator`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_FrequencyEstimator/#FrequencyEstimator): That describes how the frequency of the grid can be estimated using the grid voltages. Typically a phase-locked loop (PLL).
- [`Outer Loop Control`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/outer_control/#OuterControl): That describes the active and reactive power control dynamics.
- [`Inner Loop Control`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_InnerControl/#InnerControl): That can describe virtual impedance, voltage control and current control dynamics.
- [`Converter`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_Converter/#Converter): That describes the dynamics of the pulse width modulation (PWM) or space vector modulation (SVM).
- [`Filter`](https://sienna-platform.github.io/PowerSystems.jl/stable/model_library/generated_Filter/): Used to connect the converter output to the grid.

The following figure summarizes the components of a inverter and which variables they share:

```@raw html
<img src="https://raw.githubusercontent.com/Sienna-Platform/PowerSystems.jl/main/docs/src/assets/inv_metamodel.png" width="75%">
```

Contrary to the generator, there are many control structures that can be used to model
inverter controllers (e.g. grid-following, grid feeding or virtual synchronous machine).
For this purpose, more variables are shared among the components in order to cover all
these posibilities.

## Reference

For models, check the library in [PowerSystems.jl](https://sienna-platform.github.io/PowerSystems.jl/stable/)
