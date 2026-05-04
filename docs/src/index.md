# PowerSimulationsDynamics.jl

```@meta
CurrentModule = PowerSimulationsDynamics
```

## Overview

`PowerSimulationsDynamics.jl` is a [`Julia`](http://www.julialang.org) package for
doing Power Systems Dynamic Modeling with Low Inertia Energy Sources.

The synchronous machine components supported here are based on commercial models and
the academic components are derived from [Power System Modelling and Scripting](https://www.springer.com/gp/book/9783642136689).

Inverter models support both commercial models, such as REPC, REEC and REGC type of models; and academic models obtained
from grid-following and grid-forming literature such as in ["A Virtual Synchronous Machine implementation for
distributed control of power converters in SmartGrids"](https://www.sciencedirect.com/science/article/pii/S0378779615000024)

The background work on `PowerSimulationsDynamics.jl` is explained in [Revisiting Power Systems Time-domain Simulation Methods and Models](https://arxiv.org/pdf/2301.10043.pdf)

```bibtex
@article{lara2023revisiting,
title={Revisiting Power Systems Time-domain Simulation Methods and Models},
author={Lara, Jose Daniel and Henriquez-Auba, Rodrigo and Ramasubramanian, Deepak and Dhople, Sairaj and Callaway, Duncan S and Sanders, Seth},
journal={arXiv preprint arXiv:2301.10043},
year={2023}
}
```

## Structure

The following figure shows the interactions between `PowerSimulationsDynamics.jl`, `PowerSystems.jl`, `ForwardDiff.jl`, `DiffEqBase.jl` and the integrators.
The architecture of `PowerSimulationsDynamics.jl`  is such that the power system models are
all self-contained and return the model function evaluations. The Jacobian is calculated
using automatic differentiation through `ForwardDiff.jl`, that is used for both numerical
integration and small signal analysis. Considering that the resulting models are differential-algebraic
equations (DAE), the implementation focuses on the use of implicit solvers, in particular
BDF and Rosenbrock methods.

```@raw html
<img src="./assets/SoftwareLoop.jpg" width="65%"/>
``` ⠀

## About Sienna

`PowerSimulationsDynamics.jl` is part of the National Laboratory of the Rockies (formerly known as NREL)'s
[Sienna ecosystem](https://sienna-platform.github.io/Sienna/), an open source framework for
power system modeling, simulation, and optimization. The Sienna ecosystem can be
[found on Github](https://github.com/Sienna-Platform/). It contains three applications:

  - [Sienna\Data](https://sienna-platform.github.io/Sienna/pages/applications/sienna_data.html) enables
    efficient data input, analysis, and transformation
  - [Sienna\Ops](https://sienna-platform.github.io/Sienna/pages/applications/sienna_ops.html) enables
    enables system scheduling simulations by formulating and solving optimization problems
  - [Sienna\Dyn](https://sienna-platform.github.io/Sienna/pages/applications/sienna_dyn.html) enables
    system transient analysis including small signal stability and full system dynamic
    simulations

Each application uses multiple packages in the [`Julia`](http://www.julialang.org)
programming language.

## Installation and Quick Links

  - [Sienna installation page](https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/how-to/install/):
    Instructions to install `PowerSimulationsDynamics.jl` and other Sienna\Dyn packages
  - [Sienna Documentation Hub](https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/index.html):
    Links to other Sienna packages' documentation
