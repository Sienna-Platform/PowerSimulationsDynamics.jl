# PowerSimulationsDynamics.jl

| **Documentation** | **Build Status** |
|:---:|:---:|
| [![][docs-sienna-img]][docs-sienna-url] [![][docs-stable-img]][docs-stable-url] [![][docs-dev-img]][docs-dev-url] | [![Main - CI](https://github.com/Sienna-Platform/PowerSimulationsDynamics.jl/workflows/Main%20-%20CI/badge.svg?branch=main)](https://github.com/Sienna-Platform/PowerSimulationsDynamics.jl/actions/workflows/main-tests.yml) [![codecov](https://codecov.io/gh/Sienna-Platform/PowerSimulationsDynamics.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/Sienna-Platform/PowerSimulationsDynamics.jl) [<img src="https://img.shields.io/badge/slack-@Sienna/PSID-sienna.svg?logo=slack">](https://join.slack.com/t/core-sienna/shared_invite/zt-glam9vdu-o8A9TwZTZqqNTKHa7q3BpQ) [![DOI](https://zenodo.org/badge/280242020.svg)](https://zenodo.org/badge/latestdoi/280242020) [![PowerSimulationsDynamics.jl Downloads](https://img.shields.io/badge/dynamic/json?url=http%3A%2F%2Fjuliapkgstats.com%2Fapi%2Fv1%2Ftotal_downloads%2FPowerSimulationsDynamics&query=total_requests&label=Downloads)](http://juliapkgstats.com/pkg/PowerSimulationsDynamics) |

[docs-sienna-img]: https://img.shields.io/badge/Central_Sienna_docs-blue.svg
[docs-sienna-url]: https://sienna-platform.github.io/Sienna/SiennaDocs/docs/build/index/
[docs-stable-img]: https://img.shields.io/badge/docs-stable-blue.svg
[docs-stable-url]: https://sienna-platform.github.io/PowerSimulationsDynamics.jl/stable/
[docs-dev-img]: https://img.shields.io/badge/docs-dev-blue.svg
[docs-dev-url]: https://sienna-platform.github.io/PowerSimulationsDynamics.jl/dev/

`PowerSimulationsDynamics.jl` is a Julia package for power system modeling and simulation of Power Systems dynamics. The objectives of the package are:

- Provide a flexible modeling framework that can accommodate different device models according to modeling needs.

- Streamline the construction of large scale differential equations problems to avoid repetition of work when adding/modifying model details.

- Exploit Julia's capabilities to improve computational performance of large scale power system dynamic simulations.

- Provide State-of-Art modeling to assess Low-Inertia Power Systems.

Check the [Project Section](https://github.com/Sienna-Platform/PowerSimulationsDynamics.jl/projects/1) to see the pipelines of new models to be added.

## Installation

```julia
julia> ]
(v1.9) pkg> add PowerSystems
(v1.9) pkg> add PowerSimulationsDynamics
```

## Usage

`PowerSimulationsDynamics.jl` uses [PowerSystems.jl](https://github.com/Sienna-Platform/PowerSystems.jl) to handle the data used in the simulations.

```julia
using PowerSimulationsDynamics
using PowerSystems
```

For information on using the package, see the [stable documentation](https://sienna-platform.github.io/PowerSimulationsDynamics.jl/stable/). Use the [in-development documentation](https://sienna-platform.github.io/PowerSimulationsDynamics.jl/dev/) for the version of the documentation which contains the unreleased features.

## Citing PowerSimulationsDynamics.jl

[Paper describing `PowerSimulationsDynamics.jl`](https://arxiv.org/abs/2308.02921)

```bibtex
@misc{lara2023powersimulationsdynamicsjl,
      title={PowerSimulationsDynamics.jl -- An Open Source Modeling Package for Modern Power Systems with Inverter-Based Resources}, 
      author={Jose Daniel Lara and Rodrigo Henriquez-Auba and Matthew Bossart and Duncan S. Callaway and Clayton Barrows},
      year={2023},
      eprint={2308.02921},
      archivePrefix={arXiv},
      primaryClass={eess.SY},
      doi={https://doi.org/10.48550/arXiv.2308.02921}
}
```


## References

The background work on `PowerSimulationsDynamics.jl` is explained in [Revisiting Power Systems Time-domain Simulation Methods and Models](https://ieeexplore.ieee.org/document/10213230)

```bibtex
@ARTICLE{revLaraDynamics,
  author={Lara, Jose Daniel and Henriquez-Auba, Rodrigo and Ramasubramanian, Deepak and Dhople, Sairaj and Callaway, Duncan S. and Sanders, Seth},
  journal={IEEE Transactions on Power Systems}, 
  title={Revisiting Power Systems Time-domain Simulation Methods and Models}, 
  year={2023},
  volume={},
  number={},
  pages={1-16},
  doi={10.1109/TPWRS.2023.3303291}}
```

## Development

Contributions to the development and enahancement of PowerSimulationsDynamics.jl is welcome. Please see [CONTRIBUTING.md](https://github.com/Sienna-Platform/PowerSimulationsDynamics.jl/blob/main/CONTRIBUTING.md) for code contribution guidelines.

## License

PowerSimulationsDynamics.jl is released under a BSD [license](https://github.com/Sienna-Platform/PowerSimulationsDynamics.jl/blob/main/LICENSE).
PowerSimulationsDynamics.jl has been developed as part of the Scalable Integrated Infrastructure Planning (SIIP) initiative at the U.S. Department of Energy's National Laboratory of the Rockies ([NLR](https://www.nlr.gov/)), formerly known as NREL.
