# PowerSimulationsDynamics.jl — Claude Guide

Platform-wide Sienna conventions (performance, type stability, formatter, environments, code style) live in `.claude/Sienna.md` — read it too. This file is repo-specific and does not restate them.

## Purpose & place in the stack

PowerSimulationsDynamics.jl (PSID) runs **time-domain dynamic simulations** of power systems. It formulates and solves the DAE/ODE system describing generator and inverter dynamic device models, using the SciML stack. This is **not** a JuMP optimization package — there is no optimization-container or constraint-building layer.

PSID consumes `PowerSystems.jl` (PSY) data: dynamic-injector device models (`DynamicGenerator`, `DynamicInverter`, their machines/AVRs/governors/PSS, inverter converters/filters/controls) and the static network. It uses `PowerFlows.jl` (PF) to get the steady-state operating point that seeds initialization, and `PowerNetworkMatrices.jl` (PNM) for network matrices. Verified deps in `Project.toml`: `PowerSystems` (compat 5), `PowerFlows` (^0.16), `PowerNetworkMatrices` (^0.20), `InfrastructureSystems` (3), plus the numerics stack `SciMLBase` (2), `NLsolve` (4), `ForwardDiff` (1), `SparseArrays`, `LinearAlgebra`, `DataStructures`, `FastClosures`, `TimerOutputs`. Note: `SciMLBase` is a dep, but the concrete solvers (`Sundials`, `OrdinaryDiffEq`, `DelayDiffEq`) are supplied by the caller / test env, not by PSID itself.

## Architecture & src/ layout

Include order is authoritative — see `src/PowerSimulationsDynamics.jl`. New constants/types must precede their uses.

- `src/base/` — the simulation machinery.
  - `definitions.jl`, `ports.jl`, `bus_categories.jl` — constants, port mappings, bus classification.
  - `device_wrapper.jl` — `DynamicWrapper{T<:PSY.DynamicInjection}`: wraps each PSY dynamic device with its `ix_range` / `ode_range` (slices into the global state vector), `global_index` (`ImmutableDict{Symbol,Int}` mapping state names → global indices), inner-component refs, and connection data. `branch_wrapper.jl` — analogous wrapper for dynamic branches.
  - `simulation_model.jl` — model-variation singleton types: `ResidualModel`, `MassMatrixModel` (both `<: SimulationModel`), and `NoDelays` / `HasDelays` (`<: DelayModel`).
  - `simulation_inputs.jl` — `SimulationInputs`: assembled state vector, mass matrix, wrappers, index ranges, `get_setpoints`. `system_model.jl` — `SystemModel{T<:SimulationModel, D<:DelayModel, C<:Cache}`, the callable passed to the solver.
  - `perturbations.jl` — perturbation/event types.
  - `mass_matrix.jl` — builds the singular DAE mass matrix. `jacobian.jl` — `JacobianFunctionWrapper` and `get_jacobian`, built with `ForwardDiff`. `caches.jl` — dual-number-aware caches for ForwardDiff.
  - `simulation.jl` — `Simulation` (a `mutable struct Simulation{T<:SimulationModel}`), `execute!`, `read_results`, `get_setpoints`.
  - `nlsolve_wrapper.jl`, `simulation_initialization.jl` — steady-state solve (see below). `small_signal.jl` — `small_signal_analysis`. `simulation_results.jl` — `SimulationResults`, `get_state_series`. `model_validation.jl`.
- `src/initialization/` — `init_device.jl` plus per-component routines under `generator_components/` (machine, shaft, avr, tg, pss) and `inverter_components/` (filter, DCside, converter, frequency_estimator, inner, outer). Each solves for that component's initial states given the power-flow operating point.
- `src/models/` — the device dynamic-model equations. Common: `branch.jl`, `device.jl`, `network_model.jl`, `dynline_model.jl`, `ref_transformations.jl`, `common_controls.jl`. `generator_models/` (machine, pss, avr, tg, shaft) and `inverter_models/` (DCside, filter, frequency_estimator, outer_control, inner_control, converter, output_current_limiter). `load_models.jl`, `source_models.jl`, `saturation_models.jl`, and `system.jl` which holds the top-level residual/MM callables.
- `src/post_processing/` — `get_*_series` accessors, branch-flow series, source/load/generator post-processing, `read_initial_conditions`, `show_states_initial_value`.
- `src/utils/` — `psy_utils.jl`, `pf_utils.jl`, `immutable_dicts.jl`, `print.jl`, `kwargs_check.jl`, `logging.jl`.

## Key public API / entry points

Verified exports in `src/PowerSimulationsDynamics.jl`:

- Build & run: `Simulation`, `Simulation!`, `execute!`, `SimulationResults`, `read_results`.
- Model selection (first positional arg to `Simulation`): `ResidualModel`, `MassMatrixModel`.
- Frequency reference: `ReferenceBus`, `ConstantFrequency`.
- Perturbations: `NetworkSwitch`, `ControlReferenceChange`, `BranchTrip`, `BranchImpedanceChange`, `SourceBusVoltageChange`, `GeneratorTrip`, `LoadTrip`, `LoadChange`, `PerturbState`. (`BusTrip` is defined but commented out of exports.)
- Analysis & results: `small_signal_analysis`, `summary_participation_factors`, `summary_eigenvalues`, `get_jacobian`, `get_state_series`, `get_voltage_magnitude_series`, `get_voltage_angle_series`, `get_real_current_series`, `get_activepower_series`, `get_reactivepower_series`, `get_field_current_series`, `get_field_voltage_series`, `get_pss_output_series`, `get_mechanical_torque_series`, `get_frequency_series`, branch-flow series, `get_setpoints`, `get_solution`, `read_initial_conditions`, `show_states_initial_value`.
- Load transforms: `transform_load_to_constant_impedance` / `_current` / `_power`. Validation: `is_valid`.

## DAE formulation, residual & Jacobian

The system is a singular **mass-matrix DAE**: algebraic equations (network) have zero mass-matrix rows, differential equations (device states) have unit rows. Two equivalent formulations are chosen at `Simulation` construction:

- `MassMatrixModel` — `M du/dt = f(u,p,t)`, handed to an ODE/DAE integrator (e.g. `Rodas`, `IDA`). The callable lives at `src/models/system.jl` as `function (m::SystemModel{MassMatrixModel, NoDelays, C})(...)` (and a `HasDelays` variant for models with delays).
- `ResidualModel` — `g(du,u,p,t) = M du - f(u,p,t)`, a residual handed to a DAE solver. Callable: `function (m::SystemModel{ResidualModel, NoDelays, C})(...)`.

The Jacobian is computed with `ForwardDiff` via `JacobianFunctionWrapper` / `get_jacobian` (`src/base/jacobian.jl`), using a cached `JacobianConfig`. This is why model equation code must stay ForwardDiff-compatible (see gotchas).

## PSID-specific conventions & gotchas

- **Dynamic-injector wrapping.** Devices are never touched raw inside the residual — they are wrapped in `DynamicWrapper` (and branches in the branch wrapper) during `SimulationInputs` assembly. Per-device `ix_range` / `ode_range` are precomputed slices into the global state vector; model equations write into those ranges. Look up a state by name through the wrapper's `global_index`, not by hard-coded offsets.
- **Initialization must converge.** Before integration, PSID solves a power flow (via PowerFlows) and then a nonlinear system (NLsolve) for the steady-state device initial conditions (`src/base/simulation_initialization.jl`). Failures surface as hard errors — `@error("PowerFlow failed to solve")`, `@error("Invalid initial condition values ...")`, `error("Failed to find operating point")` — not silent fallbacks. A run that fails to initialize cannot proceed; do not paper over a non-converging init.
- **ForwardDiff-compatibility.** Model functions are differentiated with dual numbers, so do not hard-type intermediate buffers to `Float64`, do not branch on numeric type, and route any scratch storage through the dual-aware `caches.jl`. Breaking this silently corrupts the Jacobian rather than erroring.
- **No `isa`/`<:` branching; dispatch on wrapper/model type parameters instead** (platform rule — reinforced here because model code is dispatch-heavy across `SystemModel{T,D,C}`). Mirror the existing residual/MM structure when adding a parallel formulation so the two stay mechanically comparable.
- **Frequency reference** (`ReferenceBus` vs `ConstantFrequency`) changes the state set and the network equations — pick it consciously when adding device models.

## Cross-package coupling

- **PowerSystems (PSY):** all device/dynamic-injector structs, accessors, and the static system. PSID dispatches its model and init routines on PSY dynamic-component types — new device support usually means a PSY model type plus a PSID model-equation method and an init method.
- **PowerFlows (PF):** supplies the operating point that seeds initialization.
- **PowerNetworkMatrices (PNM):** network matrices for the algebraic network model.
- **InfrastructureSystems (IS):** logging, asserts (`IS.@assert_op`), base utilities.
- Downstream: when changing public series accessors or `SimulationResults`, consider notebooks/tutorials and any analysis tooling that reads them.

## Running tests, docs, formatter (verified commands)

Formatter (self-activates its own environment):

```sh
julia --project=scripts/formatter -e 'include("scripts/formatter/formatter_code.jl")'
```

Tests use a custom `@includetests` runner (a TestSetExtensions-style macro copied into `test/runtests.jl`) driven by `ARGS`; test files live in `test/` as `test_case*.jl` / `test_base.jl`. The full suite also runs Aqua checks. Always use `--project=test`.

```sh
julia --project=test -e 'using Pkg; Pkg.instantiate()'   # first time / when deps change
julia --project=test test/runtests.jl                    # full suite
julia --project=test test/runtests.jl test_case_OMIB     # a single file, by name without ".jl"
```

Note: `test/Project.toml` provides solver deps (`Sundials`, `OrdinaryDiffEq`, `DelayDiffEq`) and `PowerSystemCaseBuilder` (PSB) — heed the usual PSB shared-state caching gotchas. `test_case49_csvgn1.jl` is currently disabled in the runner.

Docs:

```sh
julia --project=docs -e 'using Pkg; Pkg.develop(PackageSpec(path=pwd())); Pkg.instantiate()'  # first time
julia --project=docs docs/make.jl                                                             # must finish without errors
```
