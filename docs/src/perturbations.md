# Perturbations

Perturbations are used to alter the system from its steady state operation. If a Simulation is properly initialized, all states will remain fixed in their initial condition if no perturbation is applied to the system.

## List of perturbations

  - [`NetworkSwitch`](@ref): allows to modify directly the admittance matrix, `Ybus`, used in the Simulation.
  - [`BranchTrip`](@ref): completely disconnects a branch from the system.
  - [`BranchImpedanceChange`](@ref): change the impedance of a branch by a user defined multiplier.
  - [`GeneratorTrip`](@ref): allows to disconnect a Dynamic Generation unit from the system.
  - [`ControlReferenceChange`](@ref): allows to change the reference setpoint provided by a generator/inverter.
  - [`LoadChange`](@ref): allows to change the active or reactive power setpoint from a load.
  - [`LoadTrip`](@ref): allows the user to disconnect a load from the system.
  - [`SourceBusVoltageChange`](@ref): allows to change the reference setpoint provided by a voltage source.

## Examples

### Example 1: Circuit Disconnection using [`NetworkSwitch`](@ref)

Consider a two bus system connected via a double circuit line, on which each circuit has parameters, `r = 0.0, x = 0.1, b = 0.0` per unit, then the admittance matrix of the original system is given by:

```julia
yb = [0.0-20.0im 0.0+20.0im
    0.0+20.0im 0.0-20.0im]
```

Triping one circuit can be modeled by doubling the impedance, i.e., dividing by 2 the admittance:

```julia
new_yb = [0.0-10.0im 0.0+10.0im
    0.0+10.0im 0.0-10.0im]
```

To apply a Network Switch, we require to use a sparse matrix, so we can do this by simply:

```julia
using SparseArrays
new_yb = sparse(new_yb)
```

Then, this perturbation ocurring at ``t = 1.0`` seconds can be included as:

```julia
ns1 = NetworkSwitch(1.0, new_yb)
```

### Example 2: Three Phase Fault using [`NetworkSwitch`](@ref)

Another perturbation that can be modeled is a three phase fault at Bus 1 with impedance `r_f = 0.0001, x_f = 0.0` per unit, then the admittance of this new system is:

```julia
new_yb2 = [10000.0-20.0im 0.0+20.0im
    0.0+20.0im 0.0-20.0im]
```

Then, this perturbation ocurring at ``t = 1.0`` seconds can be included as:

```julia
new_yb2 = sparse(new_yb2)
ns2 = NetworkSwitch(1.0, new_yb2)
```

Now, consider that the fault is cleared at ``t = 1.05`` seconds by disconnecting the Circuit 2 of the line. This can be modeled with the single circuit admittance matrix:

```julia
new_yb3 = [0.0-10.0im 0.0+10.0im
    0.0+10.0im 0.0-10.0im]
```

and the perturbation as:

```julia
new_yb3 = sparse(new_yb3)
ns3 = NetworkSwitch(1.05, new_yb3)
```

Then, the entire perturbation for the Simulation can be included in a vector of perturbations as:

```julia
three_fault = [ns2, ns3]
```

that can be passed as a perturbation argument in the Simulation construction.

### Example 3: [`BranchTrip`](@ref)

Consider the following 2 bus system defined by:

```julia
buses = [
    Bus(1, "nodeA", "REF", 0, 1.0, (min = 0.9, max = 1.05), 230, nothing, nothing),
    Bus(2, "nodeB", "PV", 0, 1.0, (min = 0.9, max = 1.05), 230, nothing, nothing),
]

line1 = Line(
    "Circuit1",
    true,
    0.0,
    0.0,
    Arc(; from = buses[1], to = buses[2]),
    0.00,
    0.1,
    (from = 0.0, to = 0.0),
    2.0,
    (min = -0.7, max = 0.7),
)
line2 = Line(
    "Circuit2",
    true,
    0.0,
    0.0,
    Arc(; from = buses[1], to = buses[2]),
    0.0,
    0.1,
    (from = 0.0, to = 0.0),
    2.0,
    (min = -0.7, max = 0.7),
)

sys = System(100.0, buses, [], [], [line1, line2])
```

A Branch Trip of Circuit 2 at time ``t = 1.0`` seconds, can be implemented as:

```julia
b_trip = BranchTrip(1.0, Line, "Circuit2")
```

**Note:** Islanding is currently not supported in `PowerSimulationsDynamics.jl`. If a `BranchTrip` isolates a generation unit, the system may diverge due to the isolated generator.

### Example 4: [`BranchImpedanceChange`](@ref)

Following the same example as before, it is possible to amplify the impedance of a single circuit by 2.0 (that would represent that this Circuit is actually composed by 2 circuits) using the following perturbation:

```julia
b_change = BranchImpedanceChange(1.0, Line, "Circuit2", 2.0)
```

### Example 5: [`GeneratorTrip`](@ref)

Consider that you have a generator at bus 102, named `"generator-102-1"` in your system called `sys`. The constructor to trip it from the system is:

```julia
g = get_component(DynamicGenerator, sys, "generator-102-1")
g_trip = GeneratorTrip(1.0, g)
```

### Example 6: [`ControlReferenceChange`](@ref)

Consider that you have a generator at bus 102, named `"generator-102-1"` in your system called `sys`. The constructor to change is active power reference to `0.5` is:

```julia
g = get_component(DynamicGenerator, sys, "generator-102-1")
crc = ControlReferenceChange(1.0, g, :P_ref, 0.5)
```

### Example 7: [`LoadChange`](@ref)

Consider that you have a load at bus 103, named `"load-103-1"` in your system called `sys`. The constructor to change is active power reference to `0.8` per unit at ``t = 1.0`` seconds is:

```julia
l_device = get_component(ElectricLoad, sys, "load-103-1")
l_change = LoadChange(1.0, l_device, :P_ref, 0.8)
```

### Example 8: [`LoadTrip`](@ref)

Consider that you have a load at bus 103, named `"load-103-1"` in your system called `sys`. The constructor to disconnect such load at ``t = 1.0`` seconds is:

```julia
l_device = get_component(ElectricLoad, sys, "load-103-1")
l_trip = LoadTrip(1.0, l_device)
```

### Example 9: [`SourceBusVoltageChange`](@ref)

Consider that you have a voltage source at bus 101, named `"source-101-1"` in your system called `sys`. The constructor to change is voltage magnitude reference to `1.02` per unit at ``t = 1.0`` seconds is:

```julia
s_device = get_component(Source, sys, "source-101-1")
s_change = SourceBusVoltageChange(1.0, s_device, 1, 1.02)
```
