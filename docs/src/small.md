# Small Signal Analysis

Here we discuss the method used to do a small signal analysis on the DAE system defined in
`PowerSimulationsDynamics.jl`. The package defines algebraic variables for both real and
imaginary voltages on all buses (except if they have a dynamic line connected, on which
the voltage of those buses are treated as differential variables). In addition, each dynamic
device can add differential variables (or states) that are concatenated to construct the
system of differential algebraic equations.

**Note:** The validation of small signal results is still work in progress due to the differences
in the way that different software packages perform the calculations.

## Automatic Differentiation

Once an equilibrium point is found, the complete jacobian of the non-linear system can be
obtained using [automatic differentiation](https://en.wikipedia.org/wiki/Automatic_differentiation)
in [Julia](https://www.juliadiff.org). In particular, the package `ForwardDiff.jl` is used to
obtain the jacobian of the non-linear algebraic system of equations. `PowerSimulationsDynamics.jl`
handles the resulting jacobian and reports the reduced jacobian and the corresponding eigenvalues and eigenvectors.

## Jacobian Reduction

We define ``y`` as the vector of algebraic variables, ``x`` as the vector of differential
variables (states) and ``p`` the parameters of the system, we can define ``g(y,x,p)`` as the
vector of algebraic equations and ``f(y,x,p)`` as the vector of differential equations.
With that, the non-linear differential algebraic system of equations can be written as:

```math
\begin{align}
\left[\begin{array}{c}
 0 \\
  \dot{x}
  \end{array}\right] = \left[\begin{array}{c}
  g(y,x,p) \\
   f(y,x,p) \end{array}\right]
\end{align}
```

For small signal analysis, we are interested in the stability around an equilbrium point
``y_{eq},x_{eq}`` that satisfies ``\dot{x} = 0`` or equivalently ``f(y_{eq},x_{eq},p) = 0``,
while obviously satisfying ``g(y_{eq}, x_{eq}, p) = 0``. To do that we use a first order
approximation:

```math
\begin{align}
\left[\begin{array}{c}
 0 \\
  \Delta\dot{x}
  \end{array}\right] = \underbrace{\left[\begin{array}{c}
  g(y_{eq},x_{eq},p) \\
   f(y_{eq},x_{eq},p) \end{array}\right]}_{= 0}
 + J[y_{eq}, x_{eq}, p] \left[\begin{array}{c}
 \Delta y \\
  \Delta x
  \end{array}\right]
  \end{align}
```

The first to note is that the jacobian matrix can be splitted in 4 blocks depending on the
specific variables we are taking the partial derivatives:

```math
\begin{align}
J[y_{eq}, x_{eq}, p] =
\left[\begin{array}{cc}
 g_y & g_x \\
 f_y & f_x \\
  \end{array}\right]
\end{align}
```

For small signal analyses, we are interested in the stability of the differential states,
while still considering that those need to evolve in the manifold defined by the linearized
algebraic equations. Assuming that ``g_y`` is not singular (see chapter 7 of Federico
Milano's book: [Power System Modelling and Scripting](https://www.springer.com/gp/book/9783642136689)
or [the following paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1323205))
we can eliminate the algebraic variables to obtain the reduced jacobian:

```math
\begin{align}
J_{\text{red}} = f_x - f_y g_y^{-1} g_x
\end{align}
```

that defines our reduced system for the differential variables

```math
\begin{align}
\Delta \dot{x} = J_{\text{red}} \Delta x
\end{align}
```

on which we can compute its eigenvalues to analyze local stability.

## Accessing the Jacobian function

You can retrieve the Jacobian function for a simulation using the [`get_jacobian`](@ref) function as follows:

```julia
jacobian = get_jacobian(ResidualModel, system)
```

optionally you can pass the number of iterations to check for sparsity as follows:

```julia
jacobian = get_jacobian(ResidualModel, system, 0)
```

if you specify 0, the jacobian function will use a full matrix.

The return of [`get_jacobian`](@ref) is known as a functor in Julia and can be used to make evaluations.
Currently, any function can be evaluated with the following inputs:

```julia
jacobian(x)
```

This version of the function is type unstable should only be used for non-critial ops. It works
to get the eigenvalues given an operating point `x`

```julia
jacobian(JM, x)
```

This version evaluates in place the value of the jacobian for an operating point `x` and writes
to the matrix `JM`

```julia
jacobian(JM, x, p, t)
```

This version complied with the requirements to be used in DiffEq for ODE solvers.
p and t aren't used they just mean to match the interfaces. See [DiffEqDocs](https://diffeq.sciml.ai/stable/features/performance_overloads/#Examples)

```julia
jacobian(JM, dx, x, p, gamma, t)
```

This version complied with the requirements to be used in DiffEq for DAE solvers.
p and t aren't used they just mean to match the interfaces. It assumes that the jacobian has
the form:

```math
\begin{align}
  JM = \gamma * I + J(x)
\end{align}
```

See [DiffEqDocs](https://diffeq.sciml.ai/stable/features/performance_overloads/#Examples) for
additional details.
