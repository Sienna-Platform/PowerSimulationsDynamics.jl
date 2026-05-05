# Outer Loop Controls

This component defines controllers for both active and reactive power. The joint design is based
on the fact that many novel control techniques can be based on joint control of active and reactive
power.

## Virtual Inertia and Q-droop [`OuterControl`](@ref)

The following model represent a virtual synchronous machine model to represent how active
power is going to be deployed. The constructor is `OuterControl{VirtualInertia, ReactivePowerDroop}`.
It defines a new SRF denoted as ``\theta_{\text{olc}}`` for the active power controller and
uses a simple voltage droop for dispatching reactive power:

```math
\begin{align}
    \dot{\omega}_{\text{olc}} &= \frac{p_{\text{ref}}}{T_a} - \frac{p_e}{T_a} - \frac{k_d(\omega_{\text{olc}} - \omega_{\text{pll}})}{T_a} - \frac{k_\omega(\omega_{\text{olc}} - \omega_{\text{ref}})}{T_a} \tag{1a} \\
    \dot{\theta}_{\text{olc}} &= \Omega_b (\omega_{\text{olc}} - \omega_{\text{sys}}) \tag{1b} \\
    \dot{q}_m &= \omega_f (q_e - q_m) \tag{1c}
\end{align}
```

with

```math
\begin{align}
    p_e &= v_ri_r + v_ii_i \tag{1d} \\
    q_e &= v_ii_r - v_ri_i \tag{1e} \\
    v_{\text{olc}}^{\text{ref}} &= v_{\text{ref}} + k_q(q_{\text{ref}} - q_m) \tag{1f}
\end{align}
```

In this case, the measurement of power are being done in the capacitor of the LCL filter.
However, depending on the model, this measurements could be different depending on where
is the point of common coupling.

## Active Power Droop (P-droop) and Q-droop [`OuterControl`](@ref)

The following model represent a ``P\text{-}f`` droop model to represent how active
power is going to be deployed. The constructor is `OuterControl{ActivePowerDroop, ReactivePowerDroop}`.
It defines a new SRF denoted as ``\theta_{\text{olc}}`` for the active power controller and
uses a simple voltage droop for dispatching reactive power. Both active and reactive power are measured via a low-pass filter:

```math
\begin{align}
    \dot{\theta}_{\text{olc}} &= \Omega_b (\omega_{\text{olc}} - \omega_{\text{sys}}) \tag{2a} \\
    \dot{p}_m &= \omega_z (p_e - p_m) \tag{2b} \\
    \dot{q}_m &= \omega_f (q_e - q_m) \tag{2c}
\end{align}
```

with

```math
\begin{align}
    p_e &= v_ri_r + v_ii_i \tag{2d} \\
    q_e &= v_ii_r - v_ri_i \tag{2e} \\
    \omega_{\text{olc}} &= \omega_{\text{ref}} + R_p (p_{\text{ref}} - p_m) \tag{2f} \\
    v_{\text{olc}}^{\text{ref}} &= v_{\text{ref}} + k_q(q_{\text{ref}} - q_m) \tag{2g}
\end{align}
```

In this case, the measurement of power are being done in the capacitor of the LCL filter.
However, depending on the model, this measurements could be different depending on where
is the point of common coupling.

## Active and Reactive Virtual Oscillator Controllers [`OuterControl`](@ref)

The following model represents a Virtual Oscillator Controller for both active and reactive power
to generate the voltage references that will be used in the Voltage Controller. The contructor is `OuterControl{ActiveVirtualOscillator, ReactiveVirtualOscillator}`
It defines a new SRF denoted as ``\theta_{\text{olc}}`` and a voltage reference ``E_{\text{olc}}``.
The equations are:

```math
\begin{align}
    \dot{\theta}_{\text{olc}} &= \Omega_b (\omega_{\text{olc}} - \omega_{\text{sys}}) \tag{3a} \\
    \dot{E}_{olc} &= \Omega_b \left(\frac{k_1}{E_{oc}} (-\sin(\gamma) (p_\text{ref} - p_e) + \cos(\gamma)(q_\text{ref} - q_e)) + k_2 (V_\text{ref} - E_{oc}^2)E_{oc} \right) \tag{3b} \\
\end{align}
```

with

```math
\begin{align}
    \gamma &= \psi - \frac{\pi}{2} \tag{3c} \\
    \omega_{\text{olc}} &=  \omega_{\text{sys}} + \frac{k_1}{E_{oc}^2} \left(\cos(\gamma) (p_\text{ref} - p_e) + \sin(\gamma)(q_\text{ref} - q_e) \right) \tag{3d} \\
    p_e &= v_ri_r + v_ii_i \tag{3e} \\
    q_e &= v_ii_r - v_ri_i \tag{3f} \\
\end{align}
```

## Active and Reactive Power PI Controllers (Grid Following) [`OuterControl`](@ref)

The following model represents a PI controller for both active and reactive power to generate
the current references that will be used in the Current Controller of the inner control
`CurrentModeControl`. The constructor is `OuterControl{ActivePowerPI, ReactivePowerPI}`.
The equations are:

```math
\begin{align}
    \dot{\sigma}_{p} &= p_\text{ref} - p_m \tag{4a} \\
    \dot{p}_m &= \omega_z (p_e - p_m) \tag{4b} \\
    \dot{\sigma}_{q} &= q_\text{ref} - q_m \tag{4c} \\
    \dot{q}_m &= \omega_f (q_e - p_m) \tag{4d} \\
\end{align}
```

with

```math
\begin{align}
    p_e &= v_ri_r + v_ii_i \tag{4e} \\
    q_e &= v_ii_r - v_ri_i \tag{4f} \\
    \omega_{\text{olc}} &= \omega_{\text{pll}} \tag{4g} \\
    \theta_{\text{olc}} &= \theta_{\text{pll}} \tag{4h} \\
    i_\text{d,cv}^\text{ref} &= k_p^q (q_\text{ref} - q_m) + k_i^q \sigma_q \tag{4i} \\
    i_\text{q,cv}^\text{ref} &= k_p^p (p_\text{ref} - p_m) + k_i^p \sigma_p \tag{4j} \\
\end{align}
```

This models requires a PLL to have a SRF for an internal ``dq`` reference frame. Contrary
to the Grid-Forming model, it cannot work without a PLL. Since this Outer Control outputs
a current reference, it can only be used with a current mode inner control (i.e. that receives
a current reference instead of a voltage reference).

## Active and Reactive Generic Renewable Controller Type AB [`OuterControl`](@ref)

The following model represents an outer controller for both active and reactive power from generic industrial
models REPCA and REECB to generate the current references that will be used in the Current Controller of the inner control
`RECurrentControlB`. The constructor is `OuterControl{ActiveRenewableControllerAB, ReactiveRenewableControllerAB}`.
The equations will depend on the flags used.

### Active part

In the case of `F_Flag = 1` the equations (without limits and freezing) are:

```math
\begin{align}
    \dot{p}_{\text{flt}} &= \frac{1}{T_p} (p_e - p_{\text{flt}}) \tag{5a} \\
    \dot{\xi}_{P} &= p_{\text{err}} \tag{5b} \\
    \dot{p}_{\text{ext}} &= \frac{1}{T_g} (K_{pg} p_{\text{err}} + K_{ig} \xi_P) \tag{5c} \\
    \dot{p}_{\text{ord}} &= \frac{1}{T_{\text{pord}}} (p_{\text{ext}} - p_{\text{ord}}) \tag{5d}
\end{align}
```

with

```math
\begin{align}
    p_e &= v_ri_r + v_ii_i \tag{5e} \\
    p_\text{err} &= p_\text{ref} + p_\text{droop} - p_\text{flt} \tag{5f} 
\end{align}
```

In the case of `F_Flag = 0` the equations (without limits) are simply

```math
\begin{align}
    \dot{p}_{\text{ord}} &= \frac{1}{T_{\text{pord}}} (p_{\text{ref}} - p_{\text{ord}}) \tag{5g}
\end{align}
```

The current command going to the Inner Loop is computed as:

```math
\begin{align}
    I_{\text{oc,pcmd}} &= \frac{p_{\text{ord}}}{V_{\text{t,flt}}} \tag{5h}
\end{align}
```

on which ``V_\text{t,flt}`` is the filtered terminal bus voltage coming from the inner controller.

### Reactive part

In the case of `VC_Flag = 0, Ref_Flag = 0, PF_Flag = 0, V_Flag = 1` the equations (without limits and freezing) are:

```math
\begin{align}
    \dot{q}_\text{flt} &= \frac{1}{T_\text{fltr}} (q_e - q_\text{flt}) \tag{5i} \\
    \dot{\xi}_\text{q,oc} &= q_\text{err} \tag{5j} \\
    \dot{q}_{LL} &= \frac{1}{T_{fv}}(Q_\text{pi} ( 1 - T_{ft}/T_{fv}) - q_{LL}) \tag{5k} \\
    \dot{\xi}_Q &= V_\text{pi,in} \tag{5l} 
\end{align}
```

with

```math
\begin{align}
    q_e &= v_ii_r - v_ri_i \tag{5m} \\
    q_\text{err} &= q_\text{ref} - q_flt \tag{5n} \\
    Q_\text{pi} &= K_p q_\text{err} + K_i \xi_\text{q,oc} \tag{5o} \\
    Q_\text{ext} &= q_{LL} + \frac{T_{ft}}{T_{fv}} Q_\text{pi} \tag{5p} \\
    V_\text{pi,in} &= Q_\text{ext} - q_e \tag{5q} 
\end{align}
```

The output to the inner controller are ``V_\text{oc,qcmd}`` if the `Q_Flag = 1` on the Inner Controller, or ``I_\text{oc,qcmd}`` if `Q_Flag = 0`:

```math
\begin{align}
    V_\text{oc,qcmd} &= (K_{qp} V_\text{pi,in} + K_{qi} \xi_Q) - V_\text{t,flt} \tag{5r} \\
    I_\text{oc,qmcd} &= \frac{Q_\text{ext}}{\max(V_\text{t,flt}, 0.01)} \tag{5s}
\end{align}
```

In the case of `VC_Flag = 0, Ref_Flag = 0, PF_Flag = 0, V_Flag = 1` the equations (without limits and freezing) are:

```math
\begin{align}
    \dot{q}_\text{flt} &= \frac{1}{T_\text{fltr}} (q_e - q_\text{flt}) \tag{5t} \\
    \dot{\xi}_\text{q,oc} &= q_\text{err} \tag{5u} \\
    \dot{q}_{LL} &= \frac{1}{T_{fv}}(Q_\text{pi} ( 1 - T_{ft}/T_{fv}) - q_{LL}) \tag{5v} \\
\end{align}
```

The remaining models for other flags will be included when implemented in `PowerSimulationsDynamics.jl`.
