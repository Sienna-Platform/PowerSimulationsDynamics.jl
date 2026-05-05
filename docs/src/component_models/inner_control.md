# Inner Loop Controls

This component defines voltage and current controllers to generate the reference signal
for the converter. Although in many controls the current and voltage control are separate blocks
we propose a more general control approach that considers them as a joint control logic.

## Integrated Virtual Impedance, Voltage and Current Controller [`VoltageModeControl`](@ref)

The following model receives both the outer loop control frequency and reference voltage
signal to generate the reference signal for the converters. The virtual impedance plays a
similar role of the impedance of a synchronous generator. A PI voltage controller is used
to generate the current signal that is used in the PI current controller to finally generate
the voltage reference signal for the converters.

```math
\begin{align*}
    \dot{\xi}_d &= v_{d,\text{vi}}^{\text{ref}} - v_d \tag{1a} \\
    \dot{\xi}_q &= v_{q,\text{vi}}^{\text{ref}} - v_q \tag{1b} \\
    \dot{\gamma}_d &= i_{d,\text{cv}}^{\text{ref}} - i_{d,\text{cv}} \tag{1c} \\
    \dot{\gamma}_q &= i_{q,\text{cv}}^{\text{ref}} - i_{q,\text{cv}} \tag{1d} \\
    \dot{\phi}_d &= \omega_{\text{ad}}(v_d - \phi_d) \tag{1e} \\
    \dot{\phi}_q &= \omega_{\text{ad}}(v_q - \phi_q) \tag{1f}
\end{align*}
```

with

```math
\begin{align}
    v_{d,\text{vi}}^{\text{ref}} &= v_{\text{olc}}^{\text{ref}} - r_v i_d + \omega_{\text{olc}} l_v i_q \tag{1g} \\
    v_{q,\text{vi}}^{\text{ref}} &= - r_v i_q - \omega_{\text{olc}} l_v i_d \tag{1h} \\
    i_{d,\text{cv}}^{\text{ref}} &= k_{pv}\left(v_{d,\text{vi}}^{\text{ref}} - v_d\right) + k_{iv} \xi_d - c_f \omega_{\text{olc}} v_q + k_{\text{ffi}}i_d \tag{1i} \\
    i_{q,\text{cv}}^{\text{ref}} &= k_{pv}\left(v_{q,\text{vi}}^{\text{ref}} - v_q\right) + k_{iv} \xi_q + c_f \omega_{\text{olc}} v_d + k_{\text{ffi}}i_q \tag{1j} \\
    v_d^{\text{ref-signal}} &= k_{pc} \left(i_{d,\text{cv}}^{\text{ref}} - i_{d,\text{cv}}\right) + k_{ic} \gamma_d - \omega_{\text{olc}} l_f i_{q,\text{cv}} + k_{\text{ffv}}v_d - k_{\text{ad}}(v_d - \phi_d) \tag{1k} \\
    v_q^{\text{ref-signal}} &= k_{pc} \left(i_{q,\text{cv}}^{\text{ref}} - i_{q,\text{cv}}\right) + k_{ic} \gamma_q + \omega_{\text{olc}} l_f i_{d,\text{cv}} + k_{\text{ffv}}v_q - k_{\text{ad}}(v_q - \phi_q) \tag{1l}
\end{align}
```

In here the transformation to the ``dq`` reference frame is using the outer-loop reference angle as:

```math
\begin{align*}
v_d + jv_q = (v_r + jv_i)e^{-j\delta\theta_{olc}} \\
i_d + ji_q = (i_r + ji_i)e^{-j\delta\theta_{olc}}
\end{align*}
```

that again ``v_r + jv_i`` could be in the capacitor or the last branch of the filter (i.e.
the point of common coupling). For LCL filters it is considered in the capacitor. In the case
of the converter, the transformation is directly

```math
\begin{align*}
i_{d,\text{cv}} + ji_{q,\text{cv}} = (i_{r,\text{cv}} + ji_{i,\text{cv}})e^{-j\delta\theta_{olc}}
\end{align*}
```

## Current Mode Controller [`CurrentModeControl`](@ref)

The following model receives the current reference (in ``dq`` axis) from an outer loop controller
that outputs current references such as the PI outer controller used for grid following inverters.
A PI current controller is used to generate the voltage reference signal for the converters.

```math
\begin{align*}
    \dot{\gamma}_d &= i_{d,\text{cv}}^{\text{ref}} - i_{d,\text{cv}} \tag{2a} \\
    \dot{\gamma}_q &= i_{q,\text{cv}}^{\text{ref}} - i_{q,\text{cv}} \tag{2b} \\
\end{align*}
```

with

```math
\begin{align}
    v_d^{\text{ref-signal}} &= k_{pc} \left(i_{d,\text{cv}}^{\text{ref}} - i_{d,\text{cv}}\right) + k_{ic} \gamma_d - \omega_{\text{olc}} l_f i_{q,\text{cv}} + k_{\text{ffv}}v_d \tag{2b} \\
    v_q^{\text{ref-signal}} &= k_{pc} \left(i_{q,\text{cv}}^{\text{ref}} - i_{q,\text{cv}}\right) + k_{ic} \gamma_q + \omega_{\text{olc}} l_f i_{d,\text{cv}} + k_{\text{ffv}}v_q \tag{2c}
\end{align}
```

The transformation for the converter current is computed as:

```math
\begin{align*}
i_{d,\text{cv}} + ji_{q,\text{cv}} = (i_{r,\text{cv}} + ji_{i,\text{cv}})e^{-j\theta_{olc}}
\end{align*}
```

In here ``\theta_{olc}`` is the outer-loop angle. In the case of grid-following models, this angle is equal to the angle provided from the PLL.

## Generic Renewable Inner Controller Type B [`RECurrentControlB`](@ref)

This models the inner control part of the [REECB](https://www.powerworld.com/WebHelp/Content/TransientModels_HTML/Exciter%20REEC_B.htm) model. The equations (without limiters) when `Q_Flag = 1` are:

```math
\begin{align}
    T_{rv} \dot{V}_\text{t,flt} &= V_t - \dot{V}_\text{t,flt} \tag{3a} \\
    \dot{\xi}_{icv} &= V_\text{oc,qcmd} \tag{3b}
\end{align}
```

on which ``V_\text{oc,qcmd}`` comes from the Outer Controller and the output current commands ``I_\text{pcmd}`` and ``I_\text{qcmd}`` are computed as:

```math
\begin{align}
    I_\text{pcmd} &= I_\text{oc, pcmd} \tag{3c} \\
    I_\text{qcmd} &= I_{icv} + I_\text{qinj} \tag{3d} \\
    I_{icv} &= K_{vp} V_\text{oc,qcmd} + K_{vi} \xi_{icv} \tag{3e} \\
    I_{\text{qinj}} &= K_{qv} (V_\text{ref0} - V_\text{t,flt}) \tag{3f}
\end{align}
```

The equations when `Q_Flag = 0` are:

```math
\begin{align}
    T_{rv} \dot{V}_\text{t,flt} &= V_t - \dot{V}_\text{t,flt} \tag{3g} \\
    T_{iq} \dot{I}_{icv} &= I_\text{oc,qcmd} - I_{icv} \tag{3h}
\end{align}
```

on which ``I_\text{oc,qcmd}`` comes from the Outer Controller and the output current commands ``I_\text{pcmd}`` and ``I_\text{qcmd}`` are computed as:

```math
\begin{align}
    I_\text{pcmd} &= I_\text{oc, pcmd} \tag{3i} \\
    I_\text{qcmd} &= I_{icv} + I_\text{qinj} \tag{3j} \\
    I_{\text{qinj}} &= K_{qv} (V_\text{ref0} - V_\text{t,flt}) \tag{3k}
\end{align}
```
