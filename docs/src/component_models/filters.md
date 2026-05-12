# Filters

## LCL Filter [`PowerSystems.LCLFilter`](@extref)

A standard LCL filter is proposed to connect the output of the converter to the grid. In
this case, ``v_r`` and ``v_i`` are voltages in the capacitor, while ``v_r^{\text{grid}}``
and ``v_i^{\text{grid}}`` represent the voltage at the bus. The L filter after the capacitor
can also include a step-up transformer to increase the voltage, that is model as an extra
impedance.

```math
\begin{align}
    \dot{i}_{r,\text{cv}} &= \frac{\Omega_b}{l_f}\left( v_r^{\text{cv}} - v_r  - r_f i_{r,\text{cv}} + \omega_{\text{grid}} l_f i_{i,\text{cv}} \right) \tag{1a} \\
    \dot{i}_{i,\text{cv}} &= \frac{\Omega_b}{l_f}\left( v_i^{\text{cv}} - v_i  - r_f i_{i,\text{cv}} - \omega_{\text{grid}} l_f i_{r,\text{cv}} \right) \tag{1b} \\
    \dot{v}_{r} &=  \frac{\Omega_b}{c_f}\left( i_r^{\text{cv}} - i_r + \omega_{\text{grid}} c_f v_i \right) \tag{1c} \\
    \dot{v}_{i} &=  \frac{\Omega_b}{c_f}\left( i_i^{\text{cv}} - i_i - \omega_{\text{grid}} c_f v_r \right) \tag{1d} \\
    \dot{i}_{r} &= \frac{\Omega_b}{l_g}\left( v_r - v_r^{\text{grid}} - r_g i_{r} + \omega_{\text{grid}} l_g i_{i} \right) \tag{1e} \\
    \dot{i}_{i} &= \frac{\Omega_b}{l_g}\left( v_i - v_i^{\text{grid}} - r_g i_{i} - \omega_{\text{grid}} l_g i_{r} \right) \tag{1f}
\end{align}
```

on which

```math
\begin{align*}
v_r^\text{cv} + jv_i^\text{cv} = (v_d^\text{cv} + jv_q^\text{cv})e^{j\delta\theta_{olc}}
\end{align*}
```

that comes from the converter model.

## RL Filter [`PowerSystems.RLFilter`](@extref)

The algebraic RL filter is used to connect the output of the converter through a RL series filter using algebraic phasor equations. The equations for the output current are:

```math
\begin{align}
    i_r + ji_i &= \frac{(v_r^\text{cv} + j v_i^\text{cv}) - (v_r^\text{grid} + jv_i^\text{grid})}{r_f + jl_f} \tag{2a}
\end{align}
```

on which ``v_r^\text{cv} + jv_i^\text{cv}`` comes from the converter model.
