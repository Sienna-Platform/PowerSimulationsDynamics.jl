# Converter

This component can be used to model the dynamics of the switching process.

## Average Model [`PowerSystems.AverageConverter`](@extref)

The average model outputs the desired reference signal since:

```math
\begin{align}
v_{d}^{\text{cv}} \approx m_{d} v_{\text{dc}} \approx \frac{v_{d}^{\text{ref-signal}}}{v_{\text{dc}}} v_{\text{dc}} \approx v_{d}^{\text{ref-signal}} \tag{1a} \\
v_{q}^{\text{cv}} \approx m_{q} v_{\text{dc}} \approx \frac{v_{q}^{\text{ref-signal}}}{v_{\text{dc}}} v_{\text{dc}} \approx v_{q}^{\text{ref-signal}} \tag{1b}
\end{align}
```

where ``m_{dq}`` is the modulation signal, and ``v_{dq}^{\text{ref-signal}}`` is the voltage reference signal from the inner loop control.

## Generic Renewable Converter Type A [`PowerSystems.RenewableEnergyConverterTypeA`](@extref)

This block represents the [REGCA](https://www.powerworld.com/WebHelp/Content/TransientModels_HTML/Machine%20Model%20REGC_A.htm) model. The equations (without the limiters) are:

```math
\begin{align}
    \dot{I}_p &= \frac{1}{T_g} (I_\text{pcmd} - I_p) \tag{2a} \\
    \dot{I}_q &= \frac{1}{T_g} (I_\text{qcmd} - I_q) \tag{2b} \\
    \dot{V}_\text{meas} &= \frac{1}{T_{fltr}} (V_t - V_\text{meas} \tag{2c})
\end{align}
```

on which ``I_\text{pcmd}`` and ``I_\text{qcmd}`` are the current commands from the inner control and ``V_t`` is the bus voltage magnitude. The additional terms and output current are computed as:

```math
\begin{align}
    I_q^\text{cv} &= -I_q - I_\text{q,extra} \tag{2d} \\
    I_\text{q,extra} &= \max(K_{hv} (V_t - V_\text{olim})) \tag{2e} \\
    I_p^\text{cv} &= G_{lv} I_p \tag{2f} 
\end{align}
```

on which ``G_{lv}`` is the gain used for Low Voltage Active Current Management and ``I_\text{q,extra}`` is the additional current for High Voltage Reactive Current Management.

It is important to note that both current commands coming from the inner control were obtained by dividing the active (or reactive) power by the magnitude voltage instead of using the correct phasor formula ``I = (S/V)^*``. For that purpose, a correction factor must be applied to obtain the correct output currents in the network reference frame:

```math
\begin{align*}
    I_r + jI_i &= (I_p + jI_q) \cdot V_t \cdot \frac{1}{V_r + jV_i} \\
    &= (I_p + jI_q) \cdot \frac{V_t}{V_t e^{j\theta}} \\
    &= (I_p + jI_q) \cdot e^{-j\theta}
\end{align*}
```

This correction factor looks like a reference transformation that must be used to properly inject current into the grid. With that the output current is computed as:

```math
\begin{align}
    I_r &= I_p^\text{cv} \cos(\theta) - I_q^\text{cv} \sin(\theta) \tag{2g} \\
    I_i &= I_p^\text{cv} \sin(\theta) + I_q^\text{cv} \cos(\theta) \tag{2h}
\end{align}
```

This current source is usually modeled as a Norton equivalent using a parallel impedance with values ``R_{sorce}`` and ``X_{sorce}`` provided in the `.raw` file. If an RL filter is used, a Voltage Source behind that RL filter (i.e. the converter output voltage) can be computed as:

```math
\begin{align*}
    Z_f &= r_f + jl_f \\
    Z_{sorce} &= R_{sorce} + jX_{sorce} \\
    I_{cv} &= I_r + jI_i \\
    v_r^\text{cv} + jv_i^\text{cv} &= \frac{I_{cv} + \frac{v^\text{grid}}{Z_f}}{\frac{1}{Z_{sorce}} + \frac{1}{Z_f}} \tag{2i}
\end{align*}
```
