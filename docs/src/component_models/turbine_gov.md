# Prime Movers and Turbine Governors (TG)

This section describes how mechanical power is modified to provide primary frequency control with synchronous generators. It is assumed that ``\tau_{\text{ref}} = P_{\text{ref}}`` since they are decided at nominal frequency ``ω = 1``.

## Fixed TG [`PowerSystems.TGFixed`](@extref)

This a simple model that set the mechanical torque to be equal to a proportion of the desired reference ``\tau_m = \eta P_{\text{ref}}``. To set the mechanical torque to be equal to the desired power, the value of ``\eta`` is set to 1.

## TG Type I [`PowerSystems.TGTypeI`](@extref)

This turbine governor is described by a droop controller and a low-pass filter to model the governor and two lead-lag blocks to model the servo and reheat of the turbine governor.

```math
\begin{align}
\dot{x}_{g1} &= \frac{1}{T_s}(p_{\text{in}} - x_{g1}) \tag{1a} \\
\dot{x}_{g2} &= \frac{1}{T_c} \left[ \left(1- \frac{t_3}{T_c}\right)x_{g1} - x_{g2} \right] \tag{1b} \\
\dot{x}_{g3} &= \frac{1}{T_5} \left[\left(1 - \frac{T_4}{T_5}\right)\left(x_{g2} + \frac{T_3}{T_c}x_{g1}\right) - x_{g3}  \right] \tag{1c} \\
\tau_m &= x_{g3} + \frac{T_4}{T_5}\left(x_{g2} + \frac{T_3}{T_c}x_{g1}\right) \tag{1d}
\end{align}
```

with

```math
\begin{align*}
p_{\text{in}} = P_{\text{ref}} + \frac{1}{R}(\omega_s - 1.0)
\end{align*}
```

## TG Type II [`PowerSystems.TGTypeII`](@extref)

This turbine governor is a simplified model of the Type I.

```math
\begin{align}
\dot{x}_g &= \frac{1}{T_2}\left[\frac{1}{R}\left(1 - \frac{T_1}{T_2}\right) (\omega_s - \omega) - x_g\right] \tag{2a} \\
\tau_m &= P_{\text{ref}} + \frac{1}{R}\frac{T_1}{T_2}(\omega_s - \omega) \tag{2b}
\end{align}
```

## TG Simple [`PowerSystems.TGSimple`](@extref)

This turbine governor represents a simple first-order model with droop control. It's a 1-state model suitable for basic frequency response studies.

```math
\begin{align}
T_m \dot{\tau}_m &= \left(P_{\text{ref}} + d_t (\omega_{\text{ref}} - \omega)\right) - \tau_m \tag{3a}
\end{align}
```

The `TGSimple` uses a linear relationship between frequency deviation and power output and a first order lag response with time constant ``T_m``. It is suitable for studies where detailed turbine dynamics are not critical since provides basic primary frequency response through droop control.

## TGOV1 [`PowerSystems.SteamTurbineGov1`](@extref)

This represents a classical Steam-Turbine Governor, known as TGOV1.

```math
\begin{align}
\dot{x}_{g1} &= \frac{1}{T_1} (\text{ref}_{in} - x_{g1}) \tag{4a} \\
\dot{x}_{g2} &= \frac{1}{T_3} \left(x_{g1}^\text{sat} \left(1 - \frac{T_2}{T_3}\right) - x_{g2}\right) \tag{4b}
\end{align}
```

with

```math
\begin{align}
\text{ref}_{in} &= \frac{1}{R} (P_{ref} - (\omega - 1.0)) \tag{4c} \\
x_{g1}^\text{sat} &= \left\{ \begin{array}{cl}
                        x_{g1} & \text{ if } V_{min} \le x_{g1} \le V_{max}\\
                        V_{max} & \text{ if } x_{g1} > V_{max} \\
                        V_{min} & \text{ if } x_{g1} < V_{min}
                    \end{array} \right. \tag{4d} \\
\tau_m &= x_{g2} + \frac{T_2}{T_3} x_{g1} - D_T(\omega - 1.0) \tag{4e}
\end{align}
```

## GAST [`PowerSystems.GasTG`](@extref)

This turbine governor represents the Gas Turbine representation, known as GAST.

```math
\begin{align}
\dot{x}_{g1} &= \frac{1}{T_1} (x_{in} - x_{g1}) \tag{5a} \\
\dot{x}_{g2} &= \frac{1}{T_2} \left(x_{g1}^\text{sat} - x_{g2}\right) \tag{5b} \\
\dot{x}_{g3} &= \frac{1}{T_3} (x_{g2} - x_{g3}) \tag{5c}
\end{align}
```

with

```math
\begin{align}
x_{in} &= \min\left\{P_{ref} - \frac{1}{R}(\omega - 1.0), A_T + K_T (A_T - x_{g3}) \right\} \tag{5d} \\
x_{g1}^\text{sat} &= \left\{ \begin{array}{cl}
                        x_{g1} & \text{ if } V_{min} \le x_{g1} \le V_{max}\\
                        V_{max} & \text{ if } x_{g1} > V_{max} \\
                        V_{min} & \text{ if } x_{g1} < V_{min}
                    \end{array} \right. \tag{5e} \\
\tau_m &= x_{g2}  - D_T(\omega - 1.0) \tag{5f}
\end{align}
```

## HYGOV [`PowerSystems.HydroTurbineGov`](@extref)

This represents a classical hydro governor, known as HYGOV.

```math
\begin{align}
T_f\dot{x}_{g1} &= P_{in} - x_{g1} \tag{6a} \\
\dot{x}_{g2} &= x_{g1} \tag{6b}\\
T_g \dot{x}_{g3} &= c - x_{g3} \tag{6c}\\
\dot{x}_{g4} &= \frac{1 - h}{T_w} \tag{6d}
\end{align}
```

with

```math
\begin{align}
P_{in} &= P_{ref} - \Delta \omega - R x_{g2} \tag{6e} \\
c &= \frac{x_{g1}}{r} + \frac{x_{g2}}{rT_r} \tag{6f} \\
h &= \left(\frac{x_{g4}}{x_{g3}}\right)^2 \tag{6g}\\
\tau_m &= h\cdot A_t(x_{g4} - q_{NL}) - D_{turb} \Delta\omega \cdot x_{g3} \tag{6h}
\end{align}
```

## DEGOV [`PowerSystems.DEGOV`](@extref)

This turbine governor represents the IEEE Type DEGOV Diesel Engine Governor Model. It's a 5-state model commonly used for diesel generators.

```math
\begin{align}
T_1 \dot{x}_{ecb1} &= -\Delta\omega - T_1 x_{ecb1} - x_{ecb2} \tag{7a} \\
\dot{x}_{ecb2} &= x_{ecb1} \tag{7b} \\
(T_5 + T_6) \dot{x}_{a1} &= y_1 - (T_5 + T_6) x_{a1} - x_{a2} \tag{7c} \\
\dot{x}_{a2} &= x_{a1} \tag{7d} \\
\dot{x}_{a3} &= K y_2 \tag{7e}
\end{align}
```

with

```math
\begin{align*}
\Delta\omega &= \omega - \omega_{sys} \\
y_1 &= \frac{T_4}{T_1 T_2} (-\Delta\omega) + \frac{T_3 - T_1 T_4 / (T_1 T_2)}{T_1 T_2} x_{ecb1} + \frac{1 - T_4 / (T_1 T_2)}{T_1 T_2} x_{ecb2} \\
y_2 &= \frac{0}{T_5 T_6} y_1 + \frac{T_4 - (T_5 + T_6) \cdot 0 / (T_5 T_6)}{T_5 T_6} x_{a1} + \frac{1 - 0 / (T_5 T_6)}{T_5 T_6} x_{a2} \\
y_{delay} = e^{-s T_d} x_{a3}
P_m &= T_d y_{delay} \cdot \omega \\
\tau_m &= \frac{P_m}{\omega}
\end{align*}
```

Note that the model includes a time delay ``T_d`` applied to the final actuator output `x_{a3}`.

## DEGOV1 [`PowerSystems.DEGOV1`](@extref)

This is an enhanced version of the DEGOV model with optional droop control. It's a 5-state or 6-state model depending on the droop flag.

```math
\begin{align}
T_1 \dot{x}_{g1} &= \text{ll\_in} - T_1 x_{g1} - x_{g2} \tag{8a} \\
\dot{x}_{g2} &= x_{g1} \tag{8b} \\
(T_5 + T_6) \dot{x}_{g3} &= y_1 - (T_5 + T_6) x_{g3} - x_{g4} \tag{8c} \\
\dot{x}_{g4} &= x_{g3} \tag{8d} \\
\dot{x}_{g5} &= K y_2 \tag{8e}
\end{align}
```

If droop flag = 1, an additional state is added:

```math
\begin{align}
T_e \dot{x}_{g6} &= \tau_e - x_{g6} \tag{8f}
\end{align}
```

with

```math
\begin{align*}
\text{ll\_in} &= P_{ref} - (\omega - 1.0) - \text{feedback} \cdot R \\
\text{feedback} &= \begin{cases}
x_{g5} & \text{if droop\_flag} = 0 \\
x_{g6} & \text{if droop\_flag} = 1
\end{cases} \\
y_1 &= \frac{0}{T_1 T_2} \text{ll\_in} + \frac{T_3}{T_1 T_2} x_{g1} + \frac{1}{T_1 T_2} x_{g2} \\
y_2 &= \frac{0}{T_5 T_6} y_1 + \frac{T_4}{T_5 T_6} x_{g3} + \frac{1}{T_5 T_6} x_{g4} \\
P_m &= x_{g5}(T_d) \cdot \omega \\
\tau_m &= \frac{P_m}{\omega}
\end{align*}
```

### Notes

  - When droop flag = 0: feedback comes from actuator output (speed droop)
  - When droop flag = 1: feedback comes from electrical power (load droop)

## WPIDHY [`PowerSystems.WPIDHY`](@extref)

This model represents a Woodward PID Hydro Turbine Governor, a 7-state model designed for hydro turbine control.

```math
\begin{align}
T_{reg} \dot{x}_{g1} &= \text{reg} \cdot x_{in} - x_{g1} \tag{9a} \\
\dot{x}_{g2} &= K_i' \cdot \text{pid\_input} + \frac{K_p'}{K_i'} x_{g2} \tag{9b} \\
T_a \dot{x}_{g3} &= \text{pid\_out} - x_{g3} \tag{9c} \\
T_a \dot{x}_{g4} &= -\frac{K_d'}{T_a} \text{pid\_input} - x_{g4} \tag{9d} \\
T_b \dot{x}_{g5} &= x_{g3} - x_{g5} \tag{9e} \\
\dot{x}_{g6} &= \text{clamp}(x_{g5}, V_{min}, V_{max}) \quad \text{with windup protection} \tag{9f} \\
\frac{T_w}{2} \dot{x}_{g7} &= \text{power\_at\_gate} + T_w x_{g7} - x_{g7} \tag{9g}
\end{align}
```

with

```math
\begin{align*}
x_{in} &= \tau_e - P_{ref} \\
\text{pid\_input} &= x_{g1} - (\omega - \omega_{sys}) \\
K_p' &= (-T_a K_i) + K_p \\
K_d' &= (T_a^2 K_i) - (T_a K_p) + K_d \\
K_i' &= K_i \\
\text{pi\_out} &= \text{pid\_input} \cdot K_p' + x_{g2} \\
\text{pd\_out} &= x_{g4} + \frac{K_d'}{T_a} \text{pid\_input} \\
\text{pid\_out} &= \text{pi\_out} + \text{pd\_out} \\
\text{power\_at\_gate} &= f(\text{clamp}(x_{g6}, G_{min}, G_{max})) \\
P_m &= \text{clamp}(x_{g7}, P_{min}, P_{max}) - D(\omega - \omega_{sys}) \\
\tau_m &= \frac{P_m}{\omega}
\end{align*}
```

Note that this model includes:

  - Includes nonlinear gate-to-power mapping function
  - Water inertia modeled with lead-lag compensation

## PIDGOV [`PowerSystems.PIDGOV`](@extref)

This model represents a PID Turbine Governor, typically used for hydro applications. It's a 7-state model with configurable feedback.

```math
\begin{align}
T_{reg} \dot{x}_{g1} &= R_{perm} \cdot x_{in} - x_{g1} \tag{10a} \\
\dot{x}_{g2} &= K_i \cdot \text{pid\_input} \tag{10b} \\
T_a \dot{x}_{g3} &= \text{pi\_out} - x_{g3} \tag{10c} \\
T_a \dot{x}_{g4} &= -\frac{K_d}{T_a} \text{pid\_input} - x_{g4} \tag{10d} \\
T_a \dot{x}_{g5} &= (x_{g3} + \text{pd\_out}) - x_{g5} \tag{10e} \\
\dot{x}_{g6} &= \text{clamp}\left(\frac{x_{g5} - x_{g6}}{T_b}, V_{min}, V_{max}\right) \quad \text{with windup protection} \tag{10f} \\
\frac{T_z}{2} \dot{x}_{g7} &= \text{power\_at\_gate} + T_z x_{g7} - x_{g7} \tag{10g}
\end{align}
```

with

```math
\begin{align*}
x_{in} &= \begin{cases}
P_{ref} - \tau_e & \text{if feedback\_flag} = 0 \\
P_{ref} - x_{g6} & \text{if feedback\_flag} = 1
\end{cases} \\
\text{pid\_input} &= x_{g1} - (\omega - \omega_{sys}) \\
\text{pi\_out} &= K_p \cdot \text{pid\_input} + x_{g2} \\
\text{pd\_out} &= x_{g4} + \frac{K_d}{T_a} \text{pid\_input} \\
T_z &= A_{tw} \cdot T_w \\
\text{power\_at\_gate} &= f(\text{clamp}(x_{g6}, G_{min}, G_{max})) \\
P_m &= x_{g7} - D_{turb}(\omega - \omega_{sys}) \\
\tau_m &= \frac{P_m}{\omega}
\end{align*}
```

Note that this model includes:

  - Configurable feedback: electrical power or gate position
  - Modified integrator with rate limiting for gate servo
  - Water column dynamics with surge tank effects
  - Nonlinear gate-to-power characteristic curve
