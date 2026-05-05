# Automatic Voltage Regulators (AVR)

AVR are used to determine the voltage in the field winding ``v_f`` (or ``V_f``) in the model.

## Fixed AVR [`AVRFixed`](@ref)

This is a simple model that set the field voltage to be equal to a desired constant value ``v_f = v_{\text{fix}}``.

## Simple AVR [`AVRSimple`](@ref)

This depicts the most basic AVR, on which the field voltage is an integrator over the difference of the measured voltage and a reference:

```math
\begin{align}
\dot{v}_f = K_v(v_{\text{ref}} - v_h) \tag{1a}
\end{align}
```

## AVR Type I [`AVRTypeI`](@ref)

This AVR is a simplified version of the IEEE DC1 AVR model:

```math
\begin{align}
\dot{v}_f &= -\frac{1}{T_e} \left[ V_f(K_e + S_e(v_f))-v_{r1} \right] \tag{2a} \\
\dot{v}_{r1} &= \frac{1}{T_a} \left[ K_a\left(v_{\text{ref}} - v_m - v_{r2} - \frac{K_f}{T_f}v_f\right) - v_{r1} \right]   \tag{2b} \\
\dot{v}_{r2} &=  -\frac{1}{T_f} \left[ \frac{K_f}{T_f}v_f + v_{r2} \right]  \tag{2c} \\
\dot{v}_m &= \frac{1}{T_r} (v_h - v_m) \tag{2d}
\end{align}
```

with the ceiling function:

```math
\begin{align*}
S_e(v_f) = A_e \exp(B_e|v_f|)
\end{align*}
```

## AVR Type II [`AVRTypeII`](@ref)

This model represents a static exciter with higher gains and faster response than the Type I:

```math
\begin{align}
\dot{v}_f &= -\frac{1}{T_e} \left[ V_f(1 + S_e(v_f))-v_{r} \right] \tag{3a} \\
\dot{v}_{r1} &= \frac{1}{T_1} \left[ K_0\left(1 - \frac{T_2}{T_1} \right)(v_{\text{ref}} - v_m) - v_{r1}  \right] \tag{3b} \\
\dot{v}_{r2} &=  \frac{1}{K_0 T_3} \left[ \left( 1 - \frac{T_4}{T_3} \right) \left( v_{r1} + K_0\frac{T_2}{T_1}(v_{\text{ref}} - v_m)\right) - K_0 v_{r2} \right]  \tag{3c} \\
\dot{v}_m &= \frac{1}{T_r} (v_h - v_m) \tag{3d}
\end{align}
```

with

```math
\begin{align*}
v_r &= K_0v_{r2} + \frac{T_4}{T_3} \left( v_{r1} + K_0\frac{T_2}{T_1}(v_{\text{ref}} - v_m)\right) \\
S_e(v_f) &= A_e \exp(B_e|v_f|)
\end{align*}
```

## Excitation System AC1A [`ESAC1A`](@ref)

The model represents the 5-states IEEE Type AC1A Excitation System Model:

```math
\begin{align}
\dot{V}_m &= \frac{1}{T_r} (V_{h} - V_m) \tag{4a} \\
\dot{V}_{r1} &= \frac{1}{T_b} \left(V_{in} \left(1 - \frac{T_c}{T_b}\right) - V_{r1}\right) \tag{4b} \\
\dot{V}_{r2} &= \frac{1}{T_a} (K_a V_{out} - V_{r2}) \tag{4c} \\
\dot{V}_e &= \frac{1}{T_e} (V_r - V_{FE}) \tag{4d} \\
\dot{V}_{r3} &= \frac{1}{T_f} \left( - \frac{K_f}{T_f}V_{FE} - V_{r3} \right) \tag{4e} \\
\end{align}
```

with

```math
\begin{align*}
I_N &= \frac{K_c}{V_e} X_{ad}I_{fd} \\
V_{FE} &= K_d X_{ad}I_{fd} + K_e V_e + S_e V_e \\
S_e &= B\frac{(V_e-A)^2}{V_e} \\
V_{F1} &= V_{r3} + \frac{K_f}{T_f} V_{FE} \\
V_{in} &= V_{ref} - V_m - V_{F1} \\
V_{out} &= V_{r1} + \frac{T_c}{T_b} V_{in} \\
V_f &= V_e f(I_N) \\
f(I_N) &= \left\{\begin{array}{cl}
    1 & \text{ if }I_N \le 0 \\
    1 - 0.577 I_N & \text{ if } 0 < I_N \le 0.433 \\
    \sqrt{0.75 - I_N^2} & \text{ if } 0.433 < I_N \le 0.75 \\
    1.732(1-I_N) & \text{ if } 0.75 <  I_N \le 1 \\
    0 & \text{ if } I_N > 1 \end{array} \right.
\end{align*}
```

on which ``X_{ad}I_{fd}`` is the field current coming from the generator and ``V_{h}`` is the terminal voltage, and ``A,B`` are the saturation coefficients computed using the ``E_1, E_2, S_e(E_1), S_e(E_2)`` data.

## Simplified Excitation System [`SEXS`](@ref)

The model for the 2 states excitation system SEXS:

```math
\begin{align}
\dot{V}_f &= \frac{1}{T_e} (V_{LL} - V_f) \tag{5a} \\
\dot{V}_r &= \frac{1}{T_b} \left[\left(1 - \frac{T_a}{T_b}\right) V_{in} - V_r \right] \tag{5b}
\end{align}
```

with

```math
\begin{align*}
V_{in} &= V_{ref} + V_s - V_h \\
V_{LL} &= V_r + \frac{T_a}{T_b}V_{in} \\
\end{align*}
```

on which ``V_h`` is the terminal voltage and ``V_s`` is the PSS output signal.

## Excitation System ST1 [`EXST1`](@ref)

The model represents the 4-states IEEE Type ST1 Excitation System Model:

```math
\begin{align}
\dot{V}_m &= \frac{1}{T_r} (V_{h} - V_m) \tag{6a} \\
\dot{V}_{rll} &= \frac{1}{T_b} \left(V_{in} \left(1 - \frac{T_c}{T_b}\right) - V_{rll}\right) \tag{6b} \\
\dot{V}_{r} &= \frac{1}{T_a} (V_{LL} - V_{r}) \tag{6c} \\
\dot{V}_{fb} &= \frac{1}{T_f} \left( - \frac{K_f}{T_f}V_{r} - V_{fb} \right) \tag{6d} \\
\end{align}
```

with

```math
\begin{align*}
V_{in} &= V_{ref} - V_m - y_{hp} \\
V_{LL} &= V_{r} + \frac{T_c}{T_b} V_{in} \\
y_{hp} &= V_{fb} + \frac{K_f}{T_f} V_r \\
V_f &= V_r \\
\end{align*}
```

on which ``V_h`` is the terminal voltage.

## Excitation System EXAC1 [`EXAC1`](@ref)

The model represents the 5-states IEEE Type EXAC1 Excitation System Model:

```math
\begin{align}
\dot{V}_m &= \frac{1}{T_r} (V_{h} - V_m) \tag{7a} \\
\dot{V}_{r1} &= \frac{1}{T_b} \left(V_{in} \left(1 - \frac{T_c}{T_b}\right) - V_{r1}\right) \tag{7b} \\
\dot{V}_{r2} &= \frac{1}{T_a} (K_a V_{out} - V_{r2}) \tag{7c} \\
\dot{V}_e &= \frac{1}{T_e} (V_r - V_{FE}) \tag{7d} \\
\dot{V}_{r3} &= \frac{1}{T_f} \left( - \frac{K_f}{T_f}V_{FE} - V_{r3} \right) \tag{7e} \\
\end{align}
```

with

```math
\begin{align*}
I_N &= \frac{K_c}{V_e} X_{ad}I_{fd} \\
V_{FE} &= K_d X_{ad}I_{fd} + K_e V_e + S_e V_e \\
S_e &= B\frac{(V_e-A)^2}{V_e} \\
V_{F1} &= V_{r3} + \frac{K_f}{T_f} V_{FE} \\
V_{in} &= V_{ref} - V_m - V_{F1} \\
V_{out} &= V_{r1} + \frac{T_c}{T_b} V_{in} \\
V_f &= V_e f(I_N) \\
f(I_N) &= \left\{\begin{array}{cl}
    1 & \text{ if }I_N \le 0 \\
    1 - 0.577 I_N & \text{ if } 0 < I_N \le 0.433 \\
    \sqrt{0.75 - I_N^2} & \text{ if } 0.433 < I_N \le 0.75 \\
    1.732(1-I_N) & \text{ if } 0.75 <  I_N \le 1 \\
    0 & \text{ if } I_N > 1 \end{array} \right.
\end{align*}
```

on which ``X_{ad}I_{fd}`` is the field current coming from the generator and ``V_{h}`` is the terminal voltage, and ``A,B`` are the saturation coefficients computed using the ``E_1, E_2, S_e(E_1), S_e(E_2)`` data.

## Excitation System ST8C [`ST8C`](@ref)

The model represents the 5-states IEEE Type ST8C Excitation System Model:

```math
\begin{align}
T_r \dot{V}_m &= V_{h} - V_m \tag{8a} \\
\dot{x}_{a1} &= \begin{cases}
V_{\pi_{in}} & \text{if } V_{\pi_{min}} < K_{pr} V_{\pi_{in}} + K_{ir} x_{a1} < V_{\pi_{max}} \\
0 & \text{otherwise}
\end{cases} \tag{8b} \\
\dot{x}_{a2} &= \begin{cases}
I_{fd_{diff}} & \text{if } V_{a_{min}} < K_{pa} I_{fd_{diff}} + K_{ia} x_{a2} < V_{a_{max}} \\
0 & \text{otherwise}
\end{cases} \tag{8c} \\
T_a \dot{x}_{a3} &= \begin{cases}
K_a \pi_{out} - x_{a3} & \text{if } V_{r_{min}} < x_{a3} < V_{r_{max}} \\
0 & \text{otherwise}
\end{cases} \tag{8d} \\
T_f \dot{x}_{a4} &= K_f I_{fd} - x_{a4} \tag{8e} \\
\end{align}
```

with

```math
\begin{align*}
V_{\pi_{in}} &= V_{ref} + V_s - V_m \\
I_{fd_{ref}} &= \text{clamp}(K_{pr} V_{\pi_{in}} + K_{ir} x_{a1}, V_{\pi_{min}}, V_{\pi_{max}}) \\
I_{fd_{diff}} &= I_{fd_{ref}} - x_{a4} \\
\pi_{out} &= \text{clamp}(K_{pa} I_{fd_{diff}} + K_{ia} x_{a2}, V_{a_{min}}, V_{a_{max}}) \\
I_{N1} &= \frac{K_{c1} I_{fd}}{V_e} \\
F_{ex} &= f(I_{N1}) \\
V_{b1} &= \min(F_{ex} V_e, V_{B1_{max}}) \\
V_{b2} &= 0 \quad \text{(feedforward current not implemented)} \\
E_{fd} &= V_{b1} x_{a3} + V_{b2} \\
f(I_N) &= \left\{\begin{array}{cl}
    1 & \text{ if }I_N \le 0 \\
    1 - 0.577 I_N & \text{ if } 0 < I_N \le 0.433 \\
    \sqrt{0.75 - I_N^2} & \text{ if } 0.433 < I_N \le 0.75 \\
    1.732(1-I_N) & \text{ if } 0.75 <  I_N \le 1 \\
    0 & \text{ if } I_N > 1 \end{array} \right.
\end{align*}
```

on which ``I_{fd}`` is the field current from the generator, ``V_{h}`` is the terminal voltage, and ``V_s`` is the PSS output signal.

### Notes

  - The rectifier function ``f(I_N)`` models the characteristic of a three-phase full-wave rectifier
  - Terminal current feedforward (``K_{i2}``) is not currently implemented and must be set to 0.0
  - Voltage compensation is not implemented (``V_e = K_p``)

## Excitation System ST6B [`ST6B`](@ref)

The model represents the 4-states IEEE Type ST6B Excitation System Model:

```math
\begin{align}
T_r \dot{V}_m &= V_{h} - V_m \tag{9a} \\
\dot{x}_{i} &= \begin{cases}
V_{\pi_{in}} & \text{if } V_{a_{min}} < K_{pa} V_{\pi_{in}} + K_{ia} x_{i} < V_{a_{max}} \\
0 & \text{otherwise}
\end{cases} \tag{9b} \\
T_{da} \dot{x}_{d} &= -K_{da} V_{\pi_{in}} - x_{d} \tag{9c} \\
\dot{V}_{g} &= \frac{1}{T_g} (K_g E_{fd} - V_g) \tag{9d} \\
\end{align}
```

with

```math
\begin{align*}
V_{\pi_{in}} &= V_{ref} + V_s - V_m \\
\text{PI output} &= \text{clamp}(K_{pa} V_{\pi_{in}} + K_{ia} x_{i}, V_{a_{min}}, V_{a_{max}}) \\
\text{PD output} &= x_{d} + \frac{K_{da}}{T_{da}} V_{\pi_{in}} \\
V_a &= \text{PI output} + \text{PD output} \\
\text{FF output} &= (V_a - V_g) K_m + K_{ff} V_a \\
V_{r1} &= \max((I_{lr} K_{ci} - X_{ad} I_{fd}) K_{lr}, V_{r_{min}}) \\
V_{r2} &= \text{clamp}(\text{FF output}, V_{r_{min}}, V_{r_{max}}) \\
V_r &= \min(V_{r1}, V_{r2}) \\
E_{fd} &= V_r V_m \\
\end{align*}
```

on which ``X_{ad} I_{fd}`` is the field current from the generator, ``V_{h}`` is the terminal voltage, and ``V_s`` is the PSS output signal. Finally, the derivative block implements a high-pass filter:

```math
\text{PD output} = x_d + \frac{K_{da}}{T_{da}} u, \quad T_{da} \dot{x}_d = -\frac{K_{da}}{T_{da}} u - x_d
```

while the current limiter ensures:

```math
V_{r1} = \max((I_{lr} K_{ci} - X_{ad} I_{fd}) K_{lr}, V_{r_{min}})
```

### Notes

  - The derivative term uses a high-pass filter implementation
  - Current limiting is active when field current exceeds the reference value ``I_{lr}``
  - The exciter output is the minimum of voltage-limited and current-limited values
  - During initialization, ``I_{lr}`` is typically adjusted to match operating conditions
