# Power System Stabilizers (PSS)

PSS are used to add an additional signal ``v_s`` to the input signal of the AVR: ``v_\text{ref} = v_\text{ref}^{\text{avr}} + v_s``.

## Fixed PSS [`PowerSystems.PSSFixed`](@extref)

This is a simple model that set the stabilization signal to be equal to a desired constant value ``v_s = v_{s}^{\text{fix}}``. The absence of PSS can be modelled using this component with ``v_s^{\text{fix}} = 0``.

## Simple PSS [`PowerSystems.PSSSimple`](@extref)

This is the most basic PSS that can be implemented, on which the stabilization signal is  a proportional controller over the frequency and electrical power:

```math
\begin{align}
v_s = K_{\omega}(\omega - \omega_s) + K_p(\omega \tau_e - P_{\text{ref}}) \tag{1a}
\end{align}
```

## IEEE Stabilizer [`PowerSystems.IEEEST`](@extref)

The 7th-order PSS model is:

```math
\begin{align}
A_4 \dot{x}_1 &= u - A_3 x_1 - x_2  \tag{2a} \\
\dot{x}_2 &= x_1 \tag{2b} \\
A_2\dot{x}_3 &= x_2 - A_1 x_3 - x_4 \tag{2c}\\
\dot{x}_4 &= x_3 \tag{2d}\\
T_2\dot{x}_5 &= \left(1 - \frac{T_1}{T_2}\right) y_f - x_5 \tag{2e}\\
T_4\dot{x}_6 &= \left(1 - \frac{T_3}{T_4}\right) y_{LL1} - x_6 \tag{2f}\\
T_6\dot{x}_7 &=  -\left(\frac{K_s T_5}{T_6} y_{LL2} + x_7 \right) \tag{2g}
\end{align}
```

with

```math
\begin{align*}
y_f &= \frac{T_4}{T_2} x_2 + \left(T_3 - T_1 \frac{T_4}{T_2}\right) x_3 + \left(1 - \frac{T_4}{T_2}\right)x_4 \\
y_{LL1} &= x_5 + \frac{T_1}{T_2} y_f \\
y_{LL2} &= x_6 + \frac{T_3}{T_4} y_{LL1} \\
y_{out} &= x_7 + \frac{K_s T_5}{T_6} y_{LL2} \\
V_s &= \text{clamp}(y_{out}, \text{Ls}_\text{min}, \text{Ls}_\text{max})
\end{align*}
```

on which ``u`` is the input signal to the PSS, that depends on the flag. Currently, rotor speed, electric torque, mechanical torque and voltage magnitude are supported inputs.

## STAB1 PSS [`PowerSystems.STAB1`](@extref)

The 3rd-order PSS model is:

```math
\begin{align}
T \dot{x}_1 &= K \omega - x_1 \tag{3a} \\
T_3\dot{x}_2 &= \left(1 - \frac{T_1}{T_3}\right) x_1 - x_2 \tag{3b} \\
T_4\dot{x}_3 &= \left(1 - \frac{T_2}{T_4}\right) y_{LL} - x_2 \tag{3c} \\
\end{align}
```

with

```math
\begin{align*}
y_{LL} = x_2 + \frac{T_1}{T_3} x_1 \\
y_{out} = x_3 +  \frac{T_2}{T_4} y_{LL} \\
V_s =  \text{clamp}(y_{out}, -H_{lim}, H_{lim})
\end{align*}
```
