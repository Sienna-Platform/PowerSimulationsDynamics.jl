# Machines

The machine component describes the stator-rotor electromagnetic dynamics.

## Classical Model (Zero Order) [`BaseMachine`](@ref)

This is the classical order model that does not have differential equations in its machine model
(``\delta`` and ``\omega`` are defined in the shaft):

```math
\begin{align}
  \left[ \begin{array}{c} i_d \\ i_q \end{array} \right] &= \left[ \begin{array}{cc} r_a & -x_d' \\ x_d' & r_a \end{array} \right]^{-1}  \left[ \begin{array}{c} -v_d \\ e_q' - v_q \end{array} \right] \tag{1a}\\
p_e \approx \tau_e &= (v_q + r_a i_q)i_q + (v_d + r_ai_d)i_d \tag{1b}
\end{align}
```

## One d- One q- Model (2nd Order) [`OneDOneQMachine`](@ref)

This model includes two transient emf with their respective differential equations:

```math
\begin{align}
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[-e_q' + (x_d-x_d')i_d + v_f\right] \tag{2a}\\
\dot{e}_d' &= \frac{1}{T_{q0}'} \left[-e_d' + (x_q-x_q')i_q \right] \tag{2b}\\
  \left[ \begin{array}{c} i_d \\ i_q \end{array} \right] &= \left[ \begin{array}{cc} r_a & -x_q' \\ x_d' & r_a \end{array} \right]^{-1}  \left[ \begin{array}{c} e_d'-v_d \\ e_q' - v_q \end{array} \right] \tag{2c}\\
p_e \approx \tau_e &= (v_q + r_a i_q)i_q + (v_d + r_ai_d)i_d \tag{2d}
\end{align}
```

## Marconato Machine (6th Order) [`MarconatoMachine`](@ref)

The Marconato model defines 6 differential equations, two for stator fluxes and 4 for transient and subtransient emfs:

```math
\begin{align}
\dot{\psi}_d &= \Omega_b(r_ai_d + \omega \psi_q + v_d) \tag{3a} \\
\dot{\psi}_q &= \Omega_b(r_ai_q - \omega \psi_d + v_q) \tag{3b} \\
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[-e_q' - (x_d-x_d'-\gamma_d)i_d + \left(1- \frac{T_{AA}}{T_{d0}'} \right) v_f\right] \tag{3c}\\
\dot{e}_d' &= \frac{1}{T_{q0}'} \left[-e_d' + (x_q-x_q'-\gamma_q)i_q \right] \tag{3d}\\
\dot{e}_q'' &= \frac{1}{T_{d0}''} \left[-e_q'' + e_q' - (x_d'-x_d''+\gamma_d)i_d + \frac{T_{AA}}{T_{d0}'}v_f \right] \tag{3e} \\
\dot{e}_d'' &= \frac{1}{T_{q0}''} \left[-e_d'' + e_d' + (x_q'-x_q''+\gamma_q)i_q \right] \tag{3f} \\
i_d &= \frac{1}{x_d''} (e_q'' - \psi_d) \tag{3g} \\
i_q &= \frac{1}{x_q''} (-e_d'' - \psi_q) \tag{3h} \\
\tau_e &= \psi_d i_q - \psi_q i_d \tag{3i}
\end{align}
```

with

```math
\begin{align*}
  \gamma_d &= \frac{T_{d0}'' x_d''}{T_{d0}' x_d'} (x_d - x_d') \\
  \gamma_q &= \frac{T_{q0}'' x_q''}{T_{q0}' x_q'} (x_q - x_q')
\end{align*}
```

## Simplified Marconato Machine (4th Order) [`SimpleMarconatoMachine`](@ref)

This model neglects the derivative of stator fluxes (``\dot{\psi}_d`` and  ``\dot{\psi}_q``) and assume that the rotor speed stays close to 1 pu (``\omega\psi_{d}=\psi_{d}`` and ``\omega\psi_{q}=\psi_{q}``) that allows to remove the stator fluxes variables from the Marconato model.

```math
\begin{align}
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[-e_q' - (x_d-x_d'-\gamma_d)i_d + \left(1- \frac{T_{AA}}{T_{d0}'} \right) v_f\right] \tag{4a}\\
\dot{e}_d' &= \frac{1}{T_{q0}'} \left[-e_d' + (x_q-x_q'-\gamma_q)i_q \right] \tag{4b}\\
\dot{e}_q'' &= \frac{1}{T_{d0}''} \left[-e_q'' + e_q' - (x_d'-x_d''+\gamma_d)i_d + \frac{T_{AA}}{T_{d0}'}v_f \right] \tag{4c} \\
\dot{e}_d'' &= \frac{1}{T_{q0}''} \left[-e_d'' + e_d' + (x_q'-x_q''+\gamma_q)i_q \right] \tag{4d} \\
\left[ \begin{array}{c} i_d \\ i_q \end{array} \right] &= \left[ \begin{array}{cc} r_a & -x_q'' \\ x_d'' & r_a \end{array} \right]^{-1}  \left[ \begin{array}{c} e_d''-v_d \\ e_q'' - v_q \end{array} \right] \tag{4e}\\
p_e \approx \tau_e &= (v_q + r_a i_q)i_q + (v_d + r_ai_d)i_d \tag{4f}
\end{align}
```

with

```math
\begin{align*}
  \gamma_d &= \frac{T_{d0}'' x_d''}{T_{d0}' x_d'} (x_d - x_d') \\
  \gamma_q &= \frac{T_{q0}'' x_q''}{T_{q0}' x_q'} (x_q - x_q')
\end{align*}
```

## Anderson-Fouad Machine (6th Order) [`AndersonFouadMachine`](@ref)

The Anderson-Fouad model also defines 6 differential equations, two for stator fluxes and 4 for transient and subtransient emfs and is derived from the Marconato model by defining ``\gamma_d \approx \gamma_q \approx T_{AA} \approx 0``:

```math
\begin{align}
\dot{\psi}_d &= \Omega_b(r_ai_d + \omega \psi_q + v_d) \tag{5a} \\
\dot{\psi}_q &= \Omega_b(r_ai_q - \omega \psi_d + v_q) \tag{5b} \\
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[-e_q' - (x_d-x_d')i_d + v_f\right] \tag{5c}\\
\dot{e}_d' &= \frac{1}{T_{q0}'} \left[-e_d' + (x_q-x_q')i_q \right] \tag{5d}\\
\dot{e}_q'' &= \frac{1}{T_{d0}''} \left[-e_q'' + e_q' - (x_d'-x_d'')i_d \right] \tag{5e} \\
\dot{e}_d'' &= \frac{1}{T_{q0}''} \left[-e_d'' + e_d' + (x_q'-x_q'')i_q \right] \tag{5f} \\
i_d &= \frac{1}{x_d''} (e_q'' - \psi_d) \tag{5g} \\
i_q &= \frac{1}{x_q''} (-e_d'' - \psi_q) \tag{5h} \\
\tau_e &= \psi_d i_q - \psi_q i_d \tag{5i}
\end{align}
```

## Simplified Anderson-Fouad Machine (4th Order) [`SimpleAFMachine`](@ref)

Similar to the Simplified Marconato Model, this model neglects the derivative of stator fluxes (``\dot{\psi}_d`` and  ``\dot{\psi}_q``) and assume that the rotor speed stays close to 1 pu (``\omega \psi_d = \psi_d`` and ``\omega \psi_q = \psi_q``) that allows to remove the stator fluxes variables from the model:

```math
\begin{align}
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[-e_q' - (x_d-x_d')i_d + v_f\right] \tag{6a}\\
\dot{e}_d' &= \frac{1}{T_{q0}'} \left[-e_d' + (x_q-x_q')i_q \right] \tag{6b}\\
\dot{e}_q'' &= \frac{1}{T_{d0}''} \left[-e_q'' + e_q' - (x_d'-x_d'')i_d \right] \tag{6c} \\
\dot{e}_d'' &= \frac{1}{T_{q0}''} \left[-e_d'' + e_d' + (x_q'-x_q'')i_q \right] \tag{6d} \\
\left[ \begin{array}{c} i_d \\ i_q \end{array} \right] &= \left[ \begin{array}{cc} r_a & -x_q'' \\ x_d'' & r_a \end{array} \right]^{-1}  \left[ \begin{array}{c} e_d''-v_d \\ e_q'' - v_q \end{array} \right] \tag{6e}\\
p_e \approx \tau_e &= (v_q + r_a i_q)i_q + (v_d + r_ai_d)i_d \tag{6f}
\end{align}
```

## Round Rotor Machine (4th Order) [`RoundRotorQuadratic`](@ref), [`RoundRotorExponential`](@ref)

This model represents the traditional round rotor models GENROU/GENROE models implemented in PSLF/PSSE/PowerWorld.
Similar to the Simplified Marconato Model, this model neglects the derivative of stator fluxes (``\dot{\psi}_d`` and  ``\dot{\psi}_q``). Round rotor machines must satisfy ``x_d'' = x_q''``.

```math
\begin{align}
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[v_f - X_{ad}I_{fd}\right] \tag{7a}\\
\dot{e}_d' &= \frac{1}{T_{q0}'} \left[-X_{aq}I_{1q} \right] \tag{7b}\\
\dot{\psi}_{kd} &= \frac{1}{T_{d0}''} \left[-\psi_{kd} + e_q' - (x_d'-x_l)i_d \right] \tag{7c} \\
\dot{\psi}_{kq} &= \frac{1}{T_{q0}''} \left[-\psi_{kq} + e_d' + (x_q'-x_l)i_q \right] \tag{7d} \\
\end{align}
```

with:

```math
\begin{align}
\gamma_{d1} &= \frac{x_d'' - x_l}{x_d' - x_l} \tag{7e}\\
\gamma_{q1} &= \frac{x_q'' - x_l}{x_q' - x_l} \tag{7f}\\
\gamma_{d2} &= \frac{x_d' - x_d''}{(x_d'-x_l)^2} \tag{7g}\\
\gamma_{q2} &= \frac{x_q' - x_q''}{(x_q' - x_l)^2} \tag{7h}\\
\gamma_{qd} &= \frac{x_q - x_l}{x_d - x_l} \tag{7i}\\
\psi_q'' &= \gamma_{q1} e_d' + \psi_{kq} (1 - \gamma_{q1}) \tag{7j}\\
\psi_d'' &= \gamma_{d1} e_q' + \gamma_{d2} (x_d' - x_l) \psi_{kd} \tag{7k}\\
\psi'' &= \sqrt{(\psi_d'')^2 + (\psi_q'')^2} \tag{7l}\\
\left[ \begin{array}{c} i_d \\ i_q \end{array} \right] &= \left[ \begin{array}{cc} -r_a & x_q'' \\ -x_d'' & r_a \end{array} \right]^{-1}  \left[ \begin{array}{c} v_d - \psi_q'' \\ -v_q + \psi_d'' \end{array} \right] \tag{7m}\\
X_{ad}I_{fd} &= e_q' + (x_d - x_d') (\gamma_{d1} i_d - \gamma_{d2} \psi_{kd} + \gamma_{d2} + e_q') + \text{Se}(\psi'') \psi_d'' \tag{7n}\\
X_{aq}I_{1q} &= e_d' + (x_q - x_q') (\gamma_{q2} e_d' - \gamma_{q2}\psi_{kq} - \gamma_{q1} i_q) + \text{Se}(\psi'') \psi_q'' \gamma_{qd} \tag{7o} \\
\tau_e &= i_d (r_a i_d + v_d) + i_q(r_a i_q + v_q) \tag{7p}
\end{align}
```

The difference between GENROU and GENROE occurs in which additive saturation function ``\text{Se}(\psi'')`` is used. Input data is provided by the saturation values at ``\psi'' = 1.0`` and ``\psi'' = 1.2`` p.u. For the GENROU model, the function used is:

```math
\begin{align}
\text{Se}(\psi'') &= \frac{B(\psi'' - A)^2 }{\psi''} \tag{7q}
\end{align}
```

and for the GENROE model the function used is:

```math
\begin{align}
\text{Se}(\psi'') &= B(\psi'')^A \tag{7r}
\end{align}
```

The parameters ``A`` and ``B`` for each function are computed using the two points given
``(1.0, \text{Se}(1.0))`` and ``(1.2, \text{Se}(1.2))``.

## Salient Pole Machine (3rd Order) [`SalientPoleQuadratic`](@ref), [`SalientPoleExponential`](@ref)

This model represents the traditional round rotor models GENSAL/GENSAE models implemented in PSLF/PSSE/PowerWorld.
Similar to the GENROU Model, this model neglects the derivative of stator fluxes (``\dot{\psi}_d`` and  ``\dot{\psi}_q``).

```math
\begin{align}
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[v_f - X_{ad}I_{fd}\right] \tag{8a}\\
\dot{\psi}_{kd} &= \frac{1}{T_{d0}''} \left[-\psi_{kd} + e_q' - (x_d'-x_l)i_d \right] \tag{8b} \\
\dot{\psi}_{q}'' &= \frac{1}{T_{q0}''} \left[-\psi_{q}'' - (x_q-x_q'')i_q \right] \tag{8c} \\
\end{align}
```

with:

```math
\begin{align}
\gamma_{d1} &= \frac{x_d'' - x_l}{x_d' - x_l} \tag{8d}\\
\gamma_{q1} &= \frac{x_q'' - x_l}{x_q' - x_l} \tag{8e}\\
\gamma_{d2} &= \frac{x_d' - x_d''}{(x_d'-x_l)^2} \tag{8f}\\
\psi_d'' &= \gamma_{d1} e_q' + \gamma_{q1} \psi_{kd} \tag{8g}\\
\left[ \begin{array}{c} i_d \\ i_q \end{array} \right] &= \left[ \begin{array}{cc} -r_a & x_q'' \\ -x_d'' & r_a \end{array} \right]^{-1}  \left[ \begin{array}{c} v_d - \psi_q'' \\ -v_q + \psi_d'' \end{array} \right] \tag{8h}\\
X_{ad}I_{fd} &= e_q' + \text{Se}(e_q') e_q' + (x_d - x_d') (i_d + \gamma_{d2} (e_q' - \psi_{kd} - (x_d' - x_l)i_d) \tag{8i}\\
\tau_e &= i_d (r_a i_d + v_d) + i_q(r_a i_q + v_q) \tag{8j}
\end{align}
```

The difference between GENSAL and GENSAE occurs in which additive saturation function ``\text{Se}(e_q')`` is used. Input data is provided by the saturation values at ``e_q' = 1.0`` and ``e_q' = 1.2`` p.u. For the GENSAL model, the function used is:

```math
\begin{align}
\text{Se}(e_q') &= \frac{B(e_q' - A)^2 }{\e_q'} \tag{8k}
\end{align}
```

and for the GENSAE model the function used is:

```math
\begin{align}
\text{Se}(e_q') &= B(e_q')^A \tag{8l}
\end{align}
```

The parameters ``A`` and ``B`` for each function are computed using the two points given
``(1.0, \text{Se}(1.0))`` and ``(1.2, \text{Se}(1.2))``.

## SauerPai Machine (6th Order) [`SauerPaiMachine`](@ref)

The Sauer Pai model defines 6 differential equations as follows:

```math
\begin{align}
\dot{\psi}_d &= \Omega_b(r_ai_d + \omega \psi_q + v_d) \tag{9a} \\
\dot{\psi}_q &= \Omega_b(r_ai_q - \omega \psi_d + v_q) \tag{9b} \\
\dot{e}_q' &= \frac{1}{T_{d0}'} \left[(-e_q' - (x_d - x_d')(i_d + \gamma_{d2} \cdot  \dot{\psi}_d'') + v_f)\right] \tag{9c}\\
\dot{e}_q' &= \frac{1}{T_{q0}'} \left[(-e_d' + (x_q - x_q')(i_q + \gamma_{q2} \cdot \dot{\psi}_q''))\right] \tag{9d}\\
\dot{\psi}_d'' &= \frac{1}{T_{d0}''} \left[(-\psi_d'' + e_q' - (x_d' - x_l)\cdot i_d)\right] \tag{9e} \\
\dot{\psi}_q'' &= \frac{1}{T_{q0}''} \left[(-\psi_q'' - e_d' - (x_q' - x_l)\cdot i_q)\right]  \tag{9f} \\
i_d &= \frac{1}{x_d''} (\gamma_{d1} \cdot  e_q' - \psi_d + (1 - \gamma_{d1}) * \psi_d'') \tag{9g} \\
i_q &= \frac{1}{x_q''} ((-\gamma_{q1} \cdot  e_d' - \psi_q + (1 - \gamma_{q1}) \cdot  \psi_q'') \tag{9h} \\
\tau_e &= \psi_d i_q - \psi_q i_d \tag{9i}
\end{align}
```

with

```math
\begin{align*}
  \gamma_{d1} &= \frac{x_d'' - x_l}{x_d' - x_l} \\
  \gamma_{q1} &= \frac{x_q'' - x_l}{x_q' - x_l} \\
  \gamma_{d2} &= \frac{1 - \gamma_d1}{x_d' - x_l} \\
  \gamma_{q2} &= \frac{1 - \gamma_q1}{x_q' - x_l}
\end{align*}
```
