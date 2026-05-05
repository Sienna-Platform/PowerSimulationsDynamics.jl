# Frequency Estimators

This component is used to estimate the frequency of the grid based on the voltage at the bus.

## Fixed Frequency [`FixedFrequency`](@ref)

This is a simple model that set the measured frequency to a desired constant value (i.e. does not measure the frequency)
``\omega_{pll} = \omega_{\text{fix}}`` (usually ``\omega_{\text{fix}} = 1.0`` p.u.). Used by default when grid-forming
inverters do not use frequency estimators.

## Phase-Locked Loop (PLL) for VSM [`KauraPLL`](@ref)

The following equations present a PLL used to estimate the frequency and PLL angle of
the grid. There are two reference frames considered in this inverter. Those are the VSM
of the outer-loop control ``\theta_{\text{olc}}`` and the PLL one
``\theta_{\text{pll}}``. The notation used a ``\delta\theta`` refers as the variation
of the respective angle ``\theta^\text{grid}`` with respect to the grid SRF (instead of the fixed
``\alpha`` component of the ``\alpha\beta`` transformation):

```math
\begin{align}

\dot{v}_{d,\text{pll}} &= \omega_{\text{lp}} \left [v_{d,\text{out}} - v_{d,\text{pll}} \right] \tag{1a} \\
\dot{v}_{q,\text{pll}} &= \omega_{\text{lp}} \left [v_{q,\text{out}} - v_{q,\text{pll}} \right] \tag{1b} \\
\dot{\varepsilon}_{\text{pll}} &= \tan^{-1}\left(\frac{v_{q,\text{pll}}}{v_{d,\text{pll}}} \right) \tag{1c} \\
\dot{\theta}_{\text{pll}} &= \Omega_b \delta \omega_{\text{pll}} \tag{1d}
\end{align}
```

with

```math
\begin{align}
\delta\omega_{\text{pll}} &= 1.0 - \omega_{\text{sys}} + k_{p,\text{pll}} \tan^{-1} \left(\frac{v_{q,\text{pll}}}{v_{d,\text{pll}}} \right) + k_{i,\text{pll}} \varepsilon_{\text{pll}} \tag{1e} \\
\omega_{\text{pll}} &= \delta\omega_{\text{pll}} + \omega_{\text{sys}} \tag{1f} \\
v_{d,\text{out}} + jv_{q,\text{out}} &= (v_r + jv_i)e^{-\delta\theta_\text{pll}}  \tag{1g}
\end{align}
```

on which ``v_r + jv_i`` is the voltage in the grid reference frame on which the PLL is
measuring (i.e. point of common coupling), that could be in the capacitor of an LCL filter
or the last branch of such filter.

## Reduced Order Phase-Locked Loop (PLL) [`ReducedOrderPLL`](@ref)

The following equations presents a simplified PLL used to estimate the frequency and PLL angle of the grid. The model attempts to steer the voltage in the q-axis to zero (i.e. lock the q-axis to zero) using a PI controller. With that the equations are given by:

```math
\begin{align}
\dot{v}_{q,\text{pll}} &= \omega_{\text{lp}} \left [v_{q,\text{out}} - v_{q,\text{pll}} \right] \tag{2a} \\
\dot{\varepsilon}_{\text{pll}} &= v_{q,\text{pll}} \tag{2b} \\
\dot{\theta}_{\text{pll}} &= \Omega_b \delta \omega_{\text{pll}} \tag{2c}
\end{align}
```

with

```math
\begin{align}
\delta\omega_{\text{pll}} &= 1.0 - \omega_{\text{sys}} + k_{p,\text{pll}} v_{q,\text{pll}} + k_{i,\text{pll}} \varepsilon_{\text{pll}} \tag{2d} \\
\omega_{\text{pll}} &= \delta\omega_{\text{pll}} + \omega_{\text{sys}} \tag{2e} \\
v_{d,\text{out}} + jv_{q,\text{out}} &= (v_r + jv_i)e^{-\delta\theta_\text{pll}}  \tag{2f}
\end{align}
```

on which ``v_r + jv_i`` is the voltage in the grid reference frame on which the PLL is
measuring (i.e. point of common coupling), that could be in the capacitor of an LCL filter
or the last branch of such filter.
