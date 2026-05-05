# Shafts

The shaft component defines the rotating mass of the synchronous generator.

## Rotor Mass Shaft [`SingleMass`](@ref)

This is the standard model, on which one single mass (typically the rotor) is used to model the entire inertia of the synchronous generator. Each generator's rotating frame use a reference frequency ``\omega_s``, that typically is the synchronous one (i.e. ``\omega_s = 1.0``). The model defines two differential equations for the rotor angle ``\delta`` and the rotor speed ``\omega``:

```math
\begin{align}
\dot{\delta} &= \Omega_b(\omega - \omega_s) \tag{1a} \\
\dot{\omega} &= \frac{1}{2H}(\tau_m - \tau_e - D(\omega-\omega_s)) \tag{1b}
\end{align}
```

## Five-Mass Shaft [`FiveMassShaft`](@ref)

This model describes model connecting a high-pressure (hp) steam turbine, intermediate-pressure (ip) steam turbine, low-pressure (lp) steam pressure, rotor and exciter (ex) connected in series (in that order) in the same shaft using a spring-mass model:

```math
\begin{align}
\dot{\delta} &= \Omega_b(\omega - \omega_s) \tag{2a} \\
\dot{\omega} &= \frac{1}{2H} \left[- \tau_e - D(\omega-\omega_s)) - D_{34} (\omega-\omega_{lp}) - D_{45}(\omega-\omega_{ex}) + K_{lp}(\delta_{lp-\delta}) +K_{ex}(\delta_{ex}-\delta) \right] \tag{2b} \\
\dot{\delta}_{hp} &= \Omega_b(\omega_{hp} - \omega_s) \tag{2c} \\
\dot{\omega}_{hp} &= \frac{1}{2H_{hp}} \left[ \tau_m - D_{hp}(\omega_{hp}-\omega_s) - D_{12}(\omega_{hp} - \omega_{ip}) + K_{hp}(\delta_{ip} - \delta_{hp}) \right] \tag{2d} \\
\dot{\delta}_{ip} &= \Omega_b(\omega_{ip} - \omega_s) \tag{2e} \\
\dot{\omega}_{ip} &= \frac{1}{2H_{ip}} \left[- D_{ip}(\omega_{ip}-\omega_s) - D_{12}(\omega_{ip} - \omega_{hp}) -D_{23}(\omega_{ip} - \omega_{lp} ) + K_{hp}(\delta_{hp} - \delta_{ip}) + K_{ip}(\delta_{lp}-\delta_{ip}) \right] \tag{2f} \\
\dot{\delta}_{lp} &= \Omega_b(\omega_{lp}-\omega_s) \tag{2g} \\
\dot{\omega}_{lp} &= \frac{1}{2H_{lp}} \left[ - D_{lp}(\omega_{lp}-\omega_s) - D_{23}(\omega_{lp} - \omega_{ip}) -D_{34}(\omega_{lp} - \omega ) + K_{ip}(\delta_{ip} - \delta_{lp}) + K_{lp}(\delta-\delta_{lp}) \right] \tag{2h} \\
\dot{\delta}_{ex} &= \Omega_b(\omega_{ex}-\omega_s) \tag{2i} \\
\dot{\omega}_{ex} &= \frac{1}{2H_{ex}} \left[ - D_{ex}(\omega_{ex}-\omega_s) - D_{45}(\omega_{ex} - \omega) + K_{ex}(\delta - \delta_{ex}) \right] \tag{2j}
\end{align}
```
