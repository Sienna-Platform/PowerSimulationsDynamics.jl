Here we discuss the models used to describe load modeling in `PowerSimulationsDynamics.jl`.
In a similar fashion of other devices, loads will withdraw power (i.e. current) from the current-injection balances at the nodal level. Based on the specified parameters and model chosen, the equations for computing such withdrawal will change.

## Static Loads (or Algebraic Loads)

### ZIP + Exponential Load Model

`PowerSimulationsDynamics.jl` uses all the static ZIP and exponential loads at each bus to obtain a single structure that creates an aggregate ZIP load model and a collection of all exponential loads.

The ZIP load model given by:

```math
\begin{align}
P_\text{zip} &= P_\text{power} + P_\text{current} \cdot \frac{V}{V_0} + P_\text{impedance} \cdot \left(\frac{V}{V_0}\right)^2 \tag{1a}\\
Q_\text{zip} &= Q_\text{power} + Q_\text{current}  \cdot \frac{V}{V_0} + Q_\text{impedance} \cdot \left(\frac{V}{V_0}\right)^2\tag{1b}
\end{align}
```

with ``V = \sqrt{V_r^2 + V_i^2}`` and ``V_0`` the voltage magnitude from the power flow solution.

The current taken for the load is computed as:

```math
\begin{align}
I_\text{zip} &= \frac{(P_\text{zip} + j Q_\text{zip})^*}{(V_r + j V_i)^*}\tag{1c} \\
I_\text{zip} &= \frac{P_\text{zip} - j Q_\text{zip}}{V_r - j V_i}\tag{1d}
\end{align}
```

For constant impedance load, the current obtained is:

```math
\begin{align}
I_\text{re}^z &= \frac{1}{V_0^2} \cdot (V_r \cdot P_\text{impedance} + V_i \cdot Q_\text{impedance})\tag{1e} \\
I_\text{im}^z &= \frac{1}{V_0^2} \cdot (V_i \cdot P_\text{impedance} - V_r \cdot Q_\text{impedance})\tag{1f}
\end{align}
```

For constant current load, the current obtained is:

```math
\begin{align}
I_\text{re}^i  &= \frac{1}{V_0} \cdot \frac{V_r * P_\text{current} + V_i * Q_\text{current}}{V} \tag{1g}\\
I_\text{im}^i  &= \frac{1}{V_0} \cdot \frac{V_i * P_\text{current} - V_r * Q_\text{current}}{V} \tag{1h}
\end{align}
```

For constant power load, the current obtained is:

```math
\begin{align}
I_\text{re}^p  =  \frac{V_r \cdot P_\text{power} + V_i \cdot Q_\text{power}}{V^2} \tag{1i} \\
I_\text{im}^p =  \frac{V_i \cdot P_\text{power} - V_r \cdot Q_\text{power}}{V^2} \tag{1j}
\end{align}
```

Then the total current withdrawed from the ZIP load model is simply

```math
\begin{align}
I_\text{zip}^\text{re}  &=  I_\text{re}^z + I_\text{re}^i + I_\text{re}^p \tag{1k} \\
I_\text{zip}^\text{im}  &=  I_\text{im}^z + I_\text{im}^i + I_\text{im}^p \tag{1l}
\end{align}
```

On the case of Exponential Loads, the model is given by:

```math
\begin{align}
P_\text{exp} = P_0 \cdot \left(\frac{V}{V_0}\right)^\alpha \tag{1m}\\
Q_\text{exp} = Q_0 \cdot \left(\frac{V}{V_0}\right)^\beta \tag{1n}
\end{align}
```

The current taken for the load is computed as:

```math
\begin{align}
I_\text{exp} &= \frac{(P_\text{exp} + j Q_\text{exp})^*}{(V_r + j V_i)^*} \tag{1o} \\
I_\text{exp} &= \frac{P_\text{exp} - j Q_\text{exp}}{V_r - j V_i} \tag{1p}
\end{align}
```

that results:

```math
\begin{align}
I_\text{exp}^\text{re}  &= V_r \cdot P_0 \cdot \frac{V^{\alpha - 2}}{V_0^\alpha} + V_i \cdot Q_0 \cdot \frac{V^{\beta - 2}}{V_0^\beta} \tag{1q}\\
I_\text{exp}^\text{im}  &= V_i \cdot P_0 \cdot \frac{V^{\alpha - 2}}{V_0^\alpha} - V_r \cdot Q_0 \cdot \frac{V^{\beta - 2}}{V_0^\beta} \tag{1r}
\end{align}
```

## Dynamic loads

### 5th-order Single Cage Induction Machine [`PowerSystems.SingleCageInductionMachine`](@extref)

The following model is used to model a 5th-order induction machine with a quadratic relationship speed-torque.
Refer to "Analysis of Electric Machinery and Drive Systems" by Paul Krause, Oleg Wasynczuk and Scott Sudhoff for the equations derivation

```math
\begin{align}
\dot{\psi}_{qs} &= \Omega_b (v_{qs} - \omega_\text{sys} \psi_{ds} - R_s i_{qs}) \tag{2a}\\
\dot{\psi}_{ds} &= \Omega_b (v_{ds} + \omega_\text{sys} \psi_{qs} - R_s i_{ds}) \tag{2b} \\
\dot{\psi}_{qr} &= \Omega_b \left(v_{qr} - (\omega_\text{sys} - \omega_r) \psi_{dr} + \frac{R_r}{X_{lr}} (\psi_{mq} - \psi_{qr})\right) \tag{2c}\\
\dot{\psi}_{dr} &= \Omega_b \left(v_{dr} + (\omega_\text{sys} - \omega_r) \psi_{qr} + \frac{R_r}{X_{lr}} (\psi_{md} - \psi_{dr})\right) \tag{2d}\\
\dot{\omega}_r &= \frac{1}{2H} (\tau_e - \tau_{m0}(A \omega_r^2 + B \omega_r + C)) \tag{2e}
\end{align}
```

where:

```math
\begin{align*}
X_{ad} = X_{aq} &= \left(\frac{1}{X_m} + \frac{1}{X_{ls}} + \frac{1}{X_{lr}}\right)^{-1} \\
v_{qs} &= V_i^\text{bus} \\
v_{ds} &= V_r^\text{bus} \\
v_{qr} = v_{dr} &= 0 \\
\psi_{mq} &= X_{aq} \left(\frac{\psi_{qs}}{X_{ls}}+ \frac{\psi_{qr}}{X_{lr}}\right) \\
\psi_{md} &= X_{ad} \left(\frac{\psi_{ds}}{X_{ls}}+ \frac{\psi_{dr}}{X_{lr}}\right) \\
i_{qs} &= \frac{1}{X_{ls}} (\psi_{qs} - \psi_{mq}) \\
i_{ds} &= \frac{1}{X_{ls}} (\psi_{ds} - \psi_{md}) \\
\tau_e &= \psi_{ds} i_{qs} - \psi_{qs} i_{ds} 
\end{align*}
```

Finally, the withdrawed current from the bus is:

```math
\begin{align*}
I_r = \left(\frac{S_\text{motor}}{S_\text{base}}\right) (i_{ds} - v_{qs} B_{sh}) \\
I_i = \left(\frac{S_\text{motor}}{S_\text{base}}\right) (i_{qs} + v_{ds} B_{sh}) 
\end{align*}
```

### 3rd-order Single Cage Induction Machine [`PowerSystems.SimplifiedSingleCageInductionMachine`](@extref)

The following models approximates the stator fluxes dynamics of the 5th-order model by using algebraic equations.

```math
\begin{align}
\dot{\psi}_{qr} &= \Omega_b \left(v_{qr} - (\omega_\text{sys} - \omega_r) \psi_{dr} - R_r i_{qr} \right) \tag{3a} \\
\dot{\psi}_{dr} &= \Omega_b \left(v_{dr} + (\omega_\text{sys} - \omega_r) \psi_{qr} - R_r i_{dr}\right) \tag{3b} \\
\dot{\omega}_r &= \frac{1}{2H} (\tau_e - \tau_{m0}(A \omega_r^2 + B \omega_r + C)) \tag{3c}
\end{align}
```

where

```math
\begin{align*}
v_{qs} &= V_i^\text{bus} \\
v_{ds} &= V_r^\text{bus} \\
v_{qr} = v_{dr} &= 0 \\
i_{qs} &= \frac{1}{R_s^2 + \omega_\text{sys}^2 X_p^2} \left( (R_s v_{qs} - \omega_\text{sys} X_p v_{ds}) - \left(R_s \omega_\text{sys} \frac{X_m}{X_{rr}} \psi_{dr} + \omega_\text{sys}^2 X_p \frac{X_m}{X_{rr}} \psi_{qr} \right) \right) \\
i_{ds} &= \frac{1}{R_s^2 + \omega_\text{sys}^2 X_p^2} \left( (R_s v_{ds} + \omega_\text{sys} X_p v_{qs}) - \left(-R_s \omega_\text{sys} \frac{X_m}{X_{rr}} \psi_{qr} + \omega_\text{sys}^2 X_p \frac{X_m}{X_{rr}} \psi_{dr} \right) \right) \\
i_{qr} &= \frac{1}{X_{rr}} (\psi_{qr} - X_m i_{qs}) \\
i_{dr} &= \frac{1}{X_{rr}} (\psi_{dr} - X_m i_{ds}) \\
\tau_e &= \psi_{qr} i_{dr} - \psi_{dr} i_{qr} 
\end{align*}
```

Finally, the withdrawed current from the bus is:

```math
\begin{align*}
I_r = \left(\frac{S_\text{motor}}{S_\text{base}}\right) (i_{ds} - v_{qs} B_{sh}) \\
I_i = \left(\frac{S_\text{motor}}{S_\text{base}}\right) (i_{qs} + v_{ds} B_{sh}) 
\end{align*}
```

### Active Constant Power Load Model

The following 12-state model Active Load model  that measures the AC side using a Phase-Lock-Loop (PLL) and regulates a DC voltage to supply a resistor $r_L$. This model induces a constant power load-like behavior as it tries to maintain a fixed DC voltage to supply ``P = v_\text{DC}^2 / r_L``. The model is based on [the following reference](https://www.sciencedirect.com/science/article/pii/S0142061516000740).

The complete model is given by:

```math
\begin{align}
    \dot{\theta} &= \Omega_b (\omega_\text{pll} - \omega_s) \tag{4a} \\
    \dot{\epsilon} &= v_\text{o}^q \tag{4b}\\
    \omega_\text{pll} &= \omega^\star + k^p_\text{pll} v_\text{o}^q + k_\text{pll}^i \epsilon \tag{4c}\\
    \dot{\zeta} &= v_\text{DC}^\star - v_\text{DC} \tag{4d} \\
    i_\text{cv}^{d,\star} &= k_\text{DC}^p ( v_\text{DC}^\star - v_\text{DC}) + k_\text{DC}^i \zeta \tag{4e}  \\
    \frac{c_\text{DC}}{\Omega_b} \dot{v}_\text{DC} &= \frac{p_\text{cv}}{v_\text{DC}} - \frac{v_\text{DC}}{r_L} \tag{4f} \\
    \dot{\gamma}_d &= i_\text{cv}^d - i_\text{cv}^{d,\star} \tag{4g}\\
    \dot{\gamma}_q &= i_\text{cv}^q - i_\text{cv}^{q,\star} \tag{4h} \\
    v_\text{cv}^{d,\star} &= k_\text{pc}( i_\text{cv}^d - i_\text{cv}^{d,\star}) + k_\text{ic} \gamma_d + \omega_\text{pll} l_f i_\text{cv}^q \tag{4i}\\
    v_\text{cv}^{q,\star} &= k_\text{pc}( i_\text{cv}^q - i_\text{cv}^{q,\star}) + k_\text{ic} \gamma_q - \omega_\text{pll} l_f i_\text{cv}^d \tag{4j}
\end{align}
```

Equations (4a)--(4c)  describes the PLL dynamics to lock the active load to the grid. Equations (4d)-(4e)  describes the DC Voltage Controller to steer the DC voltage to ``v_\text{DC}^\star``, while equation (4f) describes the DC voltage dynamics at the capacitor assuming an ideal converter. Finally, equations (4g)--(4j) describes the dynamics of the AC Current Controller. Additionally six states are defined for the LCL filter in a similar fashion of GFM inverters.
