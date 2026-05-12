# Reference Frames

Each dynamic device is defined in its own ``dq`` synchronous reference frame (SRF). It is
important to note that there are several conventions to do reference frame transformations.

## Synchronous Machines

The grid is modeled in its own real-imaginary (RI) reference frame. With such, this follows
the standard convention that for a voltage angle ``\theta = 0``, there is no imaginary part
and hence ``v_h = v_r + j0``. Traditionally, the reference frame ``dq`` with rotor angle
``\delta`` for synchronous machines connected to a bus ``v_h\angle \theta = v_r + jv_i``
follows the following convention for transformation of per-unit RMS phasors:

```math
\begin{align}
v_d + jv_q &= (v_r + jv_i) e^{-j(\delta- \pi/2)} \tag{1a} \\
v_d &=  v_h \sin(\delta - \theta) \tag{1b} \\
v_q &= v_h \cos(\delta - \theta) \tag{1c} \\
\left[ \begin{array}{c} v_d \\ v_q \end{array} \right] &= \left[ \begin{array}{cc} \sin(\delta) & -\cos(\delta) \\ \cos(\delta) & \sin(\delta) \end{array} \right] \left[ \begin{array}{c} v_r \\ v_i \end{array} \right] \tag{1d}
\end{align}
```

Note that hence in a bus of ``1.0\angle 0``, a rotor angle of ``\delta = 0`` implies that
``v_q = 1.0`` and ``v_d = 0.0``. This transformation is the one that can be found in most
books of Power Systems, such as Kundur, Sauer Pai and in Milano too, and is the convention
used in the software to model dynamic models of synchronous machines in their own reference
frame.

## Inverters

The previously convention is not the standard one used for modeling inverters. Most of
inverter and phase-lock loop (PLL) models follow the next convention:

```math
\begin{align}
v_d + jv_q &= (v_r + jv_i) e^{-j \delta} \tag{2a}  \\
v_d &=  v_h \cos(\delta - \theta) \tag{2b} \\
v_q &= -v_h \sin(\delta - \theta) \tag{2c}
\end{align}
```

That, contrary to the previous case, when ``\delta = \theta = 0`` implies that ``v_d = 1.0``
and ``v_q = 0.0``. This yields the typical PLL conditions that steer ``v_q \to 0`` when ``\delta``
locks in ``\theta``, or when both SRF lock between each other.

## Transformation used

Given the predominancy of both convention in current work, the software uses both conventions
depending on the device modeled. For synchronous machines we used the standard convention (1a)-(1d),
while for inverter models we use the predominant convention used nowadays in such models, i.e. (2a)-(2c).
