# Academic Notes: ros2_sim

This document provides a concise, academic framing of ros2_sim for researchers, educators, and control engineers.

## Summary

ros2_sim is a high‑frequency, deterministic robot arm simulation framework built around analytical rigid‑body dynamics. It is designed for controller development and educational use, emphasizing transparency and repeatability over contact‑rich realism.

## Dynamics Model

The simulator relies on Pinocchio for rigid‑body dynamics of articulated systems. This provides:

- Mass matrix $M(q)$
- Coriolis/centrifugal terms $C(q,\dot{q})$
- Gravity vector $g(q)$
- Jacobians $J(q)$ for mapping forces and velocities

The standard joint‑space dynamics model is:

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + \tau_{ext}
$$

where $q$ denotes joint positions and $\tau_{ext}$ captures external disturbances (planned). This is the classical rigid‑body manipulator equation.

## Forward Dynamics and State Integration

Given joint torques $\tau$, current joint positions $q$, and joint velocities $\dot{q}$, forward dynamics solves for $\ddot{q}$. A common approach is to compute the bias terms and mass matrix from inverse dynamics:

Bias (Coriolis + gravity) via inverse dynamics at zero acceleration:

$$
C(q,\dot{q})\dot{q} + g(q) = \mathrm{InverseDynamics}(q, \dot{q}, 0)
$$

Mass matrix columns (using unit accelerations $\ddot{q}^{(i)}$):

$$
M_i(q) = \mathrm{InverseDynamics}(q, 0, \ddot{q}^{(i)}) - \mathrm{InverseDynamics}(q, 0, 0)
$$

where $\ddot{q}^{(i)}_i = 1$ and $\ddot{q}^{(i)}_j = 0$ for $j \neq i$, and

$$
M(q) = [M_1(q) \; \dots \; M_n(q)].
$$

Then the forward dynamics step is:

$$
\ddot{q} = M(q)^{-1}\big(\tau - C(q,\dot{q})\dot{q} - g(q) - \tau_f\big)
$$

Note: $M(q)$ is symmetric positive definite, so $M(q)^{-T} = M(q)^{-1}$, but the standard form is written with $M(q)^{-1}$.

where $\tau_f$ can represent friction (e.g., Coulomb or viscous models).

For time integration, a simple semi‑implicit Euler step is often used:

$$
\dot{q}_{k+1} = \dot{q}_k + \ddot{q}_k\,\Delta t,
\qquad
q_{k+1} = q_k + \dot{q}_{k+1}\,\Delta t.
$$

### Implementation Note (Pinocchio)

Pinocchio provides functions for inverse dynamics (RNEA), mass matrix (CRBA), and forward dynamics (ABA). These align with the equations above and are the algorithms used conceptually by ros2_sim. Refer to Pinocchio’s documentation for exact API details and naming.

## Control Focus

The project targets control‑centric workflows, including:

- High‑frequency PID and model‑based control loops
- Impedance/admittance control experiments
- Reproducible software‑in‑the‑loop (SIL) testing
- Future MPC integrations with direct access to dynamics

The architecture prioritizes deterministic stepping and stable integration over contact‑heavy physics.

## Educational Value

By exposing analytical dynamics directly, ros2_sim makes core concepts observable and hackable:

- Mass matrix interpretation
- Jacobian mappings for force/velocity
- Effects of gravity and Coriolis terms
- Controller stability at kHz loop rates

This approach supports coursework, lab exercises, and research prototypes where clarity is essential.

## Intended Limitations

The project intentionally does not provide:

- Rich contact dynamics or friction cones
- Soft‑body or deformable physics
- Photorealistic rendering

These omissions are deliberate to preserve high‑frequency execution and analytical tractability.

## Research Directions (planned)

- External wrench injection $\tau_{ext}$ for disturbance studies
- Deterministic replay and noise models for SIL validation
- Kinematic obstacle avoidance (Jacobian or potential‑field based)
- MCP integration for advanced planning/control experiments

## References

- Pinocchio: https://github.com/stack-of-tasks/pinocchio
- Featherstone, R. (2008). *Rigid Body Dynamics Algorithms*.
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*.
- Siciliano, B., Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics*
