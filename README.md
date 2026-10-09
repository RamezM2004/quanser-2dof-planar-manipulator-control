# State-Space Control & Observer Design for Quanser 2-DOF Planar Manipulator
### Pole Placement, Luenberger State Observer, LQR Failure Analysis & Reachable Workspace Mapping

**Course:** ME0345 – Control Systems II  
**Authors:** Ramez Al-Masadeh & Ahmad Al-Amiri  
**Instructor:** Dr. Mutaz Ryalat  
**Institution:** Department of Mechatronics Engineering, German Jordanian University (GJU)  
**Full Reports:**
* [`docs/Control_Systems_II_Final_Report.pdf`](docs/Control_Systems_II_Final_Report.pdf) (17-Page Capstone Report)
* [`docs/Modeling_and_Linearization_Quanser_5Bar.pdf`](docs/Modeling_and_Linearization_Quanser_5Bar.pdf) (5-Page Theoretical Modeling Document)

---

## Executive Summary

This repository presents the modeling, state-space linearization, full-state feedback pole placement control, and Luenberger state observer design for the **Quanser 5-Bar Planar Manipulator** (modeled as a 2-DOF planar parallel robot).

The objective is to achieve precise task-space end-effector positioning at $(E_x, E_y) = (0.038, 0.080)\text{ m}$ with **settling time $T_s < 3\text{ s}$ and percent overshoot $\%OS < 10\%$**, while reconstructing unmeasured joint velocities via an observer and examining the failure modes of task-space Linear Quadratic Regulation (LQR).

---

## 1. Physical System & Kinematic Modeling

The Quanser 5-bar mechanism is a closed-chain parallel robot driven by two base-mounted DC motors separated by distance $L_0 = 0.076\text{ m}$. Motion occurs in the horizontal plane; hence, **gravitational potential energy is constant ($G(\theta) = 0$)**.

| System Parameter | Description | Value |
| :--- | :--- | :--- |
| **$L_0$** | Distance between base motor axes | $0.076\text{ m}$ |
| **$L_1$** | Length of driven link arms | $0.084\text{ m}$ |
| **$m_1, m_3$** | Mass of link 1 and link 2 | $0.01479\text{ kg}, 0.01918\text{ kg}$ |
| **$J_1, J_3$** | Inertia of link 1 and link 2 | $6.51 \times 10^{-7}\text{ kg}\cdot\text{m}^2, 7.78 \times 10^{-7}\text{ kg}\cdot\text{m}^2$ |
| **$J_m$** | Motor rotor inertia | $4.00 \times 10^{-6}\text{ kg}\cdot\text{m}^2$ |
| **$J_{t1}, J_{t2}$** | Total equivalent axis inertia | $4.651 \times 10^{-6}\text{ kg}\cdot\text{m}^2, 4.778 \times 10^{-6}\text{ kg}\cdot\text{m}^2$ |

### Forward Kinematics (Midpoint End-Effector)
$$E_x = \frac{-L_1 \sin\theta_0 + L_1 \sin\theta_1 + L_0}{2}$$
$$E_y = \frac{L_1 \cos\theta_0 + L_1 \cos\theta_1}{2}$$

### Inverse Kinematics via `fsolve`
At the operating point $(E_x, E_y) = (0.038\text{ m}, 0.080\text{ m})$, non-linear optimization yields:
$$\theta_0 \approx 0.000\text{ rad} \ (0^\circ), \quad \theta_1 \approx 0.000\text{ rad} \ (0^\circ)$$

---

## 2. Linearized State-Space Representation

Using generalized coordinates $x = [\theta_0, \theta_1, \dot{\theta}_0, \dot{\theta}_1]^T$ and input torques $u = [\tau_0, \tau_1]^T$:

$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$

$$A = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{1}{J_{t1}} & 0 \\ 0 & \frac{1}{J_{t2}} \end{bmatrix}$$

$$C = \begin{bmatrix} -\frac{L_1}{2}\cos\theta_0 & \frac{L_1}{2}\cos\theta_1 & 0 & 0 \\ -\frac{L_1}{2}\sin\theta_0 & -\frac{L_1}{2}\sin\theta_1 & 0 & 0 \end{bmatrix}, \quad D = \begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$$

### Controllability & Observability Rank Verification
* **Kalman Controllability Matrix:** $\mathcal{C} = [B \quad AB \quad A^2B \quad A^3B] \implies \mathbf{\text{rank}(\mathcal{C}) = 4}$ (Fully Controllable).
* **Kalman Observability Matrix:** $\mathcal{O} = [C^T \quad A^TC^T \quad (A^T)^2C^T \quad (A^T)^3C^T]^T \implies \mathbf{\text{rank}(\mathcal{O}) = 4}$ (Fully Observable).

---

## 3. Controller & Observer Design

### A. State-Feedback Pole Placement
To guarantee $T_s < 3\text{ s}$ and $\%OS < 10\%$, closed-loop poles are selected:
$$\text{Poles} = [-2 \pm 1j, -3 \pm 0.5j]$$
State-feedback gain matrix $K$ is computed via MATLAB `place(A, B, poles)`:
$$u = -Kx + N_{\text{bar}} r$$

### B. Feedforward Precompensator ($N_{\text{bar}}$)
Because full-state feedback alters system DC gain, $N_{\text{bar}}$ scales the reference input $r$ to guarantee zero steady-state tracking error:
$$N_{\text{bar}} = \text{pinv}\left( C (-A_{cl})^{-1} B \right)$$

### C. Luenberger State Observer
To estimate unmeasured joint velocities $[\dot{\theta}_0, \dot{\theta}_1]^T$ using only position feedback:
$$\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})$$
Observer poles are placed $3\times$ deeper into the stable left-half s-plane for rapid state estimation convergence:
$$\text{Poles}_{\text{obs}} = 3 \times \text{Poles} = [-6 \pm 3j, -9 \pm 1.5j]$$
$$L = \text{place}(A', C', \text{Poles}_{\text{obs}})'$$

---

## 4. Key Engineering Discovery: Why Task-Space LQR Failed

In addition to pole placement, an optimal Linear Quadratic Regulator (LQR) was synthesized minimizing:
$$J = \int_0^\infty (x^T Q x + u^T R u) dt$$

### The Failure Root-Cause Analysis:
1. **Extreme Closed-Loop Pole Magnitude:** The LQR solver yielded closed-loop poles on the order of $\mathbf{10^4\text{ rad/s}}$.
2. **Actuator Chattering & Motor Burnout Risk:** Poles at $10^4\text{ rad/s}$ represent extreme high-frequency dynamics that demand massive, instantaneous torque switching, leading to severe actuator saturation and risk of DC motor winding burnout.
3. **Precompensator Ill-Conditioning:** The calculation of $N_{\text{bar}} = \text{pinv}(C(-A + BK_{\text{lqr}})^{-1}B)$ becomes severely ill-conditioned due to the high gain matrix $K_{\text{lqr}}$, producing unstable tracking.
4. **Observer Bandwidth Violation:** An observer paired with $10^4\text{ rad/s}$ dynamics would need to operate at $>3 \times 10^4\text{ rad/s}$, amplifying sensor measurement noise catastrophically.

**Conclusion:** State-feedback pole placement with deliberately chosen damping $\zeta$ and natural frequency $\omega_n$ proved far more robust and physically realizable than unconstrained task-space LQR.

---

## 5. Reachable Workspace Visualization

The robot workspace was mapped using a 2D meshgrid sweep:
* **Theoretical Unconstrained Workspace ($-\pi \le \theta \le \pi$):** Circular planar envelope.
* **Physical Constrained Workspace ($-\pi/2 \le \theta \le \pi/2$):** Restricted diamond-like operational region enforcing mechanical link collision limits.
* Confirmed that the chosen operating point $(38\text{ mm}, 80\text{ mm})$ lies safely inside the physically reachable region.

---

## Repository Structure

```text
quanser-2dof-planar-manipulator-control/
├── docs/
│   ├── Control_Systems_II_Final_Report.pdf         # Complete 17-page capstone report
│   └── Modeling_and_Linearization_Quanser_5Bar.pdf # 5-page theoretical modeling document
├── src/
│   ├── 01_controller_and_observer_design.m         # Full pole placement, Nbar & observer
│   ├── 02_lqr_control_and_failure_analysis.m       # LQR synthesis and failure root-cause analysis
│   └── 03_reachable_workspace_visualization.m      # 2D Cartesian reachability meshgrid mapping
├── .gitignore
└── README.md
```

---

## How to Run the MATLAB Scripts

1. Open MATLAB (R2020a or later). Ensure the **Control System Toolbox** and **Optimization Toolbox** (for `fsolve`) are installed.
2. Navigate to `src/`.
3. Run `01_controller_and_observer_design.m` to simulate the full state-feedback controller and Luenberger observer.
4. Run `02_lqr_control_and_failure_analysis.m` to reproduce the LQR ill-conditioning analysis.
5. Run `03_reachable_workspace_visualization.m` to plot the theoretical vs. physically constrained workspaces.