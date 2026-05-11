# Tube-Based-Robust-MPC-for-End-Effector-Pose-Regulation-
# Tube-Based Robust MPC for End-Effector Pose Regulation

Robust convex tube MPC for holding the end-effector of a 7-DOF Franka Emika Panda still while its base is being shaken. Built as a course project at UIUC.

The arm–base coupling is treated as a bounded additive joint-acceleration disturbance, the prediction model is the feedback-linearized double integrator, and the MPC is a second-order cone program that jointly optimizes the nominal trajectory and the ellipsoidal tube radius. The implementation extends the convex tube MPC of [Wullt et al., 2025](https://arxiv.org/abs/2508.21677) to the mobile-base setting and benchmarks it against three baselines.

## Headline result

End-effector RMSE under a sinusoidal virtual base disturbance ($A = 0.5$, $f = 1.0$ Hz), 7-DOF Panda, $T_s = 50$ ms, $H = 12$:

| Controller   | RMSE [m]              | Max error [m]         | Constraint violation |
|--------------|-----------------------|-----------------------|----------------------|
| PD + GC      | $0.312 \pm 0.007$     | $0.596 \pm 0.055$     | 99.9 %               |
| DOB + PD     | $0.765 \pm 0.031$     | $1.392 \pm 0.006$     | 99.9 %               |
| Nominal MPC  | $0.0029 \pm 0.0015$   | $0.0043 \pm 0.0023$   | **0 %**              |
| **Tube MPC** | $0.0029 \pm 0.0015$   | $0.0042 \pm 0.0023$   | **0 %**              |

Both MPC variants achieve sub-millimetre regulation while incurring zero state/torque constraint violations. Tube MPC adds a recursive-feasibility guarantee on top, which the nominal MPC cannot certify. Full sweep across three disturbance types (sinusoidal, step, stochastic), three amplitudes (0.5, 1.0, 2.0), and 20 stochastic seeds is in [`EvaluationResults.csv`](EvaluationResults.csv).

## Repository contents

```
.
├── Main_Notebook.ipynb       # full pipeline: robot model + 4 controllers + evaluation
├── EvaluationResults.csv     # mean ± std across the full disturbance sweep
├── videos/                   # 12 Meshcat playbacks (4 controllers × 3 scenarios)
└── README.md                 # this file
```

`Main_Notebook.ipynb` is the single entry point. It is organized into four parts:

1. **Robot wrapper** — thin `PandaRobot` class around Pinocchio (CRBA for $M$, RNEA for inverse dynamics, ABA for forward dynamics) loaded from `robot_descriptions`, with Meshcat visualization.
2. **Virtual base disturbance** — converts a planar base acceleration $\ddot q_b = [\ddot x_b, \ddot y_b, \ddot\theta_b]$ into an equivalent joint-space disturbance through the damped Jacobian pseudo-inverse. Replace one method to swap in a true $-M_{aa}^{-1} M_{ab} \ddot q_b$ once a mobile-base URDF is available.
3. **Controllers** — `PandaTubeMPC` (SOCP solved in CVXPY with Clarabel/ECOS/SCS), plus `pd_gravity_torque` and a DOB-augmented PD baseline. Nominal MPC is recovered from `PandaTubeMPC` by setting the disturbance bound and tightening coefficients to zero.
4. **Evaluation** — a quick three-scenario benchmark and a full sweep, both producing the same row schema; metrics include position RMSE, max error, settling time, constraint-violation rate, and control effort.

## Requirements

Python ≥ 3.10. The notebook needs:

```bash
pip install numpy scipy matplotlib pandas
pip install cvxpy clarabel                 # MPC SOCP solver
pip install pin robot_descriptions         # Pinocchio + URDF loader
pip install meshcat-shapes                 # visualization
pip install imageio imageio-ffmpeg pillow  # optional, only for recording new videos
```

Tested on Python 3.11 with `pin` 2.7, `cvxpy` 1.5, `clarabel` 0.9.

## Running

Open `Main_Notebook.ipynb` and run the cells in order:

- **Cells 0–7** build the robot model and verify the PD-with-gravity-compensation loop converges on a step reference. A Meshcat tab will open in your browser; keep it visible if you intend to record videos later.
- **Cells 8–14** define the four controllers and the metrics pipeline.
- **Cell 15** runs the quick three-scenario benchmark (≈ 2 minutes on a laptop). It prints a per-run table and a mean ± std summary by (scenario type, controller).
- **Cells 16–21** are optional: they re-run the same simulations and record an MP4 of each Meshcat playback to `videos/`.
- **Cell 22** is the full sweep (commented out by default). Flipping `RUN_FULL_EVALUATION = True` runs all $3 \times 3 + 3 + 3 \times 20 = 72$ scenarios; expect ≈ 30 minutes.

### Recording videos

Video recording uses the helper module `meshcat_video.py` (imported from cell 18). The Meshcat browser tab must stay open and visible during recording; the helpers grab frames from the live viewport. If the workflow is fiddly on your machine, the existing MP4s in `videos/` cover all 12 combinations for the quick scenarios.

## Demo videos

Browse [`videos/`](videos/) for a side-by-side feel of how each controller behaves. Twelve clips are included, one per (controller, scenario) pair:

| Scenario                                | PD + GC                                                     | DOB + PD                                                       | Nominal MPC                                                       | **Tube MPC**                                                      |
|-----------------------------------------|-------------------------------------------------------------|----------------------------------------------------------------|-------------------------------------------------------------------|-------------------------------------------------------------------|
| Sinusoidal ($A=0.5$, $f=1.0$ Hz)        | [mp4](videos/PDplusGC__base_sin_A0.5_f1.0.mp4)             | [mp4](videos/DOBplusPD__base_sin_A0.5_f1.0.mp4)               | [mp4](videos/Nominal_MPC__base_sin_A0.5_f1.0.mp4)                | [mp4](videos/Tube_MPC__base_sin_A0.5_f1.0.mp4)                   |
| Step ($A=0.5$, $t_0=2.0$ s)             | [mp4](videos/PDplusGC__base_step_A0.5_t02.0.mp4)           | [mp4](videos/DOBplusPD__base_step_A0.5_t02.0.mp4)             | [mp4](videos/Nominal_MPC__base_step_A0.5_t02.0.mp4)              | [mp4](videos/Tube_MPC__base_step_A0.5_t02.0.mp4)                 |
| Stochastic ($A=0.5$, seed 0)            | [mp4](videos/PDplusGC__base_stoch_A0.5_seed0.mp4)          | [mp4](videos/DOBplusPD__base_stoch_A0.5_seed0.mp4)            | [mp4](videos/Nominal_MPC__base_stoch_A0.5_seed0.mp4)             | [mp4](videos/Tube_MPC__base_stoch_A0.5_seed0.mp4)                |

In every clip the end-effector is asked to hold a single goal pose while the virtual base disturbance pushes joint accelerations around. The two MPC clips look nearly static; the PD clips drift; the DOB+PD clips drift the most and visibly chatter.

## Method in one paragraph

After feedback linearization the arm dynamics reduce to $\ddot q_a = a + w$ with $w \in \mathcal{W}$, where $a$ is a commanded joint acceleration and $w$ lumps parametric error and the base-coupling term $-M_{aa}^{-1} M_{ab} \ddot q_b$. Discretized at $T_s = 50$ ms, this is a double integrator $x_{k+1} = Ax_k + B(a_k + w_k)$. The tube MPC plans a nominal trajectory $(\bar x_k, \bar a_k)$ along with an ellipsoidal tube radius $\delta_k$ that satisfies $\delta_{k+1} \ge \rho \delta_k + \beta_a \|\bar a_k\| + \beta_v \|\bar v_k\| + \beta_c$, where $\beta_c = \|B\|_2 \bar d$ and $\bar d$ is the empirical disturbance bound. State and input constraints are tightened linearly in $\delta_k$ so that any realization inside the tube remains feasible. The actual control applied to the plant is $a(k) = \bar a_0^\star + K(x(k) - \bar x_0^\star)$ with $K$ the discrete LQR gain for $(A, B, Q, R)$. The SOCP is solved with Clarabel/ECOS/SCS via CVXPY, warm-started between samples.

A longer description of the method, the simulation setup, and the discussion of why DOB+PD underperforms is in the project report.

## Notes and limitations

- The Panda URDF used here is **fixed-base**, so the base coupling is injected via a virtual channel ($J_v^{\#}$ of the planar base acceleration). The MPC sees this as a bounded joint-acceleration disturbance; behavior is therefore representative of the physical case but not identical. Substituting the true $-M_{aa}^{-1} M_{ab} \ddot q_b$ requires a mobile-base URDF and a one-line change in `VirtualPlanarBaseDisturbance.joint_accel_disturbance`.
- The disturbance Lipschitz constant $L_\beta$ in the tube recursion is currently absorbed into $\rho$ rather than computed offline, which is conservative.
- The MPC's auxiliary feedback gain $K$ is set to the discrete LQR gain for $(A, B, Q, R)$ rather than synthesised via an LMI; this is sufficient for the constraint-tightening here but a formal LMI synthesis would give a tighter $P$ and a less conservative invariant set.

## References

- B. Wullt, J. Köhler, P. Mattsson, M. Norrlöf, T. B. Schön. *Robust convex model predictive control with collision avoidance guarantees for robot manipulators.* arXiv:2508.21677, 2025.
- D. Q. Mayne, M. M. Seron, S. V. Raković. *Robust model predictive control of constrained linear systems with bounded disturbances.* Automatica, 41(2):219–224, 2005.
- J. Carpentier et al. *The Pinocchio C++ library.* IEEE/SII, 2019.
- S. Diamond, S. Boyd. *CVXPY: A Python-embedded modeling language for convex optimization.* JMLR, 2016.

## Acknowledgments

Course project, UIUC. Claude (Anthropic) was used to help organize the structure of the project report, suggest tables, and grammar-check.
