# `xarm_moveit/` — A Modern Robotics Reading

> Cross-walk between the code in this workspace and Lynch, Park —
> *Modern Robotics: Mechanics, Planning, and Control* (MR). Section
> numbers below are MR's. Every formula is implemented somewhere in
> `src/` and the references point you to the line.

---

## 1. System overview

The xArm 1S is a 5-DOF positioning manipulator with a 1-DOF parallel-jaw
gripper, observed by a fixed (eye-to-hand) Astra Pro RGB-D camera. The
software stack splits cleanly along the topics MR introduces:

| MR topic | Implementation in this repo |
|---|---|
| Configuration space, joint limits (MR §2) | `src/xarm/urdf/xarm_1s.urdf.xacro`, `src/xarm_moveit_config/config/joint_limits.yaml` |
| Rigid-body motions, $SE(3)$ (MR §3) | URDF link/joint origins, `tf2_ros` graph |
| Forward kinematics (MR §4) | `src/xarm_pick/xarm_pick/arm_ik.py:fk_tool0` |
| Inverse kinematics (MR §6) | `src/xarm_pick/xarm_pick/arm_ik.py:solve_ik` (custom 5-DOF numerical), KDL plugin (unused — fails) |
| Trajectory generation (MR §9) | OMPL RRT-Connect for path, `AddTimeOptimalParameterization` (TOTG) for time scaling, `xarm_hw/driver.py:execute_trajectory_cb` for execution |
| Robot motion control / actuation (MR §11) | `xarm_hw/driver.py:rad_to_units` and `send_joint_positions` |
| Camera projection / fiducials (MR §11.4 + AR appendix) | `charuco_tf_publisher/charuco_tf_node.py`, `xarm_pick/calibrate_homography.py`, `xarm_pick/pick_2d.py` |

Logical pipeline for a single pick (`pick_2d.py`):

```
camera image          ┐
camera_info (K)       ├─► ArUco detect (IPPE_SQUARE) ─► TF camera→marker ─► pixel(u,v)
ChArUco TF publisher  ┘                                                       │
                                                                              ▼
                                                              H ∈ ℝ³ˣ³  (saved YAML)
                                                                              │
                                                                              ▼
                                                              world XY = π(H · [u v 1]ᵀ)
                                                                              │
                                                              ┌────────────────┘
                                                              ▼
            5-DOF numerical IK (L-BFGS-B) ─► joint vector q ─► MoveIt MoveGroup
                                                              │
                                                              ▼
                       OMPL RRTConnect path ─► TOTG retime ─► FollowJointTrajectory
                                                              │
                                                              ▼
                                       xarm_hw driver ─► USB ─► servos
```

Three points worth noting up front, because every formula below exists
to support one of them:

1. **5-DOF on a 6-DOF task is underdetermined in orientation.** MR §6.2
   only solves 6-DOF inverse kinematics analytically; we take the
   numerical path of MR §6.2.2 and accept whatever orientation the arm
   can deliver, then *post-filter* by tilt.
2. **Hand-eye calibration was abandoned in favour of a planar
   homography.** The cube lives on a known plane (the table), so the
   full $T^{cam}_{world} \in SE(3)$ extrinsic is overkill — a single
   3×3 matrix mapping pixels to table $(X,Y)$ is enough, much easier to
   calibrate, and absorbs every linear distortion (camera, table tilt,
   focal length error) into one fit.
3. **The driver silently clamps each joint to $\pm \pi/2$.** This is
   not a property of MR's robot; it is a property of *this* robot's
   USB firmware. Every component of the IK respects it.

---

## 2. Configuration space and the URDF (MR §2.1, §4.1.2)

The robot has 9 URDF joints but only 6 actuated:

| Joint | Type | Axis | URDF origin (rpy / xyz) | Notes |
|---|---|---|---|---|
| `arm6` | revolute | $-\hat z$ | $(0,0,\pi)$ / $(0,0,0.043)$ | base yaw — origin pre-rotates by $\pi$, **and** the joint axis is $-\hat z$ |
| `arm5` | revolute | $+\hat y$ | $0$ / $(0.002,0,0.032)$ | shoulder pitch |
| `arm4` | revolute | $-\hat y$ | $0$ / $(0,0,0.09775)$ | elbow |
| `arm3` | revolute | $+\hat y$ | $0$ / $(0,0,0.099)$ | wrist pitch |
| `arm2` | revolute | $+\hat z$ | $0$ / $(-0.00125,0,0.050)$ | wrist roll-ish (last arm joint) |
| `arm1` | revolute | $+\hat x$ | $0$ / $(-0.0015,-0.014,0.0315)$ | gripper master |
| `arm1_left` | continuous | $+\hat x$ | mimics `arm1`, $-1$ | gripper mirror |
| `arm0`, `arm0_left` | continuous | $+\hat x$ | mimics `arm1` | gripper four-bar |

Only `arm2..arm6` move the tool. `arm1` is structurally a **sibling
branch off `link2`**, not a serial successor; the SRDF
(`xarm_moveit_config/config/xarm_1s.srdf`) declares the planning chain
as

```xml
<chain base_link="base_link" tip_link="tool0"/>
```

which excludes `arm1` from MoveIt's group `arm`. MoveIt therefore
plans only over the 5-DOF positioning subsystem; the gripper is driven
by `xarm_hw/gripper.py` with a one-joint `JointTrajectory`. This makes
$\mathcal{C} = (-\pi/2, +\pi/2)^5$ for the five planning joints (after
the driver-clamp tightening — see §7).

`tool0` is a **fixed** frame attached to `link2` at
$(-0.0038, 0, 0.062)$. It is the gripper midpoint when the jaws are
straddling a cube, hence the right point for IK targeting.

---

## 3. Rigid-body motions and frames (MR §3)

Lynch builds everything on $SE(3)$:

$$
T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}, \qquad R \in SO(3),\; p\in\mathbb{R}^3.
$$

Each URDF `<joint>` corresponds to one $T$. With $T_{ab}$ meaning
"frame $b$ expressed in frame $a$", the chain composes by left-to-right
multiplication (MR §3.3.2):

$$
T_{0,n} = T_{0,1} \, T_{1,2} \cdots T_{n-1,n}.
$$

In ROS this is exactly what the TF tree does. `tf2_ros` is the bookkeeping
implementation of MR's "concatenate transforms" rule; every
`lookup_transform(world, tool0)` is one product.

In the IK code, $SE(3)$ is built explicitly:

```python
# arm_ik.py:65
def _T(R, t):
    M = np.eye(4); M[:3,:3] = R; M[:3,3] = t; return M
```

Elementary rotations $R_x, R_y, R_z$ at `arm_ik.py:50–62` are MR Eq.
(3.6) verbatim.

### 3.1 The product-of-exponentials view (MR §4.1.2)

Each revolute joint can equivalently be written via the matrix
exponential of a screw axis:

$$
e^{[\mathcal{S}_i] \theta_i} = \begin{bmatrix} e^{[\omega]\theta} & (I\theta + (1-\cos\theta)[\omega] + (\theta-\sin\theta)[\omega]^2) v \\ 0 & 1 \end{bmatrix}.
$$

We don't use PoE in the code (we use frame-by-frame composition), but
the URDF is convertible: $\omega_i$ is the joint axis expressed in the
*home* (zero-pose) world frame and $q_i = -\omega_i \times p_i$ where
$p_i$ is any point on the joint axis at zero pose. The home FK
$M = T_{world,tool0}\big|_{q=0}$ is recovered by setting all five
revolute angles to zero in `fk_tool0`.

---

## 4. Forward kinematics (MR §4.1)

Implemented in `arm_ik.py:fk_tool0` (lines 72–95). Given the joint
vector $q = (q_6, q_5, q_4, q_3, q_2)$,

$$
T_{w,tool0}(q) \;=\; T_{w,b}\,T_{b,6}(q_6)\,T_{6,5}(q_5)\,T_{5,4}(q_4)\,T_{4,3}(q_3)\,T_{3,2}(q_2)\,T_{2,t}.
$$

Each link transform is **fixed-rpy times joint-axis rotation** (MR §3.3.3
"body-frame conventions"). The interesting one is the base joint:

```python
# arm_ik.py:83
T_b6 = _T(_Rz(np.pi) @ _Rz(-q6), np.array([0.0, 0.0, 0.043]))
```

Two URDF facts collapse into this product:

* `<origin rpy="0 0 3.14"/>` ⇒ a *static* $R_z(\pi)$ pre-rotation.
* `<axis xyz="0 0 -1"/>` ⇒ the joint rotates about $-\hat z$, so a
  positive $q_6$ produces $R_z(-q_6)$.

Composition:
$R_z(\pi)\,R_z(-q_6) = R_z(\pi - q_6)$.

There is a comment in the code calling out a previous bug where this
was implemented as $R_z(\pi + q_6)$. That sign error broke FK (and
therefore IK) for *every* cube not on the home azimuth — a great
illustration of MR §3.3.3's warning that joint-axis sign and the
pre-rpy must be combined correctly. The unit test of the fix is in
`_seed_from_target` (line 107):

```python
azimuth = np.pi - np.arctan2(y, x)
```

derived by setting "link6's $+\hat x$ should point toward $(x,y)$"
and inverting the same product.

The rest of the chain composes routinely:

$$
\begin{aligned}
T_{w,b} &= I_3,\; p=(0,0,0.043) \\
T_{6,5} &= R_y(q_5),\; p=(0.002, 0, 0.032) \\
T_{5,4} &= R_y(-q_4),\; p=(0, 0, 0.09775) \\
T_{4,3} &= R_y(q_3),\; p=(0, 0, 0.099) \\
T_{3,2} &= R_z(q_2),\; p=(-0.00125, 0, 0.050) \\
T_{2,t} &= I_3,\; p=(-0.0038, 0, 0.062).
\end{aligned}
$$

The end-effector position $p_{tool0} \in \mathbb{R}^3$ and frame
$R_{tool0} \in SO(3)$ are read off the upper-right and upper-left
blocks of $T_{w,tool0}$. These are the only two quantities the IK cost
function actually consumes:

* Position: `T[:3, 3]`
* Tool $+\hat z$ direction in world: `T[:3, 2]`
  (the third column of the rotation block).

For a parallel-jaw gripper, the third column **is** the closing
direction of the jaws — Lynch's "approach vector" $\hat z_e$ in the
common end-effector frame convention (MR §3.3.2).

---

## 5. Inverse kinematics (MR §6)

### 5.1 Why the 6-DOF KDL solver fails on this arm

MoveIt's default IK plugin is `KDLKinematicsPlugin`
(`xarm_moveit_config/config/kinematics.yaml:1-4`). KDL implements MR
Algorithm 6.1 — Newton-Raphson on the 6-DOF residual

$$
\mathbf{e}(q) = \begin{bmatrix} \log\bigl(R_{sd}\,R_{sb}(q)^\top\bigr)^\vee \\ p_{sd} - p_{sb}(q) \end{bmatrix} \in \mathbb{R}^6
$$

with the Jacobian update $q \leftarrow q + J^\dagger e$. With 5
joints and a $6 \times 5$ Jacobian, $J$ is rank-deficient for the full
6-DOF target almost everywhere; the pseudoinverse step blows up or
KDL's residual gate refuses to converge — and the plugin returns
`NO_IK_SOLUTION` for every reachable position, not just the
unreachable ones. This is why `pick_2d` does not call MoveIt for IK at
all and why `MoveGroupClient.move_to_joints` is the only path used in
production.

### 5.2 Numerical IK as nonlinear optimization (MR §6.2.2)

`arm_ik.solve_ik` reformulates IK as

$$
q^* = \arg\min_{q \in \mathcal{C}} \;\; \|p_{tool0}(q) - p^\star\|^2 + \lambda \, g_{orient}(q)
$$

with

* $p^\star$: the desired tool0 position (3 constraints),
* $g_{orient}(q) = (1 + R_{tool0}(q)_{[3,3]})^2$, zero exactly when
  the tool $+\hat z$ axis points along the world $-\hat z$,
* $\mathcal{C} = [-1.5707, +1.5707]^5$,
* $\lambda = 10^{-4}$.

This is a soft-constraint reformulation of a 5-equation system
(3 position + 2 orientation; the 6th, yaw about tool $+\hat z$, is
genuinely free for a symmetric jaw gripper — see MR §6.1's discussion
of redundancy). Reasoning behind the choices:

* **Position-first 2-pass.** Pass 1 minimises position only
  (`cost_pos`). Pass 2 starts from the pass-1 optimum and adds the
  tiny orientation penalty (`cost_pos_then_orient`, line 150). With
  $\lambda \ll 1$ the second pass cannot trade position for
  orientation: among the *position-equivalent* postures the optimiser
  finds the one closest to gripper-down. This is the standard
  "redundancy resolution by secondary cost" of MR §6.3.
* **Multi-start.** Because $f$ is non-convex on $[-\pi/2,\pi/2]^5$, a
  single start from the seed is not enough. `solve_ik` uses 24
  starts: one heuristic seed (line 116) plus 23 uniform draws from the
  joint box. The heuristic seed picks $q_6 = \pi - \mathrm{atan2}(y,x)$,
  i.e. it pre-aims the base yaw at the target.
* **Bounded L-BFGS-B.** MR §6.2.2 uses Newton on the unconstrained
  residual; we use box-constrained quasi-Newton (scipy L-BFGS-B). The
  L-BFGS-B Hessian approximation plays the role of $J^\top J$ in the
  damped-least-squares step

  $$ q \leftarrow q - (J^\top J + \mu I)^{-1} J^\top e $$

  with the bounded line search providing the damping implicitly.

* **Tilt filter.** Solutions with $\arccos(-R_{[3,3]}) \ge \pi/2$ are
  discarded (line 189). These are "gripper pointing up" poses — the
  arm can reach the position only by inverting itself, useless for a
  top-down pick. The caller (`pick_2d`) further enforces a tighter
  tilt tolerance (default 30°, line 56 of `pick_2d.py`).

### 5.3 Selection across multi-start solutions

`arm_ik.solve_ik:196` runs a four-tier comparison among the 24
candidates:

1. Position success (`pos_err < 5 mm`) beats failure.
2. Among successes, lowest tilt wins.
3. Among failures, lowest position error wins.
4. Otherwise tie-broken by enumeration order.

This is a discrete approximation of the continuous Pareto-front
selection MR sketches in §6.3.2 for redundancy resolution: feasibility
first, then secondary objective.

### 5.4 Driver-clamp coupling

Why the IK joint limits are $\pm 1.5707$, not the URDF limits:
`xarm_hw/driver.py:125`

```python
def rad_to_units(self, rad):
    deg = rad * 180.0 / math.pi
    deg = max(-90.0, min(90.0, deg))     # <-- silent clamp
    units = 500 + int(deg * 4.0)
    units = max(100, min(900, units))
    return units
```

If IK returns $q_5 = 1.7\,\text{rad}$ (allowed by the URDF), the driver
silently sends $\pi/2$ instead. FK on the *commanded* $q$ predicts a
position that the *actual* $q$ never reaches — typical error at the
table surface was ~30 mm and ~17° of unmodelled tilt. Tightening
`JOINT_LIMITS` (`arm_ik.py:39`) to one ULP below $\pi/2$ closes the
loop: every $q$ the optimiser proposes is one the hardware can
faithfully execute. This is the kind of constraint-feasibility
adjustment MR §6.2.2 alludes to under "joint-limit clamping".

---

## 6. Calibration

The system has two distinct calibrations stacked on each other.

### 6.1 Camera intrinsics — pinhole projection (MR §8.4 / §11.4 in older eds)

The Astra Pro publishes a `CameraInfo` message containing the camera
matrix $K \in \mathbb{R}^{3 \times 3}$. Lynch's projection model
(MR Eq. (11.1)-equivalent in vision references):

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
\sim
K \begin{bmatrix} X_c / Z_c \\ Y_c / Z_c \\ 1 \end{bmatrix},
\qquad K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}.
$$

Implemented at `calibrate_homography.py:73`:

```python
u = K[0,0] * x / z + K[0,2]
v = K[1,1] * y / z + K[1,2]
```

This is the only place pinhole projection is used in
`calibrate_homography`: we have the marker's *3D* pose in the camera
frame (from PnP), and we need its *pixel*. The reason for re-projecting
instead of using the original detected centre is that PnP smooths and
disambiguates the IPPE flip (next subsection), so the projected centroid
is more stable than the raw measurement.

### 6.2 Marker pose — IPPE PnP (MR doesn't cover this; vision appendix)

`charuco_tf_node.py:_process_single_aruco` (line 488) runs OpenCV's
**`SOLVEPNP_IPPE_SQUARE`** on the four marker corners. For a planar
square of edge $L$, the object points in the marker frame are

$$
P_o = \tfrac{L}{2}
\begin{bmatrix} -1 & +1 & +1 & -1 \\ +1 & +1 & -1 & -1 \\ 0 & 0 & 0 & 0 \end{bmatrix}.
$$

PnP solves for the rotation $R$ and translation $t$ minimising

$$
\sum_i \| \pi(K, R\,P_{o,i} + t) - p_i \|^2,
$$

where $\pi$ is the pinhole projection of §6.1 and $p_i$ are the
detected pixel corners.

A key MR-omitted subtlety: a planar square has a **two-fold pose
ambiguity** — front and back-flipped solutions both reproject within
sub-pixel error at near-frontal views. IPPE (Collins & Bartoli, 2014)
returns *both* solutions explicitly. The code disambiguates with three
filters in `_pick_ippe_solution` (line 574):

1. **Physical feasibility:** reject any $R$ with $R_{[3,3]} \ge 0$ —
   the marker is one-sided, its normal must point back toward the
   camera (i.e. the $z$-component of $R \hat z_{marker}$ in camera
   frame must be negative).
2. **Sticky-branch history:** prefer the candidate within 30° of the
   most recently accepted rvec, with a 6-frame timeout that *clears*
   history on persistent rejection. Lynch §13.6 calls this kind of
   recursive estimator a "tracker" — ours is just rotation-aware.
3. **Depth fusion bootstrap:** when there is no recent history, fit a
   robust plane to depth pixels inside the marker's image-convex hull
   and pick the IPPE branch whose normal $R[:,2]$ best matches it
   (`_depth_disambiguate`, line 763). Score is

   $$ s_i = \frac{\angle(n_i, n_{depth})}{30^\circ} + \|t_i - c\|_{n_{depth}}, $$

   a mixed-unit blend where 30° normal mismatch is comparable to a 1 m
   centroid offset.

### 6.3 Pixel → world: the planar homography (MR §8.4.2 / homography appendix)

The decisive simplification of this workspace is in
`calibrate_homography.py`. Instead of solving for the camera extrinsic
$T^{cam}_{world} \in SE(3)$ (which is the full hand-eye AX=XB problem,
MR §11 + Tsai/Park/Daniilidis), we exploit the fact that **all picks
are on a single plane** and fit a 2D homography directly.

For any plane $Z=Z_0$ in the world, the pinhole projection is
projectively linear in $(X,Y)$. Lynch §8.4.2's homography form:

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
\sim H'
\begin{bmatrix} X \\ Y \\ 1 \end{bmatrix},
$$

with $H' = K [r_1\;r_2\;\, t + Z_0 r_3]$ where $r_1, r_2, r_3$ are the
columns of the world-to-camera rotation. We invert this and solve for
the **pixel-to-world** map directly:

$$
\begin{bmatrix} wX \\ wY \\ w \end{bmatrix} = H \begin{bmatrix} u \\ v \\ 1 \end{bmatrix},
\qquad H = (H')^{-1} \in \mathbb{R}^{3\times 3},
\qquad
\begin{bmatrix} X \\ Y \end{bmatrix} = \frac{1}{w}\begin{bmatrix} wX \\ wY \end{bmatrix}.
$$

Implementation: `pick_2d.py:104`

```python
def pixel_to_world(H, u, v):
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])
```

#### 6.3.1 The data collection (two-phase)

Each calibration sample needs a $\{(u_i, v_i)\;\leftrightarrow\;(X_i, Y_i)\}$
pair. The cube can be observed pixel-wise *or* be reachable by the
gripper, but not both at once (the gripper occludes the marker in the
grasp pose). The script splits the capture in two:

* **Phase A** (`capture_phase_a`, line 166): place the cube, move the
  arm out of the way, wait for a fresh marker TF, project the
  marker centroid through $K$ to get $(u_i, v_i)$.
* **Phase B** (line 204): without disturbing the cube, drag-teach the
  arm so that `tool0` is exactly where it would be at grasp-time, then
  read $T_{world,tool0}$ from `tf2_ros` and take its translation
  $(X_i, Y_i, Z_i)$.

Both halves resolve through the TF tree, which is itself the
forward-kinematics chain of §4 evaluated continuously in
`robot_state_publisher`.

#### 6.3.2 Solving for $H$

With $N \ge 4$ correspondences, the homography satisfies
$\lambda_i \, [u_i\,v_i\,1]^\top = H^{-1} [X_i\,Y_i\,1]^\top$. Stacking
the cross-product residuals (MR §8.4.2 / Hartley-Zisserman normal
equations) gives a $2N \times 9$ linear system $A h = 0$, solved by
SVD: take the right singular vector of the smallest singular value.
The script defers this to OpenCV with **RANSAC** (line 290):

```python
H, mask = cv2.findHomography(pixels, world_xys,
                             method=cv2.RANSAC,
                             ransacReprojThreshold=0.01)
```

Threshold $0.01$ is in **world units (m)** because that is the unit of
the destination — i.e. a sample is an inlier if its predicted $(X,Y)$
is within 10 mm of the captured $(X,Y)$. RANSAC's role is to reject
the few mis-clicked phase-A captures (marker mis-detections, jitter on
the moment of ENTER) without poisoning the global fit.

Quality is reported by **per-point residual**:

$$
r_i = \bigl\| \pi(H, u_i, v_i) - (X_i, Y_i) \bigr\|_2.
$$

The current calibration (CLAUDE.md §8) sits at 9/12 inliers, ~2.5 mm
mean residual — adequate for a 40 mm cube with a 30 mm marker.

#### 6.3.3 The grasp-z trick

Phase B captures `tool0`'s **z**, not the table's z. The script uses
the median z over all calibration points as `suggested_table_z`
(line 310). Because Phase B asks the user to put `tool0` exactly where
it should be at the grasp moment (jaws straddling the cube), this
median is the correct z to drive `tool0` to during the GRASP step —
no offset needed (`pick_2d.py:42-47`). This is a clever sidestep of
the gripper-geometry calibration MR would otherwise need (the
"tool-frame calibration" of §4.1.4): we never compute the gripper's
shape because we never need it; we just *measure* the right z directly.

### 6.4 Why this beats hand-eye AX=XB for this rig

Full hand-eye calibration solves $AX = XB$ (MR §11) for the unknown
camera-to-flange transform $X \in SE(3)$ from many $(A_i, B_i)$
robot/camera motion pairs. It needs:

* good rvec stability (the IPPE flicker poisons it directly),
* enough motion to constrain rotation (≥3 non-parallel screw axes),
* an accurate FK chain to the marker, and
* a marker rigidly mounted to the flange.

This rig violated every one. The 2D homography:

* is a planar special case so it sidesteps the orientation half of
  $X$ entirely,
* needs ~6-8 samples instead of dozens of motions,
* has a built-in residual metric (the inlier $r_i$),
* and is decoupled from the IK quality (it's purely a vision-only
  fit, then arm FK is consulted exactly once per sample to get the
  ground-truth $(X,Y)$).

The cost: it only works for cubes **on the calibrated plane**. Move
the camera or change the table height and the calibration is invalid.
For a fixed-camera tabletop demo, that's a perfectly fair trade.

---

## 7. Trajectory generation (MR §9)

Given the IK output $q^\star \in \mathbb{R}^5$, MoveIt produces a
time-parameterised joint trajectory $q(t)$ over $t \in [0, T]$ with
$q(0) = q_{current}$ and $q(T) = q^\star$, satisfying joint-limit,
velocity-limit, and acceleration-limit constraints.

### 7.1 Path planning: OMPL RRT-Connect (MR §10.5, motion planning)

`ompl_planning.yaml`:

```yaml
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0
arm:
  planner_configs:
    - RRTConnectkConfigDefault
```

RRT-Connect (Kuffner & LaValle, 2000) is a bidirectional sampling-
based planner: two trees rooted at $q_{current}$ and $q^\star$ grow
toward each other in $\mathcal{C}$, attempting a "connect" extension
on every iteration. With `range: 0.0`, OMPL picks a default extension
distance from the joint extents.

The planner produces a **geometric** path
$\sigma: [0,1] \to \mathcal{C}$, with no notion of time. This matches
MR's split between path planning (the geometric problem) and time
scaling (the dynamic problem) in §9.1.

### 7.2 Time scaling: TOTG (MR §9.4)

The path is then handed to MoveIt's
**`AddTimeOptimalParameterization`** (TOTG, "time-optimal
trajectory generation" — Kunz & Stilman, 2012). This is a numerical
implementation of Lynch §9.4's time-optimal time scaling:

> Given a path $\sigma(s)$, $s \in [0,1]$, find the time scaling
> $s(t)$ that minimises $T = \int_0^1 \frac{ds}{\dot s(s)}$ subject to
> per-joint velocity bounds $|\dot q_i| \le V_i$ and acceleration
> bounds $|\ddot q_i| \le A_i$.

The bounds come from `joint_limits.yaml`:

```yaml
arm6:
  has_velocity_limits: true
  max_velocity: 0.3
  has_acceleration_limits: true
  max_acceleration: 0.5
```

with global scaling

```yaml
default_velocity_scaling_factor: 0.30
default_acceleration_scaling_factor: 0.30
```

so the *effective* velocity limit is $0.3 \times 0.3 = 0.09$ rad/s per
joint. This is intentionally conservative for safety while testing.

The launch wiring (`xarm_1s_moveit.launch.py:5-10`) carries a comment
that captures a Humble-era subtlety:

> `AddTimeParameterization` was renamed to
> `AddTimeOptimalParameterization` in MoveIt 2 Humble. The old name
> fails to load (pluginlib `InvalidClass`) and leaves planned paths
> with no time profile, which makes post-processing return FAILURE
> (99999) on every request.

Concretely: an un-retimed trajectory has every point's
`time_from_start = 0`, which the controller then refuses to execute
(no monotonic time stamps).

### 7.3 The trapezoidal velocity profile (MR §9.2)

For point-to-point motion in joint space, MR §9.2 gives three canonical
profiles:

1. Cubic polynomial (5 coefficients per joint, smooth velocity, jerky
   acceleration).
2. Quintic polynomial (continuous accel).
3. Trapezoidal velocity (linear segments — bang-coast-bang accel).

TOTG generalises (3) to a path in $\mathcal{C}$, producing a piecewise
function $\dot s(s)$ that saturates the most-constrained joint at every
point on the path. The output is a sequence of `JointTrajectoryPoint`s
each with `positions`, `velocities`, `accelerations`, and a strictly
monotonic `time_from_start`.

### 7.4 Execution (MR §11)

The controller side runs in `xarm_hw/driver.py:execute_trajectory_cb`
(line 217). For each trajectory point it:

1. Reads the desired wall time $t_i = $ `time_from_start`.
2. Computes the per-segment duration $\Delta t_i = t_i - t_{i-1}$ with
   a 200 ms floor.
3. Sends the waypoint to the servos with that duration as the move
   time argument: `arm.setPosition(servo_cmds, duration=duration_ms)`.
4. Busy-waits until the wall-clock time matches $t_i$ before
   advancing to the next point.

This is a (very simple) feedforward-only execution layer. There is no
PID; the smart-servos themselves implement a closed loop on the
commanded angle, and the duration argument is interpreted as a smooth
linear interpolation inside the servo. As long as TOTG's profile is
reachable, the cumulative wall-clock matches the planned $T$.

There is also a defensive fallback: if `time_from_start` is zero on
the last point — meaning TOTG didn't run — the driver synthesises a
linear time profile at a fixed peak velocity (line 252). This shipped
after a deploy where a misnamed pluginlib class silently turned every
trajectory into an instantaneous step.

### 7.5 Pilz cartesian limits (MR §9.5)

`pilz_cartesian_limits.yaml` is loaded into `move_group` even though
no Pilz pipeline is active. It is dead config for our pipeline today
but kept because OMPL's `AddTimeOptimalParameterization` reads no
Cartesian terms — the limits are joint-space only. Pilz's LIN/CIRC
planners *would* read these (max Cartesian speed, max Cartesian
accel) for straight-line tool paths, which MR §9.5 calls "task-space
trajectory generation". Activating Pilz is the easiest way to get a
straight-line descent for the GRASP step if the joint-space path
overshoots — it currently doesn't, so we don't.

---

## 8. The full motion of one pick

Putting everything together, the GRASP descent of `pick_2d` runs:

1. **Sense** (vision): `charuco_tf_publisher` detects the marker and
   broadcasts $T^{cam}_{marker}$. Latency: one frame (~33 ms).
2. **Project** (calibration §6.1): marker translation goes through
   $K$ to a pixel $(u, v)$ — `pick_2d.py:99`.
3. **Map** (calibration §6.3): pixel goes through $H$ to a world
   $(X, Y)$ — `pick_2d.py:195`.
4. **Add z**: $Z = $ `table_z` from calibration YAML.
5. **IK** (§5): `arm_ik.solve_ik((X,Y,Z))` returns $q^\star$ via
   24-restart L-BFGS-B.
6. **Tilt gate** (§5.3): reject if $\arccos(-R_{[3,3]}) > 30°$.
7. **MoveGroup goal** (§7.1): `MoveGroupClient.move_to_joints(q^\star)`
   sends a `MoveGroup` action with `JointConstraint` for each joint.
8. **Plan**: OMPL RRT-Connect finds a collision-free path in
   $\mathcal{C}$ from $q_{current}$ to $q^\star$.
9. **Retime**: TOTG annotates the path with $\dot q$, $\ddot q$, and
   monotonic timestamps respecting joint-velocity/accel limits.
10. **Execute** (§7.4): driver streams setpoints to the servos with
    per-segment durations, busy-waiting on wall time.
11. **Feedback**: 20 Hz `JointState` from the driver closes the loop in
    `tf2_ros`; RViz and `pick_2d` see the actual pose.

The forward kinematics chain (§4) is evaluated continuously by
`robot_state_publisher` to broadcast every link's TF, which is what
calibration Phase B reads to get ground-truth $(X,Y)$ in step 3 of the
**previous** pick's calibration run.

---

## 9. What's deliberately not in MR's playbook

A few choices in this workspace deviate from MR's textbook recipe; in
each case the deviation is forced by something concrete:

* **No analytic IK.** MR §6.1 gives closed-form IK for "kinematically
  decoupled" wrists (3R wrist + 3R arm). The xArm 1S has 5 joints and
  no spherical wrist, so no closed form exists. Numerical IK is the
  only option.
* **No Jacobian-based velocity control.** MR §5 derives the Jacobian
  $J(q)$; we never compute it explicitly. The gripper-down picks are
  point-to-point with no Cartesian-velocity constraint, so the
  Jacobian is consumed only inside L-BFGS-B's Hessian approximation
  (numerically, by finite differences in scipy) and inside TOTG's
  joint-limit checks.
* **No dynamics.** MR §8 derives $M(q)\ddot q + h(q,\dot q) = \tau$;
  `xarm_hw` doesn't model torque at all because the smart servos
  encapsulate it. Move durations end up as kinematic constraints, not
  dynamic ones — exactly the "kinematic decoupling" assumption MR §11
  flags as practical when servo bandwidth is high.
* **No SLAM, no estimation.** The camera is rigidly fixed to the
  world; the homography absorbs both intrinsic and extrinsic linear
  error in one step. The system is wholly open-loop on object
  position once the cube stops moving — a simplification that's only
  legitimate because the marker provides absolute (not differential)
  pose.

---

## 10. Quick reference

| Concept | File | Line |
|---|---|---|
| FK chain `world → tool0` | `src/xarm_pick/xarm_pick/arm_ik.py` | 72 |
| Numerical IK (L-BFGS-B, 24 starts) | `src/xarm_pick/xarm_pick/arm_ik.py` | 125 |
| Tilt gate / pose acceptance | `src/xarm_pick/xarm_pick/arm_ik.py` | 189 |
| Pinhole projection $\pi(K, p)$ | `src/xarm_pick/xarm_pick/calibrate_homography.py` | 73 |
| 2-phase calibration capture | `src/xarm_pick/xarm_pick/calibrate_homography.py` | 217 |
| Homography fit (RANSAC) | `src/xarm_pick/xarm_pick/calibrate_homography.py` | 290 |
| Pixel → world map | `src/xarm_pick/xarm_pick/pick_2d.py` | 104 |
| Pick state machine | `src/xarm_pick/xarm_pick/pick_2d.py` | 211 |
| MoveGroup joint-space goal | `src/xarm_pick/xarm_pick/moveit_client.py` | 62 |
| MoveGroup pose goal (free yaw) | `src/xarm_pick/xarm_pick/moveit_client.py` | 80 |
| OMPL RRT-Connect config | `src/xarm_moveit_config/config/ompl_planning.yaml` | 18 |
| TOTG retimer | `src/xarm_moveit_config/config/ompl_planning.yaml` | 9 |
| Joint limits / velocity scaling | `src/xarm_moveit_config/config/joint_limits.yaml` | 5 |
| ArUco / IPPE detector + TF | `src/charuco_tf_publisher/charuco_tf_publisher/charuco_tf_node.py` | 488 |
| IPPE branch picker | `src/charuco_tf_publisher/charuco_tf_publisher/charuco_tf_node.py` | 574 |
| Depth-fusion disambiguation | `src/charuco_tf_publisher/charuco_tf_publisher/charuco_tf_node.py` | 763 |
| FollowJointTrajectory execution | `src/xarm_hw/xarm_hw/driver.py` | 217 |
| Driver $\pm \pi/2$ clamp | `src/xarm_hw/xarm_hw/driver.py` | 125 |
| Gripper one-joint command | `src/xarm_hw/xarm_hw/gripper.py` | 48 |

