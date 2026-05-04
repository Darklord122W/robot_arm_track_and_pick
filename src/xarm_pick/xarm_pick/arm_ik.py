"""Numerical IK for the xArm 1S 5-DOF arm — Modern Robotics (Lynch & Park) style.

Forward kinematics via Product of Exponentials (MR §4.1):

    T(θ) = exp([S₁]θ₁) · exp([S₂]θ₂) · ... · exp([S₅]θ₅) · M

where Sᵢ ∈ ℝ⁶ are screw axes in the SPACE (world) frame at the home pose
and M ∈ SE(3) is the home-pose transform of `tool0`. Joint indices match
the URDF chain order, base → tool: S₁ ↔ arm6, S₅ ↔ arm2.

Why PoE instead of per-link composition: identical kinematics, but the
space / body Jacobian (MR §5.1) drops out as a function of the screw
axes alone, so the IK iteration step is analytical.

Inverse kinematics: damped-least-squares Newton-Raphson with task-
priority redundancy resolution (MR §6.2 generalised; Siciliano §3.5).
This arm is **5-DOF**, so for a 6-DOF body twist V_b the Jacobian
J_b ∈ ℝ⁶ˣ⁵ has more rows than columns and is not square — but the
**Moore-Penrose pseudoinverse** J_b⁺ exists and gives the least-squares
step. We split the task:

  • Primary: 3 position constraints, J_pos ∈ ℝ³ˣ⁵ (full rank in non-
    singular configs). Step: Δq_pos = J_pos⁺ · (p_target - p_current).
    The pseudoinverse picks the *minimum-norm* Δq among all that
    satisfy the linearised position equation — leaves the redundancy
    free for the secondary task.
  • Secondary: gripper-down preference, projected into the null space
    of J_pos so it never fights position. Step: Δq_orient = N · (-α ∇c_o)
    with N = (I - J_pos⁺ · J_pos).

DLS damping (`λ²·I` term) keeps the inverse well-conditioned near
kinematic singularities. Multi-start (24 seeds) handles branch
selection — different basins yield different IK branches.

KDL (MoveIt's default) returns NO_IK_SOLUTION (-31) on this arm because
arm1 is the gripper (not in the SRDF planning chain) and KDL refuses
without a workaround for the 5-DOF chain.

URDF chain (src/xarm/urdf/xarm_1s.urdf.xacro):

    world (z=0.043 fixed) -> base_link
    arm6 (axis 0,0,-1; origin xyz=0,0,0.043 rpy=0,0,π) -> link6
    arm5 (axis 0, 1, 0; origin 0.002, 0, 0.032)        -> link5
    arm4 (axis 0,-1, 0; origin 0,     0, 0.09775)      -> link4
    arm3 (axis 0, 1, 0; origin 0,     0, 0.099)        -> link3
    arm2 (axis 0, 0, 1; origin -0.00125, 0, 0.050)     -> link2
    tool0 (fixed, origin -0.0038, 0, 0.062)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


# ----------------------------------------------------------------------
#  Joint metadata
# ----------------------------------------------------------------------

# Effective joint limits as enforced by the hardware driver.
# WHY these are tighter than the URDF: src/xarm_hw/xarm_hw/driver.py
# `rad_to_units` clamps every commanded angle to ±90° (±π/2 rad) before
# sending to the servos. Any IK solution outside this range is silently
# truncated by the driver, which produces ~30 mm position error and the
# 15–20° "tilt" we saw in pick_2d. So the IK must respect the driver's
# clamp, not the URDF's nominal limits.
HALF_PI_LIMIT = 1.5707  # one ULP below π/2 to avoid edge clipping in the driver
JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    'arm2': (-HALF_PI_LIMIT, HALF_PI_LIMIT),
    'arm3': (-HALF_PI_LIMIT, HALF_PI_LIMIT),
    'arm4': (-HALF_PI_LIMIT, HALF_PI_LIMIT),
    'arm5': (-HALF_PI_LIMIT, HALF_PI_LIMIT),
    'arm6': (-HALF_PI_LIMIT, HALF_PI_LIMIT),
}
# base → tool. Index i in this list is "joint i+1" in MR notation.
JOINT_ORDER = ['arm6', 'arm5', 'arm4', 'arm3', 'arm2']


# ----------------------------------------------------------------------
#  SO(3) / SE(3) helpers (MR §3)
# ----------------------------------------------------------------------

def skew3(w: np.ndarray) -> np.ndarray:
    """3-vector → 3×3 skew-symmetric matrix [w] (MR §3.2.3, eq. 3.30)."""
    return np.array([
        [0.0,   -w[2],  w[1]],
        [w[2],   0.0,  -w[0]],
        [-w[1],  w[0],  0.0],
    ])


def exp_so3(omega: np.ndarray, theta: float) -> np.ndarray:
    """Rodrigues: exp([ω]θ) ∈ SO(3). Assumes ‖ω‖ = 1. (MR eq. 3.51)"""
    w_hat = skew3(omega)
    return (np.eye(3)
            + np.sin(theta) * w_hat
            + (1.0 - np.cos(theta)) * (w_hat @ w_hat))


def exp_se3(twist: np.ndarray, theta: float) -> np.ndarray:
    """Matrix exponential of a screw twist S scaled by θ (MR eq. 3.88).

    `twist` is the 6-vector (ω, v). For revolute joints ‖ω‖ = 1; for
    prismatic ω = 0. This robot is all-revolute.
    """
    omega = twist[:3]
    v = twist[3:]
    T = np.eye(4)
    if np.linalg.norm(omega) < 1e-12:
        T[:3, 3] = v * theta
        return T
    R = exp_so3(omega, theta)
    w_hat = skew3(omega)
    G = (theta * np.eye(3)
         + (1.0 - np.cos(theta)) * w_hat
         + (theta - np.sin(theta)) * (w_hat @ w_hat))
    T[:3, :3] = R
    T[:3, 3] = G @ v
    return T


def adjoint(T: np.ndarray) -> np.ndarray:
    """6×6 Adjoint of T ∈ SE(3) (MR eq. 3.83). Maps body twists to space."""
    R = T[:3, :3]
    p = T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, :3] = skew3(p) @ R
    Ad[3:, 3:] = R
    return Ad


# ----------------------------------------------------------------------
#  Robot kinematic constants — derived once from the URDF
# ----------------------------------------------------------------------
#
# Each row of SCREW_AXES is a screw Sᵢ = (ωₓ, ωᵧ, ω_z, vₓ, vᵧ, v_z) in
# the SPACE (world) frame at home (q = 0):
#
#   ωᵢ = unit rotation axis of joint i in world
#   qᵢ = a point on joint i's axis in world (we use the joint-child
#        link's origin in world at home)
#   vᵢ = -ωᵢ × qᵢ   (so that V_s = (ω, v) is a screw through qᵢ)
#
# Derivation walk-through:
#   • All static origins are translations except arm6's, which has rpy=π
#     about Z. Once that flip is composed into the chain, every link
#     6..2 sits in world with orientation Rz(π).
#   • Stack the static origins (rotating each by the cumulative parent
#     orientation) to get link origins in world at home:
#       link6:  ( 0,         0,    0.086   )
#       link5:  (-0.002,     0,    0.118   )
#       link4:  (-0.002,     0,    0.21575 )
#       link3:  (-0.002,     0,    0.31475 )
#       link2:  (-0.00075,   0,    0.36475 )
#       tool0:  ( 0.00305,   0,    0.42675 )   ← ends up as M's translation
#   • Rotate each URDF axis by Rz(π) to get its world direction:
#       arm6 axis (0,0,-1) in base_link (no rpy ahead): world (0, 0,-1)
#       arm5 axis (0, 1,0) ← Rz(π) → world (0,-1, 0)
#       arm4 axis (0,-1,0) ← Rz(π) → world (0, 1, 0)
#       arm3 axis (0, 1,0) ← Rz(π) → world (0,-1, 0)
#       arm2 axis (0, 0,1) ← Rz(π) → world (0, 0, 1)
#   • v = -ω × q.

SCREW_AXES = np.array([
    # ω_x   ω_y   ω_z       v_x        v_y        v_z
    [ 0.0,  0.0, -1.0,    0.0,       0.0,       0.0     ],  # S₁ = arm6
    [ 0.0, -1.0,  0.0,    0.118,     0.0,       0.002   ],  # S₂ = arm5
    [ 0.0,  1.0,  0.0,   -0.21575,   0.0,      -0.002   ],  # S₃ = arm4
    [ 0.0, -1.0,  0.0,    0.31475,   0.0,       0.002   ],  # S₄ = arm3
    [ 0.0,  0.0,  1.0,    0.0,       0.00075,   0.0     ],  # S₅ = arm2
])

# Home-configuration M = T_world_tool0 at q = 0.
# Orientation Rz(π) = diag(-1, -1, 1); translation is link offsets summed
# with the cumulative Rz(π) applied to each downstream piece.
M_HOME = np.array([
    [-1.0,  0.0,  0.0,  0.00305 ],
    [ 0.0, -1.0,  0.0,  0.0     ],
    [ 0.0,  0.0,  1.0,  0.42675 ],
    [ 0.0,  0.0,  0.0,  1.0     ],
])


# ----------------------------------------------------------------------
#  Forward kinematics + Jacobian (MR §4.1, §5.1)
# ----------------------------------------------------------------------

def fk_tool0(q: np.ndarray) -> np.ndarray:
    """T_world_tool0 via PoE: ∏ exp([Sᵢ] θᵢ) · M_HOME.

    `q` is in JOINT_ORDER (q[0] = arm6, …, q[4] = arm2).
    """
    T = np.eye(4)
    for i in range(len(JOINT_ORDER)):
        T = T @ exp_se3(SCREW_AXES[i], q[i])
    return T @ M_HOME


def space_jacobian(q: np.ndarray) -> np.ndarray:
    """Space-frame Jacobian J_s ∈ ℝ⁶ˣⁿ at q (MR eq. 5.11).

    Column i: Adjoint of the cumulative transform up to (but not
    including) joint i, applied to S_i. Column 0 is just S_0.
    """
    n = len(JOINT_ORDER)
    Js = np.zeros((6, n))
    Js[:, 0] = SCREW_AXES[0]
    T_prefix = np.eye(4)
    for i in range(1, n):
        T_prefix = T_prefix @ exp_se3(SCREW_AXES[i - 1], q[i - 1])
        Js[:, i] = adjoint(T_prefix) @ SCREW_AXES[i]
    return Js


def body_jacobian(q: np.ndarray) -> np.ndarray:
    """Body-frame Jacobian J_b ∈ ℝ⁶ˣⁿ at q (MR eq. 5.18).

    Computed via the identity J_b = Ad(T(q)⁻¹) · J_s. Useful for control
    or analysis; the IK below uses J_s directly because the position
    gradient is cleanest in the space frame.
    """
    return adjoint(np.linalg.inv(fk_tool0(q))) @ space_jacobian(q)


# ----------------------------------------------------------------------
#  IK
# ----------------------------------------------------------------------

@dataclass
class IKResult:
    joints: Dict[str, float]
    pos_err_m: float       # ‖tool0_pos - target_pos‖
    z_err_rad: float       # angle between tool0 +Z and world -Z
    success: bool          # True if pos_err < pos_tol
    message: str = ''


def _seed_from_target(target: np.ndarray) -> np.ndarray:
    """Reasonable initial guess: arm6 yaws the base toward the target,
    other joints folded so the wrist is roughly above the target.

    arm6's space-frame screw axis is (ω = (0,0,-1), q on axis = (0,0,0.086)).
    A rotation of θ about this axis is equivalent to Rz(-θ) acting on the
    XY plane. For the wrist to point toward (x, y), we need the cumulative
    base orientation Rz(π - q6) to align link6's +X with the target azimuth,
    which gives q6 = π - atan2(y, x). Wrap to [-π, π] and clip to limits.
    """
    x, y, _ = target
    azimuth = np.pi - np.arctan2(y, x)
    azimuth = ((azimuth + np.pi) % (2 * np.pi)) - np.pi
    return np.array([
        np.clip(azimuth, *JOINT_LIMITS['arm6']),  # q6
        0.5,                                       # q5: shoulder pitch fwd
        1.0,                                       # q4: elbow bend
        -0.5,                                      # q3: wrist down
        0.0,                                       # q2: tool yaw
    ])


def position_jacobian(q: np.ndarray) -> np.ndarray:
    """Position-only Jacobian: d(tool0_position)/dq ∈ ℝ³ˣⁿ.

    Each column i is the contribution of joint i to the world-frame
    velocity of the tool0 origin: ωᵢ × p + vᵢ where (ωᵢ, vᵢ) = J_s[:, i].
    Vectorised: dp/dq = -skew(p) · ω_s + v_s.
    """
    T = fk_tool0(q)
    p = T[:3, 3]
    Js = space_jacobian(q)
    return -skew3(p) @ Js[:3, :] + Js[3:, :]


def damped_pinv(J: np.ndarray, lam: float) -> np.ndarray:
    """Damped Moore-Penrose pseudoinverse: J⁺ = Jᵀ (J Jᵀ + λ²I)⁻¹.

    For a "fat" matrix (rows < cols) — e.g. our 3×5 position Jacobian
    — this gives the minimum-norm Δq solving J · Δq = b. The damping
    `λ²·I` term (Levenberg-Marquardt / Tikhonov) regularises the inverse
    near singularities where (J Jᵀ) loses rank.
    """
    rows = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(rows))


def _solve_one_seed(
    target: np.ndarray,
    q0: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    pos_tol_m: float,
    max_iter: int = 120,
    damping_far: float = 0.10,    # used when far from target — stability
    damping_near: float = 0.005,  # used at convergence — accuracy
    far_threshold_m: float = 0.05,
    secondary_gain: float = 0.3,
    step_cap_rad: float = 0.5,
    pos_converged_factor: float = 0.2,
) -> Tuple[np.ndarray, float, float]:
    """Damped Newton-Raphson with task-priority null-space orientation bias.

    Damping is adaptive: large `damping_far` when ‖p_err‖ > `far_threshold_m`
    (stable, robust to singularities); small `damping_near` once close
    (precise convergence). Loop exits early when ‖p_err‖ falls below
    `pos_tol_m · pos_converged_factor`.
    """
    q = np.clip(q0, lower, upper)
    n = len(q)
    eye_n = np.eye(n)
    convergence_bound = pos_tol_m * pos_converged_factor

    polish_iters = 30  # null-space orientation refinement after position locks
    polish_remaining = polish_iters
    converged = False

    for _ in range(max_iter):
        T = fk_tool0(q)
        p = T[:3, 3]
        z_axis = T[:3, 2]

        p_err = target - p
        err_mag = float(np.linalg.norm(p_err))

        # Once position converges, switch to "polish" mode: keep stepping
        # in the null space (orientation refinement) while the primary
        # task corrects any drift. Stop once the polish budget is spent.
        if err_mag < convergence_bound:
            converged = True
            polish_remaining -= 1
            if polish_remaining <= 0:
                break

        Js = space_jacobian(q)
        omega_s = Js[:3, :]
        v_s = Js[3:, :]

        # Position Jacobian: d(p)/dq = -skew(p) · ω_s + v_s   ∈ ℝ³ˣⁿ
        J_pos = -skew3(p) @ omega_s + v_s

        # Adaptive damping: smooth blend between "far" and "near" values.
        # Large damping when far (stability); tiny near target (accuracy).
        blend = min(1.0, err_mag / far_threshold_m)
        lam = damping_near + blend * (damping_far - damping_near)
        J_pos_pinv = damped_pinv(J_pos, lam)

        # Primary step: drive position toward target (small once converged).
        dq_pos = J_pos_pinv @ p_err

        # Null-space projector — directions that don't perturb position.
        N = eye_n - J_pos_pinv @ J_pos

        # Secondary objective: gripper-down. Cost c_o = (1 + z_axis_z)²,
        # gradient ∇c_o[i] = 2 · (1 + z_z) · (-skew(z_axis) · ω_s)[2, i].
        z_err = 1.0 + float(z_axis[2])
        dz_dq = -skew3(z_axis) @ omega_s
        grad_o = 2.0 * z_err * dz_dq[2, :]
        # Boost secondary gain during polish — once position is locked, we
        # have free joint motion in N's range; spend it on orientation.
        gain = secondary_gain * (3.0 if converged else 1.0)
        dq_orient = N @ (-gain * grad_o)

        dq = dq_pos + dq_orient

        # Step cap to avoid overshooting in tight workspaces.
        dq_norm = float(np.linalg.norm(dq))
        if dq_norm > step_cap_rad:
            dq *= step_cap_rad / dq_norm

        q = np.clip(q + dq, lower, upper)

    T = fk_tool0(q)
    pos_err = float(np.linalg.norm(T[:3, 3] - target))
    z_axis = T[:3, 2]
    cos_z = float(np.clip(-z_axis[2], -1.0, 1.0))
    z_err_rad = float(np.arccos(cos_z))
    return q, pos_err, z_err_rad


def solve_ik(
    target_xyz: Tuple[float, float, float],
    gripper_down_weight: float = 1.0,  # API compat; unused
    pos_tol_m: float = 0.005,
    n_starts: int = 24,
    reference_q: Optional[Dict[str, float]] = None,
    tilt_acceptable_rad: float = np.radians(45.0),
) -> IKResult:
    """Solve IK for tool0 at target_xyz.

    Algorithm: damped-least-squares Newton-Raphson with task-priority
    redundancy resolution (see module docstring). One Newton iteration:

        J_pos     = ∂(tool0_pos)/∂q  ∈ ℝ³ˣ⁵
        J_pos⁺    = J_posᵀ (J_pos · J_posᵀ + λ²I)⁻¹     # damped pinv
        Δq_pos    = J_pos⁺ · (target - p_current)
        N         = I - J_pos⁺ · J_pos                 # null-space projector
        Δq_orient = N · (-α · ∇c_orient(q))            # projected secondary
        q ← clip(q + (Δq_pos + Δq_orient), bounds)

    The 5-DOF rank deficiency disappears: J_pos is 3×5 with rank 3 in
    non-singular configs, so J_pos⁺ exists and Δq_pos is the minimum-
    norm step that closes the position gap to first order. The remaining
    2 DOF live in N's range and are spent on orientation.

    Multi-start (24 seeds) is still needed: Newton-Raphson converges
    locally, and 5-DOF arms have multiple IK branches (shoulder-fwd-
    elbow-up vs. shoulder-back-elbow-down etc). Each seed lands in a
    different basin; we pick the best per the tier rules below.

    `reference_q` (optional): branch-pinning. Among candidates that hit
    position tolerance with tilt < `tilt_acceptable_rad`, the one closest
    in joint space to `reference_q` wins. Keeps a pick sequence (PRE_GRASP
    → GRASP → LIFT) in one homotopy class so the wrist doesn't flip
    branches mid-trajectory.
    """
    del gripper_down_weight  # unused — see docstring

    target = np.asarray(target_xyz, dtype=np.float64)
    n = len(JOINT_ORDER)
    lower = np.array([JOINT_LIMITS[name][0] for name in JOINT_ORDER])
    upper = np.array([JOINT_LIMITS[name][1] for name in JOINT_ORDER])

    ref_arr: Optional[np.ndarray] = None
    if reference_q is not None:
        ref_arr = np.array(
            [float(np.clip(reference_q[name], *JOINT_LIMITS[name]))
             for name in JOINT_ORDER],
            dtype=np.float64,
        )

    rng = np.random.default_rng(seed=42)
    seeds = []
    if ref_arr is not None:
        seeds.append(ref_arr.copy())
    seeds.append(_seed_from_target(target))
    while len(seeds) < n_starts:
        seeds.append(np.array([rng.uniform(lo, hi) for lo, hi in zip(lower, upper)]))

    HALF_PI = np.pi / 2.0
    best: Optional[IKResult] = None
    best_jd: float = 0.0

    for q0 in seeds:
        try:
            qf, pos_err, z_err_rad = _solve_one_seed(
                target, q0, lower, upper, pos_tol_m,
            )
        except Exception:
            continue
        # Reject "gripper-up" solutions (tilt ≥ 90°) — position is reachable
        # only with the gripper inverted. Useless for top-down picks.
        if z_err_rad >= HALF_PI:
            continue
        joints = {name: float(v) for name, v in zip(JOINT_ORDER, qf)}
        candidate = IKResult(
            joints=joints, pos_err_m=pos_err, z_err_rad=z_err_rad,
            success=(pos_err < pos_tol_m), message='DLS Newton (PoE)',
        )
        cand_jd = (float(np.linalg.norm(qf - ref_arr))
                   if ref_arr is not None else 0.0)
        if best is None:
            best, best_jd = candidate, cand_jd
        else:
            # Tiered comparison:
            #   1. Position-success (pos_err < pos_tol) beats failure.
            #   2. Among successes:
            #      - Both within tilt_acceptable → reference closeness wins.
            #      - One within, other outside → within wins.
            #      - Both outside → lowest tilt wins.
            #   3. Among failures: lowest position error wins.
            if candidate.success and not best.success:
                better = True
            elif not candidate.success and best.success:
                better = False
            elif candidate.success and best.success:
                c_acc = candidate.z_err_rad < tilt_acceptable_rad
                b_acc = best.z_err_rad < tilt_acceptable_rad
                if c_acc and b_acc:
                    better = cand_jd < best_jd
                elif c_acc and not b_acc:
                    better = True
                elif not c_acc and b_acc:
                    better = False
                else:
                    better = candidate.z_err_rad < best.z_err_rad
            else:
                better = candidate.pos_err_m < best.pos_err_m
            if better:
                best, best_jd = candidate, cand_jd

    if best is None:
        return IKResult(
            joints={nm: 0.0 for nm in JOINT_ORDER},
            pos_err_m=float('inf'), z_err_rad=float('inf'),
            success=False, message='all starts failed',
        )
    return best


if __name__ == '__main__':
    # CLI: probe IK for a target.
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--xyz', nargs=3, type=float, default=[0.06, 0.22, 0.20])
    ap.add_argument('--n-starts', type=int, default=16)
    args = ap.parse_args()

    r = solve_ik(tuple(args.xyz), n_starts=args.n_starts)
    print(f'Target XYZ: {args.xyz}')
    print(f'Pos error : {r.pos_err_m * 1000:.2f} mm  (success={r.success})')
    print(f'Gripper Z deviation from down: {np.degrees(r.z_err_rad):.1f} deg')
    print(f'Joints:')
    for nm in JOINT_ORDER:
        print(f'  {nm} = {r.joints[nm]:+.4f} rad')

    q = np.array([r.joints[nm] for nm in JOINT_ORDER])
    T = fk_tool0(q)
    print(f'FK pos: {T[:3, 3]}')
    print(f'FK tool0 +Z dir: {T[:3, 2]}')
