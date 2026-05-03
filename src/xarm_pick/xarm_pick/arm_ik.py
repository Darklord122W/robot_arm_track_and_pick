"""Numerical IK for the xArm 1S 5-DOF arm.

KDL (MoveIt's default) only solves 6-DOF poses exactly and returns
NO_IK_SOLUTION on this arm because arm1 is the gripper (not in the
planning chain) and arm2..arm6 give 5 DOF — one short of a full pose.

This module solves IK as an unconstrained nonlinear optimization:
position is hard (must match), orientation is soft (we prefer
gripper-down but accept whatever the arm can reach). For a parallel-jaw
gripper on a symmetric cube, gripper-yaw is irrelevant, so we have
effectively 5 constraints on 5 DOF — well-posed.

The chain (URDF: src/xarm/urdf/xarm_1s.urdf.xacro):

    world (z=0.043 fixed) -> base_link
    arm6 (Z, +q rot, origin xyz=0,0,0.043 rpy=0,0,pi) -> link6
    arm5 (+Y, origin 0.002,0,0.032)                   -> link5
    arm4 (-Y, origin 0,0,0.09775)                     -> link4
    arm3 (+Y, origin 0,0,0.099)                       -> link3
    arm2 (Z, origin -0.00125,0,0.050)                 -> link2
    tool0 (fixed, origin -0.0038,0,0.062)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
from scipy.optimize import minimize


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
JOINT_ORDER = ['arm6', 'arm5', 'arm4', 'arm3', 'arm2']  # base -> tool


def _Rx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def fk_tool0(q: np.ndarray) -> np.ndarray:
    """Forward kinematics: joint vec [q6, q5, q4, q3, q2] -> 4x4 world->tool0."""
    q6, q5, q4, q3, q2 = q

    # world -> base_link
    T_wb = _T(np.eye(3), np.array([0.0, 0.0, 0.043]))
    # base_link -> link6: URDF has rpy=(0,0,pi) AND axis=(0,0,-1) — note the
    # NEGATIVE-Z axis. Joint rotation is Rz(-q6), composed AFTER the static
    # rpy: Rz(pi) * Rz(-q6) = Rz(pi - q6). Earlier bug used Rz(pi + q6),
    # which silently broke FK / IK for any non-zero arm6 (i.e., every cube
    # not directly along the home azimuth).
    T_b6 = _T(_Rz(np.pi) @ _Rz(-q6), np.array([0.0, 0.0, 0.043]))
    # link6 -> link5 (arm5 about +Y at origin (0.002, 0, 0.032))
    T_65 = _T(_Ry(q5), np.array([0.002, 0.0, 0.032]))
    # link5 -> link4 (arm4 about -Y at origin (0, 0, 0.09775))
    T_54 = _T(_Ry(-q4), np.array([0.0, 0.0, 0.09775]))
    # link4 -> link3 (arm3 about +Y at origin (0, 0, 0.099))
    T_43 = _T(_Ry(q3), np.array([0.0, 0.0, 0.099]))
    # link3 -> link2 (arm2 about +Z at origin (-0.00125, 0, 0.050))
    T_32 = _T(_Rz(q2), np.array([-0.00125, 0.0, 0.050]))
    # link2 -> tool0 (fixed)
    T_2t = _T(np.eye(3), np.array([-0.0038, 0.0, 0.062]))

    return T_wb @ T_b6 @ T_65 @ T_54 @ T_43 @ T_32 @ T_2t


@dataclass
class IKResult:
    joints: Dict[str, float]
    pos_err_m: float       # ||tool0_pos - target_pos||
    z_err_rad: float       # angle between tool0_z and world_neg_z
    success: bool          # True if pos_err < pos_tol
    message: str = ''


def _seed_from_target(target: np.ndarray) -> np.ndarray:
    """Reasonable initial joint guess: arm6 points base toward the target,
    other joints folded to put the wrist roughly above the target."""
    x, y, _ = target
    # T_b6's rotation = Rz(pi) * Rz(-q6) = Rz(pi - q6). For link6's +X axis to
    # point toward (x, y), we need (pi - q6) == atan2(y, x), i.e.,
    # q6 = pi - atan2(y, x). Wrap to [-pi, pi] then clip to joint limits.
    azimuth = np.pi - np.arctan2(y, x)
    azimuth = ((azimuth + np.pi) % (2 * np.pi)) - np.pi  # wrap to [-pi, pi]
    return np.array([
        np.clip(azimuth, *JOINT_LIMITS['arm6']),  # q6
        0.5,                                       # q5: shoulder pitch fwd
        1.0,                                       # q4: elbow bend
        -0.5,                                      # q3: wrist down
        0.0,                                       # q2
    ])


def solve_ik(
    target_xyz: Tuple[float, float, float],
    gripper_down_weight: float = 1.0,  # kept for API compat; ignored in current path
    pos_tol_m: float = 0.005,
    n_starts: int = 24,
    reference_q: Optional[Dict[str, float]] = None,
    tilt_acceptable_rad: float = np.radians(45.0),
) -> IKResult:
    """Solve IK for tool0 at target_xyz.

    Strategy: position is the hard constraint (we always try to hit it
    exactly). Orientation falls out of the optimization — tool0 will tilt
    by whatever angle the 5-DOF kinematics force at that position. We
    then report the tilt; the caller (pick_2d) decides if it's acceptable.

    `success` here means position error < pos_tol_m. Orientation is a
    diagnostic, not a pass/fail.

    `reference_q` (optional): joint dict of a "preferred" configuration.
    Used as (a) one of the multi-start seeds, and (b) a tiebreaker —
    among candidates that hit position tol AND have tilt under
    `tilt_acceptable_rad`, the one with smallest joint-space distance
    from `reference_q` wins. WHY: 5-DOF arms have multiple IK branches
    (e.g. base-aimed-at-target with elbow forward, vs base-flipped with
    elbow back ("over the top")). Both can reach the same point with
    similar tilt; selecting purely on lowest tilt makes the chosen
    branch hop unpredictably between targets, producing the over-the-
    head paths that RRTConnect then dutifully plans through. Pinning
    the branch via `reference_q` keeps the whole pick sequence in one
    homotopy class.
    """
    del gripper_down_weight  # unused — see docstring

    target = np.asarray(target_xyz, dtype=np.float64)
    bounds = [JOINT_LIMITS[name] for name in JOINT_ORDER]

    ref_arr: Optional[np.ndarray] = None
    if reference_q is not None:
        ref_arr = np.array(
            [float(np.clip(reference_q[name], *JOINT_LIMITS[name]))
             for name in JOINT_ORDER],
            dtype=np.float64,
        )

    def cost_pos(q: np.ndarray) -> float:
        T = fk_tool0(q)
        return float(np.sum((T[:3, 3] - target) ** 2))

    def cost_pos_then_orient(q: np.ndarray) -> float:
        # Used as a tiebreaker among position-equivalent solutions: prefer
        # the posture that's closest to gripper-down. Tiny weight so it
        # never trades position for orientation.
        T = fk_tool0(q)
        pos_err_sq = float(np.sum((T[:3, 3] - target) ** 2))
        z_axis = T[:3, 2]
        z_err = 1.0 + float(z_axis[2])  # 0 when tool0 +Z = world -Z
        return pos_err_sq + 1e-4 * (z_err ** 2)

    best: Optional[IKResult] = None
    rng = np.random.default_rng(seed=42)

    seeds = []
    if ref_arr is not None:
        seeds.append(ref_arr.copy())
    seeds.append(_seed_from_target(target))
    while len(seeds) < n_starts:
        seeds.append(np.array([rng.uniform(lo, hi) for (lo, hi) in bounds]))

    HALF_PI = np.pi / 2.0

    for q0 in seeds:
        try:
            # Pass 1: position-only — find a posture that hits the target.
            r1 = minimize(cost_pos, q0, method='L-BFGS-B', bounds=bounds,
                          options={'maxiter': 200, 'ftol': 1e-12})
            # Pass 2: refine, biased toward gripper-down. Tiny weight so we
            # don't sacrifice position.
            r2 = minimize(cost_pos_then_orient, r1.x, method='L-BFGS-B',
                          bounds=bounds, options={'maxiter': 200, 'ftol': 1e-12})
            qf = r2.x if cost_pos(r2.x) < pos_tol_m ** 2 else r1.x
        except Exception:
            continue
        T = fk_tool0(qf)
        pos_err = float(np.linalg.norm(T[:3, 3] - target))
        z_axis = T[:3, 2]
        cos_z = float(np.clip(-z_axis[2], -1.0, 1.0))
        z_err_rad = float(np.arccos(cos_z))
        # Reject "gripper-up" solutions (tilt >= 90 deg). Position-only
        # optimization sometimes finds these when the position is reachable
        # only with the gripper inverted — useless for top-down picks.
        if z_err_rad >= HALF_PI:
            continue
        joints = {name: float(v) for name, v in zip(JOINT_ORDER, qf)}
        candidate = IKResult(
            joints=joints, pos_err_m=pos_err, z_err_rad=z_err_rad,
            success=(pos_err < pos_tol_m), message='2-pass L-BFGS-B',
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
            #      - Both outside → lowest tilt wins (existing behaviour).
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
            joints={n: 0.0 for n in JOINT_ORDER},
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
    for n in JOINT_ORDER:
        print(f'  {n} = {r.joints[n]:+.4f} rad')

    # Verify FK matches
    q = np.array([r.joints[n] for n in JOINT_ORDER])
    T = fk_tool0(q)
    print(f'FK pos: {T[:3, 3]}')
    print(f'FK tool0 +Z dir: {T[:3, 2]}')
