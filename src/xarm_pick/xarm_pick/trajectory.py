"""Joint-space trajectory generation — from-scratch implementations.

Replaces MoveIt's `AddTimeOptimalParameterization` (TOTG) with the
classical methods presented in:

  • Craig, *Introduction to Robotics: Mechanics and Control* (3rd ed.),
    Ch. 7 — cubic polynomials, quintic polynomials, linear segments
    with parabolic blends (LSPB).
  • Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*,
    §9.2 (point-to-point cubic / quintic), §9.4 (time-optimal time
    scaling along a parameterised path).

All trajectories below treat the path as a straight line in joint space:

    q_i(t) = q_start_i + s(t) · (q_goal_i - q_start_i),    s ∈ [0, 1]

so the geometric path is fixed and only the time scaling s(t) changes
between methods. This is the simplest case of MR §9.4 (path
parameterised by a single scalar) and matches Craig's joint-space
schemes verbatim. Path planning (obstacle avoidance) is deliberately
out of scope — for the tabletop pick-and-place workspace there are no
collisions to avoid in the joint-space straight line, so we don't need
RRT-Connect.

The four scaling profiles s(t):

  • Cubic polynomial (Craig §7.3, MR §9.2)
        s(t) = 3(t/T)² - 2(t/T)³
        Boundary conditions: s(0)=0, s(T)=1, ṡ(0)=ṡ(T)=0.
        Peak velocity factor:     ṡ_max     = 1.5 / T   (at t=T/2)
        Peak acceleration factor: |s̈|_max  = 6   / T²  (at t=0 and t=T)

  • Quintic polynomial (Craig §7.4, MR §9.2)
        s(t) = 10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵
        Boundary conditions: also s̈(0)=s̈(T)=0, so jerk is bounded
        across the start/end (cubic has acceleration discontinuities).
        Peak velocity factor:     ṡ_max     = 1.875 / T
        Peak acceleration factor: |s̈|_max  = 10/√3 / T² ≈ 5.7735 / T²

  • LSPB / trapezoidal blend (Craig §7.5)
        Three phases — accel, coast, decel. The blend time t_b sets
        the constant acceleration during the ramp:
            a = 1 / (t_b · (T - t_b))            (peak)
            ṡ_coast = a · t_b                    (cruise speed)
        s(t) is C¹ but acceleration is discontinuous at the blend
        boundaries.

  • Time-optimal trapezoidal (MR §9.4 special case)
        Same shape as LSPB but t_b is chosen automatically to saturate
        either the velocity bound or the acceleration bound, picking
        the shorter total time. Reduces to a triangular profile when
        the segment is too short to ever reach v_max.

Convention:
  All four classes implement a `sample(t)` method returning q, qdot,
  qddot in joint space, and a `discretise(dt)` helper that yields a
  sequence of `TrajectorySample`. Total duration is `traj.duration`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence


# ----------------------------------------------------------------------
#  Tiny numpy-free vector helpers
# ----------------------------------------------------------------------
#
# Trajectory math here is intentionally implemented on plain Python
# float lists rather than np.ndarray. Two reasons:
#
#   1. The rewrite is meant to read like Craig / Lynch's textbook
#      pseudocode — explicit per-joint loops, no hidden broadcasting.
#   2. Avoids a numpy dependency in the trajectory module's public
#      surface; callers who want arrays can wrap freely.

Vec = List[float]


def _scale(v: Sequence[float], k: float) -> Vec:
    return [float(x) * float(k) for x in v]


def _delta(a: Sequence[float], b: Sequence[float]) -> Vec:
    if len(a) != len(b):
        raise ValueError(f'_delta: length mismatch {len(a)} vs {len(b)}')
    return [float(x) - float(y) for x, y in zip(b, a)]


def _add(a: Sequence[float], b: Sequence[float]) -> Vec:
    return [float(x) + float(y) for x, y in zip(a, b)]


# ----------------------------------------------------------------------
#  Sample type
# ----------------------------------------------------------------------

@dataclass
class TrajectorySample:
    """One waypoint along the trajectory.

    `q`, `qd`, `qdd` are joint-space vectors (one entry per planning
    joint, in the order the trajectory was constructed with).
    """
    t: float
    q: Vec
    qd: Vec
    qdd: Vec


# ----------------------------------------------------------------------
#  Common base
# ----------------------------------------------------------------------

class _StraightLineProfile:
    """Shared straight-line-in-joint-space scaffolding.

    Subclasses implement `_s_of_t(t)` returning (s, ṡ, s̈) ∈ ℝ³ for the
    scalar path parameter, and set `self.duration`. Position, velocity,
    and acceleration in joint space are then linear in (s, ṡ, s̈)
    because the path is q(t) = q_start + s(t) · Δq.
    """

    duration: float
    q_start: Vec
    q_goal: Vec
    delta: Vec
    _n: int

    def __init__(self, q_start: Sequence[float], q_goal: Sequence[float]):
        if len(q_start) != len(q_goal):
            raise ValueError(
                f'q_start and q_goal differ in length: '
                f'{len(q_start)} vs {len(q_goal)}'
            )
        self.q_start = [float(x) for x in q_start]
        self.q_goal = [float(x) for x in q_goal]
        self.delta = _delta(self.q_start, self.q_goal)
        self._n = len(self.q_start)

    # Subclass override --------------------------------------------------
    def _s_of_t(self, t: float):
        raise NotImplementedError

    # Public sampling ---------------------------------------------------
    def sample(self, t: float) -> TrajectorySample:
        # Clamp to [0, duration]; outside is held at the boundary state.
        if t < 0.0:
            t = 0.0
        elif t > self.duration:
            t = self.duration
        s, sd, sdd = self._s_of_t(t)
        q = [self.q_start[i] + s   * self.delta[i] for i in range(self._n)]
        qd = [                    sd  * self.delta[i] for i in range(self._n)]
        qdd = [                   sdd * self.delta[i] for i in range(self._n)]
        return TrajectorySample(t=t, q=q, qd=qd, qdd=qdd)

    def discretise(self, dt: float = 0.02) -> List[TrajectorySample]:
        """Sample the profile at uniform intervals dt, including endpoints.

        Spacing is exactly dt except for the final segment which may be
        shorter so the last sample lands exactly at t = duration. This
        matches MoveIt's TOTG output cadence (default 0.02 s) so the
        downstream xarm_hw driver (50 Hz target) sees the same shape.
        """
        if dt <= 0.0:
            raise ValueError('dt must be positive')
        out: List[TrajectorySample] = []
        n_steps = int(math.ceil(self.duration / dt))
        for k in range(n_steps):
            t = k * dt
            out.append(self.sample(t))
        out.append(self.sample(self.duration))
        return out


# ----------------------------------------------------------------------
#  Cubic polynomial — Craig §7.3, MR §9.2.1
# ----------------------------------------------------------------------

class CubicPolynomial(_StraightLineProfile):
    """s(t) = 3(t/T)² − 2(t/T)³.

    Smooth velocity (zero at boundaries), but acceleration jumps from
    +6/T² to 0 at t=0+ and from 0 to −6/T² at t=T−. Use Quintic
    instead if your servos can't tolerate those steps.
    """

    # Peak factors used by `with_limits` (max over t∈[0,T] of |·|).
    PEAK_V_FACTOR = 1.5
    PEAK_A_FACTOR = 6.0

    def __init__(self, q_start, q_goal, duration: float):
        super().__init__(q_start, q_goal)
        if duration <= 0.0:
            raise ValueError('duration must be positive')
        self.duration = float(duration)

    def _s_of_t(self, t: float):
        T = self.duration
        u = t / T
        s   = 3.0 * u * u - 2.0 * u * u * u
        sd  = (6.0 * u - 6.0 * u * u) / T
        sdd = (6.0     - 12.0 * u   ) / (T * T)
        return s, sd, sdd

    # ------------------------------------------------------------------
    @classmethod
    def with_limits(
        cls,
        q_start: Sequence[float],
        q_goal: Sequence[float],
        v_max: Sequence[float],
        a_max: Sequence[float],
        safety_margin: float = 1.05,
    ) -> 'CubicPolynomial':
        """Pick the smallest T such that no joint exceeds v_max / a_max.

        Per joint i: peak velocity is (1.5/T)·|Δq_i|, peak |accel| is
        (6/T²)·|Δq_i|. The smallest T satisfying both bounds for every
        joint is therefore

            T = max_i max(
                  PEAK_V_FACTOR · |Δq_i| / v_max_i ,
                  sqrt(PEAK_A_FACTOR · |Δq_i| / a_max_i)
                )

        `safety_margin` (default 1.05) inflates T by 5% to keep the
        actual servo response below the nominal limits — useful because
        the smart-servos round-trip with finite latency.
        """
        return _fit_polynomial(
            cls, q_start, q_goal, v_max, a_max,
            cls.PEAK_V_FACTOR, cls.PEAK_A_FACTOR, safety_margin,
        )


# ----------------------------------------------------------------------
#  Quintic polynomial — Craig §7.4, MR §9.2.2
# ----------------------------------------------------------------------

class QuinticPolynomial(_StraightLineProfile):
    """s(t) = 10(t/T)³ − 15(t/T)⁴ + 6(t/T)⁵.

    Also enforces s̈(0)=s̈(T)=0, so the trajectory is C² across the
    boundaries — no acceleration step at start / end (jerk is
    bounded). Slightly slower than cubic for the same v_max because
    peak velocity is 1.875/T vs 1.5/T.
    """

    PEAK_V_FACTOR = 1.875                  # 15/8
    PEAK_A_FACTOR = 10.0 / math.sqrt(3.0)  # ≈ 5.7735

    def __init__(self, q_start, q_goal, duration: float):
        super().__init__(q_start, q_goal)
        if duration <= 0.0:
            raise ValueError('duration must be positive')
        self.duration = float(duration)

    def _s_of_t(self, t: float):
        T = self.duration
        u = t / T
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        u5 = u4 * u
        s   = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
        sd  = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / T
        sdd = (60.0 * u  - 180.0 * u2 + 120.0 * u3) / (T * T)
        return s, sd, sdd

    @classmethod
    def with_limits(cls, q_start, q_goal, v_max, a_max,
                    safety_margin: float = 1.05) -> 'QuinticPolynomial':
        return _fit_polynomial(
            cls, q_start, q_goal, v_max, a_max,
            cls.PEAK_V_FACTOR, cls.PEAK_A_FACTOR, safety_margin,
        )


def _fit_polynomial(cls, q_start, q_goal, v_max, a_max,
                    peak_v_factor: float, peak_a_factor: float,
                    safety_margin: float):
    """Shared duration solver for CubicPolynomial / QuinticPolynomial.

    The peak velocity and peak |accel| of an `s(t)` profile have closed
    forms `peak_v_factor / T` and `peak_a_factor / T²` respectively. In
    joint space these are scaled by |Δq_i| per joint. We solve for the
    smallest T that satisfies every joint's v and a limit.
    """
    delta = _delta(q_start, q_goal)
    if not (len(v_max) == len(a_max) == len(delta)):
        raise ValueError('v_max, a_max, and joints must all match in length')
    if any(v <= 0.0 for v in v_max) or any(a <= 0.0 for a in a_max):
        raise ValueError('v_max / a_max entries must be positive')

    t_v = 0.0
    t_a = 0.0
    for i, d in enumerate(delta):
        ad = abs(d)
        if ad < 1e-12:
            continue
        t_v = max(t_v, peak_v_factor * ad / float(v_max[i]))
        t_a = max(t_a, math.sqrt(peak_a_factor * ad / float(a_max[i])))
    duration = max(t_v, t_a) * float(safety_margin)
    if duration <= 0.0:
        # All joints already at the goal — emit a 1-ms trajectory so the
        # discretiser still produces samples.
        duration = 1e-3
    return cls(q_start, q_goal, duration)


# ----------------------------------------------------------------------
#  LSPB / Linear segment with parabolic blends — Craig §7.5
# ----------------------------------------------------------------------

class LSPB(_StraightLineProfile):
    """Three-phase profile: parabolic accel → linear cruise → parabolic decel.

    Given total duration T and blend time t_b ∈ (0, T/2], the constant
    acceleration during the parabolic phases is

        a = 1 / (t_b · (T − t_b))     (in s-units; multiply by Δq for joint a)

    and the cruise velocity is ṡ = a · t_b. When t_b = T/2 the coast
    phase vanishes and the profile is a pure triangle (the time-optimal
    case at fixed T). Default `blend_frac = 1/3` matches Craig's
    illustrative example.
    """

    def __init__(self, q_start, q_goal, duration: float,
                 blend_frac: float = 1.0 / 3.0):
        super().__init__(q_start, q_goal)
        if duration <= 0.0:
            raise ValueError('duration must be positive')
        if not (0.0 < blend_frac <= 0.5):
            raise ValueError('blend_frac must be in (0, 0.5]')
        self.duration = float(duration)
        self.t_b = blend_frac * self.duration
        self._a_s   = 1.0 / (self.t_b * (self.duration - self.t_b))  # s̈ during ramp
        self._v_s   = self._a_s * self.t_b                           # ṡ during cruise

    def _s_of_t(self, t: float):
        T = self.duration
        t_b = self.t_b
        a = self._a_s
        v = self._v_s
        if t <= t_b:
            # Phase 1 — accelerate from rest.
            s   = 0.5 * a * t * t
            sd  = a * t
            sdd = a
        elif t < T - t_b:
            # Phase 2 — coast. s(t_b) = 0.5·a·t_b² and slope = v.
            s   = 0.5 * a * t_b * t_b + v * (t - t_b)
            sd  = v
            sdd = 0.0
        else:
            # Phase 3 — symmetric decel. End at s=1 with ṡ=0.
            tau = T - t
            s   = 1.0 - 0.5 * a * tau * tau
            sd  = a * tau
            sdd = -a
        return s, sd, sdd


# ----------------------------------------------------------------------
#  Time-optimal trapezoidal — MR §9.4 (path-parameterised, special case)
# ----------------------------------------------------------------------

class TimeOptimalTrapezoid(_StraightLineProfile):
    """Time-optimal ṡ(s) along a straight joint-space line.

    MR §9.4 derives the time-optimal time scaling along an arbitrary
    path σ(s) by integrating an ODE that saturates the joint
    velocity / acceleration limits. For a *straight* line in joint
    space the path is q(s) = q_start + s · Δq, so

        |q̇_i| ≤ v_max_i  ⇒  |ṡ| ≤ v_max_i / |Δq_i|
        |q̈_i| ≤ a_max_i  ⇒  |s̈| ≤ a_max_i / |Δq_i|

    The minimum-over-joints of these bounds is the limit on ṡ and s̈.
    The optimal profile is then a 1-D trapezoid on s(t):
    bang-coast-bang acceleration that saturates the most-constrained
    joint at every instant.

    Two cases:
      • If the segment is long enough to reach ṡ_max:
            t_acc = ṡ_max / s̈_max,
            d_acc = ½ · s̈_max · t_acc² = ½ · ṡ_max · t_acc,
            t_coast = (1 − 2·d_acc) / ṡ_max,
            T = 2·t_acc + t_coast.
      • Otherwise (triangular):
            t_acc = sqrt(1 / s̈_max),
            T = 2 · t_acc.

    This is the analytical answer to "given per-joint v / a bounds and
    a straight-line path, what is the shortest time to traverse it?"
    No numerical integration needed.
    """

    def __init__(
        self,
        q_start: Sequence[float],
        q_goal: Sequence[float],
        v_max: Sequence[float],
        a_max: Sequence[float],
        safety_margin: float = 1.05,
    ):
        super().__init__(q_start, q_goal)
        if not (len(v_max) == len(a_max) == self._n):
            raise ValueError('v_max / a_max must match joint count')
        if any(v <= 0.0 for v in v_max) or any(a <= 0.0 for a in a_max):
            raise ValueError('v_max / a_max must be positive')
        if safety_margin < 1.0:
            raise ValueError('safety_margin must be ≥ 1.0')

        # Tightest joint determines the s-frame bounds.
        sd_max = math.inf
        sdd_max = math.inf
        moving = False
        for i, d in enumerate(self.delta):
            ad = abs(d)
            if ad < 1e-12:
                continue
            moving = True
            sd_max  = min(sd_max,  float(v_max[i]) / ad)
            sdd_max = min(sdd_max, float(a_max[i]) / ad)

        if not moving:
            # No-op move; emit a 1 ms profile so discretise() still works.
            self.duration = 1e-3
            self._t_acc = 0.0
            self._t_coast = 0.0
            self._sd_peak = 0.0
            self._sdd_peak = 0.0
            return

        # Apply safety margin by scaling the *limits* down — this is
        # equivalent to scaling time up but cleaner because the same
        # v_max / a_max are then carried into the per-joint qd / qdd.
        sd_max  /= float(safety_margin)
        sdd_max /= float(safety_margin)

        # Distance-to-peak-velocity check.
        # Triangular if ½·s̈_max·t_acc² ≥ 0.5  (where t_acc = ṡ_max/s̈_max),
        # i.e., if 2·d_acc ≥ 1.
        t_acc_full = sd_max / sdd_max
        d_acc_full = 0.5 * sdd_max * t_acc_full * t_acc_full
        if 2.0 * d_acc_full >= 1.0:
            # Triangular — never reaches ṡ_max.
            t_acc = math.sqrt(1.0 / sdd_max)
            t_coast = 0.0
            sd_peak = sdd_max * t_acc
        else:
            t_acc = t_acc_full
            d_coast = 1.0 - 2.0 * d_acc_full
            t_coast = d_coast / sd_max
            sd_peak = sd_max

        self.duration = 2.0 * t_acc + t_coast
        self._t_acc = t_acc
        self._t_coast = t_coast
        self._sd_peak = sd_peak
        self._sdd_peak = sdd_max

    def _s_of_t(self, t: float):
        t_acc   = self._t_acc
        t_coast = self._t_coast
        T = self.duration
        a = self._sdd_peak
        v = self._sd_peak
        if t <= t_acc:
            s   = 0.5 * a * t * t
            sd  = a * t
            sdd = a
        elif t < t_acc + t_coast:
            s   = 0.5 * a * t_acc * t_acc + v * (t - t_acc)
            sd  = v
            sdd = 0.0
        else:
            tau = T - t
            s   = 1.0 - 0.5 * a * tau * tau
            sd  = a * tau
            sdd = -a
        return s, sd, sdd


# ----------------------------------------------------------------------
#  ROS bridge — convert samples to a JointTrajectory message
# ----------------------------------------------------------------------

def to_joint_trajectory_msg(
    samples: Iterable[TrajectorySample],
    joint_names: Sequence[str],
    frame_id: str = '',
):
    """Build a `trajectory_msgs/msg/JointTrajectory` from samples.

    Imports ROS messages lazily so this module can be unit-tested
    without a ROS environment (e.g. during pure-math validation).
    """
    from builtin_interfaces.msg import Duration as DurationMsg
    from std_msgs.msg import Header
    from trajectory_msgs.msg import (
        JointTrajectory as JointTrajectoryMsg,
        JointTrajectoryPoint,
    )

    msg = JointTrajectoryMsg()
    hdr = Header()
    hdr.frame_id = frame_id
    msg.header = hdr
    msg.joint_names = list(joint_names)

    for s in samples:
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in s.q]
        pt.velocities = [float(x) for x in s.qd]
        pt.accelerations = [float(x) for x in s.qdd]
        d = DurationMsg()
        d.sec = int(s.t)
        d.nanosec = int(round((s.t - int(s.t)) * 1e9))
        # Guard against 1e9 from rounding edge cases.
        if d.nanosec >= 1_000_000_000:
            d.sec += 1
            d.nanosec -= 1_000_000_000
        pt.time_from_start = d
        msg.points.append(pt)
    return msg


# ----------------------------------------------------------------------
#  Convenience factory used by the local trajectory client
# ----------------------------------------------------------------------

def build(
    method: str,
    q_start: Sequence[float],
    q_goal: Sequence[float],
    v_max: Sequence[float],
    a_max: Sequence[float],
    duration: Optional[float] = None,
    blend_frac: float = 1.0 / 3.0,
    safety_margin: float = 1.05,
) -> _StraightLineProfile:
    """Construct one of the four profiles by name.

      method ∈ {"cubic", "quintic", "lspb", "trapezoid"}.
      For "lspb", `duration` is required (LSPB doesn't auto-fit).
      For everything else, `duration` overrides the limit-based fit.
    """
    m = method.lower().strip()
    if m == 'cubic':
        if duration is not None:
            return CubicPolynomial(q_start, q_goal, duration)
        return CubicPolynomial.with_limits(q_start, q_goal, v_max, a_max,
                                           safety_margin=safety_margin)
    if m == 'quintic':
        if duration is not None:
            return QuinticPolynomial(q_start, q_goal, duration)
        return QuinticPolynomial.with_limits(q_start, q_goal, v_max, a_max,
                                             safety_margin=safety_margin)
    if m == 'lspb':
        if duration is None:
            # Fit a feasible duration by reducing to a trapezoid: same
            # peak velocity at blend_frac=1/3 reaches ṡ = 1.5/T (cubic-
            # like). Use the cubic formula as a stand-in for sizing.
            dummy = CubicPolynomial.with_limits(
                q_start, q_goal, v_max, a_max, safety_margin=safety_margin,
            )
            duration = dummy.duration
        return LSPB(q_start, q_goal, duration, blend_frac=blend_frac)
    if m in ('trapezoid', 'trapezoidal', 'time-optimal', 'optimal'):
        return TimeOptimalTrapezoid(q_start, q_goal, v_max, a_max,
                                    safety_margin=safety_margin)
    raise ValueError(
        f'unknown trajectory method "{method}"; '
        f'choose cubic / quintic / lspb / trapezoid'
    )


# ----------------------------------------------------------------------
#  CLI for inspection: print samples and peak metrics
# ----------------------------------------------------------------------

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='Trajectory generator probe.')
    ap.add_argument('--method', default='trapezoid',
                    choices=['cubic', 'quintic', 'lspb', 'trapezoid'])
    ap.add_argument('--start', nargs='+', type=float,
                    default=[0.0, 0.0, 0.0, 0.0, 0.0])
    ap.add_argument('--goal', nargs='+', type=float,
                    default=[0.6, -0.4, 0.3, 0.5, -0.2])
    ap.add_argument('--vmax', nargs='+', type=float,
                    default=[0.3, 0.3, 0.3, 0.3, 0.3])
    ap.add_argument('--amax', nargs='+', type=float,
                    default=[0.3, 0.3, 0.3, 0.3, 0.3])
    ap.add_argument('--dt', type=float, default=0.05)
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--blend', type=float, default=1.0 / 3.0)
    args = ap.parse_args()

    traj = build(
        args.method, args.start, args.goal, args.vmax, args.amax,
        duration=args.duration, blend_frac=args.blend,
    )
    samples = traj.discretise(dt=args.dt)
    print(f'method   : {args.method}')
    print(f'duration : {traj.duration:.4f} s')
    print(f'samples  : {len(samples)}')
    print()
    print(f'{"t":>6} | ' + '  '.join(f'q{i}'.rjust(7) for i in range(len(args.start)))
          + ' || ' + '  '.join(f'qd{i}'.rjust(7) for i in range(len(args.start))))
    for s in samples:
        ts = f'{s.t:6.3f}'
        qs = '  '.join(f'{v:+7.3f}' for v in s.q)
        qds = '  '.join(f'{v:+7.3f}' for v in s.qd)
        print(f'{ts} | {qs} || {qds}')

    # Peak metrics
    peak_v = [0.0] * len(args.start)
    peak_a = [0.0] * len(args.start)
    for s in samples:
        for i, (v, a) in enumerate(zip(s.qd, s.qdd)):
            if abs(v) > peak_v[i]: peak_v[i] = abs(v)
            if abs(a) > peak_a[i]: peak_a[i] = abs(a)
    print()
    print('peak |qd|  per joint:', '  '.join(f'{v:.4f}' for v in peak_v))
    print('peak |qdd| per joint:', '  '.join(f'{a:.4f}' for a in peak_a))
