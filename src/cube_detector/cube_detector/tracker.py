"""
Multi-cube identity tracker.

Two algorithms behind a single interface — both consume CubeDetection lists
and emit Track objects with stable per-cube IDs across frames:

  - kalman_hungarian : SORT-style. Constant-velocity Kalman per track,
                       Mahalanobis gate + Hungarian assignment, robust to
                       brief occlusion via predict-only "coasting" frames.
  - greedy_nn        : Plain Euclidean nearest-neighbour, one-shot per
                       frame. ~30 lines of association logic. Cheap, but
                       swaps IDs when cubes pass close or one is occluded.

Both algorithms share the lifecycle (tentative → confirmed → coasting →
deleted) and the Track output type, so callers can swap algorithms via a
single config string.

Pure Python — no ROS deps. The detector_node wires this into ROS topics.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .detector import CubeDetection


# ----------------------------------------------------------------------
# Output type — common to both trackers
# ----------------------------------------------------------------------
@dataclass
class Track:
    id: int                          # stable monotonic ID (resets on node restart)
    position: np.ndarray             # (3,) filtered xyz, metres, tracker frame
    velocity: np.ndarray             # (3,) filtered velocity m/s; zeros until 2nd hit
    latest_R: np.ndarray             # (3,3) latest measurement's rotation (no Kalman, see §14.2)
    score: float                     # latest detection score
    hits: int                        # total measurements observed
    misses: int                      # consecutive missed frames since last hit
    age: int                         # total frames since first detection
    last_stamp: float                # last update time (sec)
    status: str = 'tentative'        # 'tentative' | 'confirmed' | 'coasting'


# ----------------------------------------------------------------------
# Constant-velocity Kalman state for one cube
# ----------------------------------------------------------------------
class _CVKalman:
    """6-state constant-velocity Kalman with white-noise acceleration.

    State x = [px, py, pz, vx, vy, vz].
    Measurement z = [px, py, pz].
    """

    __slots__ = ('x', 'P', '_R', '_proc_var')

    def __init__(self, init_pos: np.ndarray, meas_var_xy: float,
                 meas_var_z: float, proc_accel_var: float,
                 init_vel_var: float = 1.0):
        self.x = np.zeros(6, dtype=np.float64)
        self.x[:3] = init_pos
        # P0: position uncertainty = measurement noise; velocity uncertainty = init_vel_var (m/s)²
        self.P = np.diag([meas_var_xy, meas_var_xy, meas_var_z,
                          init_vel_var, init_vel_var, init_vel_var]).astype(np.float64)
        self._R = np.diag([meas_var_xy, meas_var_xy, meas_var_z]).astype(np.float64)
        self._proc_var = float(proc_accel_var)

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        # White-noise accel: Q = q * G G^T,  G = [dt²/2 I_3 ; dt I_3]
        g_top = 0.5 * dt * dt
        G = np.zeros((6, 3), dtype=np.float64)
        G[:3, :] = g_top * np.eye(3)
        G[3:, :] = dt * np.eye(3)
        Q = self._proc_var * (G @ G.T)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray) -> None:
        # H selects position out of 6-state
        H = np.zeros((3, 6), dtype=np.float64)
        H[:3, :3] = np.eye(3)
        S = H @ self.P @ H.T + self._R
        K = self.P @ H.T @ np.linalg.inv(S)
        y = z - H @ self.x
        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float64)
        self.P = (I - K @ H) @ self.P

    def innovation_cov(self) -> np.ndarray:
        H = np.zeros((3, 6), dtype=np.float64)
        H[:3, :3] = np.eye(3)
        return H @ self.P @ H.T + self._R

    def predicted_pos(self) -> np.ndarray:
        return self.x[:3].copy()

    def predicted_vel(self) -> np.ndarray:
        return self.x[3:].copy()


# ----------------------------------------------------------------------
# Common base — implements lifecycle + ID assignment
# ----------------------------------------------------------------------
class _BaseTracker:
    """Shared lifecycle: tentative→confirmed→coasting→deleted."""

    def __init__(self, *, min_hits: int, min_hits_window: int,
                 max_misses: int):
        self.min_hits = int(min_hits)
        self.min_hits_window = int(min_hits_window)
        self.max_misses = int(max_misses)
        self._next_id = 0

    def _alloc_id(self) -> int:
        i = self._next_id
        self._next_id += 1
        return i

    @staticmethod
    def _refresh_status(t: Track, *, min_hits: int, min_hits_window: int,
                        max_misses: int) -> None:
        if t.status == 'tentative':
            if t.hits >= min_hits:
                t.status = 'confirmed'
            elif t.age >= min_hits_window:
                t.status = 'deleted'
        elif t.status == 'confirmed':
            if t.misses > 0:
                t.status = 'coasting'
        elif t.status == 'coasting':
            if t.misses == 0:
                t.status = 'confirmed'
            elif t.misses > max_misses:
                t.status = 'deleted'


# ----------------------------------------------------------------------
# Algorithm 1: SORT-style Kalman + Hungarian
# ----------------------------------------------------------------------
class KalmanHungarianTracker(_BaseTracker):
    """Per-cube CV Kalman + Mahalanobis gating + Hungarian assignment."""

    def __init__(self,
                 *,
                 meas_std_xy: float = 0.005,
                 meas_std_z: float = 0.007,
                 proc_accel_std: float = 0.05,
                 gate_chi2: float = 11.34,
                 min_hits: int = 3,
                 min_hits_window: int = 5,
                 max_misses: int = 10):
        super().__init__(min_hits=min_hits,
                         min_hits_window=min_hits_window,
                         max_misses=max_misses)
        self._meas_var_xy = float(meas_std_xy) ** 2
        self._meas_var_z = float(meas_std_z) ** 2
        self._proc_var = float(proc_accel_std) ** 2
        self.gate_chi2 = float(gate_chi2)
        self._tracks: List[Track] = []
        self._kalmans: List[_CVKalman] = []  # parallel list

    @property
    def algorithm(self) -> str:
        return 'kalman_hungarian'

    def update(self, detections: Sequence[CubeDetection],
               stamp: float) -> List[Track]:
        # 1) Predict every existing track to the new timestamp.
        for t, kf in zip(self._tracks, self._kalmans):
            dt = max(0.0, stamp - t.last_stamp)
            kf.predict(dt)

        n_t = len(self._tracks)
        n_d = len(detections)

        matches: List[Tuple[int, int]] = []
        unmatched_t: List[int] = list(range(n_t))
        unmatched_d: List[int] = list(range(n_d))

        # 2) Mahalanobis-gated Hungarian — only if both sides have entries.
        if n_t > 0 and n_d > 0:
            from scipy.optimize import linear_sum_assignment
            BIG = 1.0e6
            cost = np.full((n_t, n_d), BIG, dtype=np.float64)
            for i, kf in enumerate(self._kalmans):
                mu = kf.predicted_pos()
                S_inv = np.linalg.inv(kf.innovation_cov())
                for j, det in enumerate(detections):
                    y = np.asarray(det.cube_center, dtype=np.float64) - mu
                    m2 = float(y @ S_inv @ y)
                    if m2 < self.gate_chi2:
                        cost[i, j] = m2

            row_ind, col_ind = linear_sum_assignment(cost)
            matched_t = set()
            matched_d = set()
            for r, c in zip(row_ind, col_ind):
                if cost[r, c] >= self.gate_chi2:
                    continue  # gated out, treat as unmatched
                matches.append((int(r), int(c)))
                matched_t.add(int(r))
                matched_d.add(int(c))
            unmatched_t = [i for i in range(n_t) if i not in matched_t]
            unmatched_d = [j for j in range(n_d) if j not in matched_d]

        # 3) Apply measurement updates to matched tracks.
        for ti, dj in matches:
            t = self._tracks[ti]
            kf = self._kalmans[ti]
            det = detections[dj]
            kf.update(np.asarray(det.cube_center, dtype=np.float64))
            t.position = kf.predicted_pos()
            t.velocity = kf.predicted_vel()
            t.latest_R = np.asarray(det.rotation, dtype=np.float64).copy()
            t.score = float(det.score)
            t.hits += 1
            t.misses = 0
            t.age += 1
            t.last_stamp = stamp
            self._refresh_status(t, min_hits=self.min_hits,
                                 min_hits_window=self.min_hits_window,
                                 max_misses=self.max_misses)

        # 4) Coast unmatched tracks (predict-only — already predicted in step 1).
        for ti in unmatched_t:
            t = self._tracks[ti]
            kf = self._kalmans[ti]
            t.position = kf.predicted_pos()
            t.velocity = kf.predicted_vel()
            t.misses += 1
            t.age += 1
            self._refresh_status(t, min_hits=self.min_hits,
                                 min_hits_window=self.min_hits_window,
                                 max_misses=self.max_misses)

        # 5) Spawn new tracks from unmatched detections.
        for dj in unmatched_d:
            det = detections[dj]
            kf = _CVKalman(np.asarray(det.cube_center, dtype=np.float64),
                           self._meas_var_xy, self._meas_var_z, self._proc_var)
            t = Track(
                id=self._alloc_id(),
                position=kf.predicted_pos(),
                velocity=kf.predicted_vel(),
                latest_R=np.asarray(det.rotation, dtype=np.float64).copy(),
                score=float(det.score),
                hits=1, misses=0, age=1,
                last_stamp=stamp, status='tentative',
            )
            self._refresh_status(t, min_hits=self.min_hits,
                                 min_hits_window=self.min_hits_window,
                                 max_misses=self.max_misses)
            self._tracks.append(t)
            self._kalmans.append(kf)

        # 6) Drop deleted (in reverse so indexes hold).
        for i in range(len(self._tracks) - 1, -1, -1):
            if self._tracks[i].status == 'deleted':
                del self._tracks[i]
                del self._kalmans[i]

        return [t for t in self._tracks
                if t.status in ('confirmed', 'coasting')]

    @property
    def all_tracks(self) -> List[Track]:
        return list(self._tracks)


# ----------------------------------------------------------------------
# Algorithm 2: greedy nearest-neighbour
# ----------------------------------------------------------------------
class GreedyNNTracker(_BaseTracker):
    """Plain Euclidean NN. No filter; track position = latest measurement.

    Velocity is exponentially-smoothed (vel_alpha) finite difference, kept
    only so the Track output schema matches the Kalman variant.
    """

    def __init__(self,
                 *,
                 max_jump_m: float = 0.05,
                 vel_alpha: float = 0.4,
                 min_hits: int = 3,
                 min_hits_window: int = 5,
                 max_misses: int = 10):
        super().__init__(min_hits=min_hits,
                         min_hits_window=min_hits_window,
                         max_misses=max_misses)
        self.max_jump = float(max_jump_m)
        self.vel_alpha = float(vel_alpha)
        self._tracks: List[Track] = []

    @property
    def algorithm(self) -> str:
        return 'greedy_nn'

    def update(self, detections: Sequence[CubeDetection],
               stamp: float) -> List[Track]:
        n_t = len(self._tracks)
        n_d = len(detections)

        matches: List[Tuple[int, int]] = []
        used_t: set = set()
        used_d: set = set()

        if n_t > 0 and n_d > 0:
            D = np.full((n_t, n_d), np.inf, dtype=np.float64)
            for i, t in enumerate(self._tracks):
                for j, det in enumerate(detections):
                    d = float(np.linalg.norm(
                        np.asarray(det.cube_center, dtype=np.float64) - t.position
                    ))
                    if d < self.max_jump:
                        D[i, j] = d
            # Greedy: take the smallest entry; cross out its row + column; repeat.
            while True:
                if not np.isfinite(D).any():
                    break
                idx = int(np.argmin(D))
                i, j = divmod(idx, n_d)
                if not np.isfinite(D[i, j]):
                    break
                matches.append((i, j))
                used_t.add(i)
                used_d.add(j)
                D[i, :] = np.inf
                D[:, j] = np.inf

        unmatched_t = [i for i in range(n_t) if i not in used_t]
        unmatched_d = [j for j in range(n_d) if j not in used_d]

        # Update matched
        for ti, dj in matches:
            t = self._tracks[ti]
            det = detections[dj]
            new_pos = np.asarray(det.cube_center, dtype=np.float64)
            dt = max(1e-3, stamp - t.last_stamp)
            v_inst = (new_pos - t.position) / dt
            t.velocity = self.vel_alpha * v_inst + (1.0 - self.vel_alpha) * t.velocity
            t.position = new_pos
            t.latest_R = np.asarray(det.rotation, dtype=np.float64).copy()
            t.score = float(det.score)
            t.hits += 1
            t.misses = 0
            t.age += 1
            t.last_stamp = stamp
            self._refresh_status(t, min_hits=self.min_hits,
                                 min_hits_window=self.min_hits_window,
                                 max_misses=self.max_misses)

        # Coast unmatched (no Kalman, so position holds steady at last value)
        for ti in unmatched_t:
            t = self._tracks[ti]
            t.misses += 1
            t.age += 1
            self._refresh_status(t, min_hits=self.min_hits,
                                 min_hits_window=self.min_hits_window,
                                 max_misses=self.max_misses)

        # Spawn new tracks
        for dj in unmatched_d:
            det = detections[dj]
            t = Track(
                id=self._alloc_id(),
                position=np.asarray(det.cube_center, dtype=np.float64).copy(),
                velocity=np.zeros(3, dtype=np.float64),
                latest_R=np.asarray(det.rotation, dtype=np.float64).copy(),
                score=float(det.score),
                hits=1, misses=0, age=1,
                last_stamp=stamp, status='tentative',
            )
            self._refresh_status(t, min_hits=self.min_hits,
                                 min_hits_window=self.min_hits_window,
                                 max_misses=self.max_misses)
            self._tracks.append(t)

        # Drop deleted
        self._tracks = [t for t in self._tracks if t.status != 'deleted']

        return [t for t in self._tracks
                if t.status in ('confirmed', 'coasting')]

    @property
    def all_tracks(self) -> List[Track]:
        return list(self._tracks)


# ----------------------------------------------------------------------
# Factory
# ----------------------------------------------------------------------
def make_tracker(algorithm: str, **kwargs):
    """Instantiate a tracker by name. Raises ValueError on unknown name."""
    a = algorithm.lower().strip()
    if a in ('kalman_hungarian', 'kalman', 'sort'):
        # Filter kwargs to only those accepted by KalmanHungarianTracker
        allowed = {'meas_std_xy', 'meas_std_z', 'proc_accel_std',
                   'gate_chi2', 'min_hits', 'min_hits_window', 'max_misses'}
        return KalmanHungarianTracker(**{k: v for k, v in kwargs.items()
                                         if k in allowed})
    if a in ('greedy_nn', 'greedy', 'nn'):
        allowed = {'max_jump_m', 'vel_alpha',
                   'min_hits', 'min_hits_window', 'max_misses'}
        return GreedyNNTracker(**{k: v for k, v in kwargs.items()
                                  if k in allowed})
    raise ValueError(
        f"Unknown tracker algorithm '{algorithm}'. "
        f"Use 'kalman_hungarian' or 'greedy_nn'."
    )
