"""
Pure-Python cube detector.  No ROS dependencies — see detector_node.py for the
ROS wrapper.  Tunable via the constructor.

Pipeline
--------
1. RANSAC plane fit on a sub-sampled point cloud → table plane (n, d).
2. Per-pixel height above table.
3. Mask = height in [h_min, cube_size + tol]  → connected components.
4. Filter components by predicted pixel area at their median depth.
5. For each surviving candidate, extract the top-face polygon (height ≈ cube_size)
   and approximate to a quadrilateral.
6. Optional edge-based corner refinement on the colour image.
7. Recover 3D pose by intersecting each corner ray with the top-face plane.

Frame conventions
-----------------
- Camera optical frame: X right, Y down, Z forward.
- Table plane:  n · p + d = 0, with n forced to point AWAY from the table
  (n.z < 0 in optical coords).  Height above table = n·p + d > 0.
- Cube +Z = same direction as n (out of the top face).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class CubeDetection:
    cube_center: np.ndarray          # (3,)  geometric centre, metres, camera frame
    top_center: np.ndarray           # (3,)  centre of top face
    rotation: np.ndarray             # (3,3) cube axes as columns in camera frame
    table_normal: np.ndarray         # (3,)  unit normal of table, points up
    top_face_corners_2d: np.ndarray  # (4,2) ordered top-face polygon, pixels
    score: float                     # 0..1  geometric confidence


class CubeDetector:
    def __init__(
        self,
        cube_size: float = 0.025,
        height_min: float = 0.005,
        height_max_tol: float = 0.010,
        ransac_iters: int = 200,
        ransac_threshold: float = 0.004,
        ransac_subsample: int = 4000,
        min_blob_area_px: int = 40,
        min_top_face_px: int = 30,
        edge_refine: bool = True,
        rng_seed: Optional[int] = None,
    ):
        self.cube_size = float(cube_size)
        self.h_min = float(height_min)
        self.h_max_tol = float(height_max_tol)
        self.ransac_iters = int(ransac_iters)
        self.ransac_threshold = float(ransac_threshold)
        self.ransac_subsample = int(ransac_subsample)
        self.min_blob_area_px = int(min_blob_area_px)
        self.min_top_face_px = int(min_top_face_px)
        self.edge_refine = bool(edge_refine)
        self._rng = np.random.default_rng(rng_seed)

    # ------------------------------------------------------------------
    def detect(
        self, rgb: np.ndarray, depth_m: np.ndarray, K: np.ndarray
    ) -> Tuple[Optional[CubeDetection], dict]:
        debug: dict = {}
        if depth_m.ndim != 2:
            return None, debug

        plane = self._fit_table_plane(depth_m, K)
        if plane is None:
            debug['stage_failed'] = 'plane_fit'
            return None, debug
        n, d = plane
        debug['table_normal'] = n
        debug['table_d'] = d

        height_map = self._compute_height_map(depth_m, K, n, d)
        debug['height_map'] = height_map

        h_max = self.cube_size + self.h_max_tol
        above = (depth_m > 0) & (height_map > self.h_min) & (height_map < h_max)
        above_u8 = above.astype(np.uint8) * 255
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        above_u8 = cv2.morphologyEx(above_u8, cv2.MORPH_OPEN, kern, iterations=1)
        above_u8 = cv2.morphologyEx(above_u8, cv2.MORPH_CLOSE, kern, iterations=2)
        debug['above_mask'] = above_u8

        candidates = self._candidates_from_mask(above_u8, depth_m, K)
        debug['n_candidates'] = len(candidates)
        if not candidates:
            debug['stage_failed'] = 'no_candidates'
            return None, debug

        best: Optional[CubeDetection] = None
        for cand in candidates:
            quad_2d, score = self._extract_top_face(
                cand, height_map, rgb, K, n, d
            )
            if quad_2d is None:
                continue
            pose = self._pose_from_top_face(quad_2d, K, n, d)
            if pose is None:
                continue
            cube_center, top_center, R = pose
            det = CubeDetection(
                cube_center=cube_center,
                top_center=top_center,
                rotation=R,
                table_normal=n,
                top_face_corners_2d=quad_2d,
                score=score,
            )
            if best is None or det.score > best.score:
                best = det

        if best is None:
            debug['stage_failed'] = 'top_face_or_pose'
        return best, debug

    # ------------------------------------------------------------------
    # Plane fit
    # ------------------------------------------------------------------
    def _fit_table_plane(self, depth_m: np.ndarray, K: np.ndarray):
        ys, xs = np.where(depth_m > 0)
        if ys.size < 200:
            return None
        if ys.size > self.ransac_subsample:
            idx = self._rng.choice(ys.size, self.ransac_subsample, replace=False)
            ys, xs = ys[idx], xs[idx]
        Z = depth_m[ys, xs]
        fx, fy = K[0, 0], K[1, 1]
        cx0, cy0 = K[0, 2], K[1, 2]
        X = (xs - cx0) * Z / fx
        Y = (ys - cy0) * Z / fy
        pts = np.stack([X, Y, Z], axis=1)

        n_pts = pts.shape[0]
        best_inl = 0
        best: Optional[Tuple[np.ndarray, float, np.ndarray]] = None
        for _ in range(self.ransac_iters):
            tri = self._rng.choice(n_pts, 3, replace=False)
            p1, p2, p3 = pts[tri[0]], pts[tri[1]], pts[tri[2]]
            v = np.cross(p2 - p1, p3 - p1)
            ln = np.linalg.norm(v)
            if ln < 1e-6:
                continue
            v = v / ln
            d_ = -float(np.dot(v, p1))
            err = np.abs(pts @ v + d_)
            inl = err < self.ransac_threshold
            cnt = int(inl.sum())
            if cnt > best_inl:
                best_inl = cnt
                best = (v, d_, inl)

        if best is None or best_inl < 100:
            return None

        # Refine plane via SVD on inliers
        _, _, inl = best
        inl_pts = pts[inl]
        c = inl_pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(inl_pts - c, full_matrices=False)
        normal = Vt[-1]
        d_ = -float(np.dot(normal, c))
        # Force normal to point toward camera (negative Z in optical frame)
        if normal[2] > 0:
            normal = -normal
            d_ = -d_
        return normal.astype(np.float64), float(d_)

    # ------------------------------------------------------------------
    @staticmethod
    def _compute_height_map(depth_m: np.ndarray, K: np.ndarray,
                            n: np.ndarray, d: float) -> np.ndarray:
        H, W = depth_m.shape
        fx, fy = K[0, 0], K[1, 1]
        cx0, cy0 = K[0, 2], K[1, 2]
        u = np.arange(W, dtype=np.float32)
        v = np.arange(H, dtype=np.float32)
        U, V = np.meshgrid(u, v)
        Z = depth_m.astype(np.float32)
        X = (U - cx0) * Z / fx
        Y = (V - cy0) * Z / fy
        height = X * n[0] + Y * n[1] + Z * n[2] + d
        height[Z == 0] = 0.0
        return height

    # ------------------------------------------------------------------
    def _candidates_from_mask(self, mask_u8: np.ndarray, depth_m: np.ndarray,
                              K: np.ndarray):
        n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, 8)
        out = []
        fx = K[0, 0]
        for i in range(1, n_lab):
            x, y, w, h, area = stats[i]
            if area < self.min_blob_area_px:
                continue
            comp = (labels == i)
            zs = depth_m[comp]
            zs = zs[zs > 0]
            if zs.size == 0:
                continue
            Z = float(np.median(zs))
            if Z < 0.05 or Z > 2.0:
                continue
            # Projected size at this depth.  Top-down view of a cube top face
            # is ≈ (s*fx/Z)^2 px²; full silhouette can be up to ~3x that for an
            # angled view (3 visible faces).  Allow a wide band.
            pred = (self.cube_size * fx / Z) ** 2
            if area < 0.25 * pred or area > 5.0 * pred:
                continue
            out.append({
                'mask': comp, 'Z': Z, 'area': int(area), 'pred': pred,
                'bbox': (int(x), int(y), int(w), int(h)),
            })
        # Best size match first
        out.sort(key=lambda c: abs(np.log(c['area'] / max(c['pred'], 1.0))))
        return out

    # ------------------------------------------------------------------
    def _extract_top_face(self, cand, height_map, rgb, K, n, d):
        comp = cand['mask']
        comp_h = height_map[comp]
        if comp_h.size == 0:
            return None, 0.0

        # Adaptive top-of-cube height from the candidate itself
        top_h = float(np.percentile(comp_h, 92))
        # Constrain to physically plausible range
        top_h = max(self.cube_size - 0.010,
                    min(self.cube_size + 0.010, top_h))

        tol = 0.005
        face_mask = (height_map > top_h - tol) & (height_map < top_h + tol) & comp
        if int(face_mask.sum()) < self.min_top_face_px:
            tol = max(0.008, self.h_max_tol)
            face_mask = (height_map > self.cube_size - tol) & \
                        (height_map < self.cube_size + tol) & comp
            if int(face_mask.sum()) < self.min_top_face_px:
                return None, 0.0

        face_u8 = face_mask.astype(np.uint8) * 255
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        face_u8 = cv2.morphologyEx(face_u8, cv2.MORPH_CLOSE, kern, iterations=2)

        contours, _ = cv2.findContours(
            face_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, 0.0
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.min_top_face_px:
            return None, 0.0

        # Approximate to a quadrilateral; fall back to minAreaRect
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            quad = approx.reshape(4, 2).astype(np.float32)
        else:
            rect = cv2.minAreaRect(contour)
            quad = cv2.boxPoints(rect).astype(np.float32)

        quad = self._order_quad(quad)
        if self.edge_refine:
            quad = self._refine_with_edges(quad, rgb)

        score = self._score_quad(quad, K, n, d - self.cube_size)
        return quad, score

    # ------------------------------------------------------------------
    def _refine_with_edges(self, quad: np.ndarray, rgb: np.ndarray) -> np.ndarray:
        if rgb is None or rgb.size == 0:
            return quad
        try:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        except cv2.error:
            return quad
        gray = cv2.bilateralFilter(gray, 5, 50, 50)
        try:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01)
            refined = cv2.cornerSubPix(
                gray, quad.copy(), (5, 5), (-1, -1), term
            )
        except cv2.error:
            return quad
        # Reject moves > 4 px (subpixel snap should be small)
        if np.all(np.linalg.norm(refined - quad, axis=1) < 4.0):
            return refined
        return quad

    # ------------------------------------------------------------------
    @staticmethod
    def _order_quad(quad: np.ndarray) -> np.ndarray:
        """Order 4 points CCW around their centroid."""
        c = quad.mean(axis=0)
        ang = np.arctan2(quad[:, 1] - c[1], quad[:, 0] - c[0])
        return quad[np.argsort(ang)]

    # ------------------------------------------------------------------
    def _score_quad(self, quad_2d: np.ndarray, K: np.ndarray,
                    n: np.ndarray, d_top: float) -> float:
        if quad_2d is None or len(quad_2d) != 4:
            return 0.0
        c3d = self._rays_to_plane(quad_2d, K, n, d_top)
        if c3d is None:
            return 0.0

        sides = np.linalg.norm(
            np.diff(np.vstack([c3d, c3d[0:1]]), axis=0), axis=1
        )
        if np.any(sides < 0.005):
            return 0.0
        mean_s = float(np.mean(sides))
        side_match = 1.0 - min(1.0, float(np.std(sides)) / mean_s)
        size_match = 1.0 - min(
            1.0, abs(mean_s - self.cube_size) / self.cube_size
        )

        # 90° corner check
        devs = []
        for i in range(4):
            a = c3d[i] - c3d[(i - 1) % 4]
            b = c3d[(i + 1) % 4] - c3d[i]
            la, lb = np.linalg.norm(a), np.linalg.norm(b)
            if la < 1e-6 or lb < 1e-6:
                return 0.0
            cos_ = float(np.clip(np.dot(a, b) / (la * lb), -1.0, 1.0))
            devs.append(abs(np.arccos(cos_) - np.pi / 2.0))
        ang_match = 1.0 - min(1.0, float(np.mean(devs)) / (np.pi / 4.0))

        return float(0.4 * size_match + 0.3 * side_match + 0.3 * ang_match)

    # ------------------------------------------------------------------
    @staticmethod
    def _rays_to_plane(uv: np.ndarray, K: np.ndarray,
                       n: np.ndarray, d_plane: float) -> Optional[np.ndarray]:
        fx, fy = K[0, 0], K[1, 1]
        cx0, cy0 = K[0, 2], K[1, 2]
        out = np.empty((len(uv), 3), dtype=np.float64)
        for i, (u, v) in enumerate(uv):
            ray = np.array([(u - cx0) / fx, (v - cy0) / fy, 1.0])
            denom = float(np.dot(n, ray))
            if abs(denom) < 1e-6:
                return None
            t = -d_plane / denom
            if t <= 0:
                return None
            out[i] = t * ray
        return out

    # ------------------------------------------------------------------
    def _pose_from_top_face(self, quad_2d, K, n, d):
        c3d = self._rays_to_plane(quad_2d, K, n, d - self.cube_size)
        if c3d is None:
            return None
        top_center = c3d.mean(axis=0)
        cube_center = top_center - 0.5 * self.cube_size * n  # n points up; cube body is below

        z_axis = n / np.linalg.norm(n)
        edge = c3d[1] - c3d[0]
        edge = edge - np.dot(edge, z_axis) * z_axis
        ln = np.linalg.norm(edge)
        if ln < 1e-6:
            return None
        x_axis = edge / ln
        y_axis = np.cross(z_axis, x_axis)
        R = np.stack([x_axis, y_axis, z_axis], axis=1)
        return cube_center, top_center, R


# ----------------------------------------------------------------------
# Helpers used by the ROS node for visualisation / messaging
# ----------------------------------------------------------------------
def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Return quaternion (x, y, z, w) from a 3x3 rotation matrix."""
    m = np.asarray(R, dtype=np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def draw_debug(rgb: np.ndarray, det: Optional[CubeDetection], debug: dict) -> np.ndarray:
    """Annotate an RGB image for visualization on the debug topic."""
    out = rgb.copy()
    above = debug.get('above_mask')
    if above is not None:
        # Tint above-table pixels lightly
        tint = np.zeros_like(out)
        tint[..., 1] = above
        out = cv2.addWeighted(out, 1.0, tint, 0.25, 0)

    if det is not None:
        pts = det.top_face_corners_2d.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        for p in det.top_face_corners_2d:
            cv2.circle(out, (int(p[0]), int(p[1])), 3, (0, 0, 255), -1)
        cx = int(det.top_face_corners_2d[:, 0].mean())
        cy = int(det.top_face_corners_2d[:, 1].mean())
        cv2.drawMarker(out, (cx, cy), (0, 255, 255),
                       cv2.MARKER_CROSS, 14, 2)
        text = (
            f"score={det.score:.2f}  "
            f"xyz=({det.cube_center[0]*100:.1f},"
            f"{det.cube_center[1]*100:.1f},"
            f"{det.cube_center[2]*100:.1f})cm"
        )
        cv2.putText(out, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(out, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        stage = debug.get('stage_failed', 'unknown')
        cv2.putText(out, f"no detection ({stage})", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
    return out
