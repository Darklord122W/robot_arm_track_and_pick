"""Generate matplotlib figures embedded in xarm_pick_report.pdf."""
from __future__ import annotations
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'report_figs')
os.makedirs(OUT, exist_ok=True)

# Make the IK module importable.
SRC = os.path.normpath(os.path.join(HERE, '..', 'src', 'xarm_pick'))
sys.path.insert(0, SRC)
from xarm_pick.arm_ik import (  # type: ignore
    fk_tool0, JOINT_ORDER, JOINT_LIMITS, solve_ik,
)


# ----------------------------------------------------------------------
#  Figure 1 — kinematic chain side view at home pose
# ----------------------------------------------------------------------

def fig_kinematic_chain():
    """Side-view stick figure of the URDF chain at q = 0 (home)."""
    # Link origins in world at home, z up, x forward (we render x-z plane).
    # (Numbers from the screw-axes derivation — see arm_ik.py SCREW_AXES comments.)
    pts = np.array([
        [0.000,   0.043],   # world → base_link
        [0.000,   0.086],   # link6 origin
        [-0.002,  0.118],   # link5 origin
        [-0.002,  0.21575], # link4 origin
        [-0.002,  0.31475], # link3 origin
        [-0.00075, 0.36475],# link2 origin
        [0.00305, 0.42675], # tool0 origin
    ])
    labels = ['world', 'link6', 'link5', 'link4', 'link3', 'link2', 'tool0']

    fig, ax = plt.subplots(figsize=(4.0, 5.5))
    ax.plot(pts[:, 0], pts[:, 1], '-o', color='#1f77b4', lw=2, markersize=8)
    for (x, z), lbl in zip(pts, labels):
        ax.annotate(lbl, (x, z), textcoords='offset points',
                    xytext=(10, -3), fontsize=9, color='#333333')
    # ground line
    ax.axhline(0.0, color='gray', lw=0.5, linestyle='--')
    ax.text(0.025, -0.005, 'world frame', fontsize=8, color='gray', va='top')
    ax.set_xlim(-0.05, 0.10)
    ax.set_ylim(-0.02, 0.48)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('z (m)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_title('xArm 1S — kinematic chain at home pose')
    fig.tight_layout()
    out = os.path.join(OUT, 'kinematic_chain.png')
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


# ----------------------------------------------------------------------
#  Figure 2 — gripper-down workspace map
# ----------------------------------------------------------------------

def fig_workspace():
    """Sweep XY at fixed table-z, color by IK success and gripper tilt."""
    # Coarse to keep runtime sane (each call → 24-restart Newton).
    xs = np.linspace(-0.22, 0.22, 23)
    ys = np.linspace(-0.05, 0.30, 19)
    Z = 0.11
    grid_succ = np.full((len(ys), len(xs)), np.nan)
    grid_tilt = np.full((len(ys), len(xs)), np.nan)
    for i, y in enumerate(ys):
        for j, x in enumerate(xs):
            r = solve_ik((float(x), float(y), Z), n_starts=8)
            if r.success:
                grid_succ[i, j] = 1.0
                grid_tilt[i, j] = float(np.degrees(r.z_err_rad))

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 4.0))

    # Left: success map.
    ax = axes[0]
    im = ax.imshow(np.where(np.isnan(grid_succ), 0.0, 1.0),
                   extent=[xs.min(), xs.max(), ys.min(), ys.max()],
                   origin='lower', cmap='Greens', aspect='equal',
                   vmin=0.0, vmax=1.0)
    ax.set_title(f'IK success @ z = {Z:.2f} m')
    ax.set_xlabel('world X (m)')
    ax.set_ylabel('world Y (m)')
    ax.scatter([0.0], [0.0], c='red', marker='+', s=80, label='base')
    ax.legend(loc='upper right', fontsize=8)

    # Right: tilt map.
    ax = axes[1]
    im2 = ax.imshow(grid_tilt,
                    extent=[xs.min(), xs.max(), ys.min(), ys.max()],
                    origin='lower', cmap='magma_r', aspect='equal',
                    vmin=0.0, vmax=45.0)
    ax.set_title('gripper tilt off-vertical (deg)')
    ax.set_xlabel('world X (m)')
    ax.set_ylabel('world Y (m)')
    cbar = plt.colorbar(im2, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('deg', rotation=270, labelpad=12)

    fig.suptitle('Gripper-down reachable workspace (xArm 1S, ±π/2 driver clamp)')
    fig.tight_layout()
    out = os.path.join(OUT, 'workspace.png')
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


# ----------------------------------------------------------------------
#  Figure 3 — IK convergence trace on a sample target
# ----------------------------------------------------------------------

def fig_ik_convergence():
    """Replay a single seed of the DLS Newton loop and plot |p_err|/iter."""
    from xarm_pick.arm_ik import (
        SCREW_AXES, M_HOME, fk_tool0, space_jacobian, skew3, damped_pinv,
    )
    target = np.array([0.05, 0.18, 0.12])
    lower = np.array([JOINT_LIMITS[n][0] for n in JOINT_ORDER])
    upper = np.array([JOINT_LIMITS[n][1] for n in JOINT_ORDER])

    rng = np.random.default_rng(0)
    seeds = [rng.uniform(lower, upper) for _ in range(4)]

    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.5))
    for k, q0 in enumerate(seeds):
        q = np.clip(q0.copy(), lower, upper)
        eye_n = np.eye(len(q))
        errs, tilts = [], []
        polish = 30
        converged = False
        for _ in range(120):
            T = fk_tool0(q)
            p = T[:3, 3]
            z_axis = T[:3, 2]
            p_err = target - p
            err = float(np.linalg.norm(p_err))
            errs.append(err * 1000.0)  # mm
            cos_z = float(np.clip(-z_axis[2], -1.0, 1.0))
            tilts.append(float(np.degrees(np.arccos(cos_z))))
            if err < 0.001 * 0.2:
                converged = True
                polish -= 1
                if polish <= 0:
                    break
            Js = space_jacobian(q)
            omega_s = Js[:3, :]
            v_s = Js[3:, :]
            J_pos = -skew3(p) @ omega_s + v_s
            blend = min(1.0, err / 0.05)
            lam = 0.005 + blend * (0.10 - 0.005)
            Jp = damped_pinv(J_pos, lam)
            dq_pos = Jp @ p_err
            N = eye_n - Jp @ J_pos
            z_err = 1.0 + float(z_axis[2])
            dz_dq = -skew3(z_axis) @ omega_s
            grad_o = 2.0 * z_err * dz_dq[2, :]
            gain = 0.3 * (3.0 if converged else 1.0)
            dq_orient = N @ (-gain * grad_o)
            dq = dq_pos + dq_orient
            n_dq = float(np.linalg.norm(dq))
            if n_dq > 0.5:
                dq *= 0.5 / n_dq
            q = np.clip(q + dq, lower, upper)
        axes[0].plot(errs, alpha=0.85, label=f'seed {k+1}')
        axes[1].plot(tilts, alpha=0.85, label=f'seed {k+1}')

    axes[0].set_yscale('log')
    axes[0].set_xlabel('iteration')
    axes[0].set_ylabel('|p_target - p(q)| (mm, log)')
    axes[0].set_title('Position-error decay')
    axes[0].axhline(5.0, color='gray', linestyle='--', lw=0.6, alpha=0.7)
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8, loc='upper right')

    axes[1].set_xlabel('iteration')
    axes[1].set_ylabel('tilt off vertical (deg)')
    axes[1].set_title('Orientation refinement (null-space polish)')
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8, loc='upper right')

    fig.suptitle('DLS Newton IK — target (0.05, 0.18, 0.12) m, 4 seeds')
    fig.tight_layout()
    out = os.path.join(OUT, 'ik_convergence.png')
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


# ----------------------------------------------------------------------

if __name__ == '__main__':
    paths = []
    for name, fn in [('chain', fig_kinematic_chain),
                     ('workspace', fig_workspace),
                     ('convergence', fig_ik_convergence)]:
        try:
            p = fn()
            paths.append(p)
            print(f'wrote {p}')
        except Exception as e:
            print(f'!! {name}: {e}', file=sys.stderr)
    print(f'\n{len(paths)} figures generated.')
