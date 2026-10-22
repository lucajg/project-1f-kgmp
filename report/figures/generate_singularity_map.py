"""Render corresponding arm singularity curves in joint and wrist space.

Run from any directory. Requires NumPy and Matplotlib. The PDF is the vector
asset included by the report; the PNG is a preview. The formula and dimensions
match notes/jacobian_detailed.tex and Matlab/models/robot_dimensions.m.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np


def arm_factor(q2, q3):
    """D(q2,q3), where det(J) = sin(q5) D, for the supplied CuRo6R."""
    a1, a2, d2, d3, d4 = 4 / 5, 2, 5 / 4, 3 / 4, 5 / 2
    s2, c2, s3, c3 = np.sin(q2), np.cos(q2), np.sin(q3), np.cos(q3)
    u = a2 + d4 * c3
    x = u * c2 + d3 * s2
    z = -u * s2 + d3 * c2
    return d4 / 2 * (
        a1 * z * c3
        - np.sqrt(3) * a1 * u * s3
        - np.sqrt(3) * x * (a2 * s3 - d2 * c3)
    )


def wrist_position(q2, q3):
    """Wrist centre in base_link at q1=0 (metres).

    Other q1 values rotate this point about the base z-axis, preserving rho,z.
    """
    u = 2 + 2.5 * np.cos(q3)
    v = 1.25 + 2.5 * np.sin(q3)
    x = u * np.cos(q2) + 0.75 * np.sin(q2)
    z = -u * np.sin(q2) + 0.75 * np.cos(q2)
    return 0.8 + x, np.sqrt(3) / 2 * v - z / 2, 1 + v / 2 + np.sqrt(3) / 2 * z


def wrist_section(q2, q3):
    x, y, z = wrist_position(q2, q3)
    return np.hypot(x, y), z


def verified_cusps():
    """Four numerically located cusps, checked through the arm IK quartic.

    These were located by solving D=0 and grad(D).(z_q3,-z_q2)=0.
    A stationary z can also satisfy these equations without being a cusp.
    Checking P=P'=P''=0 and P''' != 0 rejects such false candidates.
    The polynomial coefficients below are in descending powers of tan(q1/2),
    exactly as in the analytical IK section, after rotating the target to y=0.
    """
    cusps = np.array([
        [2.9095335489797884, -2.7500126139918746],
        [2.5141213095727082, 1.1553908324257272],
        [-0.36868905732949386, -2.2904008172381474],
        [-1.8439112435799765, 0.87677557117750471],
    ])
    a1, a2, d2, d3, d4 = 0.8, 2, 1.25, 0.75, 2.5
    max_residual = 0.0
    for q2, q3 in cusps:
        assert abs(arm_factor(q2, q3)) < 1e-10
        x, y, z = wrist_position(q2, q3)
        rho, h = np.hypot(x, y), z - 1
        t = np.tan(-np.arctan2(y, x) / 2)
        denominator = np.array([1., 0., 1.])
        y_num = np.array([h / 2, -np.sqrt(3) * rho, h / 2])
        n0 = rho**2 + h**2 + a1**2
        n_num = np.array([n0 + 2 * a1 * rho, 0., n0 - 2 * a1 * rho])
        k = d2**2 - a2**2 - d3**2 - d4**2
        sine = y_num - d2 * denominator
        cosine = n_num + k * denominator - 2 * d2 * y_num
        polynomial = (4 * a2**2 * np.convolve(sine, sine)
                      + np.convolve(cosine, cosine)
                      - 4 * a2**2 * d4**2 * np.convolve(denominator, denominator))
        polynomial /= np.max(np.abs(polynomial))
        residual = max(abs(np.polyval(np.polyder(polynomial, order), t))
                       for order in range(3))
        assert residual < 1e-10, f"Cusp triple-root residual: {residual}"
        assert abs(np.polyval(np.polyder(polynomial, 3), t)) > 1e-2
        max_residual = max(max_residual, residual)
    return cusps, max_residual


def main():
    angles = np.linspace(-np.pi, np.pi, 1201)
    q2, q3 = np.meshgrid(angles, angles)
    determinant = arm_factor(q2, q3)
    assert np.allclose(determinant[:, 0], determinant[:, -1], atol=1e-12)
    assert np.allclose(determinant[0, :], determinant[-1, :], atol=1e-12)

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["STIXGeneral"],
        "mathtext.fontset": "cm",
        "font.size": 12,
        "axes.labelsize": 12,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
        "pdf.fonttype": 42,
    })
    negative, positive, ink, cusp_colour = "#d9d9ff", "#ffe6e6", "#202020", "#a32932"
    # Match the report's text width so the plot labels retain their 12 pt size.
    fig = plt.figure(figsize=(6.2677, 3.65))
    ax = fig.add_axes([0.085, 0.17, 0.40, 0.687])
    workspace = fig.add_axes([0.61, 0.14, 0.35, 0.717])
    extent = float(np.max(np.abs(determinant))) + 1
    ax.contourf(q2, q3, determinant, levels=[-extent, 0, extent],
                colors=[negative, positive])
    curves = ax.contour(q2, q3, determinant, levels=[0], colors=[ink],
                        linewidths=1.0)
    residual = max(
        float(np.max(np.abs(arm_factor(segment[:, 0], segment[:, 1]))))
        for segment in curves.allsegs[0] if len(segment)
    )
    assert residual < 3e-4, f"Contour interpolation residual: {residual}"
    for segment in curves.allsegs[0]:
        if len(segment):
            workspace.plot(*wrist_section(segment[:, 0], segment[:, 1]),
                           color=ink, linewidth=1.0)

    cusps, cusp_residual = verified_cusps()
    joint_offsets = [(-25, 9), (-24, 8), (5, 9), (7, 7)]
    workspace_offsets = [(8, -11), (-24, -14), (-16, -7), (22, -16)]
    for index, q in enumerate(cusps):
        for panel, position, offset in [
            (ax, q, joint_offsets[index]),
            (workspace, wrist_section(*q), workspace_offsets[index]),
        ]:
            panel.plot(*position, "o", color=cusp_colour, markersize=3.5, zorder=4)
            panel.annotate(rf"$C_{index + 1}$", position, xytext=offset,
                           textcoords="offset points", color=cusp_colour,
                           fontsize=12)

    ticks = np.linspace(-np.pi, np.pi, 5)
    labels = [r"$-\pi$", r"$-\pi/2$", "$0$", r"$\pi/2$", r"$\pi$"]
    ax.set_xticks(ticks, labels)
    ax.set_yticks(ticks, labels)
    ax.set(xlim=(-np.pi, np.pi), ylim=(-np.pi, np.pi),
           xlabel=r"$q_2$ (rad)", ylabel=r"$q_3$ (rad)")
    workspace.set(xlim=(-0.15, 5.8), ylim=(-3, 6.35),
                  xlabel=r"$\rho_w$ (m)", ylabel=r"$z_w$ (m)")
    workspace.set_xticks([0, 2, 4])
    workspace.set_yticks([-2, 0, 2, 4, 6])
    ax.set_title("(a) Joint space", fontsize=12, pad=9)
    workspace.set_title("(b) Wrist-centre workspace", fontsize=12, pad=9)
    for panel in (ax, workspace):
        panel.set_aspect("equal", adjustable="box")
        panel.tick_params(direction="out", length=3, width=0.7, pad=3)
        for spine in panel.spines.values():
            spine.set_linewidth(0.7)
    fig.legend(
        handles=[Patch(facecolor=negative, edgecolor="#999999", label="$D<0$"),
                 Patch(facecolor=positive, edgecolor="#999999", label="$D>0$"),
                 Line2D([], [], color=ink, linewidth=1, label="$D=0$"),
                 Line2D([], [], color=cusp_colour, marker="o", markersize=3.5,
                        linestyle="none", label="Cusps")],
        loc="upper center", bbox_to_anchor=(0.52, 1.015), ncol=4,
        frameon=False, handlelength=1.2, handletextpad=0.4, columnspacing=1.0,
    )
    out = Path(__file__).resolve().parent
    fig.savefig(out / "singularity_map.pdf", metadata={
        "Title": "CuRo6R arm singularities in joint space and wrist-centre workspace",
        "Subject": "D = 0 and its wrist-centre image; four verified cusp points",
    })
    fig.savefig(out / "singularity_map.png", dpi=240)
    plt.close(fig)
    print(f"Saved singularity_map.pdf and .png; maximum contour residual {residual:.3e}; "
          f"normalised cusp triple-root residual {cusp_residual:.3e}.")


if __name__ == "__main__":
    main()
