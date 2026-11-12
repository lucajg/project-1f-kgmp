"""Render corresponding arm singularity curves in joint and wrist space.

Run from any directory. Requires NumPy and Matplotlib. The PDF keeps curves
and text as vectors, with raster root-count shading; the PNG is a preview.
The formula and dimensions
match sections/jacobian.tex and Matlab/models/robot_dimensions.m.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np


def arm_determinant(q2, q3):
    """det(J_v_arm), where det(J) = -sin(q5) det(J_v_arm), for CuRo6R."""
    a1, a2, d2, d3, d4 = 4 / 5, 2, 5 / 4, 3 / 4, 5 / 2
    s2, c2, s3, c3 = np.sin(q2), np.cos(q2), np.sin(q3), np.cos(q3)
    u = a2 + d4 * c3
    x = u * c2 + d3 * s2
    z = -u * s2 + d3 * c2
    return -d4 / 2 * (
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


def quartic_coefficients(rho, z):
    """Descending coefficients of P(t), t=tan(q3/2), for wrist targets."""
    rho, z = np.broadcast_arrays(rho, z)
    a1, a2, d2, d3, d4 = 0.8, 2, 1.25, 0.75, 2.5
    h = z - 1
    denominator = np.array([1., 0., 1.])
    length_sum = a1**2 + a2**2 + d2**2 + d3**2 + d4**2
    f1_num = np.array([a2 - d4, 0., a2 + d4])
    b_num = np.array([d2, 2 * d4, d2])
    f3_num = np.array([length_sum - 2 * a2 * d4, 4 * d2 * d4,
                      length_sum + 2 * a2 * d4])
    u = (rho**2 + h**2)[..., None] * denominator - f3_num
    v = h[..., None] * denominator - 0.5 * b_num
    polynomial = np.zeros(rho.shape + (5,))
    for i in range(3):
        for j in range(3):
            polynomial[..., i + j] += 0.75 * u[..., i] * u[..., j]
            polynomial[..., i + j] += 4 * a1**2 * v[..., i] * v[..., j]
    polynomial -= 3 * a1**2 * (np.convolve(f1_num, f1_num)
                              + d3**2 * np.convolve(denominator, denominator))
    return polynomial


def real_root_counts(rho, z):
    """Count real projective roots at regular targets, including q3=pi.

    Reverse coefficients where helpful to avoid a poorly conditioned leading
    coefficient near t=infinity. Reversal preserves the projective root count.
    The grid samples open regions; counts on multiple-root curves are not used.
    """
    polynomial = quartic_coefficients(rho, z)
    reverse = np.abs(polynomial[..., 0]) < np.abs(polynomial[..., -1])
    polynomial = np.where(reverse[..., None], polynomial[..., ::-1], polynomial)
    assert np.all(np.abs(polynomial[..., 0]) > 1e-10)
    companion = np.zeros(polynomial.shape[:-1] + (4, 4))
    companion[..., 0, :] = -polynomial[..., 1:] / polynomial[..., :1]
    companion[..., 1:, :-1] = np.eye(3)
    roots = np.linalg.eigvals(companion)
    counts = np.sum(np.abs(roots.imag) < 1e-8 * (1 + np.abs(roots.real)), axis=-1)
    assert np.all(np.isin(counts, [0, 2, 4]))
    return counts


def verified_cusps():
    """Four numerically located cusps, checked through the arm IK quartic.

    These solve det(J_v_arm)=0 and grad(det(J_v_arm)).(z_q3,-z_q2)=0.
    A stationary z can also satisfy these equations without being a cusp.
    Checking P=P'=P''=0 and P''' != 0 rejects such false candidates.
    The polynomial coefficients below are in descending powers of tan(q3/2),
    exactly as in the current analytical IK section. Its target invariants
    rho^2+h^2 and h do not depend on q1.
    """
    cusps = np.array([
        [2.9095335489797884, -2.7500126139918746],
        [2.5141213095727082, 1.1553908324257272],
        [-0.36868905732949386, -2.2904008172381474],
        [-1.8439112435799765, 0.87677557117750471],
    ])
    max_residual = 0.0
    for q2, q3 in cusps:
        assert abs(arm_determinant(q2, q3)) < 1e-10
        x, y, z = wrist_position(q2, q3)
        t = np.tan(q3 / 2)
        polynomial = quartic_coefficients(np.hypot(x, y), z)
        polynomial /= np.max(np.abs(polynomial))
        residual = max(abs(np.polyval(np.polyder(polynomial, order), t))
                       for order in range(3))
        assert residual < 1e-10, f"Cusp triple-root residual: {residual}"
        assert abs(np.polyval(np.polyder(polynomial, 3), t)) > 1e-2
        max_residual = max(max_residual, residual)
    return cusps, max_residual


def create_singularity_map():
    """Build the two map panels, also used by the connectivity overlay."""
    angles = np.linspace(-np.pi, np.pi, 1201)
    q2, q3 = np.meshgrid(angles, angles)
    determinant = arm_determinant(q2, q3)
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
    negative, positive, ink = "#d9d9ff", "#ffe6e6", "#202020"
    # Match the report's text width so the plot labels retain their 12 pt size.
    fig = plt.figure(figsize=(6.2677, 4.85))
    ax = fig.add_axes([0.085, 0.35, 0.40, 0.517])
    workspace = fig.add_axes([0.61, 0.327, 0.35, 0.54])
    root_colours = ["#ffffff", "#dcebf2", "#a3cdb9"]
    rho_edges = np.linspace(0, 5.8, 581)
    z_edges = np.linspace(-3, 6.35, 936)
    rho_grid, z_grid = np.meshgrid((rho_edges[:-1] + rho_edges[1:]) / 2,
                                  (z_edges[:-1] + z_edges[1:]) / 2)
    counts = real_root_counts(rho_grid, z_grid)
    workspace.pcolormesh(rho_edges, z_edges, counts,
                         cmap=ListedColormap(root_colours),
                         norm=BoundaryNorm([-1, 1, 3, 5], 3),
                         rasterized=True, shading="flat")
    extent = float(np.max(np.abs(determinant))) + 1
    ax.contourf(q2, q3, determinant, levels=[-extent, 0, extent],
                colors=[negative, positive])
    curves = ax.contour(q2, q3, determinant, levels=[0], colors=[ink],
                        linewidths=1.0)
    residual = max(
        float(np.max(np.abs(arm_determinant(segment[:, 0], segment[:, 1]))))
        for segment in curves.allsegs[0] if len(segment)
    )
    assert residual < 3e-4, f"Contour interpolation residual: {residual}"
    for segment in curves.allsegs[0]:
        if len(segment):
            workspace.plot(*wrist_section(segment[:, 0], segment[:, 1]),
                           color=ink, linewidth=1.0)

    _, cusp_residual = verified_cusps()

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
    legend_options = dict(frameon=False, handlelength=1.2, handletextpad=0.4,
                          columnspacing=0.9, fontsize=11, borderaxespad=0)
    ax.legend(
        handles=[Patch(facecolor=negative, edgecolor="#999999",
                       label=r"$\det(J_{v,\mathrm{arm}})<0$"),
                 Line2D([], [], color=ink, linewidth=1,
                        label=r"$\det(J_{v,\mathrm{arm}})=0$ (arm singularities)"),
                 Patch(facecolor=positive, edgecolor="#999999",
                       label=r"$\det(J_{v,\mathrm{arm}})>0$")],
        loc="upper center", bbox_to_anchor=(0.285, 0.22),
        bbox_transform=fig.transFigure, ncol=1, **legend_options,
    )
    root_legend = workspace.legend(
        handles=[Patch(facecolor=colour, edgecolor="#999999", label=f"${count}$")
                 for colour, count in zip(root_colours, [0, 2, 4])],
        title=r"Number of real roots of $P(t)$", title_fontsize=11,
        loc="upper center", bbox_to_anchor=(0.77, 0.20),
        bbox_transform=fig.transFigure, ncol=3, **legend_options,
    )
    workspace.add_artist(root_legend)
    workspace.legend(
        handles=[Line2D([], [], color=ink, linewidth=1,
                        label=r"$\operatorname{disc}_t P=0$")],
        loc="upper center", bbox_to_anchor=(0.77, 0.105),
        bbox_transform=fig.transFigure, **legend_options,
    )
    return fig, ax, workspace, residual, cusp_residual


def main():
    fig, _, _, residual, cusp_residual = create_singularity_map()
    out = Path(__file__).resolve().parent
    fig.savefig(out / "singularity_map.pdf", dpi=600, metadata={
        "Title": "CuRo6R arm singularities in joint space and wrist-centre workspace",
        "Subject": "Arm Jacobian determinant; IK quartic discriminant and real-root counts",
    })
    fig.savefig(out / "singularity_map.png", dpi=240)
    plt.close(fig)
    print(f"Saved singularity_map.pdf and .png; maximum contour residual {residual:.3e}; "
          f"normalised cusp triple-root residual {cusp_residual:.3e}.")


if __name__ == "__main__":
    main()
