"""Render the arm singularity map from the verified symbolic determinant.

Run from any directory. Requires NumPy and Matplotlib. The PDF is the vector
asset included by the report; the PNG is a preview. The formula and dimensions
match notes/jacobian_detailed.tex and Matlab/robot_dimensions.m.
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
    negative, positive, ink = "#d9d9ff", "#ffe6e6", "#202020"
    fig, ax = plt.subplots(figsize=(4.15, 3.75))
    fig.subplots_adjust(left=0.19, right=0.95, bottom=0.16, top=0.89)
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

    ticks = np.linspace(-np.pi, np.pi, 5)
    labels = [r"$-\pi$", r"$-\pi/2$", "$0$", r"$\pi/2$", r"$\pi$"]
    ax.set_xticks(ticks, labels)
    ax.set_yticks(ticks, labels)
    ax.set(xlim=(-np.pi, np.pi), ylim=(-np.pi, np.pi),
           xlabel=r"$q_2$ (rad)", ylabel=r"$q_3$ (rad)")
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(direction="out", length=3, width=0.7, pad=3)
    for spine in ax.spines.values():
        spine.set_linewidth(0.7)
    fig.legend(
        handles=[Patch(facecolor=negative, edgecolor="#999999", label="$D<0$"),
                 Patch(facecolor=positive, edgecolor="#999999", label="$D>0$"),
                 Line2D([], [], color=ink, linewidth=1, label="$D=0$")],
        loc="upper center", bbox_to_anchor=(0.54, 1), ncol=3,
        frameon=False, handlelength=1.2, handletextpad=0.4, columnspacing=1.0,
    )
    out = Path(__file__).resolve().parent
    fig.savefig(out / "singularity_map.pdf", metadata={
        "Title": "CuRo6R arm singularity map",
        "Subject": "D(q2,q3) = det(J) at q5 = pi/2; black curves are D = 0",
    })
    fig.savefig(out / "singularity_map.png", dpi=240)
    plt.close(fig)
    print(f"Saved singularity_map.pdf and .png; maximum contour residual {residual:.3e}.")


if __name__ == "__main__":
    main()
