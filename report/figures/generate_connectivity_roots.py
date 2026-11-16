"""Plot the quartic and circle/conic for the connectivity reference target.

Run from any directory with Python 3, NumPy and Matplotlib. The exact target
and existing root identities come from connectivity_linear_data.json; no
rounded joint coordinates are used. Outputs are vector PDFs, PNG previews,
and numerical data beside this script. The main report is not rebuilt.
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from generate_singularity_map import quartic_coefficients, wrist_position


HERE = Path(__file__).resolve().parent
INK = "#202020"
BLUE = "#2164A1"
GRAY = "#78838C"
WIDTH = 6.2677  # The report's text width, in inches.


def conic_coefficients(rho, z):
    """Coefficients [c^2, c*s, s^2, c, s, 1] of C(c,s) = cos(gamma)^2 G.

    F3 = L + 2*a2*d4*c + 2*d2*d4*s uses c^2+s^2=1. This defines the
    quadratic extension of the report's closure away from the unit circle.
    Its restriction to the circle is exactly the closure in the report.
    """
    a1, a2, d1, d2, d3, d4 = 0.8, 2.0, 1.0, 1.25, 0.75, 2.5
    cg2, sg = 0.75, 0.5
    h = z - d1
    length_sum = a1**2 + a2**2 + d2**2 + d3**2 + d4**2
    k = rho**2 + h**2 - length_sum
    b, e = 2 * a2 * d4, 2 * d2 * d4
    v = h - sg * d2
    return np.array([
        cg2 * b**2 - 4 * a1**2 * cg2 * d4**2,
        2 * cg2 * b * e,
        cg2 * e**2 + 4 * a1**2 * sg**2 * d4**2,
        -2 * cg2 * k * b - 8 * a1**2 * cg2 * a2 * d4,
        -2 * cg2 * k * e - 8 * a1**2 * v * sg * d4,
        cg2 * k**2 + 4 * a1**2 * v**2
        - 4 * a1**2 * cg2 * (a2**2 + d3**2),
    ])


def evaluate_conic(coef, c, s):
    a, b, d, e, f, g = coef
    return a * c**2 + b * c * s + d * s**2 + e * c + f * s + g


def reference_case():
    """Recover the exact target; check algebra, branch labels and arm FK."""
    data = json.loads((HERE / "connectivity_linear_data.json").read_text())
    target = np.asarray(data["target"])
    wrist = target[:3, 3] - 1.5 * target[:3, 0]
    rho, z = np.hypot(*wrist[:2]), wrist[2]
    polynomial = quartic_coefficients(rho, z)
    raw_roots = np.roots(polynomial)
    assert np.max(np.abs(raw_roots.imag)) < 1e-12
    roots = np.sort(raw_roots.real)
    assert np.min(np.diff(roots)) > 1e-4
    q3 = 2 * np.arctan(roots)
    points = np.column_stack((np.cos(q3), np.sin(q3)))
    conic = conic_coefficients(rho, z)

    # Check the entire polynomial identity, not only its zeros:
    # P(t) = (1+t^2)^2 C((1-t^2)/(1+t^2), 2t/(1+t^2)).
    t = np.linspace(-20, 20, 2001)
    delta = 1 + t**2
    conic_on_circle = evaluate_conic(conic, (1 - t**2) / delta, 2 * t / delta)
    identity_error = np.max(np.abs(np.polyval(polynomial, t) / delta**2
                                   - conic_on_circle))
    assert identity_error < 1e-10
    root_error = np.max(np.abs(np.polyval(polynomial, roots)))
    conic_error = np.max(np.abs(evaluate_conic(conic, *points.T)))
    assert root_error < 1e-10 and conic_error < 1e-10
    assert np.max(np.abs(np.sum(points**2, axis=1) - 1)) < 1e-14
    assert np.min(np.abs(np.polyval(np.polyder(polynomial), roots))) > 1

    # Each arm root occurs twice in the full IK list, once per wrist branch.
    solutions = np.asarray(data["Q_ik"])
    exported_roots = np.sort(np.tan(solutions[2] / 2))
    assert np.allclose(exported_roots, np.repeat(roots, 2), atol=1e-12, rtol=0)
    assert abs(np.tan(data["q_start"][2] / 2) - roots[3]) < 1e-12
    assert abs(np.tan(data["q_goal"][2] / 2) - roots[1]) < 1e-12

    # Independently recover q1, q2, then evaluate the arm position.
    f1 = 2 + 2.5 * np.cos(q3)
    b = 1.25 + 2.5 * np.sin(q3)
    h = z - 1
    f3 = 0.8**2 + f1**2 + b**2 + 0.75**2
    x = (rho**2 + h**2 - f3) / 1.6
    d = (h - 0.5 * b) / (np.sqrt(3) / 2)
    q2 = np.arctan2(0.75 * x - f1 * d, f1 * x + 0.75 * d)
    a = 0.8 + x
    y = np.sqrt(3) / 2 * b - d / 2
    q1 = np.arctan2(a * wrist[1] - y * wrist[0],
                    a * wrist[0] + y * wrist[1])
    wx, wy, wz = wrist_position(q2, q3)
    recovered_wrist = np.array([
        np.cos(q1) * wx - np.sin(q1) * wy,
        np.sin(q1) * wx + np.cos(q1) * wy,
        wz,
    ])
    fk_error = np.max(np.linalg.norm(recovered_wrist - wrist[:, None], axis=0))
    assert fk_error < 1e-10
    return {
        "wrist_target_m": wrist.tolist(),
        "rho_w_m": float(rho), "z_w_m": float(z),
        "h_m": float(h), "R_m2": float(rho**2 + h**2),
        "polynomial_descending": polynomial.tolist(),
        "conic_c2_cs_s2_c_s_constant": conic.tolist(),
        "roots_t1_to_t4": roots.tolist(),
        "q3_rad_t1_to_t4": q3.tolist(),
        "circle_points_t1_to_t4": points.tolist(),
        "checks": {
            "polynomial_circle_identity_max_abs": float(identity_error),
            "polynomial_root_max_abs": float(root_error),
            "conic_root_max_abs": float(conic_error),
            "recovered_arm_fk_max_error_m": float(fk_error),
        },
    }


def style_axes(ax):
    ax.tick_params(direction="out", length=3, width=0.7, pad=4)
    for spine in ax.spines.values():
        spine.set_linewidth(0.7)
    ax.grid(color="#e8e8e8", linewidth=0.45, zorder=0)


def target_subtitle(fig, data, y):
    fig.text(0.5, y,
             rf"$\rho_{{w,d}}={data['rho_w_m']:.6f}\,\mathrm{{m}},"
             rf"\quad z_{{w,d}}={data['z_w_m']:.6f}\,\mathrm{{m}}$",
             ha="center", va="center", fontsize=11)


def save(fig, stem, title):
    fig.savefig(HERE / f"{stem}.pdf", metadata={
        "Title": title,
        "Subject": "Exact reference target and root labels of connectivity_linear.pdf",
    })
    fig.savefig(HERE / f"{stem}.png", dpi=240)
    plt.close(fig)


def plot_quartic(data):
    p = np.asarray(data["polynomial_descending"])
    roots = np.asarray(data["roots_t1_to_t4"])
    fig = plt.figure(figsize=(WIDTH, 4.4))
    fig.text(0.5, 0.955, "Quartic at the reference target", ha="center", fontsize=12)
    target_subtitle(fig, data, 0.901)
    ax = fig.add_axes([0.12, 0.34, 0.845, 0.48])
    t = np.linspace(-0.68, 1.57, 1800)
    ax.plot(t, np.polyval(p, t), color=BLUE, lw=1.5, zorder=3)
    ax.axhline(0, color=INK, lw=0.8, zorder=2)
    ax.plot(roots, np.zeros(4), "o", ms=5.3, mfc="white", mec=BLUE,
            mew=1.15, zorder=5)
    offsets = [(-15, -10), (5, 14), (-8, 14), (16, -10)]
    for i, (root, offset) in enumerate(zip(roots, offsets), 1):
        ax.annotate(rf"$t_{i}$", (root, 0), xytext=offset,
                    textcoords="offset points", ha="center",
                    va="bottom" if offset[1] > 0 else "top",
                    fontsize=12, color=INK,
                    arrowprops={"arrowstyle": "-", "lw": 0.55, "color": GRAY},
                    bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.5},
                    zorder=6)
    ax.set(xlim=(-0.68, 1.57), ylim=(-12, 64), xlabel=r"$t=\tan(q_3/2)$",
           ylabel=r"$P(t)$ ($\mathrm{m}^4$)")
    ax.set_xticks([-0.5, 0, 0.5, 1, 1.5])
    ax.set_yticks([0, 20, 40, 60])
    style_axes(ax)
    fig.text(0.5, 0.167,
             rf"$P(t)\simeq {p[0]:.6f}t^4 {p[1]:+.6f}t^3 {p[2]:+.6f}t^2$",
             ha="center", fontsize=11)
    fig.text(0.5, 0.115, rf"${p[3]:+.6f}t {p[4]:+.6f}$",
             ha="center", fontsize=11)
    for i, (x, root) in enumerate(zip([0.14, 0.38, 0.62, 0.86], roots), 1):
        fig.text(x, 0.040, rf"$t_{i}={root:.6f}$", ha="center", fontsize=10.5)
    save(fig, "connectivity_quartic", "CuRo6R: quartic and four labelled arm roots")


def plot_conic(data):
    coef = np.asarray(data["conic_c2_cs_s2_c_s_constant"])
    points = np.asarray(data["circle_points_t1_to_t4"])
    matrix = np.array([[coef[0], coef[1] / 2], [coef[1] / 2, coef[2]]])
    eigenvalues, basis = np.linalg.eigh(matrix)
    centre = -np.linalg.solve(matrix, coef[3:5]) / 2
    level = centre @ matrix @ centre - coef[5]
    assert eigenvalues[0] < 0 < eigenvalues[1] and level > 0

    fig = plt.figure(figsize=(WIDTH, 5.5))
    fig.text(0.5, 0.965, "Circle and closure conic at the reference target",
             ha="center", fontsize=12)
    target_subtitle(fig, data, 0.922)
    ax = fig.add_axes([0.14, 0.29, 0.74, 0.57])
    angle = np.linspace(0, 2 * np.pi, 1001)
    ax.plot(np.cos(angle), np.sin(angle), color=INK, lw=1.25, zorder=3)
    # Exact parametrisation of both hyperbola branches; avoids contour-grid
    # artefacts near the two closely spaced upper intersections.
    u = np.linspace(-4.5, 4.5, 2201)
    for sign in [-1, 1]:
        principal = np.array([
            np.sqrt(level / -eigenvalues[0]) * np.sinh(u),
            sign * np.sqrt(level / eigenvalues[1]) * np.cosh(u),
        ])
        curve = centre[:, None] + basis @ principal
        assert np.max(np.abs(evaluate_conic(coef, *curve))) < 1e-9
        ax.plot(*curve, color=BLUE, lw=1.35, zorder=4)

    ax.axhline(0, color="#b7b7b7", lw=0.65, zorder=1)
    ax.axvline(0, color="#b7b7b7", lw=0.65, zorder=1)
    ax.plot(*points.T, "o", ms=5.0, mfc="white", mec=BLUE,
            mew=1.15, zorder=6)
    offsets = [(-20, 5), (15, 5), (19, 17), (-23, 12)]
    for i, (point, offset) in enumerate(zip(points, offsets), 1):
        ax.annotate(rf"$t_{i}$", point, xytext=offset,
                    textcoords="offset points", ha="center", va="center",
                    fontsize=12, zorder=7,
                    arrowprops={"arrowstyle": "-", "color": GRAY, "lw": 0.6},
                    bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.6})

    # The omitted half-angle point is finite in the circle representation.
    # Here C(-1,0)=P's leading coefficient != 0, so it is not an IK root.
    ax.plot(-1, 0, "x", ms=4.5, mew=0.8, color=GRAY, zorder=5)
    ax.annotate(r"$q_3=\pi$" + "\n" + r"$t=\infty$", (-1, 0),
                xytext=(7, 9), textcoords="offset points", ha="left",
                va="bottom", color=GRAY, fontsize=10)
    ax.set(xlim=(-1.28, 1.28), ylim=(-1.18, 1.35),
           xlabel=r"$c=\cos q_3$", ylabel=r"$s=\sin q_3$")
    ax.set_xticks([-1, -0.5, 0, 0.5, 1])
    ax.set_yticks([-1, -0.5, 0, 0.5, 1])
    ax.set_aspect("equal", adjustable="box")
    style_axes(ax)
    fig.legend(handles=[
        Line2D([], [], color=INK, lw=1.25, label=r"$c^2+s^2=1$"),
        Line2D([], [], color=BLUE, lw=1.35, label=r"$C(c,s)=0$ (hyperbola)"),
    ], loc="center", bbox_to_anchor=(0.5, 0.179), ncol=2,
        frameon=False, fontsize=11, handlelength=1.8, columnspacing=2.0)
    fig.text(0.5, 0.122,
             rf"$C(c,s)\simeq {coef[0]:g}c^2+{coef[1]:g}cs+{coef[2]:.6f}s^2$",
             ha="center", fontsize=11)
    fig.text(0.5, 0.080,
             rf"${coef[3]:+.6f}c{coef[4]:+.6f}s{coef[5]:+.6f}$",
             ha="center", fontsize=11)
    fig.text(0.5, 0.025,
             r"$P(t)=(1+t^2)^2\,C\!\left(\frac{1-t^2}{1+t^2},"
             r"\frac{2t}{1+t^2}\right)$",
             ha="center", fontsize=11)
    save(fig, "connectivity_circle_conic",
         "CuRo6R: unit circle, closure hyperbola and four labelled intersections")


def main():
    plt.rcParams.update({
        "font.family": "serif", "font.serif": ["STIXGeneral"],
        "mathtext.fontset": "cm", "font.size": 12,
        "axes.labelsize": 12, "xtick.labelsize": 11, "ytick.labelsize": 11,
        "pdf.fonttype": 42,
    })
    data = reference_case()
    plot_quartic(data)
    plot_conic(data)
    (HERE / "connectivity_roots_data.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data, indent=2))
    print("Saved connectivity_quartic.pdf/.png and connectivity_circle_conic.pdf/.png")


if __name__ == "__main__":
    main()
