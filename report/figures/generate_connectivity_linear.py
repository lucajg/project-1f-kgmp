"""Overlay the fixed-orientation experiment on the two singularity maps.

Run from any directory after export_connectivity_linear.m. The workspace
panel uses the wrist centre, matching the map, rather than the tool endpoint.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from generate_singularity_map import (
    arm_determinant, create_singularity_map, verified_cusps,
    wrist_position, wrist_section,
)


def label_workspace_singularities(workspace):
    """Name the 4/2 folds using continuation of the displayed arm roots.

    Labels start in increasing t=tan(q3/2) order at the marked target and
    are continued within the four-root region, including through infinity.
    In verified_cusps() order, the coalescing roots in local angular order
    are (3,4,1), (2,3,4), (4,1,2), and (2,3,4). The middle root names each
    cusp. Both the upper and right cusp therefore have index 3.
    Pair names are not extended around the two-root region: continuation
    around a cusp can change the branch reached at the reference target.
    """
    ink = '#202020'
    box = {'facecolor': 'white', 'alpha': .92,
           'edgecolor': 'none', 'pad': .5}
    leader = {'arrowstyle': '-', 'color': ink, 'lw': .45,
              'shrinkA': 2, 'shrinkB': 2}
    cusps, _ = verified_cusps()
    cusp_labels = [
        (r'$C_4$', (0.85, 0.12)),
        (r'$C_3^{r}$', (5.05, 0.35)),
        (r'$C_1$', (1.00, 1.95)),
        (r'$C_3^{u}$', (0.65, 5.35)),
    ]
    for q, (label, text_position) in zip(cusps, cusp_labels):
        point = wrist_section(*q)
        workspace.plot(*point, 'o', color=ink, ms=2.7, zorder=10)
        workspace.annotate(label, point, xytext=text_position, fontsize=10,
                           ha='center', va='center', bbox=box,
                           arrowprops=leader, zorder=11)

    # Anchors on the five open 4/2 arcs, checked by root continuation.
    # F_34 occurs on two separate arcs. Snap to the plotted zero contour.
    curves = np.concatenate([
        np.column_stack((line.get_xdata(), line.get_ydata()))
        for line in workspace.lines if len(line.get_xdata()) > 1
    ])
    folds = [
        (r'$F_{34}$', (2.17853, 4.71636), (0.95, 4.65)),
        (r'$F_{23}$', (4.00991, 2.51006), (4.85, 2.45)),
        (r'$F_{34}$', (2.59419, 1.49531), (2.90, 0.85)),
        (r'$F_{14}$', (1.55892, 0.98410), (0.60, 0.85)),
        (r'$F_{12}$', (1.79537, 1.71598), (2.80, 2.60)),
    ]
    for label, anchor, text_position in folds:
        point = curves[np.argmin(np.linalg.norm(curves - anchor, axis=1))]
        workspace.annotate(label, point, xytext=text_position, fontsize=10,
                           ha='center', va='center', bbox=box,
                           arrowprops=leader, zorder=11)


def main():
    here = Path(__file__).resolve().parent
    data = json.loads((here / 'connectivity_linear_data.json').read_text())
    q, p, solutions = [np.asarray(data[key]) for key in ('q', 'p', 'Q_ik')]
    start = np.asarray(data['q_start']).ravel()
    goal_id = int(data['selected_id']) - 1
    start_id = np.argmin(np.linalg.norm(solutions - start[:, None], axis=0))
    rho, z = wrist_section(q[1], q[2])

    # Independently compare the arm formula with the exported tool positions.
    # Fixed orientation gives p_w = p_6 - d_6 R_d e_x, with d_6 = 1.5 m.
    x0, y0, z0 = wrist_position(q[1], q[2])
    c1, s1 = np.cos(q[0]), np.sin(q[0])
    wrist = np.array([c1*x0 - s1*y0, s1*x0 + c1*y0, z0])
    exported_wrist = p - 1.5 * np.asarray(data['target'])[:3, 0, None]
    projection_error = float(np.max(np.linalg.norm(wrist - exported_wrist, axis=0)))
    assert projection_error < 1e-10, f'Wrist projection error: {projection_error}'
    assert np.linalg.norm(wrist[:, -1] - wrist[:, 0]) < 1e-10
    assert np.allclose(wrist_section(solutions[1], solutions[2]),
                       np.array([[rho[0]], [z[0]]]), atol=1e-10, rtol=0)
    assert np.allclose(arm_determinant(q[1], q[2]), data['arm_det'],
                       atol=1e-10, rtol=0)

    fig, ax, workspace, residual, cusp_residual = create_singularity_map()
    label_workspace_singularities(workspace)
    blue, path_colour = '#2164A1', '#D18A00'
    # Retain the full maps and equal axis scaling from the original figure.
    for panel, x, y in ((ax, q[1], q[2]), (workspace, rho, z)):
        panel.plot(x, y, color='white', lw=1.7, zorder=5)
        panel.plot(x, y, color=path_colour, lw=.9, zorder=6)
        k = len(x) // 2
        panel.annotate('', xy=(x[k+40], y[k+40]), xytext=(x[k-40], y[k-40]),
                       arrowprops={'arrowstyle': '->', 'color': path_colour,
                                   'lw': .9, 'mutation_scale': 9}, zorder=7)

    remaining = list(range(solutions.shape[1]))
    groups = []
    while remaining:
        idx = remaining[0]
        group = [j for j in remaining
                 if np.linalg.norm(solutions[1:3, j]-solutions[1:3, idx]) < 1e-6]
        groups.append(group)
        remaining = [j for j in remaining if j not in group]

    # Number the four arm branches by increasing finite half-angle root at
    # the common target. Keep these identities when continuing elsewhere.
    root_values = np.tan(solutions[2, [group[0] for group in groups]] / 2)
    assert len(groups) == 4 and np.all(np.isfinite(root_values))
    assert np.all(np.diff(np.sort(root_values)) > 1e-6)
    display_numbers = np.argsort(np.argsort(root_values)) + 1
    offsets = [(7, -8), (-8, -4), (0, 8), (7, 4)]
    group_numbers = {}
    for group_number, group in enumerate(groups):
        idx = group[0]
        # Both wrist branches share one arm point and one label.
        display_number = display_numbers[group_number]
        group_numbers.update({j: display_number for j in group})
        ax.plot(*solutions[1:3, idx], 'o', color=blue, ms=5,
                mfc='white', mew=1, zorder=8)
        offset = offsets[group_number]
        ax.annotate(rf'$t_{{{display_number}}}$', solutions[1:3, idx],
                    xytext=offset, textcoords='offset points', fontsize=10,
                    ha='center' if offset[0] == 0 else ('left' if offset[0] > 0 else 'right'),
                    bbox={'facecolor': 'white', 'alpha': .9,
                          'edgecolor': 'none', 'pad': .4}, zorder=9)

    assert (group_numbers[start_id], group_numbers[goal_id]) == (4, 2), \
        'The endpoint labels must match the connectivity section.'

    workspace.plot(rho[0], z[0], 'o', ms=5, mfc='white', mec=path_colour,
                   mew=.9, zorder=8)
    workspace.annotate(rf'$t_{{{group_numbers[start_id]}}}/t_{{{group_numbers[goal_id]}}}$',
                       (rho[0], z[0]), xytext=(-8, -12), ha='right',
                       textcoords='offset points', color=path_colour, fontsize=11,
                       zorder=9)

    fig.savefig(here / 'connectivity_linear.pdf', dpi=600, metadata={
        'Title': 'CuRo6R singularity maps and a nonsingular solution change',
        'Subject': 'Joint-space arm segment and wrist-centre projection at fixed orientation',
    })
    fig.savefig(here / 'connectivity_linear.png', dpi=240)
    plt.close(fig)
    print(f'Ordered roots t_1,...,t_4: {np.sort(root_values)}.')
    print(f'Saved connectivity_linear.pdf and .png; wrist projection error '
          f'{projection_error:.3e} m; contour residual {residual:.3e}; '
          f'cusp triple-root residual {cusp_residual:.3e}.')


if __name__ == '__main__':
    main()
