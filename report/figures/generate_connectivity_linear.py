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
    arm_determinant, create_singularity_map, wrist_position, wrist_section,
)


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
    offsets = [(7, -8), (-8, -4), (0, 8), (7, 4)]
    # Display labels in solver-group order: start t_1, goal t_3.
    display_numbers = (4, 3, 2, 1)
    group_number = 0
    group_numbers = {}
    while remaining:
        idx = remaining[0]
        group = [j for j in remaining
                 if np.linalg.norm(solutions[1:3, j]-solutions[1:3, idx]) < 1e-6]
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
        remaining = [j for j in remaining if j not in group]
        group_number += 1

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
    print(f'Saved connectivity_linear.pdf and .png; wrist projection error '
          f'{projection_error:.3e} m; contour residual {residual:.3e}; '
          f'cusp triple-root residual {cusp_residual:.3e}.')


if __name__ == '__main__':
    main()
