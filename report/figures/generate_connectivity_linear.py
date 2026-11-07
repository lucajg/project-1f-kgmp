"""Plot the MATLAB experiment exported by export_connectivity_linear.m."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

from generate_singularity_map import arm_factor


def main():
    here = Path(__file__).resolve().parent
    data = json.loads((here / 'connectivity_linear_data.json').read_text())
    s, q, p, solutions = [np.asarray(data[key]) for key in ('s', 'q', 'p', 'Q_ik')]
    sigma, margin = [np.asarray(data[key]) for key in ('sigma_min', 'joint_margin')]
    start = np.asarray(data['q_start']).ravel()
    goal_id = int(data['selected_id']) - 1
    start_id = np.argmin(np.linalg.norm(solutions - start[:, None], axis=0))
    blue, green, orange = '#2164A1', '#337A55', '#B84338'
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8,
                         'axes.titlesize': 9, 'axes.labelsize': 8,
                         'xtick.labelsize': 7, 'ytick.labelsize': 7,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'pdf.fonttype': 42})
    fig = plt.figure(figsize=(6.25, 3.55), layout='constrained')
    grid = fig.add_gridspec(2, 2, height_ratios=[1.08, 1], wspace=.10, hspace=.10)
    ax = fig.add_subplot(grid[0, 0])
    all_points = np.column_stack((solutions[1:3], q[1:3]))
    low = np.maximum(-np.pi, all_points.min(axis=1) - .35)
    high = np.minimum(np.pi, all_points.max(axis=1) + .35)
    q2, q3 = np.meshgrid(np.linspace(low[0], high[0], 601),
                         np.linspace(low[1], high[1], 601))
    factor = arm_factor(q2, q3)
    ax.contourf(q2, q3, factor, [-100, 0, 100],
                cmap=ListedColormap(['#E2EFF8', '#F8E3E9']))
    ax.contour(q2, q3, factor, [0], colors='#243747', linewidths=.8)
    ax.plot(q[1], q[2], color=green, lw=1.6, zorder=4)
    k = len(s)//2
    ax.annotate('', xy=q[1:3, k+40], xytext=q[1:3, k-40],
                arrowprops={'arrowstyle': '->', 'color': green, 'lw': 1.5})
    remaining = list(range(solutions.shape[1]))
    group_number = 0
    while remaining:
        idx = remaining[0]
        group = [j for j in remaining
                 if np.linalg.norm(solutions[1:3, j]-solutions[1:3, idx]) < 1e-6]
        labels = []
        for j in group:
            positive = np.sin(solutions[4, j]) > 0
            ax.plot(*solutions[1:3, j], 'o' if positive else 's',
                    color=blue if positive else orange, ms=6 if positive else 2.8,
                    mfc='none' if positive else orange, mew=1, zorder=5)
            labels.append(f'{j+1}{"+" if positive else "−"}')
        prefix = 'A: ' if start_id in group else ('B: ' if goal_id in group else '')
        offset = [(5, 9), (5, -15), (-5, 9), (-5, -15)][group_number]
        ax.annotate(prefix + ', '.join(labels), solutions[1:3, idx],
                    xytext=offset, textcoords='offset points', fontsize=7,
                    ha='left' if offset[0] > 0 else 'right',
                    bbox={'facecolor': 'white', 'alpha': .85, 'edgecolor': 'none', 'pad': .6})
        remaining = [j for j in remaining if j not in group]
        group_number += 1
    ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]),
           xlabel=r'$q_2$ (rad)', ylabel=r'$q_3$ (rad)',
           title='(a) Arm segment and IK wrist signs')

    ax = fig.add_subplot(grid[0, 1], projection='3d')
    ax.plot(*p, color=blue, lw=1.5)
    ax.scatter(*p[:, 0], s=22, facecolors='white', edgecolors=orange,
               depthshade=False, zorder=4)
    k = len(s)//2
    direction = (p[:, k+3] - p[:, k-3])
    direction *= .12 * np.linalg.norm(np.ptp(p, axis=1)) / np.linalg.norm(direction)
    ax.quiver(*p[:, k], *direction, color=blue, arrow_length_ratio=.4, linewidth=1.2)
    ax.text(*p[:, 0], '  A / B', fontsize=7, color=orange)
    ax.set(xlabel='$x$ (m)', ylabel='$y$ (m)', zlabel='$z$ (m)',
           title='(b) Closed tool path, fixed orientation')
    # Look approximately along the loop's least-varying direction so its
    # enclosed area remains visible instead of collapsing in projection.
    ax.view_init(elev=45, azim=82)
    ax.set_box_aspect(np.maximum(np.ptp(p, axis=1), .1))
    ax.tick_params(pad=0, labelsize=6)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.labelpad = 0
        axis.set_major_locator(plt.MaxNLocator(3))
    ax.set_proj_type('ortho')

    ax = fig.add_subplot(grid[1, 0])
    colors = [blue, green, '#A66816', orange, '#79509A', '#243747']
    for j in range(6):
        ax.plot(s, q[j], color=colors[j], lw=1.1, ls='-' if j < 3 else '--',
                label=f'$q_{j+1}$')
    ax.legend(loc='upper center', ncol=6, fontsize=7, frameon=False,
              handlelength=1.1, columnspacing=.65, handletextpad=.25)
    ax.set(xlim=(0, 1), ylim=(-3.1, 2.7), xlabel='Path parameter $s$',
           ylabel='Angle (rad)', title='(c) Arm lines and continued wrist angles')
    ax.grid(alpha=.15)

    ax = fig.add_subplot(grid[1, 1])
    ax.plot(s, sigma, color=blue, lw=1.5)
    ax.axhline(data['summary']['sigma_tol'], color=blue, lw=.8, ls=':')
    ax.text(.98, .007, r'Threshold $10^{-3}$', color=blue, fontsize=6.5,
            ha='right')
    k = np.argmin(sigma)
    ax.plot(s[k], sigma[k], 'o', color=blue, ms=3)
    ax.set(xlim=(0, 1), ylim=(0, max(sigma)*1.22), xlabel='Path parameter $s$',
           ylabel=r'$\sigma_{\min}(\widetilde J)$', title='(d) Full-Jacobian and joint-limit margins')
    ax.tick_params(axis='y', colors=blue)
    ax.yaxis.label.set_color(blue)
    twin = ax.twinx()
    twin.plot(s, margin, color=orange, lw=1.3, ls='--')
    k = np.argmin(margin)
    twin.plot(s[k], margin[k], 'o', color=orange, ms=3)
    twin.set(ylim=(0, max(margin)*1.22), ylabel=r'$m_q$ (rad)')
    twin.yaxis.label.set_color(orange)
    twin.tick_params(axis='y', colors=orange)
    twin.spines['right'].set_visible(True)
    ax.grid(alpha=.15)
    fig.savefig(here / 'connectivity_linear.pdf')
    fig.savefig(here / 'connectivity_linear.png', dpi=200)
    plt.close(fig)


if __name__ == '__main__':
    main()
