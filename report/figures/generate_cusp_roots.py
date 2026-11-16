"""Near-upper-cusp companions to the connectivity reference-root figures.

The primed roots are the original indexed branches at a different target U.
They are matched by angular continuation through the four-root region,
not renumbered independently. Run with Python 3, NumPy and Matplotlib.
"""

import itertools
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np

from generate_connectivity_roots import (
    HERE, WIDTH, INK, BLUE, GRAY, conic_coefficients, evaluate_conic,
    reference_case, style_axes,
)
from generate_singularity_map import (
    quartic_coefficients, real_root_counts, verified_cusps, wrist_section,
)


def four_root_midpoint(z):
    """Find the interior of the upper four-root region at a fixed height."""
    rho = np.linspace(1.2, 3.3, 4097)
    inside = np.flatnonzero(real_root_counts(rho, z) == 4)
    assert len(inside) >= 3 and np.all(np.diff(inside) == 1)
    return (rho[inside[0]] + rho[inside[-1]]) / 2


def near_cusp_case():
    ref = reference_case()
    a = np.array([ref['rho_w_m'], ref['z_w_m']])
    b = np.array([1.4863, 5.38])
    cusp_q = verified_cusps()[0][-1]
    cusp = np.array(wrist_section(*cusp_q))
    # The direct A--cusp segment crosses a fold. Instead, move horizontally
    # into the region, follow its interior up the slice, then reach U.
    height = np.linspace(a[1], b[1], 401)
    middle = np.array([four_root_midpoint(z) for z in height])
    middle_path = np.column_stack((middle, height))
    path = np.vstack((
        np.column_stack((np.linspace(a[0], middle[0], 101), np.full(101, a[1]))),
        middle_path[1:],
        np.column_stack((np.linspace(middle[-1], b[0], 31), np.full(31, b[1])))[1:],
    ))
    polynomials = quartic_coefficients(*path.T)
    assert np.min(polynomials[:, 0]) > 100  # No half-angle root at infinity.
    permutations = np.array(list(itertools.permutations(range(4))))
    previous = np.asarray(ref['q3_rad_t1_to_t4'])
    tracked = []
    max_step, min_gap = 0., np.inf
    for p in polynomials:
        roots = np.roots(p)
        assert np.max(np.abs(roots.imag)) < 1e-11
        angles = 2 * np.arctan(roots.real)
        candidates = angles[permutations]
        distances = np.arctan2(np.sin(candidates - previous),
                               np.cos(candidates - previous))
        best = np.argmin(np.sum(distances**2, axis=1))
        ordered = candidates[best]
        step = np.max(np.abs(distances[best]))
        circular_gaps = np.diff(np.r_[np.sort(ordered), np.min(ordered) + 2*np.pi])
        gap = np.min(circular_gaps)
        assert step < gap / 3, 'Reduce the continuation step.'
        assert np.all(np.diff(ordered) > 0)
        previous = ordered
        tracked.append(np.tan(ordered / 2))
        max_step, min_gap = max(max_step, step), min(min_gap, gap)
    roots = np.array(tracked[-1])
    angles = 2 * np.arctan(roots)
    p = quartic_coefficients(*b)
    coef = conic_coefficients(*b)
    points = np.column_stack((np.cos(angles), np.sin(angles)))
    assert np.max(np.abs(np.polyval(p, roots))) < 1e-10
    assert np.max(np.abs(evaluate_conic(coef, *points.T))) < 1e-10
    assert np.max(np.abs(np.sum(points**2, axis=1) - 1)) < 1e-14
    derivatives = np.polyval(np.polyder(p), roots)
    assert np.min(np.abs(derivatives)) > 0.1
    t = np.linspace(-10, 10, 2001)
    delta = 1+t*t
    identity_error = np.max(np.abs(np.polyval(p, t)/delta**2 -
                                   evaluate_conic(coef, (1-t*t)/delta, 2*t/delta)))
    assert identity_error < 1e-10
    # Recover q2 for all four roots and independently check the target slice.
    f1 = 2 + 2.5*np.cos(angles)
    arm_b = 1.25 + 2.5*np.sin(angles)
    f3 = .8**2 + f1**2 + .75**2 + arm_b**2
    x = (b[0]**2 + (b[1]-1)**2 - f3)/1.6
    d = (b[1]-1-.5*arm_b)/(np.sqrt(3)/2)
    q2 = np.arctan2(.75*x-f1*d, f1*x+.75*d)
    error = np.max(np.linalg.norm(np.array(wrist_section(q2, angles)) - b[:,None], axis=0))
    assert error < 1e-10
    tc = np.tan(cusp_q[1]/2)
    cp = quartic_coefficients(*cusp)
    assert abs(np.polyval(np.polyder(cp, 3), tc)) > 1
    data = {
        'target_U_rho_z_m': b.tolist(),
        'reference_A_rho_z_m': a.tolist(),
        'upper_cusp_rho_z_m': cusp.tolist(),
        'upper_cusp_t': float(tc),
        'upper_cusp_q3_rad': float(cusp_q[1]),
        'distance_to_cusp_m': float(np.linalg.norm(b-cusp)),
        'polynomial_descending': p.tolist(),
        'conic_c2_cs_s2_c_s_constant': coef.tolist(),
        'primed_roots_t1_to_t4': roots.tolist(),
        'q3_rad_t1_to_t4': angles.tolist(),
        'circle_points_t1_to_t4': points.tolist(),
        'checks': {
            'continuation_samples': len(path),
            'minimum_angular_separation_rad': float(min_gap),
            'maximum_continuation_step_rad': float(max_step),
            'minimum_abs_P_derivative_at_U': float(np.min(np.abs(derivatives))),
            'polynomial_circle_identity_max_abs': float(identity_error),
            'recovered_arm_slice_error_m': float(error),
        },
        'continuation_path_rho_z_m': path.tolist(),
        'continued_t_values': np.asarray(tracked).tolist(),
    }
    return data


def root_markers(ax, points, indices, offsets):
    for point, i, offset in zip(points, indices, offsets):
        ax.plot(*point, 'o', ms=4.6, mfc='white', mec=BLUE, mew=1, zorder=6)
        if offset is None:
            continue
        ax.annotate(rf"$t_{{{i}}}'$", point, xytext=offset,
                    textcoords='offset points', ha='center', va='center',
                    fontsize=11, zorder=7,
                    arrowprops={'arrowstyle':'-', 'color':GRAY, 'lw':.55},
                    bbox={'facecolor':'white', 'edgecolor':'none', 'pad':.5})


def header(fig, data, title=None, y=.898):
    if title is not None:
        fig.text(.5, .958, title, ha='center', fontsize=12)
    rho,z = data['target_U_rho_z_m']
    fig.text(.5, y,
             rf"$U:\ (\rho_w,z_w)=({rho:.4f},\,{z:.4f})\,\mathrm{{m}}$"
             + r" $\quad$Four distinct real roots",
             ha='center', fontsize=11)


def footer(fig, data, explain=True, y=.111):
    for i,(x,t) in enumerate(zip([.14,.38,.62,.86],data['primed_roots_t1_to_t4']),1):
        fig.text(x,y,rf"$t_{{{i}}}'={t:.6f}$",ha='center',fontsize=10.5)
    if explain:
        fig.text(.5,.038,r"Primes denote values at $U$; branch indices are continued from the reference target.",
                 ha='center',fontsize=10)


def save_figure(fig, stem, title):
    fig.savefig(HERE/f'{stem}.pdf',metadata={
        'Title':title,
        'Subject':'Near upper cusp inside the four-root region; indices continued from connectivity_linear.pdf',
    })
    fig.savefig(HERE/f'{stem}.png',dpi=240)
    plt.close(fig)


def quartic_plot(data):
    p = np.asarray(data['polynomial_descending'])
    t = np.asarray(data['primed_roots_t1_to_t4'])
    fig = plt.figure(figsize=(WIDTH,3.95))
    header(fig,data,r"Near $C_3^u$: three roots approaching coalescence")
    overview=fig.add_axes([.10,.32,.36,.46])
    zoom=fig.add_axes([.61,.32,.355,.46])
    x=np.linspace(-.115,.65,1501)
    overview.plot(x,np.polyval(p,x),color=BLUE,lw=1.3)
    overview.axhline(0,color=INK,lw=.65)
    overview.set(xlim=(-.115,.65),ylim=(-2.7,5.5),xlabel=r'$t=\tan(q_3/2)$',ylabel=r'$P_U(t)$ ($\mathrm{m}^4$)')
    overview.set_xticks([0,.2,.4,.6])
    overview.set_yticks([-2,0,2,4])
    root_markers(overview,np.column_stack((t,np.zeros(4))),range(1,5),[(15,12),None,None,None])
    overview.annotate(r"$t_2',t_3',t_4'$",(t[2],0),xytext=(0,33),
                      textcoords='offset points',ha='center',fontsize=11,
                      arrowprops={'arrowstyle':'-','color':GRAY,'lw':.6})
    overview.set_title('(a) All four roots',fontsize=11,pad=8)
    left,right=t[1]-.035,t[3]+.035
    zx=np.linspace(left,right,1001)
    zoom.plot(zx,np.polyval(p,zx),color=BLUE,lw=1.3)
    zoom.axhline(0,color=INK,lw=.65)
    zoom.set(xlim=(left,right),ylim=(-.085,.10),xlabel=r'$t=\tan(q_3/2)$',ylabel=r'$P_U(t)$ ($\mathrm{m}^4$)')
    zoom.set_xticks([.4,.45,.5,.55,.6])
    zoom.set_yticks([-.05,0,.05,.10])
    root_markers(zoom,np.column_stack((t[1:],np.zeros(3))),range(2,5),[(-12,12),(9,20),(-13,20)])
    zoom.set_title('(b) Enlargement of the cluster',fontsize=11,pad=8)
    overview.add_patch(Rectangle((left,-.085),right-left,.185,
                                 facecolor='#dcebf2',edgecolor=GRAY,lw=.7,zorder=0))
    for ax in (overview,zoom):style_axes(ax)
    footer(fig,data)
    save_figure(fig,'cusp_quartic','CuRo6R: near-cusp quartic and enlarged three-root cluster')


def conic_plot(data):
    coef=np.asarray(data['conic_c2_cs_s2_c_s_constant'])
    points=np.asarray(data['circle_points_t1_to_t4'])
    matrix=np.array([[coef[0],coef[1]/2],[coef[1]/2,coef[2]]])
    values,basis=np.linalg.eigh(matrix)
    centre=-np.linalg.solve(matrix,coef[3:5])/2
    level=centre@matrix@centre-coef[5]
    assert values[0]<0<values[1] and level<0
    u=np.linspace(-4.5,4.5,6001)
    curves=[]
    for sign in [-1,1]:
        principal=np.array([sign*np.sqrt(level/values[0])*np.cosh(u),
                            np.sqrt(-level/values[1])*np.sinh(u)])
        curve=centre[:,None]+basis@principal
        assert np.max(np.abs(evaluate_conic(coef,*curve)))<1e-9
        curves.append(curve)
    fig=plt.figure(figsize=(WIDTH,3.20))
    overview=fig.add_axes([.09,.30,.37,.61])
    zoom=fig.add_axes([.60,.30,.37,.61])
    a=np.linspace(0,2*np.pi,5001)
    for ax in (overview,zoom):
        for curve in curves:ax.plot(*curve,color=BLUE,lw=1.2,zorder=3)
        # Dashed circle makes the two curves distinguishable where close.
        ax.plot(np.cos(a),np.sin(a),color=INK,lw=1.05,ls=(0,(4,2)),zorder=4)
        ax.set(xlabel=r'$c_3$',ylabel=r'$s_3$')
        ax.set_aspect('equal',adjustable='box')
        style_axes(ax)
    overview.set(xlim=(-1.18,1.25),ylim=(-1.13,1.30))
    overview.set_xticks([-1,0,1]);overview.set_yticks([-1,0,1])
    overview.set_title('(a) All four intersections',fontsize=11,pad=8)
    root_markers(overview,points,range(1,5),[(-18,-8),None,None,None])
    overview.annotate(r"$t_2',t_3',t_4'$",points[2],xytext=(-48,15),
                      textcoords='offset points',ha='center',fontsize=11,
                      arrowprops={'arrowstyle':'-','color':GRAY,'lw':.6})
    xmin,xmax=.45,.82;ymin,ymax=.63,1.00
    overview.add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,
                                 fill=False,edgecolor=GRAY,lw=.7,zorder=5))
    zoom.set(xlim=(xmin,xmax),ylim=(ymin,ymax))
    zoom.set_xticks([.5,.6,.7,.8]);zoom.set_yticks([.7,.8,.9,1.0])
    zoom.set_title('(b) Enlargement of the cluster',fontsize=11,pad=8)
    root_markers(zoom,points[1:],range(2,5),[(-18,2),(16,13),(-16,-12)])
    fig.legend(handles=[
        Line2D([],[],color=INK,lw=1.05,ls=(0,(4,2)),label=r'$c_3^2+s_3^2=1$'),
        Line2D([],[],color=BLUE,lw=1.2,label=r'$C_U(c_3,s_3)=0$'),
    ],loc='center',bbox_to_anchor=(.5,.155),ncol=2,frameon=False,
        fontsize=11,handlelength=2,columnspacing=2)
    footer(fig,data,explain=False,y=.05)
    save_figure(fig,'cusp_circle_conic','CuRo6R: near-cusp circle-conic intersections and enlarged cluster')


def main():
    plt.rcParams.update({
        'font.family':'serif','font.serif':['STIXGeneral'],'mathtext.fontset':'cm',
        'font.size':11,'axes.labelsize':11,'xtick.labelsize':10,'ytick.labelsize':10,
        'pdf.fonttype':42,
    })
    data=near_cusp_case()
    quartic_plot(data)
    conic_plot(data)
    (HERE/'cusp_roots_data.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({k:v for k,v in data.items() if k not in
                      ('continuation_path_rho_z_m','continued_t_values')},indent=2))


if __name__=='__main__':main()
