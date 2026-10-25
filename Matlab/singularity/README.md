# Singularity analysis and pose explorer

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Explore arm singularities, tool workspace slices, and the IK branches of an interactive target pose.

## Files

| File | Purpose |
| --- | --- |
| [singularity_map.m](singularity_map.m) | Interactive pose controls, determinant map, workspace slice, and IK table. |
| [plot_ik_path.m](plot_ik_path.m) | Static arm singularity map with numbered IK wrist branches and a connecting path. |
| [check_singularity_map.m](check_singularity_map.m) | Check branch tracking, workspace curves, and actual UI callbacks. |
| [private/match_ik_solutions.m](private/match_ik_solutions.m) | Match unordered IK solutions to persistent IDs. |
| [private/singularity_map_data.m](private/singularity_map_data.m) | Cache the determinant grid and arm singularity contours. |
| [private/singularity_workspace_curve.m](private/singularity_workspace_curve.m) | Map wrist-center curves to a tool-position slice. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
fig = singularity_map;
% fig.UserData.T: desired tool pose.
% fig.UserData.Q: 6 x 8 joint configurations; absent IDs contain NaN.
```

## Dependencies

[Robot models](../models/README.md), [FK](../fk/README.md), [IK](../ik/README.md), and [Jacobians](../jacobian/README.md). Requires MATLAB graphics, but no additional toolbox. The helpers stay private to this folder, where both the app and its checks can call them.

## Inputs, outputs, and implementation

**[plot_ik_path.m](plot_ik_path.m)** plots target-pose IK branches and a joint path on the arm singularity map. Its optional fifth argument is an existing axes handle, allowing the planner to include the map in a single figure alongside the tool path, joint angles, and sampled margins. Without that argument it opens its own figure. The fourth argument remains `path_complete` (default `true`); use `false` to label a failed attempt's valid prefix.

**[singularity_map.m](singularity_map.m)** opens an interactive pose/IK explorer. Run `fig = singularity_map` after running `curo6r_setup`. Six sliders update while dragging; text fields allow precise values. You can also click and drag the star in the workspace plot: horizontal movement changes `rho`, vertical movement changes `z`, and the current azimuth and orientation stay fixed. The sliders, fields, and IK solutions update together. Dragging is bounded by the plot and slider ranges, and the axes stay fixed until release. Position is in metres, roll/pitch/yaw in degrees, with `R = Rz(yaw)*Ry(pitch)*Rx(roll)` for the physical `link_6` tool frame. The initial pose has eight regular solutions. Reset pose restores it and resets numbering; selecting a table row highlights its marker. Unreachable targets clear the markers and show a message.

The left plot retains the binary determinant map: pale blue for negative and pale pink for positive, with arm singularities in black. For this robot, `det(J) = D(q2,q3)*sin(q5)`, independently of `q1`, `q4`, and `q6`; the background uses `q5 = pi/2`. It therefore shows the arm factor's sign, while each IK solution has its own wrist configuration. Each of up to eight solutions has a fixed ID and color in the table, which lists all six angles in degrees. Wrist flips share a point in this plot, so they use concentric rings and a combined numbered label. Near-singular arms/wrists and exact wrist families are flagged in the table; continuous families have the representative returned by the IK solver.

IDs follow a minimum-cost, one-to-one matching of all six joint angles modulo `2*pi`, rather than the root solver's output order. Missing solutions retain their last configuration for later matching. This preserves local continuity through regular motion and angle wrapping; identities can be ambiguous at branch mergers or after large pose jumps. The plotted angles still wrap at the edges of `[-pi,pi]`. The helper `private/match_ik_solutions.m` enumerates the small set of `8!` assignments, requiring no additional toolbox. `fig.UserData.Q` exposes a `6 x 8` array in radians, with IDs as columns and absent solutions as `NaN`; `fig.UserData.T` is the target transform.

The right plot shows the **tool** target in `(rho,z)`, with `rho = hypot(x,y)` in metres. Its black curves are arm singularities at the **current orientation and position azimuth**, not a projection over all orientations. The numerical joint-space contours are mapped to wrist-center circles of radius `rw` and height `zw`. At fixed orientation, the tool offset is `o = d6*R(:,1)`; on the ray `u = [cos(phi);sin(phi)]`, the intersections satisfy `|rho*u-o_xy|^2 = rw^2` and `z = zw+o_z`. Both nonnegative roots are drawn. This accounts for the tool offset even when it is not radial. On the base axis, the last azimuth is retained because azimuth is undefined there. Wrist singularities are flagged in the solution list, rather than drawn as arm boundaries. Black curves are critical values: crossing one can change the number of IK solutions, but a point on a curve may also have other, regular solutions.

`private/singularity_map_data.m` caches a `241 x 241` numerical PoE determinant grid and its interpolated zero contours. Only IK, branch matching, the translated workspace curves, and existing graphics update while dragging. The first launch takes a few seconds; later launches reuse the cache. No Symbolic Math Toolbox is needed. Increasing the grid size in `singularity_map` refines the contours; finite grids can miss zeros without a sign change.

## Checks

**[check_singularity_map.m](check_singularity_map.m)** checks root-order independence, missing/reappearing solutions, angle wrapping, continuous tracking along a regular eight-solution motion, and independent DH forward-kinematics agreement. It also checks both workspace-ray intersections, the mapped singularity curves, actual live-slider and text-field callbacks, unreachable targets, reset, row selection, and wrist-family reporting. Run `check_singularity_map` to exercise these checks in an invisible app window.

The full runner includes this check by default; `run_checks(false, false)` omits it when graphics are unavailable. See the report's [Jacobian and singularity notes](../../report/notes/jacobian_detailed.tex).
