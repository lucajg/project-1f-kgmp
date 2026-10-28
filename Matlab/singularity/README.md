# Singularity analysis and pose explorer

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Explore arm singularities in wrist-center coordinates and the full 6R IK branches of an interactive target.

## Files

| File | Purpose |
| --- | --- |
| [singularity_map.m](singularity_map.m) | Wrist-center and tool-orientation controls, stationary singularity maps, and full 6R IK table. |
| [plot_ik_path.m](plot_ik_path.m) | Static arm singularity map with numbered IK wrist branches and a connecting path. |
| [check_singularity_map.m](check_singularity_map.m) | Check branch tracking, workspace curves, and actual UI callbacks. |
| [private/match_ik_solutions.m](private/match_ik_solutions.m) | Match unordered IK solutions to persistent IDs. |
| [private/singularity_map_data.m](private/singularity_map_data.m) | Cache the determinant grid and arm singularity contours. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
fig = singularity_map;
% fig.UserData.T: desired tool pose.
% fig.UserData.wristPosition: desired wrist center [x_w; y_w; z_w], in metres.
% fig.UserData.pose: [x_w; y_w; z_w; tool roll; tool pitch; tool yaw], metres/degrees.
% fig.UserData.Q: 6 x 8 joint configurations; absent IDs contain NaN.
```

## Dependencies

[Robot models](../models/README.md), [FK](../fk/README.md), [IK](../ik/README.md), and [Jacobians](../jacobian/README.md). Requires MATLAB graphics, but no additional toolbox. The helpers stay private to this folder, where both the app and its checks can call them.

## Inputs, outputs, and implementation

**[plot_ik_path.m](plot_ik_path.m)** plots target-pose IK branches and a joint path on the arm singularity map. Its optional fifth argument is an existing axes handle, allowing the planner to include the map in a single figure alongside the tool path, joint angles, and sampled margins. Without that argument it opens its own figure. The fourth argument remains `path_complete` (default `true`); use `false` to label a failed attempt's valid prefix.

**[singularity_map.m](singularity_map.m)** opens an interactive pose/IK explorer. Run `fig = singularity_map` after running `curo6r_setup`. The first three sliders and fields place the **wrist center**, in metres. The other three set the **tool orientation** using roll/pitch/yaw in degrees, with `R = Rz(yaw)*Ry(pitch)*Rx(roll)` for the physical `link_6` frame. All six sliders update while dragging; text fields allow precise values. The full tool target passed to the 6R IK solver is `T = [R, p_w + d6*R(:,1); 0 0 0 1]`. Position edits preserve tool orientation, with the wrist joints adjusting accordingly. Orientation edits preserve the wrist center and arm configurations while changing the wrist joints and, generally, the tool-tip position.

You can also click and drag the star in the workspace plot: horizontal movement changes `rho_w`, vertical movement changes `z_w`, and the current wrist-center azimuth and tool orientation stay fixed. The sliders, fields, and IK solutions update together. Dragging is bounded by the plot and wrist-position slider ranges, and the axes stay fixed until release. The default wrist center is `(3,0,3)` m with zero tool roll, pitch, and yaw, giving eight regular IK solutions and a starting drag plane of `y_w = 0`. Reset pose restores these defaults and resets numbering; selecting a table row highlights its marker. Unreachable targets clear the markers and show a message.

The left plot shows the arm factor `D(q2,q3)`: pale blue for negative and pale pink for positive, with arm singularities in black. For this robot, `det(J) = D(q2,q3)*sin(q5)`, independently of `q1`, `q4`, and `q6`; the background is computed using `q5 = pi/2`. Each IK solution has its own wrist configuration, so the background does not give every solution's full determinant sign. Each of up to eight solutions has a fixed ID and color in the table, which lists all six angles in degrees. Wrist flips share a point in this plot, so they use concentric rings and a combined numbered label. Near-singular arms/wrists and exact wrist families are flagged in the table; continuous families have the representative returned by the IK solver.

IDs follow a minimum-cost, one-to-one matching of all six joint angles modulo `2*pi`, rather than the root solver's output order. Missing solutions retain their last configuration for later matching. This preserves local continuity through regular motion and angle wrapping; identities can be ambiguous at branch mergers or after large pose jumps. The plotted angles still wrap at the edges of `[-pi,pi]`. The helper `private/match_ik_solutions.m` enumerates the small set of `8!` assignments, requiring no additional toolbox. `fig.UserData.Q` exposes a `6 x 8` array in radians, with IDs as columns and absent solutions as `NaN`; `fig.UserData.T` is the target transform.

The right plot shows the **wrist-center** target in `(rho_w,z_w)`, with `rho_w = hypot(x_w,y_w)` in metres. Its black curves are the cached wrist-center images of the arm singularity contours. They are independent of the target position, azimuth, and tool orientation, so moving any control leaves the curves unchanged. Rotation about joint 1 preserves `(rho_w,z_w)`; the displayed target azimuth only determines the Cartesian direction used when dragging. On the base axis, the last azimuth is retained because azimuth is undefined there. Wrist singularities are flagged in the solution list, rather than drawn as arm boundaries. Black curves are critical values: crossing one can change the number of IK solutions, but a point on a curve may also have other, regular solutions. The app retains full 6R IK, including both regular wrist flips for each arm solution.

`private/singularity_map_data.m` caches a `241 x 241` numerical PoE determinant grid and its interpolated zero contours. Both singularity maps are drawn once; only IK, branch matching, and the target/solution graphics update while dragging. The first launch takes a few seconds; later launches reuse the cache. No Symbolic Math Toolbox is needed. Increasing the grid size in `singularity_map` refines the contours; finite grids can miss zeros without a sign change.

## Checks

**[check_singularity_map.m](check_singularity_map.m)** checks root-order independence, missing/reappearing solutions, angle wrapping, continuous tracking along a regular eight-solution motion, and independent DH forward-kinematics agreement. It also verifies the wrist-center singularity curves, stationary boundaries, position edits at fixed tool orientation, orientation edits at a fixed wrist center with both flips retained, actual slider/text-field/drag callbacks, unreachable targets, reset, row selection, and wrist-family reporting. Run `check_singularity_map` to exercise these checks in an invisible app window.

The full runner includes this check by default; `run_checks(false, false)` omits it when graphics are unavailable. See the report's [Jacobian and singularity notes](../../report/notes/jacobian_detailed.tex).
