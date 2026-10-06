# project-1f-kgmp

MATLAB forward kinematics for CuRo6R, a robot with six revolute joints, using the Product of Exponentials (PoE), standard Denavit-Hartenberg (DH), and directly composed URDF joint transforms. All three use `base_link` as the base, `link_6` as the endpoint, and the same joint order, positive directions, and zero configuration. Distances are in metres and angles in radians; no Robotics Toolbox is required.

From the repository root in MATLAB:

```matlab
addpath('Matlab');
poe_data;
dh_data;
urdf_data;
q = [20; -35; 50; 15; -40; 60]*pi/180;
[T_poe, p_poe, w_poe] = poe_fk(q, poe_model);
[T_dh, p_dh, w_dh] = dh_fk(q, dh_model);
[T_urdf, p_urdf, w_urdf] = urdf_fk(q, urdf_model);
check_dh_poe;

curo6r_compare(poe_model, dh_model); % Side-by-side, shared joint controls.
% curo6r_viewer(poe_model);          % PoE alone.
% curo6r_viewer(dh_model);           % DH alone.
```

All three FK functions accept a six-element row or column vector `q`. They return a homogeneous pose `T = [R, t; 0, 0, 0, 1]`, where `R` is the endpoint orientation and `t` its position in the base frame. The `3 x 6` arrays `p` and `w` contain points on the joint axes and their unit directions, also in the base frame. PoE and URDF return physical joint origins; DH returns origins of its chosen frames, which may lie elsewhere along the same axes.

**[robot_dimensions.m](Matlab/robot_dimensions.m)** defines the nine lengths shared by DH and PoE: `h1`, `h2`, `l2`, `l3`, `w3`, `l4`, `h4`, `l5`, and `l6`. Their data scripts run it automatically. Rerun both scripts after editing it to rebuild those model structs. It stores lengths only; the fixed angular geometry remains in the data scripts. URDF data is transcribed independently, so geometry changes must also be reflected in the URDF and `urdf_data.m` for the triple check to agree.

**[poe_data.m](Matlab/poe_data.m)** builds `poe_model` at the home configuration (`q = 0`). Each column of `omega` gives a joint's unit rotation axis, and `r` gives its physical joint origin. For a revolute joint, the screw axis is `S = [omega; v]`, with `v = r x omega = -omega x r`; this represents rotation about an axis that need not pass through the base origin. The script stores the screw axes in `S`, their `4 x 4` twist matrices in `xi_hat`, and the endpoint's home pose in `M`. This is the geometry needed for the [space-frame PoE formula (Modern Robotics, section 4.1.1)](https://modernrobotics.northwestern.edu/nu-gm-book-resource/4-1-1-product-of-exponentials-formula-in-the-space-frame/).

**[poe_fk.m](Matlab/poe_fk.m)** computes `T = expm(xi_hat(:,:,1)*q(1)) * ... * expm(xi_hat(:,:,6)*q(6)) * M`. Each matrix exponential describes one joint's rigid rotation about its home screw axis. The function validates `q`, converts it to a column, and accumulates these transformations in `F`.

Before adding joint `i`'s motion, `F` contains only joints `1,...,i-1`. Its rotation and translation carry joint `i`'s home origin and axis into their current positions: `p(:,i) = R*r(:,i) + t` and `w(:,i) = R*omega(:,i)`. Thus `p` tracks the physical joint origins specified in `poe_data`.

**[dh_data.m](Matlab/dh_data.m)** builds `dh_model` using standard DH frames, with joint `i` rotating about `z_(i-1)`. The four parameter arrays describe the angle offset `theta`, translation `d` along the preceding z-axis, translation `a` along the new x-axis, and twist `alpha` about that x-axis. In particular, `d = [h1+h2, w3, h4, l4+l5, 0, l6]` and `a = [l2, l3, 0, 0, 0, 0]` use the shared dimensions. The actual joint angle is `theta(i) + q(i)`, so the third joint's `pi/2` offset is present even at home. See also the [MathWorks DH parameter reference](https://www.mathworks.com/help/robotics/ref/rigidbodyjoint.setfixedtransform.html).

The fixed `tool_transform` expresses endpoint frame `link_6` in DH frame 6. Their origins coincide, but their axes differ: `x_E = z_6`, `y_E = x_6`, and `z_E = y_6`. Appending this transform on the right makes the DH endpoint frame match the one used by PoE.

**[dh_fk.m](Matlab/dh_fk.m)** constructs each standard DH transform explicitly as `A_i = Rz(theta(i)+q(i)) * Tz(d(i)) * Tx(a(i)) * Rx(alpha(i))`, then computes `T = A_1 * ... * A_6 * tool_transform`. Before multiplying by `A_i`, `F` is the pose of frame `i-1`, so its origin `F(1:3,4)` and z-axis `F(1:3,3)` give `p(:,i)` and `w(:,i)`. A DH frame origin is a point on the corresponding joint axis; it can lie elsewhere along that line than the physical origin returned by `poe_fk`, without changing the joint's motion.

**[urdf_data.m](Matlab/urdf_data.m)** transcribes the six revolute joints from [CuRo6R.urdf](CuRo6R/CuRo6R.urdf) into `urdf_model`, with one joint per column of `xyz`, `rpy`, and `axis`. No file parsing is performed. Positions and roll-pitch-yaw angles place each joint frame relative to its parent link; axes are expressed in those local joint frames. The URDF's `0.52359877559` roll is interpreted as the intended exact `pi/6`, matching DH and PoE. The endpoint is `link_6`; `tool0` branches from `link_3_d3`, and visual/collision origins only place meshes.

**[urdf_fk.m](Matlab/urdf_fk.m)** alternates fixed placement and joint motion. For joint `i`, it forms `origin = [Rz(yaw)*Ry(pitch)*Rx(roll), xyz; 0, 0, 0, 1]`, then updates `F = F * origin`. At this point, `F` places the joint in the base frame: its translation gives `p(:,i)` and its rotation maps the local axis to `w(:,i)`. It then appends the local rotation by `q(i)`, using Rodrigues' formula `R = I + sin(q)*K + (1-cos(q))*K^2`, where `K*v = cross(axis,v)`. After joint 6, `F` is already the endpoint pose. Thus URDF composes local link geometry, DH composes specially chosen joint frames, and PoE moves home screw axes expressed in the base frame.

**[curo6r_viewer.m](Matlab/curo6r_viewer.m)** provides a shared viewer for either model: call `curo6r_viewer(poe_model)` or `curo6r_viewer(dh_model)`. Passing both structs opens two plots with one joint-angle vector. It uses classic MATLAB `figure`, `axes`, and `uicontrol` objects, with no web-based UI, resize callbacks, or continuous camera links. A small adapter selects the FK function; both models use the same `[T,p,w]` drawing interface. Only joint rotation axes and an endpoint marker are drawn, without coordinate-frame triads.

Sliders update on release; you can also type angles between -180 and 180 degrees. Home zeros the angles, and the joint selector isolates individual axes. Plot limits stay fixed and cover the robot's reach. Use the standard figure toolbar to rotate, pan, or zoom a plot, then click Match views to copy that plot's camera and limits to the other one. Reset view restores both plots. The comparison readout reports the three errors used by `check_dh_poe`. Solid PoE connectors join physical joint origins; dashed DH connectors join frame origins and need not follow physical links.

If the previous viewer is still stuck in the current MATLAB session, interrupt it with Ctrl+C, then run `delete(findall(groot,'Type','figure','Name','CuRo6R | Joint-axis explorer'))` to remove its windows. Run `clear curo6r_viewer curo6r_compare`, then reopen with `curo6r_compare`. This closes only windows with the old viewer's name.

**[curo6r_compare.m](Matlab/curo6r_compare.m)** is a thin entry point for the comparison: `curo6r_compare(poe_model,dh_model)` uses existing models, while `curo6r_compare` loads the two data scripts automatically. Both routes call the shared viewer, so the single-model and comparison controls behave identically.

**[private/curo6r_scene.m](Matlab/private/curo6r_scene.m)** creates the graphics for one panel and returns an update function that accepts `[T,p,w]` plus display options. It reuses the graphics objects as angles change. Coloured dots mark each model's chosen origins, arrows show positive joint-axis directions, and dotted guides extend the axes. Each guide is centred at `p - w*dot(w,p)`, the point on the axis closest to the base origin for unit `w`. This point is unchanged when `p` slides along the axis, so equivalent PoE and DH axes receive identical guide segments despite different origin choices. The black diamond `E` marks `link_6`.

**[check_dh_poe.m](Matlab/check_dh_poe.m)** rebuilds all three models and compares DH-PoE, DH-URDF, and PoE-URDF at 514 configurations. The columns of `Q` contain the home pose, 12 individual joint motions (each joint at `+pi/2` and `-pi/2`, with the others zero), one mixed pose, and 500 random joint vectors with angles between `-pi` and `pi`. A local random stream with seed 42 makes the samples repeatable without changing MATLAB's global random stream. Each loop iteration sends the same column of `Q` to all three FK functions; the third dimension of `T`, `p`, and `w` selects the model.

For each configuration and model pair, it measures three errors. Taking DH-PoE as an example: `norm(T_dh-T_poe,'fro')` compares the complete endpoint poses, `norm(w_dh-w_poe,'fro')` compares joint-axis directions, and `norm(cross(p_dh-p_poe,w_poe,1),'fro')` compares axis locations. The Frobenius norm is the square root of the sum of squared matrix entries. The cross product is taken separately for each column: a displacement parallel to an axis has zero cross product with that axis. Since `w_poe` is a unit vector, each cross-product magnitude is the perpendicular distance to the PoE axis line. Together with equal directions, zero distance means the two representations describe the same axis, even when their chosen origins differ. A separate check compares PoE and URDF joint origins directly, since these physical origins should coincide.

Each row of `max_error` retains the largest pose, direction, and axis-line errors for one pair across all configurations. All comparisons, including the physical-origin check, must be below `1e-10`; using exact `pi/6` consistently leaves only floating-point roundoff. The function also repeats each model's final evaluation with a row vector and checks that all outputs match the column-vector result. On success it prints the sample count and a table of maximum errors; a failed comparison names the pair and measure. The pose norm combines rotation entries and position entries, so it is a numerical consistency check rather than a physical distance. These sampled checks support agreement between the implementations; they are not a proof for every configuration or an independent validation of the physical robot geometry.
