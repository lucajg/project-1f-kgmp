# project-1f-kgmp

MATLAB forward kinematics, analytical inverse kinematics, and geometric Jacobians for CuRo6R, a robot with six revolute joints. Forward kinematics uses the Product of Exponentials (PoE), standard Denavit-Hartenberg (DH), and directly composed URDF joint transforms. All three use `base_link` as the base, `link_6` as the endpoint, and the same joint order, positive directions, and zero configuration. Distances are in metres and angles in radians. Numerical calculations, including IK, require no Robotics Toolbox; symbolic calculations require Symbolic Math Toolbox.

From the repository root in MATLAB:

```matlab
addpath('Matlab');
g = robot_dimensions();
poe_model = poe_data(g);
dh_model = dh_data(g);
urdf_data;
q = [20; -35; 50; 15; -40; 60]*pi/180;
[T_poe, p_poe, w_poe] = poe_fk(q, poe_model);
[T_dh, p_dh, w_dh] = dh_fk(q, dh_model);
[T_urdf, p_urdf, w_urdf] = urdf_fk(q, urdf_model);
check_dh_poe;
check_jacobians(false); % Numerical checks; omit false to include symbolic checks.
check_analytical_ik;

curo6r_compare(poe_model, dh_model); % Side-by-side, shared joint controls.
% curo6r_viewer(poe_model);          % PoE alone.
% curo6r_viewer(dh_model);           % DH alone.
```

All three FK functions accept a six-element row or column vector `q`. They return a homogeneous pose `T = [R, t; 0, 0, 0, 1]`, where `R` is the endpoint orientation and `t` its position in the base frame. The `3 x 6` arrays `p` and `w` contain points on the joint axes and their unit directions, also in the base frame. PoE and URDF return physical joint origins; DH returns origins of its chosen frames, which may lie elsewhere along the same axes.

**[robot_dimensions.m](Matlab/robot_dimensions.m)** defines the nine lengths shared by DH and PoE: `h1`, `h2`, `l2`, `l3`, `w3`, `l4`, `h4`, `l5`, and `l6`. `robot_dimensions()` returns their numerical values in a struct; `robot_dimensions(true)` returns real symbolic variables with the same field names. Both `dh_data(g)` and `poe_data(g)` accept this struct; omitting `g` selects the default numerical dimensions. Rebuild the models after changing dimensions. Fixed angular geometry remains in the model builders, which use exact symbolic angles for symbolic dimensions. URDF data is transcribed independently, so geometry changes must also be reflected in the URDF and `urdf_data.m` for the triple check to agree.

**[poe_data.m](Matlab/poe_data.m)** builds `poe_model` at the home configuration (`q = 0`). Each column of `omega` gives a joint's unit rotation axis, and `r` gives its physical joint origin. For a revolute joint, the screw axis is `S = [omega; v]`, with `v = r x omega = -omega x r`; this represents rotation about an axis that need not pass through the base origin. The function stores the screw axes in `S`, their `4 x 4` twist matrices in `xi_hat`, and the endpoint's home pose in `M`. This is the geometry needed for the [space-frame PoE formula (Modern Robotics, section 4.1.1)](https://modernrobotics.northwestern.edu/nu-gm-book-resource/4-1-1-product-of-exponentials-formula-in-the-space-frame/).

The model also stores `R_home(:,:,f+1)`, the orientation of named DH frame `f` at home, with frame 0 in the first slice. These orientations are generated from the fixed angle offsets and twists in `dh_data.m`. This defines the coordinate axes selected by the Jacobian's `frame` input; the screw axes and endpoint home pose are still constructed from the PoE geometry above. The Jacobian obtains each current frame orientation by multiplying its home orientation by the rotation of the corresponding PoE partial product.

**[poe_fk.m](Matlab/poe_fk.m)** computes the product `T = expm(xi_hat(:,:,1)*q(1)) * ... * expm(xi_hat(:,:,6)*q(6)) * M`. For the robot's unit revolute axes, each exponential is evaluated directly with `R = I + sin(q)*W + (1-cos(q))*W^2` and translation `(I-R)*r`. This closed form supports both numbers and exact symbols. The optional fourth output `F0(:,:,i+1)` contains the product through joint `i`, with identity in the first slice. Symbolic cumulative transforms are simplified at each step.

The optional third input, `poe_fk(q,poe_model,joint)`, limits the product to the first `joint` factors and requires that many joint angles. Omitting it uses all model joints. The endpoint home pose `M` is still appended: shortening the product holds later joints at zero, while retaining their geometry. To track the wrist center with three active joints, copy the model and set `M(1:3,4) = r(:,5)` in that copy.

Before adding joint `i`'s motion, `F` contains only joints `1,...,i-1`. Its rotation and translation carry joint `i`'s home origin and axis into their current positions: `p(:,i) = R*r(:,i) + t` and `w(:,i) = R*omega(:,i)`. Thus `p` tracks the physical joint origins specified in `poe_data`.

**[dh_data.m](Matlab/dh_data.m)** builds `dh_model` using standard DH frames, with joint `i` rotating about `z_(i-1)`. The four parameter arrays describe the angle offset `theta`, translation `d` along the preceding z-axis, translation `a` along the new x-axis, and twist `alpha` about that x-axis. In particular, `d = [h1+h2, w3, h4, l4+l5, 0, l6]` and `a = [l2, l3, 0, 0, 0, 0]` use the shared dimensions. The actual joint angle is `theta(i) + q(i)`, so the third joint's `pi/2` offset is present even at home. See also the [MathWorks DH parameter reference](https://www.mathworks.com/help/robotics/ref/rigidbodyjoint.setfixedtransform.html).

The fixed `tool_transform` expresses endpoint frame `link_6` in DH frame 6. Their origins coincide, but their axes differ: `x_E = z_6`, `y_E = x_6`, and `z_E = y_6`. Appending this transform on the right makes the DH endpoint frame match the one used by PoE.

**[dh_fk.m](Matlab/dh_fk.m)** uses the shared `private/dh_transform.m` to construct each standard DH transform as `A_i = Rz(theta(i)+q(i)) * Tz(d(i)) * Tx(a(i)) * Rx(alpha(i))`, then computes `T = A_1 * ... * A_6 * tool_transform`. Before multiplying by `A_i`, `T0(:,:,i)` is the pose of frame `i-1`; its origin and z-axis give `p(:,i)` and `w(:,i)`. The optional fourth output `T0(:,:,i+1)` holds the pose of DH frame `i`, including frame 0 in the first slice. Numeric and symbolic inputs use the same computation. A DH frame origin is a point on the corresponding joint axis; it can lie elsewhere along that line than the physical origin returned by `poe_fk`, without changing the joint's motion.

**[urdf_data.m](Matlab/urdf_data.m)** transcribes the six revolute joints from [CuRo6R.urdf](CuRo6R/CuRo6R.urdf) into `urdf_model`, with one joint per column of `xyz`, `rpy`, and `axis`. No file parsing is performed. Positions and roll-pitch-yaw angles place each joint frame relative to its parent link; axes are expressed in those local joint frames. The URDF's `0.52359877559` roll is interpreted as the intended exact `pi/6`, matching DH and PoE. The endpoint is `link_6`; `tool0` branches from `link_3_d3`, and visual/collision origins only place meshes.

**[urdf_fk.m](Matlab/urdf_fk.m)** alternates fixed placement and joint motion. For joint `i`, it forms `origin = [Rz(yaw)*Ry(pitch)*Rx(roll), xyz; 0, 0, 0, 1]`, then updates `F = F * origin`. At this point, `F` places the joint in the base frame: its translation gives `p(:,i)` and its rotation maps the local axis to `w(:,i)`. It then appends the local rotation by `q(i)`, using Rodrigues' formula `R = I + sin(q)*K + (1-cos(q))*K^2`, where `K*v = cross(axis,v)`. After joint 6, `F` is already the endpoint pose. Thus URDF composes local link geometry, DH composes specially chosen joint frames, and PoE moves home screw axes expressed in the base frame.

**[analytical_ik_3r.m](Matlab/analytical_ik_3r.m)** solves the positioning problem with `Q_arm = analytical_ik_3r(T_d,poe_model,dh_model)`. Each column is a candidate `[q1; q2; q3]`. The input is the desired **link_6 pose**, not a wrist-center pose: the solver first computes `p_w = T_d(1:3,4) - d6*T_d(1:3,1)`. The tool offset is along the physical tool's x-axis. This removes `d6` from the arm geometry while retaining `d4`, which reaches the wrist center.

The solver expresses the wrist center relative to the shoulder, undoes the fixed 30-degree tilt, and eliminates `q2` using the unchanged y-component and preserved vector length. Combining the resulting expressions for `sin(q3)` and `cos(q3)` gives a quartic in `t = tan(q1/2)`. Its coefficient arrays use descending powers, as expected by MATLAB's `roots`; `conv(P,P)` forms the coefficients of a squared polynomial. Each real root gives a candidate `q1`, and `q1 = pi` is checked separately because it is absent from the finite half-angle chart. Back-substitution recovers `q3` and then `q2` with `atan2`. Candidates are checked against PoE wrist-center FK and deduplicated modulo `2*pi`.

**[analytical_ik_6r.m](Matlab/analytical_ik_6r.m)** calls the positioning solver, then solves the wrist for every arm candidate. The physical arm orientation is `R_arm = R_F*poe_model.M(1:3,1:3)`, where `R_F` is the rotation of the first three PoE factors; the home orientation must be included. The desired wrist rotation is `R_arm.'*T_d(1:3,1:3)`, which decomposes as `Rx(q4)*Rz(q5)*Rx(q6)` for this robot's local wrist axes.

For a regular wrist, `rho = hypot(R(2,1),R(3,1))` gives `abs(sin(q5))`. Both signs are retained, giving two wrist branches per arm candidate and up to eight isolated solutions in the generic case. The first column gives `q4`, including the chosen sign of `sin(q5)`. To recover `q6`, the solver removes `Rx(q4)` on the left: the remaining `Rz(q5)*Rx(q6)` has third row `[0, sin(q6), cos(q6)]`. This avoids dividing by a small `sin(q5)` a second time and preserves branches close to wrist singularities. At `q5 = 0`, only `q4 + q6` is determined; at `q5 = pi`, only `q4 - q6` is determined. The solver chooses `q4 = 0` for these singular families and solves the remaining angle. Every returned six-angle column is checked against the complete PoE tool pose, with separate position and rotation tolerances.

Both solvers take a numeric rigid transform and consistent numeric models built from the same dimensions. They are specific to CuRo6R's axes and fixed tilt, and require nonzero `a2` and `d4`. Angles lie in `[-pi,pi]`, matching the supplied URDF limits; there is no separate custom-limit input. Unreachable targets return `zeros(3,0)` or `zeros(6,0)`. Continuous families have representative solutions rather than an exhaustive parameterization: for example, the positioning solver chooses `q1 = 0` when its constraint is identically zero. Root finding and singularity decisions use numerical tolerances; the analytic reduction does not require a numerical IK initial guess.

```matlab
q_test = [20; -35; 50; 15; -40; 60]*pi/180;
T_d = poe_fk(q_test,poe_model);
Q_arm = analytical_ik_3r(T_d,poe_model,dh_model); % 3 x number of arm solutions.
Q = analytical_ik_6r(T_d,poe_model,dh_model);    % 6 x number of full solutions.
if ~isempty(Q)
    T_check = poe_fk(Q(:,1),poe_model);
    position_error = norm(T_check(1:3,4)-T_d(1:3,4));
    rotation_error = norm(T_check(1:3,1:3)-T_d(1:3,1:3),'fro');
end
```

Different joint vectors can reproduce the same pose, so check FK agreement rather than demanding that every solution equal `q_test`.

**[check_analytical_ik.m](Matlab/check_analytical_ik.m)** performs repeatable round trips against independent DH and URDF forward kinematics. It checks default and modified dimensions, recovery of the original regular arm and both wrist branches, `q1 = pi`, exact and near wrist singularities, representative solutions on the base axis, duplicate removal, unreachable targets, and rejection of non-rigid target matrices. Run `check_analytical_ik` after adding the `Matlab` folder to the path.

**[dh_jacobian.m](Matlab/dh_jacobian.m)** and **[poe_jacobian.m](Matlab/poe_jacobian.m)** both accept `(q, frame, joint, model)` and return a `6 x n` matrix with linear velocity above tool angular velocity. All joints contribute. `joint` selects the point at which the tool's velocity field is evaluated; `frame` selects the DH axes used to express both base-relative velocity vectors. This does not subtract the selected frame's own motion. DH selects a DH origin, while PoE selects a physical joint origin. Points 5 (wrist center) and 6 (endpoint) coincide across the models; other matching indices need not select the same point.

```matlab
J06 = dh_jacobian(q,0,6,dh_model);
J05 = dh_jacobian(q,0,5,dh_model);
J35 = poe_jacobian(q,3,5,poe_model);
J_position = J35(1:3,1:3);
```

The Jacobians reuse the forward kinematics and a shared cross-product calculation. The existing `dh_jacobian_numerical` and `poe_jacobian_numerical` names remain thin wrappers. To derive an exact Jacobian, supply symbolic joint coordinates and models built from symbolic dimensions:

```matlab
gs = robot_dimensions(true);
qs = sym('q',[6,1],'real');
Jdh_sym = simplify(dh_jacobian(qs,3,5,dh_data(gs)));
Jpoe_sym = simplify(poe_jacobian(qs,3,5,poe_data(gs)));
simplify(expand(Jdh_sym - Jpoe_sym)) % A 6 x 6 zero matrix.
```

**[dh_jacobian_symbolic.m](Matlab/dh_jacobian_symbolic.m)** and **[poe_jacobian_symbolic.m](Matlab/poe_jacobian_symbolic.m)** are short entry scripts using these same functions; edit `k` and `f` to select the point and expression frame. They produce `Jdh_sym` and `Jpoe_sym` in the shared dimension symbols, plus `Jdh_ad_sym` and `Jpoe_ad_sym` in DH parameters. Their model and joint variables remain separate from the numerical examples.

**[dh_parameter_form.m](Matlab/dh_parameter_form.m)** rewrites symbolic expressions using `a1 = l2`, `a2 = l3`, `d1 = h1+h2`, `d2 = w3`, `d3 = h4`, `d4 = l4+l5`, and `d6 = l6`. Zero DH entries and fixed angles retain their values. The substitution keeps `h1` and `l4` available because the DH table does not determine the individual splits of `d1` and `d4`; these residual dimensions cancel from the Jacobians at points 5 and 6. Other physical origins may still need them. A DH parameter can also disappear from the Jacobian: `d1` only translates this robot vertically and does not affect its velocity mapping.

**[check_jacobians.m](Matlab/check_jacobians.m)** compares the numerical DH and PoE Jacobians at 114 configurations, for points 5 and 6 and every expression frame from 0 to 6, using both default and modified dimensions. It also checks row-vector inputs, compatibility wrappers, the wrist's zero linear columns and retained sixth angular column, the closed-form PoE motions against matrix exponentials, and finite differences of position and tool rotation. By default it additionally proves symbolic DH-PoE equality for `(frame, point) = (0,6), (0,5), (3,5)` and compares numerical evaluations of those symbolic results with the numerical models. The DH parameter forms are checked against direct derivation from symbolic `a`/`d` arrays and against numerical models. Call `check_jacobians(false)` to run only the numerical checks.

**[curo6r_viewer.m](Matlab/curo6r_viewer.m)** provides a shared viewer for either model: call `curo6r_viewer(poe_model)` or `curo6r_viewer(dh_model)`. Passing both structs opens two plots with one joint-angle vector. It uses classic MATLAB `figure`, `axes`, and `uicontrol` objects, with no web-based UI, resize callbacks, or continuous camera links. A small adapter selects the FK function; both models use the same `[T,p,w]` drawing interface. Only joint rotation axes and an endpoint marker are drawn, without coordinate-frame triads.

Sliders update on release; you can also type angles between -180 and 180 degrees. Home zeros the angles, and the joint selector isolates individual axes. Plot limits stay fixed and cover the robot's reach. Use the standard figure toolbar to rotate, pan, or zoom a plot, then click Match views to copy that plot's camera and limits to the other one. Reset view restores both plots. The comparison readout reports the three errors used by `check_dh_poe`. Solid PoE connectors join physical joint origins; dashed DH connectors join frame origins and need not follow physical links.

If the previous viewer is still stuck in the current MATLAB session, interrupt it with Ctrl+C, then run `delete(findall(groot,'Type','figure','Name','CuRo6R | Joint-axis explorer'))` to remove its windows. Run `clear curo6r_viewer curo6r_compare`, then reopen with `curo6r_compare`. This closes only windows with the old viewer's name.

**[curo6r_compare.m](Matlab/curo6r_compare.m)** is a thin entry point for the comparison: `curo6r_compare(poe_model,dh_model)` uses existing models, while `curo6r_compare` builds the two default numerical models automatically. Both routes call the shared viewer, so the single-model and comparison controls behave identically.

**[private/curo6r_scene.m](Matlab/private/curo6r_scene.m)** creates the graphics for one panel and returns an update function that accepts `[T,p,w]` plus display options. It reuses the graphics objects as angles change. Coloured dots mark each model's chosen origins, arrows show positive joint-axis directions, and dotted guides extend the axes. Each guide is centred at `p - w*dot(w,p)`, the point on the axis closest to the base origin for unit `w`. This point is unchanged when `p` slides along the axis, so equivalent PoE and DH axes receive identical guide segments despite different origin choices. The black diamond `E` marks `link_6`.

**[check_dh_poe.m](Matlab/check_dh_poe.m)** rebuilds all three models and compares DH-PoE, DH-URDF, and PoE-URDF at 514 configurations. The columns of `Q` contain the home pose, 12 individual joint motions (each joint at `+pi/2` and `-pi/2`, with the others zero), one mixed pose, and 500 random joint vectors with angles between `-pi` and `pi`. A local random stream with seed 42 makes the samples repeatable without changing MATLAB's global random stream. Each loop iteration sends the same column of `Q` to all three FK functions; the third dimension of `T`, `p`, and `w` selects the model.

For each configuration and model pair, it measures three errors. Taking DH-PoE as an example: `norm(T_dh-T_poe,'fro')` compares the complete endpoint poses, `norm(w_dh-w_poe,'fro')` compares joint-axis directions, and `norm(cross(p_dh-p_poe,w_poe,1),'fro')` compares axis locations. The Frobenius norm is the square root of the sum of squared matrix entries. The cross product is taken separately for each column: a displacement parallel to an axis has zero cross product with that axis. Since `w_poe` is a unit vector, each cross-product magnitude is the perpendicular distance to the PoE axis line. Together with equal directions, zero distance means the two representations describe the same axis, even when their chosen origins differ. A separate check compares PoE and URDF joint origins directly, since these physical origins should coincide.

Each row of `max_error` retains the largest pose, direction, and axis-line errors for one pair across all configurations. All comparisons, including the physical-origin check, must be below `1e-10`; using exact `pi/6` consistently leaves only floating-point roundoff. The function also repeats each model's final evaluation with a row vector and checks that all outputs match the column-vector result. On success it prints the sample count and a table of maximum errors; a failed comparison names the pair and measure. The pose norm combines rotation entries and position entries, so it is a numerical consistency check rather than a physical distance. These sampled checks support agreement between the implementations; they are not a proof for every configuration or an independent validation of the physical robot geometry.
