# project-1f-kgmp

MATLAB forward kinematics for CuRo6R, a robot with six revolute joints, using the Product of Exponentials (PoE) and standard Denavit-Hartenberg (DH) formulations. Both use `base_link` as the base, `link_6` as the endpoint, and the same joint order, positive directions, and zero configuration. Distances are in metres and angles in radians; no Robotics Toolbox is required.

From the repository root in MATLAB:

```matlab
addpath('Matlab');
poe_data;
dh_data;
q = [20; -35; 50; 15; -40; 60]*pi/180;
[T_poe, p_poe, w_poe] = poe_fk(q, poe_model);
[T_dh, p_dh, w_dh] = dh_fk(q, dh_model);
check_dh_poe;
```

Both FK functions accept a six-element row or column vector `q`. They return a homogeneous pose `T = [R, t; 0, 0, 0, 1]`, where `R` is the endpoint orientation and `t` its position in the base frame. The `3 x 6` arrays `p` and `w` contain points on the joint axes and their unit directions, also in the base frame; the meaning of `p` differs between the two formulations, as explained below.

**[robot_dimensions.m](Matlab/robot_dimensions.m)** defines the nine shared lengths: `h1`, `h2`, `l2`, `l3`, `w3`, `l4`, `h4`, `l5`, and `l6`. Both data scripts run it automatically, so length changes belong here. Rerun both data scripts after editing it to rebuild the model structs. It stores lengths only; the fixed angular geometry remains in the data scripts.

**[poe_data.m](Matlab/poe_data.m)** builds `poe_model` at the home configuration (`q = 0`). Each column of `omega` gives a joint's unit rotation axis, and `r` gives its physical joint origin. For a revolute joint, the screw axis is `S = [omega; v]`, with `v = r x omega = -omega x r`; this represents rotation about an axis that need not pass through the base origin. The script stores the screw axes in `S`, their `4 x 4` twist matrices in `xi_hat`, and the endpoint's home pose in `M`. This is the geometry needed for the [space-frame PoE formula (Modern Robotics, section 4.1.1)](https://modernrobotics.northwestern.edu/nu-gm-book-resource/4-1-1-product-of-exponentials-formula-in-the-space-frame/).

**[poe_fk.m](Matlab/poe_fk.m)** computes `T = expm(xi_hat(:,:,1)*q(1)) * ... * expm(xi_hat(:,:,6)*q(6)) * M`. Each matrix exponential describes one joint's rigid rotation about its home screw axis. The function validates `q`, converts it to a column, and accumulates these transformations in `F`.

Before adding joint `i`'s motion, `F` contains only joints `1,...,i-1`. Its rotation and translation carry joint `i`'s home origin and axis into their current positions: `p(:,i) = R*r(:,i) + t` and `w(:,i) = R*omega(:,i)`. Thus `p` tracks the physical joint origins specified in `poe_data`.

**[dh_data.m](Matlab/dh_data.m)** builds `dh_model` using standard DH frames, with joint `i` rotating about `z_(i-1)`. The four parameter arrays describe the angle offset `theta`, translation `d` along the preceding z-axis, translation `a` along the new x-axis, and twist `alpha` about that x-axis. In particular, `d = [h1+h2, w3, h4, l4+l5, 0, l6]` and `a = [l2, l3, 0, 0, 0, 0]` use the shared dimensions. The actual joint angle is `theta(i) + q(i)`, so the third joint's `pi/2` offset is present even at home. See also the [MathWorks DH parameter reference](https://www.mathworks.com/help/robotics/ref/rigidbodyjoint.setfixedtransform.html).

The fixed `tool_transform` expresses endpoint frame `link_6` in DH frame 6. Their origins coincide, but their axes differ: `x_E = z_6`, `y_E = x_6`, and `z_E = y_6`. Appending this transform on the right makes the DH endpoint frame match the one used by PoE.

**[dh_fk.m](Matlab/dh_fk.m)** constructs each standard DH transform explicitly as `A_i = Rz(theta(i)+q(i)) * Tz(d(i)) * Tx(a(i)) * Rx(alpha(i))`, then computes `T = A_1 * ... * A_6 * tool_transform`. Before multiplying by `A_i`, `F` is the pose of frame `i-1`, so its origin `F(1:3,4)` and z-axis `F(1:3,3)` give `p(:,i)` and `w(:,i)`. A DH frame origin is a point on the corresponding joint axis; it can lie elsewhere along that line than the physical origin returned by `poe_fk`, without changing the joint's motion.

**[check_dh_poe.m](Matlab/check_dh_poe.m)** rebuilds both models and compares them at 114 configurations. The columns of `Q` contain the home pose, 12 individual joint motions (each joint at `+pi/2` and `-pi/2`, with the others zero), one mixed pose, and 100 random joint vectors with angles in `[-pi, pi)`. A local random stream with seed 7 makes the samples repeatable without changing MATLAB's global random stream. Each loop iteration sends the same column of `Q` to both FK functions.

For each configuration, it measures three errors: `norm(T_dh-T_poe,'fro')` compares the complete endpoint poses, `norm(w_dh-w_poe,'fro')` compares joint-axis directions, and `norm(cross(p_dh-p_poe,w_poe,1),'fro')` compares axis locations. The Frobenius norm is the square root of the sum of squared matrix entries. The cross product is taken separately for each column: a displacement parallel to an axis has zero cross product with that axis. Since `w_poe` is a unit vector, each cross-product magnitude is the perpendicular distance to the PoE axis line. Together with equal directions, zero distance means the two representations describe the same axis, even when their chosen origins differ.

`max_error` retains the largest value of each error across all configurations, and `assert` requires all three to be below `1e-10`, allowing for floating-point roundoff. The function also repeats the final DH evaluation with a row vector and checks that all outputs match the column-vector result. On success it prints the sample count and maximum errors; otherwise it raises an error. The pose norm combines rotation entries and position entries, so it is a numerical consistency check rather than a physical distance. These sampled checks support agreement between the implementations; they are not a proof for every configuration or an independent validation of the robot geometry.
