# Forward kinematics

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Evaluate CuRo6R poses and joint axes using DH, PoE, or direct URDF transforms. Keep the three formulations together for comparison.

## Files

| File | Purpose |
| --- | --- |
| [dh_fk.m](dh_fk.m) | Standard DH forward kinematics and cumulative DH frames. |
| [poe_fk.m](poe_fk.m) | Space-frame Product of Exponentials and partial products. |
| [urdf_fk.m](urdf_fk.m) | Direct composition of fixed URDF placements and local joint rotations. |
| [check_dh_poe.m](check_dh_poe.m) | Repeatable comparison of poses, joint axes, and physical origins. |
| [private/validate_joint_vector.m](private/validate_joint_vector.m) | Validate the joint-vector size and numerical values. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
dh_model = dh_data();
poe_model = poe_data();
urdf_data;
q = [20; -35; 50; 15; -40; 60]*pi/180;
[T_dh, p_dh, w_dh] = dh_fk(q, dh_model);
[T_poe, p_poe, w_poe] = poe_fk(q, poe_model);
[T_urdf, p_urdf, w_urdf] = urdf_fk(q, urdf_model);
check_dh_poe;
```

## Dependencies

[Robot models](../models/README.md) and [shared transforms](../shared/README.md). DH and PoE accept numerical or symbolic inputs; symbolic evaluation requires Symbolic Math Toolbox. URDF FK is numerical.

## Inputs, outputs, and implementation

All three FK functions accept a six-element row or column vector `q`. They return a homogeneous pose `T = [R, t; 0, 0, 0, 1]`, where `R` is the endpoint orientation and `t` its position in the base frame. The `3 x 6` arrays `p` and `w` contain points on the joint axes and their unit directions, also in the base frame. PoE and URDF return physical joint origins; DH returns origins of its chosen frames, which may lie elsewhere along the same axes.

**[dh_fk.m](dh_fk.m)** uses the shared [dh_transform.m](../shared/dh_transform.m) to construct each standard DH transform as `A_i = Rz(theta(i)+q(i)) * Tz(d(i)) * Tx(a(i)) * Rx(alpha(i))`, then computes `T = A_1 * ... * A_6 * tool_transform`. Before multiplying by `A_i`, `T0(:,:,i)` is the pose of frame `i-1`; its origin and z-axis give `p(:,i)` and `w(:,i)`. The optional fourth output `T0(:,:,i+1)` holds the pose of DH frame `i`, including frame 0 in the first slice. Numeric and symbolic inputs use the same computation. A DH frame origin is a point on the corresponding joint axis; it can lie elsewhere along that line than the physical origin returned by `poe_fk`, without changing the joint's motion.

**[poe_fk.m](poe_fk.m)** computes the product `T = expm(xi_hat(:,:,1)*q(1)) * ... * expm(xi_hat(:,:,6)*q(6)) * M`. For the robot's unit revolute axes, each exponential is evaluated directly with `R = I + sin(q)*W + (1-cos(q))*W^2` and translation `(I-R)*r`. This closed form supports both numbers and exact symbols. The optional fourth output `F0(:,:,i+1)` contains the product through joint `i`, with identity in the first slice. Symbolic cumulative transforms are simplified at each step.

The optional third input, `poe_fk(q,poe_model,joint)`, limits the product to the first `joint` factors and requires that many joint angles. Omitting it uses all model joints. The endpoint home pose `M` is still appended: shortening the product holds later joints at zero, while retaining their geometry. To track the wrist center with three active joints, copy the model and set `M(1:3,4) = r(:,5)` in that copy.

Before adding joint `i`'s motion, `F` contains only joints `1,...,i-1`. Its rotation and translation carry joint `i`'s home origin and axis into their current positions: `p(:,i) = R*r(:,i) + t` and `w(:,i) = R*omega(:,i)`. Thus `p` tracks the physical joint origins specified in `poe_data`.

**[urdf_fk.m](urdf_fk.m)** alternates fixed placement and joint motion. For joint `i`, it forms `origin = [Rz(yaw)*Ry(pitch)*Rx(roll), xyz; 0, 0, 0, 1]`, then updates `F = F * origin`. At this point, `F` places the joint in the base frame: its translation gives `p(:,i)` and its rotation maps the local axis to `w(:,i)`. It then appends the local rotation by `q(i)`, using Rodrigues' formula `R = I + sin(q)*K + (1-cos(q))*K^2`, where `K*v = cross(axis,v)`. After joint 6, `F` is already the endpoint pose. Thus URDF composes local link geometry, DH composes specially chosen joint frames, and PoE moves home screw axes expressed in the base frame.

## Checks

**[check_dh_poe.m](check_dh_poe.m)** rebuilds all three models and compares DH-PoE, DH-URDF, and PoE-URDF at 514 configurations. The columns of `Q` contain the home pose, 12 individual joint motions (each joint at `+pi/2` and `-pi/2`, with the others zero), one mixed pose, and 500 random joint vectors with angles between `-pi` and `pi`. A local random stream with seed 42 makes the samples repeatable without changing MATLAB's global random stream. Each loop iteration sends the same column of `Q` to all three FK functions; the third dimension of `T`, `p`, and `w` selects the model.

For each configuration and model pair, it measures three errors. Taking DH-PoE as an example: `norm(T_dh-T_poe,'fro')` compares the complete endpoint poses, `norm(w_dh-w_poe,'fro')` compares joint-axis directions, and `norm(cross(p_dh-p_poe,w_poe,1),'fro')` compares axis locations. The Frobenius norm is the square root of the sum of squared matrix entries. The cross product is taken separately for each column: a displacement parallel to an axis has zero cross product with that axis. Since `w_poe` is a unit vector, each cross-product magnitude is the perpendicular distance to the PoE axis line. Together with equal directions, zero distance means the two representations describe the same axis, even when their chosen origins differ. A separate check compares PoE and URDF joint origins directly, since these physical origins should coincide.

Each row of `max_error` retains the largest pose, direction, and axis-line errors for one pair across all configurations. All comparisons, including the physical-origin check, must be below `1e-10`; using exact `pi/6` consistently leaves only floating-point roundoff. The function also repeats each model's final evaluation with a row vector and checks that all outputs match the column-vector result. On success it prints the sample count and a table of maximum errors; a failed comparison names the pair and measure. The pose norm combines rotation entries and position entries, so it is a numerical consistency check rather than a physical distance. These sampled checks support agreement between the implementations; they are not a proof for every configuration or an independent validation of the physical robot geometry.

See the report's [FK verification](../../report/sections/fk_verification.tex).
