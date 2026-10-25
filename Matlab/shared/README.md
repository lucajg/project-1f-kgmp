# Shared kinematics helpers

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Keep transform and Jacobian calculations in one place for callers across topic folders.

## Files

| File | Purpose |
| --- | --- |
| [dh_transform.m](dh_transform.m) | Construct `Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`. |
| [point_jacobian.m](point_jacobian.m) | Build a geometric Jacobian from joint origins and axes already computed by FK. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
A = dh_transform(pi/2, 1, 0.5, -pi/3); % 4 x 4 homogeneous transform.
```

## Dependencies

No other project folder. Numerical inputs need no additional toolbox; symbolic inputs require Symbolic Math Toolbox.

## Inputs, outputs, and implementation

`A = dh_transform(theta, d, a, alpha)` accepts scalar DH parameters, with distances in metres and angles in radians, and returns a `4 x 4` homogeneous transform. It supports numerical and symbolic values.

[dh_data](../models/dh_data.m) uses the rotation block to build named home-frame orientations; [dh_fk](../fk/dh_fk.m) uses the complete transform for every joint.

`J = point_jacobian(p, w, pk, R0frame)` accepts `3 x n` revolute joint origins `p` and unit axes `w`, a `3 x 1` reference point `pk`, and a `3 x 3` rotation from the expression frame to the base frame. Positions and axes are expressed in the base frame. It returns a `6 x n` geometric Jacobian with linear velocity above angular velocity; use `eye(3)` for base-frame expression. Inputs may be numerical or symbolic.

[dh_jacobian](../jacobian/dh_jacobian.m) and [poe_jacobian](../jacobian/poe_jacobian.m) use this calculation after FK. The [path tracer](../planning/private/trace_arm_segment.m) reuses one FK result for the arm-position block, tool Jacobian, and pose diagnostics at each sample.

These helpers are on the MATLAB path because their callers live in different folders. Helpers used by just one topic belong in that topic's `private/` folder.

## Checks

Covered through `check_dh_poe` and `check_jacobians`; use `check_jacobians(true)` to include symbolic coverage.
