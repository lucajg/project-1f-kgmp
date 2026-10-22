# Shared transform helper

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Keep the standard DH transform in one place because model construction and forward kinematics both need it.

## Files

| File | Purpose |
| --- | --- |
| [dh_transform.m](dh_transform.m) | Construct `Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`. |

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

[dh_data](../models/dh_data.m) uses the rotation block to build named home-frame orientations; [dh_fk](../fk/dh_fk.m) uses the complete transform for every joint. This helper is on the MATLAB path because those callers live in different folders. Helpers used by just one topic belong in that topic's `private/` folder.

## Checks

Covered through `check_dh_poe` and `check_jacobians`; use `check_jacobians(true)` to include symbolic coverage.
