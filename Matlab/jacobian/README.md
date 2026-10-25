# Geometric Jacobians

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Compute DH and PoE geometric Jacobians numerically or symbolically, and express symbolic results in DH parameters.

## Files

| File | Purpose |
| --- | --- |
| [dh_jacobian.m](dh_jacobian.m) | Full tool velocity evaluated at a DH origin. |
| [poe_jacobian.m](poe_jacobian.m) | Full tool velocity evaluated at a physical joint origin. |
| [dh_jacobian_symbolic.m](dh_jacobian_symbolic.m) | Entry script for symbolic DH results. |
| [poe_jacobian_symbolic.m](poe_jacobian_symbolic.m) | Entry script for symbolic PoE results. |
| [dh_parameter_form.m](dh_parameter_form.m) | Rewrite dimension symbols using nonzero DH parameters. |
| [check_jacobians.m](check_jacobians.m) | Compare formulations, finite differences, and optional symbolic results. |
| [point_jacobian.m](../shared/point_jacobian.m) | Shared cross-product calculation from precomputed joint origins and axes. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
dh_model = dh_data();
poe_model = poe_data();
q = [20; -35; 50; 15; -40; 60]*pi/180;
J06 = dh_jacobian(q, 0, 6, dh_model);
J05 = dh_jacobian(q, 0, 5, dh_model);
J35 = poe_jacobian(q, 3, 5, poe_model);
J_position = J35(1:3,1:3);
check_jacobians(false);
```

## Dependencies

[Robot models](../models/README.md), [forward kinematics](../fk/README.md), and [shared kinematics helpers](../shared/README.md). Numerical evaluation needs no additional toolbox. Symbolic models, entry scripts, `dh_parameter_form`, and the optional symbolic checks require Symbolic Math Toolbox.

## Inputs, outputs, and implementation

**[dh_jacobian.m](dh_jacobian.m)** and **[poe_jacobian.m](poe_jacobian.m)** both accept `(q, frame, joint, model)` and return a `6 x n` matrix with linear velocity above tool angular velocity. All joints contribute. `joint` selects the point at which the tool's velocity field is evaluated; `frame` selects the DH axes used to express both base-relative velocity vectors. This does not subtract the selected frame's own motion. DH selects a DH origin, while PoE selects a physical joint origin. Points 5 (wrist center) and 6 (endpoint) coincide across the models; other matching indices need not select the same point.

The Jacobians reuse the forward kinematics and a shared cross-product calculation. Both functions accept numerical or symbolic inputs. To derive an exact Jacobian, supply symbolic joint coordinates and models built from symbolic dimensions:

```matlab
gs = robot_dimensions(true);
qs = sym('q',[6,1],'real');
Jdh_sym = simplify(dh_jacobian(qs,3,5,dh_data(gs)));
Jpoe_sym = simplify(poe_jacobian(qs,3,5,poe_data(gs)));
simplify(expand(Jdh_sym - Jpoe_sym)) % A 6 x 6 zero matrix.
```

**[dh_jacobian_symbolic.m](dh_jacobian_symbolic.m)** and **[poe_jacobian_symbolic.m](poe_jacobian_symbolic.m)** are short entry scripts using these same functions; edit `k` and `f` to select the point and expression frame. They produce `Jdh_sym` and `Jpoe_sym` in the shared dimension symbols, plus `Jdh_ad_sym` and `Jpoe_ad_sym` in DH parameters. Their model and joint variables remain separate from the numerical examples.

**[dh_parameter_form.m](dh_parameter_form.m)** rewrites symbolic expressions using `a1 = l2`, `a2 = l3`, `d1 = h1+h2`, `d2 = w3`, `d3 = h4`, `d4 = l4+l5`, and `d6 = l6`. Zero DH entries and fixed angles retain their values. The substitution keeps `h1` and `l4` available because the DH table does not determine the individual splits of `d1` and `d4`; these residual dimensions cancel from the Jacobians at points 5 and 6. Other physical origins may still need them. A DH parameter can also disappear from the Jacobian: `d1` only translates this robot vertically and does not affect its velocity mapping.

## Checks

**[check_jacobians.m](check_jacobians.m)** compares the numerical DH and PoE Jacobians at 114 configurations, for points 5 and 6 and every expression frame from 0 to 6, using both default and modified dimensions. It also checks row-vector inputs, the wrist's zero linear columns and retained sixth angular column, the closed-form PoE motions against matrix exponentials, and finite differences of position and tool rotation. By default it additionally proves symbolic DH-PoE equality for `(frame, point) = (0,6), (0,5), (3,5)` and compares numerical evaluations of those symbolic results with the numerical models. The DH parameter forms are checked against direct derivation from symbolic `a`/`d` arrays and against numerical models. Call `check_jacobians(false)` to run only the numerical checks.

See the report's [Jacobian derivation](../../report/sections/jacobian.tex) and [detailed notes](../../report/notes/jacobian_detailed.tex).
