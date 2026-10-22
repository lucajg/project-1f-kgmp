# Analytical inverse kinematics

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Solve the positioning arm, wrist orientation, or complete CuRo6R tool pose, retaining the available solution branches.

## Files

| File | Purpose |
| --- | --- |
| [analytical_ik_3r.m](analytical_ik_3r.m) | Find arm candidates from the desired tool pose. |
| [analytical_ik_wrist.m](analytical_ik_wrist.m) | Find wrist alternatives for a prescribed arm configuration. |
| [analytical_ik_6r.m](analytical_ik_6r.m) | Combine arm and wrist branches and verify the complete pose. |
| [check_analytical_ik.m](check_analytical_ik.m) | Check round trips, branch completeness, singularities, and invalid targets. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
poe_model = poe_data();
dh_model = dh_data();
q_test = [20; -35; 50; 15; -40; 60]*pi/180;
T_d = poe_fk(q_test, poe_model);
Q_arm = analytical_ik_3r(T_d, poe_model, dh_model);
Q = analytical_ik_6r(T_d, poe_model, dh_model);
[wrist, is_singular] = analytical_ik_wrist(q_test(1:3), T_d(1:3,1:3), poe_model);
if ~isempty(Q)
    T_check = poe_fk(Q(:,1), poe_model);
    position_error = norm(T_check(1:3,4)-T_d(1:3,4));
    rotation_error = norm(T_check(1:3,1:3)-T_d(1:3,1:3), 'fro');
end
```

## Dependencies

[Robot models](../models/README.md) and [forward kinematics](../fk/README.md). All solvers use numerical inputs and require no additional toolbox. Models must use consistent dimensions.

## Inputs, outputs, and implementation

**[analytical_ik_3r.m](analytical_ik_3r.m)** solves the positioning problem with `Q_arm = analytical_ik_3r(T_d,poe_model,dh_model)`. Each column is a candidate `[q1; q2; q3]`. The input is the desired **link_6 pose**, not a wrist-center pose: the solver first computes `p_w = T_d(1:3,4) - d6*T_d(1:3,1)`. The tool offset is along the physical tool's x-axis. This removes `d6` from the arm geometry while retaining `d4`, which reaches the wrist center.

The solver expresses the wrist center relative to the shoulder, undoes the fixed 30-degree tilt, and eliminates `q2` using the unchanged y-component and preserved vector length. Combining the resulting expressions for `sin(q3)` and `cos(q3)` gives a quartic in `t = tan(q1/2)`. Its coefficient arrays use descending powers, as expected by MATLAB's `roots`; `conv(P,P)` forms the coefficients of a squared polynomial. Each real root gives a candidate `q1`, and `q1 = pi` is checked separately because it is absent from the finite half-angle chart. Back-substitution recovers `q3` and then `q2` with `atan2`. Candidates are checked by applying the [first three PoE factors](../fk/README.md#inputs-outputs-and-implementation) to the wrist-center home point `poe_model.r(:,5)`, then deduplicated modulo `2*pi`. This uses the original model directly and leaves its endpoint home pose `M` unchanged.

**[analytical_ik_wrist.m](analytical_ik_wrist.m)** solves orientation for a prescribed three-joint arm configuration:
`[wrist,is_singular] = analytical_ik_wrist(q_arm,R_d,poe_model,singular_tol)`.
Here `R_d` is the desired `3 x 3` orientation of `link_6` in `base_link`; `q_arm` is a three-element row or column vector in radians. The optional `singular_tol` defaults to `1e-10`. Regular wrists return a `3 x 2` matrix `[q4;q5;q6]`, with positive `sin(q5)` first and negative second. At or below the singularity threshold, the function returns one `q4 = 0` representative and sets `is_singular = true`. A trajectory caller should use this flag to detect a wrist obstruction, then handle continuity, angle representations, joint limits, and full-Jacobian margins separately. The helper exposes all wrist alternatives without choosing a tracking branch.

**[analytical_ik_6r.m](analytical_ik_6r.m)** calls the positioning solver, then calls `analytical_ik_wrist` for every arm candidate. The physical arm orientation is `R_arm = R_F*poe_model.M(1:3,1:3)`, where `R_F` is the rotation of the first three PoE factors; the home orientation must be included. The desired wrist rotation is `R_arm.'*T_d(1:3,1:3)`, which decomposes as `Rx(q4)*Rz(q5)*Rx(q6)` for this robot's local wrist axes.

For a regular wrist, `rho = hypot(R(2,1),R(3,1))` gives `abs(sin(q5))`. Both signs are retained, giving two wrist branches per arm candidate and up to eight isolated solutions in the generic case. The first column gives `q4`, including the chosen sign of `sin(q5)`. To recover `q6`, the solver removes `Rx(q4)` on the left: the remaining `Rz(q5)*Rx(q6)` has third row `[0, sin(q6), cos(q6)]`. This avoids dividing by a small `sin(q5)` a second time and preserves branches close to wrist singularities. At `q5 = 0`, only `q4 + q6` is determined; at `q5 = pi`, only `q4 - q6` is determined. The solver chooses `q4 = 0` for these singular families and solves the remaining angle. Every returned six-angle column is checked against the complete PoE tool pose, with separate position and rotation tolerances.

Both solvers take a numeric rigid transform and consistent numeric models built from the same dimensions. They are specific to CuRo6R's axes and fixed tilt, and require nonzero `a2` and `d4`. Angles lie in `[-pi,pi]`, matching the supplied URDF limits; there is no separate custom-limit input. Unreachable targets return `zeros(3,0)` or `zeros(6,0)`. Continuous families have representative solutions rather than an exhaustive parameterization: for example, the positioning solver chooses `q1 = 0` when its constraint is identically zero. Root finding and singularity decisions use numerical tolerances; the analytic reduction does not require a numerical IK initial guess.

Different joint vectors can reproduce the same pose, so check FK agreement rather than demanding that every solution equal `q_test`.

## Checks

**[check_analytical_ik.m](check_analytical_ik.m)** performs repeatable round trips against independent DH and URDF forward kinematics. It checks default and modified dimensions, recovery of the original regular arm and both wrist branches, `q1 = pi`, exact and near wrist singularities, representative solutions on the base axis, duplicate removal, unreachable targets, and rejection of non-rigid target matrices. It also checks standalone wrist reconstruction, branch ordering, singularity flags and threshold overrides, and row-vector arm inputs. Run `check_analytical_ik` after running `curo6r_setup`.

See the report's [analytical IK derivation](../../report/sections/analytical_ik.tex).
