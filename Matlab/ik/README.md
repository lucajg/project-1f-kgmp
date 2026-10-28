# Analytical inverse kinematics

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Solve the positioning arm, wrist orientation, or complete CuRo6R tool pose, retaining the available solution branches.

The active solver follows the [report's LaTeX derivation](../../report/sections/analytical_ik.tex), using a quartic in `tan(q3/2)`. The former `tan(q1/2)` implementation, its checks, and its documentation are preserved in [history/q1_quartic](history/q1_quartic/README.md). `curo6r_setup` excludes the archive from the MATLAB path.

## Files

| File | Purpose |
| --- | --- |
| [analytical_ik_3r.m](analytical_ik_3r.m) | Find arm candidates from the desired tool pose. |
| [analytical_ik_wrist.m](analytical_ik_wrist.m) | Find wrist alternatives for a prescribed arm configuration. |
| [analytical_ik_6r.m](analytical_ik_6r.m) | Combine arm and wrist branches and verify the complete pose. |
| [check_analytical_ik.m](check_analytical_ik.m) | Check round trips, branch completeness, singularities, and invalid targets. |
| [Report derivation](../../report/sections/analytical_ik.tex) | Supplied derivation integrated into the current report. |
| [history/q1_quartic](history/q1_quartic/README.md) | Historical solver and instructions for running it separately. |

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

The solver forms the target invariants `h = z_w-d1` and `Rcal = x_w^2+y_w^2+h^2`. With the fixed URDF tilt `alpha = pi/6`, the derivation defines `F1 = a2+d4*cos(q3)`, `B = d2+d4*sin(q3)`, and `F3 = a1^2+F1^2+d3^2+B^2`. The required coordinates are `X = (Rcal-F3)/(2*a1)` and `D = (h-sin(alpha)*B)/cos(alpha)`. Imposing `X^2+D^2 = F1^2+d3^2` eliminates `q1` and `q2` and gives the quartic in `t = tan(q3/2)`.

Its coefficient arrays use descending powers, as expected by MATLAB's `roots`; `conv(P,P)` forms a squared polynomial. Each real root gives `q3 = 2*atan(t)`, and `q3 = pi` is checked separately because it is absent from the finite half-angle chart. For numerical stability near `pi`, the solver also reverses the coefficients to solve the same quartic in `u = 1/t = cot(q3/2)`, retaining bounded roots from each chart. This avoids very large tangent roots, including repeated roots at arm singularities. Back-substitution recovers `q2 = atan2(d3*X-F1*D,F1*X+d3*D)`. Then `A = a1+X`, `Y = cos(alpha)*B-sin(alpha)*D` give `q1 = atan2(A*y_w-Y*x_w,A*x_w+Y*y_w)`. On the base axis, `q1 = 0` represents the free base angle; the additional identity `B = sin(alpha)*h` avoids numerical splitting of the repeated quartic root there. Candidates are checked by applying the [first three PoE factors](../fk/README.md#inputs-outputs-and-implementation) to the wrist-center home point `poe_model.r(:,5)`, then deduplicated modulo `2*pi`.

**[analytical_ik_wrist.m](analytical_ik_wrist.m)** solves orientation for a prescribed three-joint arm configuration:
`[wrist,is_singular] = analytical_ik_wrist(q_arm,R_d,poe_model,singular_tol)`.
Here `R_d` is the desired `3 x 3` orientation of `link_6` in `base_link`; `q_arm` is a three-element row or column vector in radians. The optional `singular_tol` defaults to `1e-10`. Regular wrists return a `3 x 2` matrix `[q4;q5;q6]`, with positive `sin(q5)` first and negative second. At or below the singularity threshold, the function returns one `q4 = 0` representative and sets `is_singular = true`. A trajectory caller should use this flag to detect a wrist obstruction, then handle continuity, angle representations, joint limits, and full-Jacobian margins separately. The helper exposes all wrist alternatives without choosing a tracking branch.

**[analytical_ik_6r.m](analytical_ik_6r.m)** calls the positioning solver, then calls `analytical_ik_wrist` for every arm candidate. The physical arm orientation is `R_arm = R_F*poe_model.M(1:3,1:3)`, where `R_F` is the rotation of the first three PoE factors; the home orientation must be included. The desired wrist rotation is `R_arm.'*T_d(1:3,1:3)`, which decomposes as `Rx(q4)*Rz(q5)*Rx(q6)` for this robot's local wrist axes.

For a regular wrist, `eta = hypot(R(2,1),R(3,1))` gives `abs(sin(q5))`. For each sign `e = +1,-1`, the supplied formulas give `q5 = atan2(e*eta,R(1,1))`, `q4 = atan2(e*R(3,1),e*R(2,1))`, and `q6 = atan2(e*R(1,3),-e*R(1,2))`. The same sign is used in all three expressions. Both branches are retained, giving up to eight isolated full-pose solutions. When `eta < 1e-6` but exceeds `singular_tol`, `q6` uses the algebraically equivalent third row of `Rx(-q4)*R`, `[0,sin(q6),cos(q6)]`. This prevents roundoff in the tiny first-row entries from corrupting the coupled wrist angles. At `q5 = 0`, only `q4 + q6` is determined; at `q5 = pi`, only `q6 - q4` is determined. The solver chooses `q4 = 0` for these singular families and solves the remaining angle. Every returned six-angle column is checked against the complete PoE tool pose, with separate position and rotation tolerances.

Both solvers take a numeric rigid transform and consistent numeric models built from the same dimensions. They are specific to CuRo6R's axes and fixed tilt. The arm solver requires nonzero `a1`, `d3`, and `d4`; changing the shoulder tilt requires consistent changes to the models as well. Angles lie in `[-pi,pi]`, matching the supplied URDF limits; there is no separate custom-limit input. Unreachable targets return `zeros(3,0)` or `zeros(6,0)`. Continuous families have representative solutions rather than an exhaustive parameterization. Root finding and singularity decisions use numerical tolerances; the analytic reduction does not require a numerical IK initial guess. Raw arm column order can differ from the historical solver; the planner preserves wrist sign order, and the singularity viewer matches branch identities across updates.

Different joint vectors can reproduce the same pose, so check FK agreement rather than demanding that every solution equal `q_test`.

## Checks

**[check_analytical_ik.m](check_analytical_ik.m)** performs repeatable round trips against independent DH and URDF forward kinematics. It checks default and modified dimensions, recovery of the original regular arm and both wrist branches, `q1 = pi`, `q3 = +/-pi` and nearby roots, repeated arm roots, exact and near wrist singularities, representative solutions on the base axis, duplicate removal, the four-arm/eight-pose branch bound, unreachable targets, and rejection of non-rigid target matrices and unsupported degenerate dimensions. It also checks standalone wrist reconstruction, branch ordering, singularity flags and threshold overrides, and row-vector arm inputs. Run `check_analytical_ik` after running `curo6r_setup`.

Run `run_checks(false,true)` to include the dependent planner and singularity-viewer checks. The report's [historical IK derivation](../../report/history/analytical_ik.tex) describes the archived `q1` method; the [current report section](../../report/sections/analytical_ik.tex) describes the active implementation.
