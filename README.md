# project-1f-kgmp

MATLAB forward kinematics, analytical inverse kinematics, geometric Jacobians, and singularity exploration for CuRo6R, a robot with six revolute joints. Forward kinematics uses Product of Exponentials (PoE), standard Denavit-Hartenberg (DH), and directly composed URDF joint transforms.

Numerical calculations, including IK, require no additional toolbox. Symbolic calculations require Symbolic Math Toolbox; interactive viewers and their UI checks require MATLAB graphics.

## Quick start

From the repository root in MATLAB:

```matlab
addpath('Matlab');
curo6r_setup;

g = robot_dimensions();
poe_model = poe_data(g);
dh_model = dh_data(g);
urdf_data; % Creates urdf_model.
q = [20; -35; 50; 15; -40; 60]*pi/180;
[T_poe, p_poe, w_poe] = poe_fk(q, poe_model);
[T_dh, p_dh, w_dh] = dh_fk(q, dh_model);
[T_urdf, p_urdf, w_urdf] = urdf_fk(q, urdf_model);
Q = analytical_ik_6r(T_poe, poe_model, dh_model);
J = poe_jacobian(q, 0, 6, poe_model);

run_checks; % Numerical checks and invisible singularity-app checks.
% run_checks(true);        % Also include symbolic checks.
% run_checks(false,false); % Numerical checks without graphics.

curo6r_compare(poe_model, dh_model); % Shared joint controls for both models.
% singularity_map;                  % Interactive pose and IK explorer.
```

Setup adds the explicit topic folders using paths relative to `curo6r_setup.m`, so subsequent calls work from any current folder. Run setup once per MATLAB session. See [MATLAB setup and checks](Matlab/README.md) for running from elsewhere, individual checks, and migrating from the old flat layout.

## Topic documentation

Each folder's README describes its files, runnable examples, assumptions, dependencies, and checks.

| Topic | Contents |
| --- | --- |
| [Robot models](Matlab/models/README.md) | Shared dimensions and DH, PoE, and URDF model construction. |
| [Forward kinematics](Matlab/fk/README.md) | Poses, joint axes, cumulative transforms, and three-model verification. |
| [Inverse kinematics](Matlab/ik/README.md) | Arm, wrist, and full-pose solvers with their solution branches. |
| [Jacobians](Matlab/jacobian/README.md) | Numerical and symbolic Jacobians, expression frames, and DH parameter forms. |
| [Singularity analysis](Matlab/singularity/README.md) | Determinant maps, workspace slices, branch tracking, and the pose explorer. |
| [Visualization](Matlab/visualization/README.md) | Joint-axis viewers and the shared DH/PoE comparison. |
| [Shared helper](Matlab/shared/README.md) | The DH transform used by model construction and FK. |
| [Report](report/README.md) | LaTeX derivations, figures, detailed notes, and build instructions. |

## Conventions

All three models use `base_link` as the base, `link_6` as the endpoint, and the same joint order, positive directions, and zero configuration. Distances are in metres and joint angles are in radians; UI controls explicitly use degrees. The robot description and meshes are in [CuRo6R](CuRo6R/CuRo6R.urdf).

A pose is `T = [R, t; 0, 0, 0, 1]`, expressing the endpoint in the base frame. FK accepts six-element row or column joint vectors. PoE and URDF report physical joint origins; DH reports chosen frame origins, which may lie elsewhere along the same axes. The [FK documentation](Matlab/fk/README.md) explains these outputs and partial products.

Geometric Jacobians place linear velocity above angular velocity. Their `joint` input selects the reference point, while `frame` selects the DH axes used to express base-relative velocities. IK targets the physical `link_6` pose and returns candidate joint vectors as columns. See each topic's README for its detailed contract and singular cases.
