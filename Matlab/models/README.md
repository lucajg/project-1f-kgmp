# Robot models

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Build the shared CuRo6R dimensions and the DH, PoE, and independently transcribed URDF models. Frame and unit conventions are defined in the [project overview](../../README.md#conventions).

## Files

| File | Purpose |
| --- | --- |
| [robot_dimensions.m](robot_dimensions.m) | Numerical lengths or real symbolic dimension variables. |
| [dh_data.m](dh_data.m) | Standard DH parameters, endpoint alignment, and home frame orientations. |
| [poe_data.m](poe_data.m) | Home joint origins, screw axes, twists, and endpoint pose. |
| [urdf_data.m](urdf_data.m) | Script that creates the independently transcribed `urdf_model`. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
g = robot_dimensions();
dh_model = dh_data(g);
poe_model = poe_data(g);
urdf_data; % Creates urdf_model in the caller's workspace.
% gs = robot_dimensions(true); % Requires Symbolic Math Toolbox.
```

## Dependencies

[Shared transforms](../shared/README.md). `poe_data` also calls `dh_data` for named home-frame orientations. Numerical construction needs no additional toolbox; symbolic dimensions require Symbolic Math Toolbox.

## Inputs, outputs, and implementation

**[robot_dimensions.m](robot_dimensions.m)** defines the nine lengths shared by DH and PoE: `h1`, `h2`, `l2`, `l3`, `w3`, `l4`, `h4`, `l5`, and `l6`. `robot_dimensions()` returns their numerical values in a struct; `robot_dimensions(true)` returns real symbolic variables with the same field names. Both `dh_data(g)` and `poe_data(g)` accept this struct; omitting `g` selects the default numerical dimensions. Rebuild the models after changing dimensions. Fixed angular geometry remains in the model builders, which use exact symbolic angles for symbolic dimensions. URDF data is transcribed independently, so geometry changes must also be reflected in the URDF and `urdf_data.m` for the triple check to agree.

**[dh_data.m](dh_data.m)** builds `dh_model` using standard DH frames, with joint `i` rotating about `z_(i-1)`. The four parameter arrays describe the angle offset `theta`, translation `d` along the preceding z-axis, translation `a` along the new x-axis, and twist `alpha` about that x-axis. In particular, `d = [h1+h2, w3, h4, l4+l5, 0, l6]` and `a = [l2, l3, 0, 0, 0, 0]` use the shared dimensions. The actual joint angle is `theta(i) + q(i)`, so the third joint's `pi/2` offset is present even at home.

The fixed `tool_transform` expresses endpoint frame `link_6` in DH frame 6. Their origins coincide, but their axes differ: `x_E = z_6`, `y_E = x_6`, and `z_E = y_6`. Appending this transform on the right makes the DH endpoint frame match the one used by PoE.

**[poe_data.m](poe_data.m)** builds `poe_model` at the home configuration (`q = 0`). Each column of `omega` gives a joint's unit rotation axis, and `r` gives its physical joint origin. For a revolute joint, the screw axis is `S = [omega; v]`, with `v = r x omega = -omega x r`; this represents rotation about an axis that need not pass through the base origin. The function stores the screw axes in `S`, their `4 x 4` twist matrices in `xi_hat`, and the endpoint's home pose in `M`.

The model also stores `R_home(:,:,f+1)`, the orientation of named DH frame `f` at home, with frame 0 in the first slice. These orientations are generated from the fixed angle offsets and twists in `dh_data.m`. This defines the coordinate axes selected by the Jacobian's `frame` input; the screw axes and endpoint home pose are still constructed from the PoE geometry above. The Jacobian obtains each current frame orientation by multiplying its home orientation by the rotation of the corresponding PoE partial product.

**[urdf_data.m](urdf_data.m)** transcribes the six revolute joints from [CuRo6R.urdf](../../CuRo6R/CuRo6R.urdf) into `urdf_model`, with one joint per column of `xyz`, `rpy`, and `axis`. No file parsing is performed. Positions and roll-pitch-yaw angles place each joint frame relative to its parent link; axes are expressed in those local joint frames. The URDF's `0.52359877559` roll is interpreted as the intended exact `pi/6`, matching DH and PoE. The endpoint is `link_6`; `tool0` branches from `link_3_d3`, and visual/collision origins only place meshes.

## Checks

Run `check_dh_poe` to compare all three default models. The [Jacobian checks](../jacobian/README.md#checks) and [IK checks](../ik/README.md#checks) also exercise modified dimensions. See the report's [DH derivation](../../report/sections/dh_parameters.tex) and [PoE derivation](../../report/sections/poe_model.tex).
