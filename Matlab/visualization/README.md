# Joint-axis visualization

[Project overview](../../README.md) · [MATLAB setup and checks](../README.md)

Inspect the PoE and DH robot geometry using one model or a side-by-side comparison with shared joint controls.

## Files

| File | Purpose |
| --- | --- |
| [curo6r_viewer.m](curo6r_viewer.m) | Shared viewer for one or two model structs. |
| [curo6r_compare.m](curo6r_compare.m) | Convenience entry point for the two-model comparison. |
| [private/curo6r_scene.m](private/curo6r_scene.m) | Create and update the graphics for one model panel. |

## Example

Run this example from the repository root. Setup is needed once per MATLAB session.

```matlab
addpath('Matlab');
curo6r_setup;
fig = curo6r_compare();
% Or open a single model:
% fig = curo6r_viewer(poe_data());
% fig = curo6r_viewer(dh_data());
```

## Dependencies

[Robot models](../models/README.md), [forward kinematics](../fk/README.md), and MATLAB graphics. No additional toolbox is needed.

## Inputs, outputs, and implementation

**[curo6r_viewer.m](curo6r_viewer.m)** provides a shared viewer for either model: call `curo6r_viewer(poe_model)` or `curo6r_viewer(dh_model)`. Passing both structs opens two plots with one joint-angle vector. It uses classic MATLAB `figure`, `axes`, and `uicontrol` objects, with no web-based UI, resize callbacks, or continuous camera links. A small adapter selects the FK function; both models use the same `[T,p,w]` drawing interface. Only joint rotation axes and an endpoint marker are drawn, without coordinate-frame triads.

Sliders update on release; you can also type angles between -180 and 180 degrees. Home zeros the angles, and the joint selector isolates individual axes. Plot limits stay fixed and cover the robot's reach. Use the standard figure toolbar to rotate, pan, or zoom a plot, then click Match views to copy that plot's camera and limits to the other one. Reset view restores both plots. The comparison readout reports the three errors used by `check_dh_poe`. Solid PoE connectors join physical joint origins; dashed DH connectors join frame origins and need not follow physical links.

If the previous viewer is still stuck in the current MATLAB session, interrupt it with Ctrl+C, then run `delete(findall(groot,'Type','figure','Name','CuRo6R | Joint-axis explorer'))` to remove its windows. Run `clear curo6r_viewer curo6r_compare`, then reopen with `curo6r_compare`. This closes only windows with the old viewer's name.

**[curo6r_compare.m](curo6r_compare.m)** is a thin entry point for the comparison: `curo6r_compare(poe_model,dh_model)` uses existing models, while `curo6r_compare` builds the two default numerical models automatically. Both routes call the shared viewer, so the single-model and comparison controls behave identically.

**[private/curo6r_scene.m](private/curo6r_scene.m)** creates the graphics for one panel and returns an update function that accepts `[T,p,w]` plus display options. It reuses the graphics objects as angles change. Coloured dots mark each model's chosen origins, arrows show positive joint-axis directions, and dotted guides extend the axes. Each guide is centred at `p - w*dot(w,p)`, the point on the axis closest to the base origin for unit `w`. This point is unchanged when `p` slides along the axis, so equivalent PoE and DH axes receive identical guide segments despite different origin choices. The black diamond `E` marks `link_6`.

## Checks

Run `check_dh_poe` for the numerical comparisons shown by the viewer. To check the UI manually, open `curo6r_compare`, move a slider, type an angle, select an axis, and use Home, Match views, and Reset view. Both panels should follow the same joint vector. There is no dedicated automated viewer check in `run_checks`.
