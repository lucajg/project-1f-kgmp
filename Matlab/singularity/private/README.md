# Private singularity helpers

[Topic documentation](../README.md)

These helpers are called by functions in the parent folder. Do not add this folder directly to the MATLAB path.

| File | Purpose |
| --- | --- |
| [match_ik_solutions.m](match_ik_solutions.m) | Assign unordered IK candidates to persistent IDs using wrapped joint-angle distances. |
| [singularity_map_data.m](singularity_map_data.m) | Cache the numerical determinant grid and wrist-center singularity curves. |
