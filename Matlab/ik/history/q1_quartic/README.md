# Historical IK: quartic in the base angle

Archived on 2026-10-28 before replacing the solver with the supplied derivation
in `tan(q3/2)`. The four MATLAB files and [original documentation](README.original.md)
are unchanged snapshots of the previous implementation and checks. Links in
the original documentation are relative to its former location, `Matlab/ik/`.

This version eliminates the shoulder and elbow angles to obtain a quartic in
`tan(q1/2)`. The report's [earlier derivation](../../../../report/history/analytical_ik.tex)
describes that method. The active solver and its documentation are in
[Matlab/ik](../../README.md).

`curo6r_setup` deliberately excludes this directory. To run the historical
checks, start from the repository root in a separate MATLAB session:

```matlab
addpath('Matlab');
curo6r_setup;
archive = fullfile(pwd,'Matlab','ik','history','q1_quartic');
addpath(archive,'-begin');
clear analytical_ik_3r analytical_ik_wrist analytical_ik_6r check_analytical_ik;
check_analytical_ik;
rmpath(archive);
clear analytical_ik_3r analytical_ik_wrist analytical_ik_6r check_analytical_ik;
```

Use `curo6r_setup` for normal work rather than recursively adding `Matlab/`:
the archived functions have the same public names as the active solvers.
