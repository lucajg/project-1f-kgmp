# Historical analytical inverse kinematics

[analytical_ik.tex](analytical_ik.tex) is the unchanged previous report section,
preserved on 2026-10-28. It derives the quartic in `tan(q1/2)` and corresponds to
the [archived MATLAB solver](../../Matlab/ik/history/q1_quartic/README.md).

The active report includes [the new q3 derivation](../sections/analytical_ik.tex).
To compile the previous section in the same report layout, run from `report/`:

```sh
latexmk -pdf historical_ik.tex
```

This produces `historical_ik.pdf` without replacing `report.pdf`.
