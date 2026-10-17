# DH report draft

`report.tex` is the standalone entry point; `report.pdf` is its compiled preview.
The two-page section consists of a convention and construction guide, a six-row
DH table, endpoint alignment, and six vector TikZ sketches.

Compile from this directory:

```sh
latexmk -pdf report.tex
# Alternatively: pdflatex report.tex, twice; or tectonic report.tex.
```

The prose is in `sections/dh_parameters.tex`. The six diagrams in `figures/` are
ordinary editable TikZ sources, requiring no external images or shell escape.
`figures/generate_dh_diagrams.py` regenerates them from shared home-frame
geometry and a fixed projection (Python 3 with NumPy). Manual changes to the
generated TikZ files are overwritten if that script is rerun.

To integrate into a larger report, copy the packages, colour definitions and
`DHPanel` macro from the preamble, then input `sections/dh_parameters.tex` from
this directory. Remove its `clearpage` if the final report should float the
illustration naturally. The prose does not require the standalone page header.

## Source interpretation

The handwritten `DH Parameters Project.pdf`, the project brief, and the existing
`Matlab/dh_data.m`, `Matlab/robot_dimensions.m` and `CuRo6R/CuRo6R.urdf` were used.
The brief supplies context; this draft covers only the requested DH section.

- Standard DH row i maps frame i-1 to frame i; joint i rotates about z_(i-1).
- With frame 0 at `base_link`, d1 is 1 m, combining the two 0.5 m heights.
  The earlier handwritten 0.5 m value measures from the physical first-joint
  origin and is not the full base-relative DH offset. The final handwritten
  table and MATLAB already use 1 m.
- The redundant identity row in the final handwritten table is omitted. There
  are exactly six variable DH rows, plus a fixed endpoint alignment.
- Frame 3 lies at the joint-3/joint-4 axis intersection, 1.5 m before the
  physical joint-4 origin. This gives a3=0 and d4=1.5+1=2.5 m.
- Frames 4 and 5 coincide at the wrist centre. Joint 6's physical origin is
  1.5 m farther along its axis; that distance belongs to d6, not d5.
- The endpoint is `link_6`, matching the MATLAB model. The URDF's auxiliary
  `tool0` branches from link 3 and is not this six-joint endpoint.
- The tiny decimal truncation of the URDF roll is interpreted as exactly pi/6,
  as in the MATLAB models. Table lengths are metres; the table uses radians
  and the drawings use degrees for legibility.

The diagrams show one shared home configuration, using a fixed projection with
the base x-axis horizontal and the base z-axis vertical. Cylinders are schematic joint
markers, not mesh reconstructions. DH origins, physical joint centres and
extended axis lines are deliberately distinct.

## Verification

The six transforms and fixed endpoint alignment were checked against direct
composition of the URDF at home and 100 reproducible random configurations.
The maximum position residual was 4.98e-11 m and the maximum rotation-matrix
Frobenius residual was 1.18e-11, consistent with the URDF roll's decimal
truncation. All physical joint centres were also checked to lie on the
corresponding DH joint-axis lines. The checks concern the DH section only.
