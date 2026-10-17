# DH report draft

`report.tex` is the standalone entry point; `report.pdf` is its compiled preview.
The one-page section discusses the DH parameters obtained for CuRo6R: a six-row
table with geometric justifications, two selected TikZ sketches, and the endpoint
alignment. It assumes familiarity with DH rather than explaining the general method.
All six frame drawings are collected on one appendix page in
`sections/dh_appendix.tex`, with a reference from the main DH section. The current
draft has four pages in total, including the appendix.

The previous detailed section is preserved in `notes/dh_parameters_detailed.tex`,
with the six-panel illustration now shared through the appendix. To compile the
detailed version separately from this directory,
use `latexmk -pdf working_notes.tex` (or `tectonic working_notes.tex`). It shares the
main preamble, diagram sources and appendix; its extended DH explanation is not
included in the main report.

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
frame/joint notation macros from the preamble, then input
`sections/dh_parameters.tex` from this directory. The section has no forced page
break; its table and figure can float naturally. The prose does not require the
standalone page header.
Include `sections/dh_appendix.tex` after `\appendix` to retain the full drawing
sequence and resolve the main section's appendix reference. The appendix also
uses the `\DHPanel` macro defined in the preamble.

## Analytical IK illustration preview

`ik_illustrations.tex` compiles to a one-page preview, separate from the main
report: `latexmk -pdf ik_illustrations.tex` (or `tectonic ik_illustrations.tex`).
It shows one configuration, `(q1,q2,q3) = (40,25,75)` degrees, in the base frame
and auxiliary shoulder frame, followed by the planar construction for `q2`.

The reusable TikZ panels are `figures/ik_base_frame.tex`,
`figures/ik_shoulder_frame.tex`, and `figures/ik_shoulder_projection.tex`.
They use the report's existing colour definitions and the `arrows.meta` TikZ
library. The first two views share the same projection and length scale.
The base-frame panel retains a faint arm outline, while the shoulder-frame
panel isolates the preserved axial coordinate and vector length. The planar
panel uses two vectors and their angle; the accompanying text derives the
scaled sine and cosine before applying `atan2`.

Run `python3 figures/generate_ik_diagrams.py` from this directory to regenerate
the three panels (Python 3 with NumPy). This overwrites manual edits to those
generated files. The generator checks joint centres and axes against the URDF,
the preserved shoulder-frame component and norm, and the sign of the `q2`
rotation in the planar diagram. The preview itself compiles without Python or
shell escape.

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
