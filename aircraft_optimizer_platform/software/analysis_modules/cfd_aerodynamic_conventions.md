# CFD Aerodynamic Conventions

This contract defines the platform-level aerodynamic coordinate and coefficient
conventions for CFD smoke runs and future scoring CFD.

## Coordinate System

- `X`: longitudinal / streamwise axis.
- `Y`: spanwise axis.
- `Z`: vertical axis.

Default freestream for zero-angle smoke runs:

- velocity direction: `+X`
- OpenFOAM velocity vector at 50 mph: `22.352,0,0` m/s

## Force Coefficient Axes

Default force directions:

- drag direction: `1,0,0`
- lift direction: `0,0,1`
- pitch axis: `0,1,0`

Older sandbox runs that used lift direction `0,1,0` should be interpreted as
side-force-like evidence. Do not compare those `Cl` values directly against
current vertical-lift `Cl`.

## Reference Values

Current smoke-run defaults are provisional:

- density: `1.225 kg/m^3`
- freestream speed: `22.352 m/s`
- reference length: `0.7111267 m`
- reference area: `0.120 m^2`
- moment center: `0.338,0,0`

These values are acceptable for solver plumbing and repeatability checks only.
Before enabling scoring CFD, the optimizer must compute or receive:

- reference area from aircraft planform or an explicit candidate field
- reference length from mean aerodynamic chord or an explicit candidate field
- moment center from current CG or a declared aerodynamic reference point
- angle-of-attack definition and sweep policy
- coefficient sign convention for lift, drag, side force, roll, pitch, and yaw

## Scoring Readiness

Passing a 60-step smoke-run force stability check is not enough for aerodynamic
scoring. Scoring CFD also needs:

- mesh-quality gates
- solver convergence criteria
- angle-of-attack policy
- near-wall / boundary-layer treatment
- reference-value provenance
- force/moment history stability criteria over a meaningful window

Until those are defined, OpenFOAM force coefficients are solver-plumbing
evidence, not optimizer scoring values.
