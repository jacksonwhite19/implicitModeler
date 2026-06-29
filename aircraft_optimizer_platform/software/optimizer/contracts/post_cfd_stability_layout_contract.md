# Post-CFD Stability Layout Adjustment Contract

Static margin is not calculated during pre-export screening. It is evaluated only after a CFD/stability analysis provides an aerodynamic neutral point.

## Inputs

- Candidate design variables.
- Virtual component assumptions used for layout.
- CFD/stability-derived `neutral_point_x_mm`.
- Desired static margin.
- Static margin acceptance bounds.

## Adjustment Rules

- EDF remains fixed.
- Fixed payload remains fixed unless a later config explicitly marks it movable.
- Battery and avionics may shift only within their configured x-ranges.
- All current internal components remain on the aircraft X axis for the simplified v0.1 model.
- Component size, mass, and keepout constraints remain active.
- The layout pass tries to move CG toward:

```text
target_cg_x = neutral_point_x - desired_static_margin * reference_chord
```

## Outputs

- Final component placement.
- Final CG.
- Final static margin.
- Static margin error from target.
- Remaining layout wiggle room.
- Whether the target margin was achieved within policy.
- Failed checks, if any.

## Failure Cases

- Internal layout no longer fits after constraints are applied.
- Movable components cannot shift far enough to meet static margin target.
- Static margin remains outside min/max policy.
- CG error exceeds policy.

