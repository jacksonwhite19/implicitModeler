# Three Targeted Fixes - Completion Summary

## Fix 1: Tier-1 Mount Sources Now Generate Solid Contact Shelf Geometry

Changed file:
- `src/scripting/api.rs`

What changed:
- Tier-1 no longer feeds the normal discrete beam path when tier-2 sources are present.
- A solid shelf/contact volume is now created from `mount_sources_tier1` and intersected with an outward-offset parent skin.
- That shelf is unioned into the mount before the lattice corridor is built.

Code change:

```rust
            let mut candidates_tier1: Vec<(f32, Vec3, Vec3)> = Vec::new();
            if mount_sources_tier2.is_none() {
                let candidates = tiered_candidate_targets(
                    &parent.0,
                    &expanded_attach_region,
                    &forbidden_union,
                    &source_points_tier1,
                    &guide_points,
                    max_reach,
                );
                push_distinct_connections_from_candidates(
                    &mut chosen,
                    &candidates,
                    &source_bounds_tier1,
                    beam_half_width,
                    desired_tier1_connections,
                    true,
                );
                candidates_tier1 = candidates;
            }
```

```rust
            let (fallback_source, fallback_bounds) = if let (Some(s), Some(bounds)) = (mount_sources_tier2.as_ref(), source_bounds_tier2.as_ref()) {
                (&s.0, bounds)
            } else {
                (&mount_sources_tier1.0, &source_bounds_tier1)
            };
            let fallback_seeds = seed_points_from_sources(fallback_source, fallback_bounds, &fallback_dirs);
```

```rust
            let mount_raw = union_all(mounts);
            let tier1_contact_relief: Arc<dyn crate::sdf::Sdf> = Arc::new(Offset::new(
                Arc::clone(&mount_sources_tier1.0),
                beam_half_width * 2.0,
            ));
            let allowed_contact_zone: Arc<dyn crate::sdf::Sdf> = if let Some(s) = mount_sources_tier2.as_ref() {
                Arc::new(Union::new(
                    Arc::clone(&tier1_contact_relief),
                    Arc::new(Offset::new(Arc::clone(&s.0), beam_half_width * 1.25)),
                ))
            } else {
                Arc::clone(&tier1_contact_relief)
            };
            let keepout_clip: Arc<dyn crate::sdf::Sdf> = Arc::new(Subtract::new(
                Arc::clone(&keepout.0),
                Arc::clone(&allowed_contact_zone),
            ));
            let mount_with_tier1: Arc<dyn crate::sdf::Sdf> = mount_raw;
            let tier1_shelf: Arc<dyn crate::sdf::Sdf> = Arc::new(Offset::new(
                Arc::clone(&mount_sources_tier1.0),
                beam_half_width * 1.5,
            ));
            let parent_contact_skin: Arc<dyn crate::sdf::Sdf> = Arc::new(Offset::new(
                Arc::clone(&parent.0),
                beam_half_width * 0.75,
            ));
            let tier1_contact: Arc<dyn crate::sdf::Sdf> = Arc::new(Intersect::new(
                tier1_shelf,
                parent_contact_skin,
            ));
            let mount_with_tier1_contact: Arc<dyn crate::sdf::Sdf> = Arc::new(Union::new(
                Arc::clone(&mount_with_tier1),
                Arc::clone(&tier1_contact),
            ));
            let corridor: Arc<dyn crate::sdf::Sdf> = Arc::new(Offset::new(
                Arc::clone(&mount_with_tier1_contact),
                corridor_radius as f32,
            ));
```

```rust
            let solid_anchors: Arc<dyn crate::sdf::Sdf> = Arc::new(Subtract::new(
                mount_with_tier1_contact,
                Arc::clone(&keepout_clip),
            ));
```

Result:
- Tier-1 tabs now contribute a continuous contact shelf against the parent shell.
- Tier-2 still drives the beam/lattice path.
- I did not change `tiered_candidate_targets`, `push_distinct_connections_from_candidates`, `freeform_target_constrained`, `freeform_target_to_guide_constrained`, or the final boss union.

## Fix 2: Servo Component Updated to Datasheet-Scale Proportions

Changed file:
- `components/servo_9g.rhai`

What changed:
- Updated body width/depth/height.
- Recentered the body around the tab plane convention.
- Widened the tabs to the datasheet span.
- Updated shaft diameter and protrusion.
- Updated all dependent keepout/boss/tab metadata.

Code change:

```rhai
    let body_x = 21.8;
    let body_y = 12.2;
    let body_z = 24.1;

    let body_top_z = 9.7;
    let body_bottom_z = -14.4;

    let tab_span_y = 29.7;
    let tab_inner_span_y = 26.9;
    let tab_z = 3.3;
    let tab_center_z = 0.0;
```

```rhai
    let shaft_radius = 5.92 * 0.5;
    let shaft_protrusion_z = 2.8;
    let shaft_z = body_top_z + shaft_protrusion_z * 0.5;
```

```rhai
    let body = translate(
        box_(body_x, body_y, body_z),
        0.0, 0.0, (body_top_z + body_bottom_z) * 0.5
    );

    let tabs = translate(
        box_(body_x, tab_span_y, tab_z),
        0.0, 0.0, tab_center_z
    );
```

```rhai
    let keepout_body = translate(
        box_(body_x + clearance_xy * 2.0, body_y + clearance_xy * 2.0, body_z + clearance_z * 2.0),
        0.0, 0.0, (body_top_z + body_bottom_z) * 0.5
    );
    let keepout_tabs = translate(
        box_(
            body_x + clearance_xy * 2.0,
            tab_span_y + clearance_xy * 2.0,
            tab_z + clearance_z * 2.0
        ),
        0.0, 0.0, tab_center_z
    );
```

```rhai
    let side_boss_pos = translate(
        box_(body_x, 8.0, body_z),
        0.0, (tab_span_y + 8.0) * 0.5, (body_top_z + body_bottom_z) * 0.5
    );
    let side_boss_neg = translate(
        box_(body_x, 8.0, body_z),
        0.0, -(tab_span_y + 8.0) * 0.5, (body_top_z + body_bottom_z) * 0.5
    );
    let bottom_boss = translate(
        box_(body_x, tab_inner_span_y, 6.0),
        0.0, 0.0, body_bottom_z - 3.0
    );
```

```rhai
    let mount_sources_tier1 = tabs;
    let mount_sources_tier2 = boss;
```

```rhai
        body_x: body_x,
        body_y: body_y,
        body_z: body_z,
        tab_span_y: tab_span_y
```

Result:
- The servo now reads as a shorter body with much wider tabs.
- The body spans the tab-plane convention instead of starting at `Z=0`.

## Fix 3: Swept Volume Replaced with Static Authored Box

Changed file:
- `components/servo_9g.rhai`

Old behavior:
- Cylindrical/generated swept volume.

New behavior:
- Static conservative box above the shaft.

Code change:

```rhai
    // Static arm travel envelope - conservative box enclosing full 90-degree arc above shaft.
    // User adjusts this per installation if needed.
    let swept_volume = translate(
        box_(arm_half_x * 2.0 + 6.0, arm_half_x * 2.0 + 6.0, 8.0),
        0.0, shaft_y, body_top_z + 4.0
    );
```

Result:
- No cylinder.
- No computed sweep radius.
- It is now a static authored box.

## Validation Run

Build/test:

```text
cargo build
  Finished `dev` profile [unoptimized + debuginfo] target(s) successfully

cargo test test_imported_servo_component_module_builds --lib
  ok
```

Generated validation sections:

```text
servo_component_demo_yz_x84.png
servo_component_demo_yz_x90.png
servo_component_demo_xz_y0.png
servo_component_yz_x0.png
servo_component_xz_y0.png
servo_component_xz_y5.png
```

What those checks show:

1. Tier-1 shelf contact
- `servo_component_demo_yz_x84.png`
- `servo_component_demo_yz_x90.png`

These now show a continuous contact band from the tab region into the wing shell rather than discrete beam posts starting directly off the tab face. Tier-2 lattice still exists above/around that contact path.

2. Servo proportions
- `servo_component_yz_x0.png`
- `servo_component_xz_y0.png`

These show:
- wider tabs
- shorter body above the tab plane
- deeper body below the tab plane

3. Swept volume is a flat box
- `servo_component_xz_y5.png`

This shows the authored rectangular top envelope rather than a cylindrical sweep body.

## Errors / Warnings Encountered

No blocking errors after the final code changes.

Existing warnings still present:

```text
warning: unused variable: `lattice_density`
warning: function `coverage_source_points` is never used
warning: function `best_target_for_source` is never used
warning: function `freeform_bridge_constrained` is never used
warning: function `freeform_bridge_to_guide_constrained` is never used
```

These warnings were pre-existing cleanup debt or caused by this targeted change making some old helper paths unused. They did not block build or validation.
