// Generic 9g servo component
// Local coordinates:
//   X = span
//   Y = chord (front is +Y)
//   Z = up
// Origin = center of mounting face on the bottom of the servo body.

fn component() {
    let body_x = 21.8;
    let body_y = 12.2;
    let body_z = 24.1;

    let body_top_z = 9.7;
    let body_bottom_z = -14.4;

    let tab_span_y = 29.7;
    let tab_inner_span_y = 26.9;
    let tab_z = 3.3;
    let tab_center_z = 0.0;

    let clearance_xy = 0.3;
    let clearance_z = 0.4;

    let shaft_radius = 5.92 * 0.5;
    let shaft_protrusion_z = 2.8;
    let shaft_z = body_top_z + shaft_protrusion_z * 0.5;
    let shaft_y = 5.0;
    let arm_half_x = 11.0;
    let horn_thickness = 2.0;

    let body = translate(
        box_(body_x, body_y, body_z),
        0.0, 0.0, (body_top_z + body_bottom_z) * 0.5
    );

    let tabs = translate(
        box_(body_x, tab_span_y, tab_z),
        0.0, 0.0, tab_center_z
    );

    let horn_hub = translate(
        cylinder(shaft_radius, shaft_protrusion_z),
        0.0, shaft_y, shaft_z
    );
    let horn_arm = translate(
        box_(arm_half_x * 2.0, 2.2, horn_thickness),
        0.0, shaft_y, body_top_z + horn_thickness * 0.5
    );

    let physical = union(
        union(body, tabs),
        union(horn_hub, horn_arm)
    );

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
    let keepout = union(keepout_body, keepout_tabs);

    // Static arm travel envelope - conservative box enclosing full 90-degree arc above shaft.
    // User adjusts this per installation if needed.
    let swept_volume = translate(
        box_(arm_half_x * 2.0 + 6.0, arm_half_x * 2.0 + 6.0, 8.0),
        0.0, shaft_y, body_top_z + 4.0
    );

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
    let front_lip = translate(
        box_(body_x, 3.0, 3.0),
        0.0, tab_span_y * 0.5 + 1.5, body_top_z - 1.5
    );
    let rear_lip = translate(
        box_(body_x, 3.0, 3.0),
        0.0, -(tab_span_y * 0.5 + 1.5), body_top_z - 1.5
    );

    let boss = union(
        union(side_boss_pos, side_boss_neg),
        union(bottom_boss, union(front_lip, rear_lip))
    );
    let mount_points = [
        mount_point(point(0.0,  tab_inner_span_y * 0.5, 0.0), point(0.0,  1.0, 0.0), 1, 2.2),
        mount_point(point(0.0, -tab_inner_span_y * 0.5, 0.0), point(0.0, -1.0, 0.0), 1, 2.2),
        mount_point(point(0.0, 0.0, body_bottom_z), point(0.0, 0.0, -1.0), 2, 1.8)
    ];

    #{
        parts: #{
            body: body,
            tabs: tabs,
            horn_hub: horn_hub,
            horn_arm: horn_arm,
            swept_volume: swept_volume,
            boss: boss
        },
        keepout: keepout,
        swept_volume: swept_volume,
        boss: boss,
        mount_points: mount_points,
        physical: physical,
        body_x: body_x,
        body_y: body_y,
        body_z: body_z,
        tab_span_y: tab_span_y
    }
}

// Standalone preview when this file is opened directly in the app.
let preview = component();
union(preview.physical, preview.swept_volume)
