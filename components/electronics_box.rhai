// Generic electronics box component
// Local coordinates:
//   X = length
//   Y = width
//   Z = up
// Origin = center of the box bottom face.

fn screw_at(x, y, shaft_r, head_r, head_d, shaft_d, z_top) {
    translate(
        countersink(shaft_r, head_r, head_d, shaft_d),
        x, y, z_top
    )
}

fn screw_keepout_at(x, y, shaft_r, head_r, head_d, shaft_d, z_top, above_h, below_extra, top_open_below) {
    let hole = translate(
        countersink(shaft_r, head_r, head_d, shaft_d),
        x, y, z_top
    );
    let head_clear = translate(
        cylinder(head_r, above_h + top_open_below),
        x, y, z_top + (above_h - top_open_below) * 0.5
    );
    let tail_clear = translate(
        cylinder(shaft_r, below_extra),
        x, y, z_top - head_d - shaft_d - below_extra * 0.5
    );
    union(union(hole, head_clear), tail_clear)
}

fn screw_pad_seed_at(x, y, shaft_r, shaft_d, z_top, head_d, below_extra) {
    translate(
        cylinder(shaft_r, shaft_d + below_extra),
        x, y, z_top - head_d - (shaft_d + below_extra) * 0.5
    )
}

fn screw_pad_limit_at(x, y, limit_r, shaft_d, z_top, head_d, below_extra) {
    translate(
        cylinder(limit_r, shaft_d + below_extra),
        x, y, z_top - head_d - (shaft_d + below_extra) * 0.5
    )
}

fn component() {
    let body_x = 50.0;
    let body_y = 40.0;
    let body_z = 10.0;
    let screw_edge_inset = 5.0;
    let screw_shaft_r = 1.7;
    let screw_head_r = 3.1;
    let screw_head_d = 2.0;
    let screw_penetration_below = 3.0;
    let screw_shaft_d = body_z - screw_head_d + screw_penetration_below;
    let screw_cut_head_radius_extra = 0.2;

    let wall_clearance = 0.5;
    let top_keepout_z = body_z * 0.5;
    let top_keepout_h = body_z * 0.5 + 6.0;
    let screw_keepout_margin = 0.35;
    let screw_head_relief_above = 1.5;
    let screw_head_relief_below = 0.8;
    let screw_head_clear_above = 1.0;
    let screw_tail_keepout_extra = 0.8;
    let fastener_pad_limit_r = screw_head_r + 1.8;

    let screw_x = body_x * 0.5 - screw_edge_inset;
    let screw_y = body_y * 0.5 - screw_edge_inset;

    let screw_cut = union(
        union(
            screw_keepout_at( screw_x,  screw_y, screw_shaft_r, screw_head_r + screw_cut_head_radius_extra, screw_head_d, screw_shaft_d, body_z, screw_head_relief_above, 0.0, screw_head_relief_below),
            screw_keepout_at(-screw_x,  screw_y, screw_shaft_r, screw_head_r + screw_cut_head_radius_extra, screw_head_d, screw_shaft_d, body_z, screw_head_relief_above, 0.0, screw_head_relief_below)
        ),
        union(
            screw_keepout_at( screw_x, -screw_y, screw_shaft_r, screw_head_r + screw_cut_head_radius_extra, screw_head_d, screw_shaft_d, body_z, screw_head_relief_above, 0.0, screw_head_relief_below),
            screw_keepout_at(-screw_x, -screw_y, screw_shaft_r, screw_head_r + screw_cut_head_radius_extra, screw_head_d, screw_shaft_d, body_z, screw_head_relief_above, 0.0, screw_head_relief_below)
        )
    );

    let fastener_keepout = union(
        union(
            screw_keepout_at( screw_x,  screw_y, screw_shaft_r + screw_keepout_margin, screw_head_r + screw_keepout_margin, screw_head_d + 0.2, screw_shaft_d + 0.2, body_z, screw_head_clear_above, screw_tail_keepout_extra, screw_head_relief_below + 0.2),
            screw_keepout_at(-screw_x,  screw_y, screw_shaft_r + screw_keepout_margin, screw_head_r + screw_keepout_margin, screw_head_d + 0.2, screw_shaft_d + 0.2, body_z, screw_head_clear_above, screw_tail_keepout_extra, screw_head_relief_below + 0.2)
        ),
        union(
            screw_keepout_at( screw_x, -screw_y, screw_shaft_r + screw_keepout_margin, screw_head_r + screw_keepout_margin, screw_head_d + 0.2, screw_shaft_d + 0.2, body_z, screw_head_clear_above, screw_tail_keepout_extra, screw_head_relief_below + 0.2),
            screw_keepout_at(-screw_x, -screw_y, screw_shaft_r + screw_keepout_margin, screw_head_r + screw_keepout_margin, screw_head_d + 0.2, screw_shaft_d + 0.2, body_z, screw_head_clear_above, screw_tail_keepout_extra, screw_head_relief_below + 0.2)
        )
    );

    let fastener_pad_seed = union(
        union(
            screw_pad_seed_at( screw_x,  screw_y, screw_shaft_r + screw_keepout_margin, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra),
            screw_pad_seed_at(-screw_x,  screw_y, screw_shaft_r + screw_keepout_margin, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra)
        ),
        union(
            screw_pad_seed_at( screw_x, -screw_y, screw_shaft_r + screw_keepout_margin, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra),
            screw_pad_seed_at(-screw_x, -screw_y, screw_shaft_r + screw_keepout_margin, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra)
        )
    );

    let fastener_pad_limit = union(
        union(
            screw_pad_limit_at( screw_x,  screw_y, fastener_pad_limit_r, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra),
            screw_pad_limit_at(-screw_x,  screw_y, fastener_pad_limit_r, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra)
        ),
        union(
            screw_pad_limit_at( screw_x, -screw_y, fastener_pad_limit_r, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra),
            screw_pad_limit_at(-screw_x, -screw_y, fastener_pad_limit_r, screw_shaft_d + 0.2, body_z, screw_head_d + 0.2, screw_tail_keepout_extra)
        )
    );

    let physical_outer = translate(
        box_(body_x, body_y, body_z),
        0.0, 0.0, body_z * 0.5
    );
    let physical = subtract(physical_outer, fastener_keepout);

    let keepout = translate(
        box_(
            body_x + wall_clearance * 2.0,
            body_y + wall_clearance * 2.0,
            body_z + wall_clearance * 2.0
        ),
        0.0, 0.0, body_z * 0.5
    );

    // Top-half service/access keepout: mounts should avoid this region.
    let service_keepout = translate(
        box_(body_x + 8.0, body_y + 8.0, top_keepout_h),
        0.0, 0.0, top_keepout_z + top_keepout_h * 0.5
    );

    let boss = union(
        translate(box_(body_x, 4.0, 4.0), 0.0, body_y * 0.5 + 2.0, body_z * 0.5),
        translate(box_(body_x, 4.0, 4.0), 0.0, -(body_y * 0.5 + 2.0), body_z * 0.5)
    );

    // Explicit tray seed: bottom + both side faces. This is the region the generator
    // should turn into one connected offset tray shell before adding supports.
    let tray_bottom = translate(
        box_(body_x, body_y, 0.4),
        0.0, 0.0, 0.2
    );
    let tray_side_pos = translate(
        box_(body_x, 0.4, body_z),
        0.0, body_y * 0.5, body_z * 0.5
    );
    let tray_side_neg = translate(
        box_(body_x, 0.4, body_z),
        0.0, -body_y * 0.5, body_z * 0.5
    );
    let tray_seed = union(tray_bottom, union(tray_side_pos, tray_side_neg));

    let bottom_mount = translate(
        box_(body_x - 8.0, body_y - 8.0, 2.0),
        0.0, 0.0, 1.0
    );
    let side_mount_pos = translate(
        box_(body_x - 10.0, 2.5, body_z - 4.0),
        0.0, body_y * 0.5, body_z * 0.5
    );
    let side_mount_neg = translate(
        box_(body_x - 10.0, 2.5, body_z - 4.0),
        0.0, -body_y * 0.5, body_z * 0.5
    );
    let lower_side_pos = translate(
        box_(body_x, 10.0, 2.0),
        0.0, body_y * 0.5 - 5.0, 1.0
    );
    let lower_side_neg = translate(
        box_(body_x, 10.0, 2.0),
        0.0, -(body_y * 0.5 - 5.0), 1.0
    );
    let lower_front = translate(
        box_(10.0, body_y - 20.0, 2.0),
        body_x * 0.5 - 5.0, 0.0, 1.0
    );
    let lower_rear = translate(
        box_(10.0, body_y - 20.0, 2.0),
        -(body_x * 0.5 - 5.0), 0.0, 1.0
    );
    let lower_outer = union(
        union(lower_side_pos, lower_side_neg),
        union(lower_front, lower_rear)
    );
    let lower_inner = translate(
        box_(body_x - 20.0, body_y - 20.0, 2.0),
        0.0, 0.0, 1.0
    );
    let mount_points = [
        // Tier 1: outer 10 mm of the lower face. Ordered as strips so density expansion
        // can fill coverage along the box length.
        mount_point(point(-body_x * 0.35,  body_y * 0.5 - 5.0, 0.0), point(0.0, 0.0, -1.0), 1, 1.2),
        mount_point(point( body_x * 0.35,  body_y * 0.5 - 5.0, 0.0), point(0.0, 0.0, -1.0), 1, 1.2),
        mount_point(point(-body_x * 0.35, -body_y * 0.5 + 5.0, 0.0), point(0.0, 0.0, -1.0), 1, 1.2),
        mount_point(point( body_x * 0.35, -body_y * 0.5 + 5.0, 0.0), point(0.0, 0.0, -1.0), 1, 1.2),

        // Tier 1: side faces, also ordered front-to-rear for density expansion.
        mount_point(point(-body_x * 0.30,  body_y * 0.5, body_z * 0.35), point(0.0,  1.0, 0.0), 1, 1.2),
        mount_point(point( body_x * 0.30,  body_y * 0.5, body_z * 0.35), point(0.0,  1.0, 0.0), 1, 1.2),
        mount_point(point(-body_x * 0.30, -body_y * 0.5, body_z * 0.35), point(0.0, -1.0, 0.0), 1, 1.2),
        mount_point(point( body_x * 0.30, -body_y * 0.5, body_z * 0.35), point(0.0, -1.0, 0.0), 1, 1.2),

        // Tier 2: inner lower face, feasible if easy. Ordered as two inner strips.
        mount_point(point(-body_x * 0.22,  body_y * 0.18, 0.0), point(0.0, 0.0, -1.0), 2, 0.9),
        mount_point(point( body_x * 0.22,  body_y * 0.18, 0.0), point(0.0, 0.0, -1.0), 2, 0.9),
        mount_point(point(-body_x * 0.22, -body_y * 0.18, 0.0), point(0.0, 0.0, -1.0), 2, 0.9),
        mount_point(point( body_x * 0.22, -body_y * 0.18, 0.0), point(0.0, 0.0, -1.0), 2, 0.9)
    ];

    #{
        parts: #{
            body: physical,
            fastener_pad_limit: fastener_pad_limit,
            fastener_pad_seed: fastener_pad_seed,
            fastener_keepout: fastener_keepout,
            service_keepout: service_keepout,
            boss: boss,
            tray_seed: tray_seed,
            mount_points: mount_points
        },
        keepout: keepout,
        service_keepout: service_keepout,
        fastener_pad_limit: fastener_pad_limit,
        fastener_pad_seed: fastener_pad_seed,
        fastener_keepout: fastener_keepout,
        tray_seed: tray_seed,
        mount_points: mount_points,
        support_density: 3,
        lattice_density: 0.35,
        boss: boss,
        physical: physical,
        body_x: body_x,
        body_y: body_y,
        body_z: body_z
    }
}

let preview = component();
preview.physical
