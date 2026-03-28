import "components/electronics_box" as ebox;

let shell_thickness = 1.6;

// Simple example fuselage shell with enough room for the box and mounts.
let fuselage_outer = fuselage_elliptical(220.0, 90.0, 70.0, 45.0, 50.0, 0.9, 0.8, 8.0);
let fuselage_shell = shell(fuselage_outer, shell_thickness);

let fuse_center = bbox_center(fuselage_outer);

let box_x = fuse_center.x;
let box_y = 0.0;
let box_z = fuse_center.z + 2.0;

let c = ebox::component();
let c = bracket_offset(c, 0.5);
let c = tray_clearance(c, 0.5);
let c = tray_thickness(c, 1.0);
let c = tray_style(c, "u_channel");
let c = tray_wall_height(c, 10.5);
let c = tray_lip_height(c, 1.0);
let c = tray_extend(c, 0.8);
let c = support_density(c, 6);
c.bracket_config = #{
    step_size_mm: 0.8,
    max_path_iters: 240,
    tier1_radius_end_factor: 1.15,
    tier2_bridge_radius_factor: 0.45,
    dilate_keepout_mm: 0.8,
    pocket_offset_mm: 0.2,
    host_blend_k: 28.0
};

let placed = mount_component_granular(fuselage_shell, c, box_x, box_y, box_z);
// Example visibility control:
// let placed = hide_part(placed, "component_physical");

let visible_fuselage = intersect(fuselage_shell, fuselage_outer);
let visible_bracket = intersect(placed.bracket, fuselage_outer);
let visible_mounts = subtract(union(visible_fuselage, visible_bracket), placed.fastener_keepout);

let preview_parts = #{
    parts: #{
        component_physical: placed.component_physical,
        tray: intersect(placed.tray, fuselage_outer),
        raw_bracket: intersect(placed.raw_bracket, fuselage_outer),
        cut_bracket: intersect(placed.cut_bracket, fuselage_outer),
        fastener_pads: placed.fastener_pads,
        bracket: intersect(placed.bracket, fuselage_outer),
        fuselage_shell: visible_fuselage,
        fastener_keepout: placed.fastener_keepout,
        keepout: placed.keepout
    }
};

union(
    visible_mounts,
    placed.component_physical
)
