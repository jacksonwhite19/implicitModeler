import "components/electronics_box" as ebox;

let shell_thickness = 1.6;
let fuselage_outer = fuselage_elliptical(220.0, 90.0, 70.0, 45.0, 50.0, 0.9, 0.8, 8.0);
let fuselage_shell = shell(fuselage_outer, shell_thickness);
let fuse_center = bbox_center(fuselage_outer);

let c = ebox::component();
let c = bracket_offset(c, 0.5);
let c = tray_clearance(c, 0.5);
let c = tray_thickness(c, 1.0);
let c = support_density(c, 6);

let placed = mount_component_granular(
    fuselage_shell,
    c,
    fuse_center.x, 0.0, fuse_center.z + 2.0
);

placed.fastener_pads
