import "components/electronics_box" as ebox;

let shell_thickness = 1.6;
let fuselage_outer = fuselage_elliptical(220.0, 90.0, 70.0, 45.0, 50.0, 0.9, 0.8, 8.0);
let fuselage_shell = shell(fuselage_outer, shell_thickness);

let fuse_center = bbox_center(fuselage_outer);

let box_x = fuse_center.x;
let box_y = 0.0;
let box_z = fuse_center.z + 2.0;

let c = ebox::component();
let placed = mount_component_granular(fuselage_shell, c, box_x, box_y, box_z);

placed.raw_bracket
