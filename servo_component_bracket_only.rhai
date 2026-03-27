import "components/servo_9g" as servo;

let root_chord = 180.0;
let span = 260.0;
let shell_thickness = 1.6;

let wing_outer = wing_with_airfoil("0018", root_chord, root_chord, span, 0.0, 0.0, 0.0);
let wing_shell = shell(wing_outer, shell_thickness);

let wing_center = bbox_center(wing_outer);
let wing_min = bbox_min(wing_outer);

let servo_x = wing_center.x;
let servo_y = wing_center.y;
let servo_z = wing_min.z + 10.0;

let s = servo::component();
s.bracket_config = #{
    step_size_mm: 0.7,
    max_path_iters: 220,
    dilate_keepout_mm: 1.0,
    pocket_offset_mm: 0.2
};

let placed = mount_component_granular(
    wing_shell,
    s,
    servo_x, servo_y, servo_z
);

placed.raw_bracket
