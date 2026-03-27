import "components/servo_9g" as servo;

// Large generic symmetric wing section sized to house a 9g servo internally.
let root_chord = 180.0;
let span = 260.0;
let shell_thickness = 1.6;

let wing_outer = wing_with_airfoil("0018", root_chord, root_chord, span, 0.0, 0.0, 0.0);
let wing_shell = shell(wing_outer, shell_thickness);

let wing_center = bbox_center(wing_outer);
let wing_min = bbox_min(wing_outer);

// Servo local origin is the center of the mounting face on the bottom of the body.
// Place it internally, slightly above the lower skin, so the granular bracket engine
// has to trace mount-point paths out to the wing shell.
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

placed.assembly
