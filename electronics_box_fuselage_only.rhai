let shell_thickness = 1.6;
let fuselage_outer = fuselage_elliptical(220.0, 90.0, 70.0, 45.0, 50.0, 0.9, 0.8, 8.0);
let fuselage_shell = shell(fuselage_outer, shell_thickness);

fuselage_shell
