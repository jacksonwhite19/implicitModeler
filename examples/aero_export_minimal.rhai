let body = sphere(12.0);

let aero_export = #{
    parts: [
        #{
            name: "body_oml",
            sdf: body,
            aero_role: "outer_mold_line",
            patch_name: "body"
        }
    ]
};

body
