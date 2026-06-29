from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--velocity", default="22.352,0,0")
    args = parser.parse_args()

    velocity = parse_vector(args.velocity)
    write_case(args.case_dir, velocity)


def write_case(case_dir: Path, velocity: tuple[float, float, float]) -> None:
    (case_dir / "0").mkdir(parents=True, exist_ok=True)
    (case_dir / "system").mkdir(parents=True, exist_ok=True)
    (case_dir / "constant").mkdir(parents=True, exist_ok=True)
    for generated in (case_dir / "0" / "Phi", case_dir / "0" / "phi"):
        if generated.exists():
            generated.unlink()

    velocity_text = f"({velocity[0]} {velocity[1]} {velocity[2]})"
    patch_names = read_boundary_patches(case_dir)
    u_patches = []
    p_patches = []
    farfield_patches = {"farfield", "inlet", "outlet", "side_ymin", "side_ymax", "side_zmin", "side_zmax"}
    for patch_name in patch_names:
        if patch_name in farfield_patches:
            u_patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform {velocity_text};
    }}"""
            )
            p_patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform 0;
    }}"""
            )
        else:
            u_patches.append(
                f"""    {patch_name}
    {{
        type            slip;
    }}"""
            )
            p_patches.append(
                f"""    {patch_name}
    {{
        type            zeroGradient;
    }}"""
            )
    (case_dir / "0" / "U").write_text(
        foam_header("volVectorField", "0", "U")
        + f"""
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform {velocity_text};

boundaryField
{{
{chr(10).join(u_patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
    )

    (case_dir / "0" / "p").write_text(
        foam_header("volScalarField", "0", "p")
        + f"""
dimensions      [0 2 -2 0 0 0 0];
internalField   uniform 0;

boundaryField
{{
{chr(10).join(p_patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
    )

    (case_dir / "system" / "controlDict").write_text(
        foam_header("dictionary", "system", "controlDict")
        + """
application     potentialFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         1;
deltaT          1;
writeControl    timeStep;
writeInterval   1;
purgeWrite      0;
writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;

// ************************************************************************* //
""",
        encoding="utf-8",
    )

    (case_dir / "system" / "fvSchemes").write_text(
        foam_header("dictionary", "system", "fvSchemes")
        + """
ddtSchemes
{
    default         steadyState;
}

gradSchemes
{
    default         Gauss linear;
}

divSchemes
{
    default         none;
    div(phi,U)      Gauss linear;
    div(div(phi,U)) Gauss linear;
}

laplacianSchemes
{
    default         Gauss linear corrected;
}

interpolationSchemes
{
    default         linear;
}

snGradSchemes
{
    default         corrected;
}

// ************************************************************************* //
""",
        encoding="utf-8",
    )

    (case_dir / "system" / "fvSolution").write_text(
        foam_header("dictionary", "system", "fvSolution")
        + """
solvers
{
    Phi
    {
        solver          PCG;
        preconditioner  DIC;
        tolerance       1e-08;
        relTol          0.01;
    }

    p
    {
        solver          PCG;
        preconditioner  DIC;
        tolerance       1e-08;
        relTol          0.01;
    }
}

potentialFlow
{
    nNonOrthogonalCorrectors 10;
}

// ************************************************************************* //
""",
        encoding="utf-8",
    )


def foam_header(class_name: str, location: str, object_name: str) -> str:
    return f"""/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM
   \\\\    /   O peration     |
    \\\\  /    A nd           |
     \\\\/     M anipulation  |
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    format      ascii;
    class       {class_name};
    location    \"{location}\";
    object      {object_name};
}}

"""


def parse_vector(value: str) -> tuple[float, float, float]:
    parts = [float(part) for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError("Expected velocity as vx,vy,vz")
    return parts[0], parts[1], parts[2]


def read_boundary_patches(case_dir: Path) -> list[str]:
    boundary_path = case_dir / "constant" / "polyMesh" / "boundary"
    if not boundary_path.exists():
        return ["aircraft", "farfield"]
    lines = boundary_path.read_text(encoding="utf-8", errors="replace").splitlines()
    start_index = None
    for index, line in enumerate(lines[:-1]):
        if line.strip().isdigit() and lines[index + 1].strip() == "(":
            start_index = index + 2
            break
    if start_index is None:
        return ["aircraft", "farfield"]

    patch_names: list[str] = []
    for index in range(start_index, len(lines) - 1):
        line = lines[index]
        name = line.strip()
        if name == ")":
            break
        if not name or name == "(" or name.startswith("//"):
            continue
        next_line = lines[index + 1].strip()
        if next_line == "{" and not name[0].isdigit():
            patch_names.append(name)
    return patch_names or ["aircraft", "farfield"]


if __name__ == "__main__":
    main()
