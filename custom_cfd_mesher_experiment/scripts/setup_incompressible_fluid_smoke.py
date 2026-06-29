from __future__ import annotations

import argparse
import math
import shutil
from pathlib import Path

from setup_potential_foam_smoke import foam_header, parse_vector, read_boundary_patches


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-case-dir", type=Path, required=True)
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--velocity", default="22.352,0,0")
    parser.add_argument("--nu", type=float, default=1.5e-5)
    parser.add_argument("--turbulence-model", choices=["laminar", "kOmegaSST"], default="laminar")
    parser.add_argument(
        "--wall-treatment",
        choices=["wall_function", "low_re"],
        default="wall_function",
        help="Aircraft-wall turbulence boundary condition set for kOmegaSST cases.",
    )
    parser.add_argument("--turbulent-k", type=float, default=0.24)
    parser.add_argument("--turbulent-omega", type=float, default=1.78)
    parser.add_argument("--end-time", type=int, default=30)
    parser.add_argument("--write-interval", type=int, default=10)
    parser.add_argument("--farfield-mode", choices=["fixed", "freestream", "split"], default="freestream")
    parser.add_argument("--p-relax", type=float, default=0.3)
    parser.add_argument("--u-relax", type=float, default=0.7)
    parser.add_argument("--n-outer-correctors", type=int, default=1)
    parser.add_argument("--n-correctors", type=int, default=1)
    parser.add_argument("--n-non-orthogonal-correctors", type=int, default=3)
    parser.add_argument("--potential-flow-correctors", type=int, default=10)
    parser.add_argument("--force-coeffs", action="store_true")
    parser.add_argument("--rho-inf", type=float, default=1.225)
    parser.add_argument("--mag-u-inf", type=float, default=0.0)
    parser.add_argument("--l-ref", type=float, default=1.0)
    parser.add_argument("--a-ref", type=float, default=1.0)
    parser.add_argument("--cofr", default="0,0,0")
    parser.add_argument("--lift-dir", default="0,0,1")
    parser.add_argument("--drag-dir", default="1,0,0")
    parser.add_argument("--pitch-axis", default="0,1,0")
    parser.add_argument("--force-write-interval", type=int, default=1)
    args = parser.parse_args()
    velocity = parse_vector(args.velocity)

    setup_case(
        args.source_case_dir,
        args.case_dir,
        velocity=velocity,
        nu=args.nu,
        turbulence_model=args.turbulence_model,
        turbulent_k=args.turbulent_k,
        turbulent_omega=args.turbulent_omega,
        wall_treatment=args.wall_treatment,
        end_time=args.end_time,
        write_interval=args.write_interval,
        farfield_mode=args.farfield_mode,
        p_relax=args.p_relax,
        u_relax=args.u_relax,
        n_outer_correctors=args.n_outer_correctors,
        n_correctors=args.n_correctors,
        n_non_orthogonal_correctors=args.n_non_orthogonal_correctors,
        potential_flow_correctors=args.potential_flow_correctors,
        force_coeffs=args.force_coeffs,
        rho_inf=args.rho_inf,
        mag_u_inf=args.mag_u_inf or vector_magnitude(velocity),
        l_ref=args.l_ref,
        a_ref=args.a_ref,
        cofr=parse_vector(args.cofr),
        lift_dir=parse_vector(args.lift_dir),
        drag_dir=parse_vector(args.drag_dir),
        pitch_axis=parse_vector(args.pitch_axis),
        force_write_interval=args.force_write_interval,
    )


def setup_case(
    source_case_dir: Path,
    case_dir: Path,
    *,
    velocity: tuple[float, float, float],
    nu: float,
    turbulence_model: str,
    turbulent_k: float,
    turbulent_omega: float,
    wall_treatment: str,
    end_time: int,
    write_interval: int,
    farfield_mode: str,
    p_relax: float,
    u_relax: float,
    n_outer_correctors: int,
    n_correctors: int,
    n_non_orthogonal_correctors: int,
    potential_flow_correctors: int,
    force_coeffs: bool,
    rho_inf: float,
    mag_u_inf: float,
    l_ref: float,
    a_ref: float,
    cofr: tuple[float, float, float],
    lift_dir: tuple[float, float, float],
    drag_dir: tuple[float, float, float],
    pitch_axis: tuple[float, float, float],
    force_write_interval: int,
) -> None:
    poly_mesh = source_case_dir / "constant" / "polyMesh"
    if not poly_mesh.exists():
        raise FileNotFoundError(f"Missing source polyMesh: {poly_mesh}")

    if (case_dir / "constant" / "polyMesh").exists():
        shutil.rmtree(case_dir / "constant" / "polyMesh")
    (case_dir / "constant").mkdir(parents=True, exist_ok=True)
    shutil.copytree(poly_mesh, case_dir / "constant" / "polyMesh")
    if turbulence_model == "kOmegaSST":
        set_boundary_patch_type(case_dir, patch_name="aircraft", patch_type="wall")

    (case_dir / "0").mkdir(parents=True, exist_ok=True)
    (case_dir / "system").mkdir(parents=True, exist_ok=True)
    for generated in (case_dir / "0" / "Phi", case_dir / "0" / "phi"):
        if generated.exists():
            generated.unlink()

    patch_names = read_boundary_patches(case_dir)
    velocity_text = f"({velocity[0]} {velocity[1]} {velocity[2]})"
    write_u(case_dir, patch_names, velocity_text, farfield_mode=farfield_mode)
    write_p(case_dir, patch_names, farfield_mode=farfield_mode)
    if turbulence_model == "kOmegaSST":
        write_rans_fields(
            case_dir,
            patch_names,
            turbulent_k=turbulent_k,
            turbulent_omega=turbulent_omega,
            wall_treatment=wall_treatment,
        )
    write_constant(case_dir, nu, turbulence_model=turbulence_model)
    write_system(
        case_dir,
        end_time=end_time,
        write_interval=write_interval,
        p_relax=p_relax,
        u_relax=u_relax,
        n_outer_correctors=n_outer_correctors,
        n_correctors=n_correctors,
        n_non_orthogonal_correctors=n_non_orthogonal_correctors,
        potential_flow_correctors=potential_flow_correctors,
        force_coeffs=force_coeffs,
        rho_inf=rho_inf,
        mag_u_inf=mag_u_inf,
        l_ref=l_ref,
        a_ref=a_ref,
        cofr=cofr,
        lift_dir=lift_dir,
        drag_dir=drag_dir,
        pitch_axis=pitch_axis,
        force_write_interval=force_write_interval,
        turbulence_model=turbulence_model,
    )


def write_u(case_dir: Path, patch_names: list[str], velocity_text: str, *, farfield_mode: str) -> None:
    patches = []
    for patch_name in patch_names:
        role = patch_role(patch_name)
        if role == "single_farfield" and farfield_mode == "freestream":
            patches.append(
                f"""    {patch_name}
    {{
        type            freestreamVelocity;
        freestreamValue uniform {velocity_text};
    }}"""
            )
        elif role == "single_farfield":
            patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform {velocity_text};
    }}"""
            )
        elif role == "inlet":
            patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform {velocity_text};
    }}"""
            )
        elif role == "outlet":
            patches.append(
                f"""    {patch_name}
    {{
        type            zeroGradient;
    }}"""
            )
        elif role == "side":
            patches.append(
                f"""    {patch_name}
    {{
        type            freestreamVelocity;
        freestreamValue uniform {velocity_text};
    }}"""
            )
        else:
            patches.append(
                f"""    {patch_name}
    {{
        type            noSlip;
    }}"""
            )
    (case_dir / "0" / "U").write_text(
        foam_header("volVectorField", "0", "U")
        + f"""
dimensions      [0 1 -1 0 0 0 0];
internalField   uniform {velocity_text};

boundaryField
{{
{chr(10).join(patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def write_p(case_dir: Path, patch_names: list[str], *, farfield_mode: str) -> None:
    patches = []
    for patch_name in patch_names:
        role = patch_role(patch_name)
        if role == "single_farfield" and farfield_mode == "freestream":
            patches.append(
                f"""    {patch_name}
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}"""
            )
        elif role == "single_farfield":
            patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform 0;
    }}"""
            )
        elif role == "inlet":
            patches.append(
                f"""    {patch_name}
    {{
        type            zeroGradient;
    }}"""
            )
        elif role == "outlet":
            patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform 0;
    }}"""
            )
        elif role == "side":
            patches.append(
                f"""    {patch_name}
    {{
        type            freestreamPressure;
        freestreamValue uniform 0;
    }}"""
            )
        else:
            patches.append(
                f"""    {patch_name}
    {{
        type            zeroGradient;
    }}"""
            )
    (case_dir / "0" / "p").write_text(
        foam_header("volScalarField", "0", "p")
        + f"""
dimensions      [0 2 -2 0 0 0 0];
internalField   uniform 0;

boundaryField
{{
{chr(10).join(patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def patch_role(patch_name: str) -> str:
    if patch_name == "aircraft":
        return "aircraft"
    if patch_name == "farfield":
        return "single_farfield"
    if patch_name == "inlet":
        return "inlet"
    if patch_name == "outlet":
        return "outlet"
    if patch_name.startswith("side_"):
        return "side"
    return "aircraft"


def set_boundary_patch_type(case_dir: Path, *, patch_name: str, patch_type: str) -> None:
    boundary_path = case_dir / "constant" / "polyMesh" / "boundary"
    lines = boundary_path.read_text(encoding="utf-8", errors="replace").splitlines()
    in_patch = False
    brace_depth = 0
    for index, line in enumerate(lines):
        stripped = line.strip()
        if not in_patch and stripped == patch_name:
            in_patch = True
            brace_depth = 0
            continue
        if not in_patch:
            continue
        if stripped == "{":
            brace_depth += 1
            continue
        if stripped == "}":
            brace_depth -= 1
            if brace_depth <= 0:
                in_patch = False
            continue
        if brace_depth > 0 and stripped.startswith("type"):
            indent = line[: len(line) - len(line.lstrip())]
            lines[index] = f"{indent}type            {patch_type};"
            continue
        if brace_depth > 0 and stripped.startswith("physicalType"):
            indent = line[: len(line) - len(line.lstrip())]
            lines[index] = f"{indent}physicalType    {patch_type};"
    boundary_path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def vector_magnitude(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def foam_vector(vector: tuple[float, float, float]) -> str:
    return f"({vector[0]} {vector[1]} {vector[2]})"


def write_rans_fields(
    case_dir: Path,
    patch_names: list[str],
    *,
    turbulent_k: float,
    turbulent_omega: float,
    wall_treatment: str,
) -> None:
    write_k(case_dir, patch_names, turbulent_k, wall_treatment=wall_treatment)
    write_omega(case_dir, patch_names, turbulent_omega)
    write_nut(case_dir, patch_names, wall_treatment=wall_treatment)


def write_k(
    case_dir: Path,
    patch_names: list[str],
    turbulent_k: float,
    *,
    wall_treatment: str,
) -> None:
    patches = []
    for patch_name in patch_names:
        role = patch_role(patch_name)
        if role == "aircraft":
            wall_type = "kLowReWallFunction" if wall_treatment == "low_re" else "kqRWallFunction"
            patches.append(
                f"""    {patch_name}
    {{
        type            {wall_type};
        value           uniform {turbulent_k};
    }}"""
            )
        elif role == "outlet":
            patches.append(
                f"""    {patch_name}
    {{
        type            inletOutlet;
        inletValue      uniform {turbulent_k};
        value           uniform {turbulent_k};
    }}"""
            )
        else:
            patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform {turbulent_k};
    }}"""
            )
    (case_dir / "0" / "k").write_text(
        foam_header("volScalarField", "0", "k")
        + f"""
dimensions      [0 2 -2 0 0 0 0];
internalField   uniform {turbulent_k};

boundaryField
{{
{chr(10).join(patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def write_omega(case_dir: Path, patch_names: list[str], turbulent_omega: float) -> None:
    patches = []
    for patch_name in patch_names:
        role = patch_role(patch_name)
        if role == "aircraft":
            patches.append(
                f"""    {patch_name}
    {{
        type            omegaWallFunction;
        value           uniform {turbulent_omega};
    }}"""
            )
        elif role == "outlet":
            patches.append(
                f"""    {patch_name}
    {{
        type            inletOutlet;
        inletValue      uniform {turbulent_omega};
        value           uniform {turbulent_omega};
    }}"""
            )
        else:
            patches.append(
                f"""    {patch_name}
    {{
        type            fixedValue;
        value           uniform {turbulent_omega};
    }}"""
            )
    (case_dir / "0" / "omega").write_text(
        foam_header("volScalarField", "0", "omega")
        + f"""
dimensions      [0 0 -1 0 0 0 0];
internalField   uniform {turbulent_omega};

boundaryField
{{
{chr(10).join(patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def write_nut(case_dir: Path, patch_names: list[str], *, wall_treatment: str) -> None:
    patches = []
    for patch_name in patch_names:
        role = patch_role(patch_name)
        if role == "aircraft":
            wall_type = "nutLowReWallFunction" if wall_treatment == "low_re" else "nutkWallFunction"
            patches.append(
                f"""    {patch_name}
    {{
        type            {wall_type};
        value           uniform 0;
    }}"""
            )
        else:
            patches.append(
                f"""    {patch_name}
    {{
        type            calculated;
        value           uniform 0;
    }}"""
            )
    (case_dir / "0" / "nut").write_text(
        foam_header("volScalarField", "0", "nut")
        + f"""
dimensions      [0 2 -1 0 0 0 0];
internalField   uniform 0;

boundaryField
{{
{chr(10).join(patches)}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def write_constant(case_dir: Path, nu: float, *, turbulence_model: str) -> None:
    (case_dir / "constant" / "physicalProperties").write_text(
        foam_header("dictionary", "constant", "physicalProperties")
        + f"""
viscosityModel  constant;

nu              [0 2 -1 0 0 0 0] {nu};

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )
    if turbulence_model == "kOmegaSST":
        momentum_transport = """
simulationType RAS;

RAS
{
    model               kOmegaSST;
    turbulence          on;
}

// ************************************************************************* //
"""
    else:
        momentum_transport = """
simulationType laminar;

// ************************************************************************* //
"""
    (case_dir / "constant" / "momentumTransport").write_text(
        foam_header("dictionary", "constant", "momentumTransport")
        + momentum_transport,
        encoding="utf-8",
        newline="\n",
    )


def write_system(
    case_dir: Path,
    *,
    end_time: int,
    write_interval: int,
    p_relax: float,
    u_relax: float,
    n_outer_correctors: int,
    n_correctors: int,
    n_non_orthogonal_correctors: int,
    potential_flow_correctors: int,
    force_coeffs: bool,
    rho_inf: float,
    mag_u_inf: float,
    l_ref: float,
    a_ref: float,
    cofr: tuple[float, float, float],
    lift_dir: tuple[float, float, float],
    drag_dir: tuple[float, float, float],
    pitch_axis: tuple[float, float, float],
    force_write_interval: int,
    turbulence_model: str,
) -> None:
    functions_block = ""
    if force_coeffs:
        if mag_u_inf <= 0.0:
            raise ValueError("forceCoeffs requires positive magUInf")
        if l_ref <= 0.0 or a_ref <= 0.0:
            raise ValueError("forceCoeffs requires positive lRef and Aref")
        functions_block = f"""
functions
{{
    aircraftForceCoeffs
    {{
        type            forceCoeffs;
        libs            ("libforces.so");

        writeControl    timeStep;
        writeInterval   {force_write_interval};
        log             true;

        patches         (aircraft);

        rho             rhoInf;
        rhoInf          {rho_inf};
        liftDir         {foam_vector(lift_dir)};
        dragDir         {foam_vector(drag_dir)};
        CofR            {foam_vector(cofr)};
        pitchAxis       {foam_vector(pitch_axis)};
        magUInf         {mag_u_inf};
        lRef            {l_ref};
        Aref            {a_ref};
    }}
}}
"""

    (case_dir / "system" / "controlDict").write_text(
        foam_header("dictionary", "system", "controlDict")
        + f"""
solver          incompressibleFluid;

startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         {end_time};
deltaT          1;

writeControl    timeStep;
writeInterval   {write_interval};
purgeWrite      0;

writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;

runTimeModifiable true;
{functions_block}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )

    turbulence_div_schemes = ""
    if turbulence_model == "kOmegaSST":
        turbulence_div_schemes = """
    div(phi,k)      bounded Gauss upwind;
    div(phi,omega)  bounded Gauss upwind;"""

    (case_dir / "system" / "fvSchemes").write_text(
        foam_header("dictionary", "system", "fvSchemes")
        + f"""
ddtSchemes
{{
    default         steadyState;
}}

gradSchemes
{{
    default         Gauss linear;
    limited         cellLimited Gauss linear 1;
    grad(U)         $limited;
}}

divSchemes
{{
    default         none;
    div(phi,U)      bounded Gauss upwind;
{turbulence_div_schemes}
    div(div(phi,U)) Gauss linear;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}}

laplacianSchemes
{{
    default         Gauss linear limited 0.5;
}}

interpolationSchemes
{{
    default         linear;
}}

snGradSchemes
{{
    default         limited 0.5;
}}

wallDist
{{
    method meshWave;
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )

    turbulence_solvers = ""
    turbulence_relaxation = ""
    if turbulence_model == "kOmegaSST":
        turbulence_solvers = """

    k
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-7;
        relTol          0.05;
    }

    omega
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-7;
        relTol          0.05;
    }"""
        turbulence_relaxation = """
        k               0.5;
        omega           0.5;"""

    (case_dir / "system" / "fvSolution").write_text(
        foam_header("dictionary", "system", "fvSolution")
        + f"""
solvers
{{
    p
    {{
        solver          GAMG;
        smoother        GaussSeidel;
        tolerance       1e-6;
        relTol          0.05;
    }}

    U
    {{
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-7;
        relTol          0.05;
    }}

    Phi
    {{
        solver          PCG;
        preconditioner  DIC;
        tolerance       1e-8;
        relTol          0.01;
    }}
{turbulence_solvers}
}}

PIMPLE
{{
    nOuterCorrectors {n_outer_correctors};
    nCorrectors {n_correctors};
    nNonOrthogonalCorrectors {n_non_orthogonal_correctors};
    pRefCell        0;
    pRefValue       0;
}}

potentialFlow
{{
    nNonOrthogonalCorrectors {potential_flow_correctors};
}}

relaxationFactors
{{
    fields
    {{
        p               {p_relax};
    }}
    equations
    {{
        U               {u_relax};
{turbulence_relaxation}
    }}
}}

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


if __name__ == "__main__":
    main()
