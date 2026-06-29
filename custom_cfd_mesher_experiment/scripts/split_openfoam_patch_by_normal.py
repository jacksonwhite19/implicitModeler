from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--patch", required=True)
    parser.add_argument("--axis", choices=["x", "y", "z"], default="y")
    parser.add_argument("--negative-name", default="cap_negative")
    parser.add_argument("--positive-name", default="cap_positive")
    parser.add_argument("--backup", action="store_true")
    args = parser.parse_args()

    poly_dir = args.case_dir / "constant" / "polyMesh"
    points = parse_points(poly_dir / "points")
    face_lines, faces = parse_faces(poly_dir / "faces")
    owner_lines = parse_label_lines(poly_dir / "owner")
    boundary_text = (poly_dir / "boundary").read_text(encoding="utf-8")
    patch = find_patch(boundary_text, args.patch)

    axis_index = {"x": 0, "y": 1, "z": 2}[args.axis]
    negative: list[int] = []
    positive: list[int] = []
    for face_index in range(patch["start"], patch["start"] + patch["count"]):
        normal = newell_normal([points[i] for i in faces[face_index]])
        if normal[axis_index] < 0:
            negative.append(face_index)
        else:
            positive.append(face_index)

    if not negative or not positive:
        raise RuntimeError(f"Patch {args.patch!r} did not split into two non-empty normal groups")

    if args.backup:
        backup_dir = poly_dir / "_pre_patch_split_backup"
        if backup_dir.exists():
            shutil.rmtree(backup_dir)
        backup_dir.mkdir()
        for name in ("faces", "owner", "boundary"):
            shutil.copy2(poly_dir / name, backup_dir / name)

    reordered = negative + positive
    old_range = list(range(patch["start"], patch["start"] + patch["count"]))
    for old, new in zip(old_range, reordered):
        face_lines[old] = format_face(faces[new])
        owner_lines[old] = owner_lines[new]

    write_list_file(poly_dir / "faces", object_name="faces", class_name="faceList", lines=face_lines)
    write_list_file(poly_dir / "owner", object_name="owner", class_name="labelList", lines=owner_lines)
    (poly_dir / "boundary").write_text(
        replace_patch_with_split(
            boundary_text,
            args.patch,
            args.negative_name,
            len(negative),
            args.positive_name,
            len(positive),
            patch["start"],
        ),
        encoding="utf-8",
        newline="\n",
    )
    print(
        {
            "patch": args.patch,
            "axis": args.axis,
            "negative_name": args.negative_name,
            "negative_faces": len(negative),
            "positive_name": args.positive_name,
            "positive_faces": len(positive),
            "start_face": patch["start"],
        }
    )


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


def parse_points(path: Path) -> list[tuple[float, float, float]]:
    text = strip_comments(path.read_text(encoding="utf-8"))
    match = re.search(r"\n\s*(\d+)\s*\(\s*(.*)\s*\)\s*$", text, flags=re.S)
    if not match:
        raise RuntimeError(f"Could not parse points file {path}")
    return [
        (float(x), float(y), float(z))
        for x, y, z in re.findall(r"\(([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\)", match.group(2))
    ]


def parse_faces(path: Path) -> tuple[list[str], list[list[int]]]:
    lines = parse_list_body_lines(path)
    faces: list[list[int]] = []
    for line in lines:
        match = re.match(r"\s*\d+\(([^)]*)\)\s*$", line)
        if not match:
            raise RuntimeError(f"Could not parse face line: {line!r}")
        faces.append([int(item) for item in match.group(1).split()])
    return lines, faces


def parse_label_lines(path: Path) -> list[str]:
    return parse_list_body_lines(path)


def parse_list_body_lines(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    start = text.index("(\n") + 2
    end = text.rindex("\n)")
    return [line.strip() for line in text[start:end].splitlines() if line.strip()]


def write_list_file(path: Path, *, object_name: str, class_name: str, lines: list[str]) -> None:
    body = "\n".join(lines)
    path.write_text(
        f"""/*--------------------------------*- C++ -*----------------------------------*\\
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
    location    "constant/polyMesh";
    object      {object_name};
}}

{len(lines)}
(
{body}
)

// ************************************************************************* //
""",
        encoding="utf-8",
        newline="\n",
    )


def format_face(face: list[int]) -> str:
    return f"{len(face)}({' '.join(str(item) for item in face)})"


def find_patch(boundary_text: str, patch_name: str) -> dict[str, int]:
    match = re.search(
        rf"(?ms)^\s*{re.escape(patch_name)}\s*\{{.*?nFaces\s+(\d+);\s*startFace\s+(\d+);.*?^\s*\}}",
        boundary_text,
    )
    if not match:
        raise RuntimeError(f"Could not find patch {patch_name!r}")
    return {"count": int(match.group(1)), "start": int(match.group(2)), "block_start": match.start(), "block_end": match.end()}


def replace_patch_with_split(
    boundary_text: str,
    patch_name: str,
    negative_name: str,
    negative_count: int,
    positive_name: str,
    positive_count: int,
    start_face: int,
) -> str:
    patch = find_patch(boundary_text, patch_name)
    split_block = f"""    {negative_name}
    {{
        type            patch;
        nFaces          {negative_count};
        startFace       {start_face};
    }}
    {positive_name}
    {{
        type            patch;
        nFaces          {positive_count};
        startFace       {start_face + negative_count};
    }}"""
    updated = boundary_text[: patch["block_start"]] + split_block + boundary_text[patch["block_end"] :]
    count_match = re.search(r"(?ms)(// \*.*?\n\n)(\d+)(\s*\n\()", updated)
    if not count_match:
        raise RuntimeError("Could not update boundary patch count")
    old_count = int(count_match.group(2))
    return updated[: count_match.start(2)] + str(old_count + 1) + updated[count_match.end(2) :]


def newell_normal(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    nx = ny = nz = 0.0
    for a, b in zip(points, points[1:] + points[:1]):
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return nx, ny, nz


if __name__ == "__main__":
    main()
