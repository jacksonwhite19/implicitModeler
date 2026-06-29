from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    report = validate(args.mesh)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["valid"]:
        raise SystemExit(1)


def validate(path: Path) -> dict[str, object]:
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    cursor = 0
    ndime = read_assignment(lines[cursor], "NDIME")
    cursor += 1
    npoin = None
    nelem = None
    point_lines: list[str] = []
    elem_lines: list[str] = []
    marker_blocks: list[tuple[str, list[str]]] = []

    while cursor < len(lines):
        line = lines[cursor]
        if line.startswith("NPOIN="):
            npoin = read_assignment(line, "NPOIN")
            cursor += 1
            point_lines = lines[cursor : cursor + npoin]
            cursor += npoin
        elif line.startswith("NELEM="):
            nelem = read_assignment(line, "NELEM")
            cursor += 1
            elem_lines = lines[cursor : cursor + nelem]
            cursor += nelem
        elif line.startswith("NMARK="):
            nmark = read_assignment(line, "NMARK")
            cursor += 1
            for _ in range(nmark):
                tag = read_tag(lines[cursor], "MARKER_TAG")
                cursor += 1
                count = read_assignment(lines[cursor], "MARKER_ELEMS")
                cursor += 1
                marker_blocks.append((tag, lines[cursor : cursor + count]))
                cursor += count
        else:
            raise ValueError(f"Unexpected SU2 section line: {line}")

    if npoin is None or nelem is None:
        raise ValueError("Missing NPOIN or NELEM section")

    max_point = npoin - 1
    volume_types: dict[str, int] = {}
    marker_counts: dict[str, int] = {}
    bad_refs = 0

    for line in point_lines:
        parts = line.split()
        if len(parts) != 4:
            raise ValueError(f"Bad point line: {line}")

    for line in elem_lines:
        parts = [int(part) for part in line.split()]
        elem_type = parts[0]
        node_count = su2_node_count(elem_type)
        node_ids = parts[1 : 1 + node_count]
        if any(node < 0 or node > max_point for node in node_ids):
            bad_refs += 1
        volume_types[str(elem_type)] = volume_types.get(str(elem_type), 0) + 1

    for tag, marker_lines in marker_blocks:
        marker_counts[tag] = len(marker_lines)
        for line in marker_lines:
            parts = [int(part) for part in line.split()]
            elem_type = parts[0]
            node_count = su2_node_count(elem_type)
            node_ids = parts[1 : 1 + node_count]
            if any(node < 0 or node > max_point for node in node_ids):
                bad_refs += 1

    valid = ndime == 3 and bad_refs == 0
    return {
        "mesh": str(path),
        "valid": valid,
        "ndime": ndime,
        "points": npoin,
        "volume_elements": nelem,
        "volume_types": volume_types,
        "markers": marker_counts,
        "bad_node_references": bad_refs,
        "unparsed_lines": 0,
    }


def read_assignment(line: str, key: str) -> int:
    prefix = f"{key}="
    if not line.startswith(prefix):
        raise ValueError(f"Expected {key}= line, got: {line}")
    return int(line.split("=", 1)[1].strip())


def read_tag(line: str, key: str) -> str:
    prefix = f"{key}="
    if not line.startswith(prefix):
        raise ValueError(f"Expected {key}= line, got: {line}")
    return line.split("=", 1)[1].strip()


def su2_node_count(elem_type: int) -> int:
    counts = {
        5: 3,
        9: 4,
        10: 4,
        12: 8,
        13: 6,
        14: 5,
    }
    if elem_type not in counts:
        raise ValueError(f"Unsupported SU2 element type {elem_type}")
    return counts[elem_type]


if __name__ == "__main__":
    main()
