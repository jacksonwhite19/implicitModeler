from __future__ import annotations

import argparse
from pathlib import Path

from paraview.simple import ExtractSurface, LegacyVTKReader, SaveData


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert a VTK aircraft patch surface to STL with ParaView.")
    parser.add_argument("--input-vtk", type=Path, required=True)
    parser.add_argument("--output-stl", type=Path, required=True)
    args = parser.parse_args()

    args.output_stl.parent.mkdir(parents=True, exist_ok=True)
    reader = LegacyVTKReader(FileNames=[str(args.input_vtk)])
    surface = ExtractSurface(Input=reader)
    SaveData(str(args.output_stl), proxy=surface)
    print(args.output_stl)


if __name__ == "__main__":
    main()
