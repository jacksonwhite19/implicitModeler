from __future__ import annotations

import argparse
import math
from pathlib import Path

from paraview.simple import (
    CreateView,
    Delete,
    LegacyVTKReader,
    Render,
    ResetCamera,
    SaveScreenshot,
    SetActiveView,
    Show,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--aircraft-vtk", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument("--representation", choices=["Surface", "Surface With Edges"], default="Surface With Edges")
    args = parser.parse_args()

    args.output_png.parent.mkdir(parents=True, exist_ok=True)

    view = CreateView("RenderView")
    SetActiveView(view)
    view.ViewSize = [args.width, args.height]
    view.Background = [1.0, 1.0, 1.0]
    view.OrientationAxesVisibility = 0

    reader = LegacyVTKReader(FileNames=[str(args.aircraft_vtk)])
    display = Show(reader, view)
    display.Representation = args.representation
    display.DiffuseColor = [0.78, 0.80, 0.82]
    if args.representation == "Surface With Edges":
        display.EdgeColor = [0.05, 0.05, 0.05]

    Render(view)
    bounds = reader.GetDataInformation().GetBounds()
    cx = 0.5 * (bounds[0] + bounds[1])
    cy = 0.5 * (bounds[2] + bounds[3])
    cz = 0.5 * (bounds[4] + bounds[5])
    dx = bounds[1] - bounds[0]
    dy = bounds[3] - bounds[2]
    dz = bounds[5] - bounds[4]
    diagonal = math.sqrt(dx * dx + dy * dy + dz * dz)

    ResetCamera(view)
    view.CameraFocalPoint = [cx, cy, cz]
    view.CameraPosition = [cx + 1.55 * diagonal, cy - 1.85 * diagonal, cz + 1.15 * diagonal]
    view.CameraViewUp = [0.0, 0.0, 1.0]
    view.CameraParallelProjection = 1
    view.CameraParallelScale = 0.58 * diagonal
    Render(view)

    SaveScreenshot(str(args.output_png), view, ImageResolution=[args.width, args.height])
    Delete(reader)
    Delete(view)


if __name__ == "__main__":
    main()
