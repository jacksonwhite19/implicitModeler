from __future__ import annotations

import argparse
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
    parser.add_argument("--hotspot-vtk", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1200)
    args = parser.parse_args()

    args.output_png.parent.mkdir(parents=True, exist_ok=True)

    view = CreateView("RenderView")
    SetActiveView(view)
    view.ViewSize = [args.width, args.height]
    view.Background = [1.0, 1.0, 1.0]
    view.OrientationAxesVisibility = 0

    aircraft = LegacyVTKReader(FileNames=[str(args.aircraft_vtk)])
    aircraft_display = Show(aircraft, view)
    aircraft_display.Representation = "Surface With Edges"
    aircraft_display.DiffuseColor = [0.72, 0.74, 0.76]
    aircraft_display.EdgeColor = [0.15, 0.15, 0.15]

    hotspots = LegacyVTKReader(FileNames=[str(args.hotspot_vtk)])
    hotspot_display = Show(hotspots, view)
    hotspot_display.Representation = "Points"
    hotspot_display.PointSize = 14
    hotspot_display.DiffuseColor = [1.0, 0.05, 0.02]

    Render(view)
    bounds = aircraft.GetDataInformation().GetBounds()
    cx = 0.5 * (bounds[0] + bounds[1])
    cy = 0.5 * (bounds[2] + bounds[3])
    cz = 0.5 * (bounds[4] + bounds[5])
    dx = bounds[1] - bounds[0]
    dy = bounds[3] - bounds[2]
    dz = bounds[5] - bounds[4]
    diagonal = (dx * dx + dy * dy + dz * dz) ** 0.5

    ResetCamera(view)
    view.CameraFocalPoint = [cx, cy, cz]
    view.CameraPosition = [cx + 1.55 * diagonal, cy - 1.85 * diagonal, cz + 1.15 * diagonal]
    view.CameraViewUp = [0.0, 0.0, 1.0]
    view.CameraParallelProjection = 1
    view.CameraParallelScale = 0.58 * diagonal
    Render(view)

    SaveScreenshot(str(args.output_png), view, ImageResolution=[args.width, args.height])
    Delete(hotspots)
    Delete(aircraft)
    Delete(view)


if __name__ == "__main__":
    main()
