from __future__ import annotations

from dataclasses import dataclass
from math import hypot, sqrt
from statistics import median
from typing import Callable

Sdf2D = Callable[[float, float], float]


@dataclass(frozen=True)
class BBox2D:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def expanded(self, amount: float) -> "BBox2D":
        return BBox2D(
            xmin=self.xmin - amount,
            ymin=self.ymin - amount,
            xmax=self.xmax + amount,
            ymax=self.ymax + amount,
        )

    def contains(self, x: float, y: float) -> bool:
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax


@dataclass(frozen=True)
class ConditioningFixtureScenario:
    name: str
    before: Sdf2D
    after: Sdf2D
    dirty_region: BBox2D
    domain: BBox2D
    grid_spacing: float
    halo_mm: float
    interface_band_mm: float


@dataclass(frozen=True)
class ConditioningFixtureResult:
    scenario_name: str
    dirty_region: BBox2D
    update_region: BBox2D
    grid_point_count: int
    dirty_block_count: int
    halo_block_count: int
    conditioned_sample_count: int
    changed_sample_count: int
    max_abs_error_inside_update: float
    max_abs_error_near_interface: float
    sign_mismatch_count_near_interface: int
    unexpected_sign_flip_count_outside_update: int
    gradient_abs_error_p50: float
    gradient_abs_error_p95: float
    gradient_abs_error_max: float
    conditioning_confidence: float

    def metrics(self) -> dict[str, dict[str, object]]:
        return {
            "conditioning.dirty_block_count": _metric(self.dirty_block_count, "count"),
            "conditioning.halo_block_count": _metric(self.halo_block_count, "count"),
            "conditioning.conditioned_sample_count": _metric(
                self.conditioned_sample_count, "count"
            ),
            "conditioning.sign_flip_count_near_interface": _metric(
                self.sign_mismatch_count_near_interface, "count"
            ),
            "conditioning.local_topology_delta": _metric(
                self.unexpected_sign_flip_count_outside_update, "count"
            ),
            "conditioning.gradient_abs_error_p50": _metric(
                self.gradient_abs_error_p50, None
            ),
            "conditioning.gradient_abs_error_p95": _metric(
                self.gradient_abs_error_p95, None
            ),
            "conditioning.gradient_abs_error_max": _metric(
                self.gradient_abs_error_max, None
            ),
            "conditioning.confidence": _metric(self.conditioning_confidence, None),
        }


def default_conditioning_fixture_scenarios() -> list[ConditioningFixtureScenario]:
    domain = BBox2D(-48.0, -32.0, 48.0, 32.0)
    return [
        ConditioningFixtureScenario(
            name="sphere_radius_edit",
            before=_circle(0.0, 0.0, 10.0),
            after=_circle(0.0, 0.0, 11.0),
            dirty_region=BBox2D(-12.0, -12.0, 12.0, 12.0),
            domain=domain,
            grid_spacing=1.0,
            halo_mm=2.0,
            interface_band_mm=2.5,
        ),
        ConditioningFixtureScenario(
            name="box_corner_edit",
            before=_box(0.0, 0.0, 14.0, 8.0),
            after=_box(0.0, 0.0, 16.0, 9.0),
            dirty_region=BBox2D(-18.0, -11.0, 18.0, 11.0),
            domain=domain,
            grid_spacing=1.0,
            halo_mm=2.0,
            interface_band_mm=2.5,
        ),
        ConditioningFixtureScenario(
            name="wing_like_tip_edit",
            before=_smooth_union(
                _ellipse(-8.0, 0.0, 23.0, 5.5),
                _ellipse(13.0, 0.0, 12.0, 3.8),
                blend=1.5,
            ),
            after=_smooth_union(
                _ellipse(-8.0, 0.0, 23.0, 5.5),
                _ellipse(15.0, 0.0, 14.0, 4.3),
                blend=1.5,
            ),
            dirty_region=BBox2D(0.0, -8.0, 38.0, 8.0),
            domain=domain,
            grid_spacing=1.0,
            halo_mm=3.0,
            interface_band_mm=3.0,
        ),
        ConditioningFixtureScenario(
            name="thin_shell_thickness_edit",
            before=_annulus(0.0, 0.0, outer_radius=15.0, inner_radius=13.0),
            after=_annulus(0.0, 0.0, outer_radius=15.0, inner_radius=12.0),
            dirty_region=BBox2D(-16.0, -16.0, 16.0, 16.0),
            domain=domain,
            grid_spacing=1.0,
            halo_mm=2.0,
            interface_band_mm=2.5,
        ),
    ]


def run_conditioning_fixture(
    scenario: ConditioningFixtureScenario,
) -> ConditioningFixtureResult:
    points = list(_grid_points(scenario.domain, scenario.grid_spacing))
    before = {point: scenario.before(*point) for point in points}
    full_after = {point: scenario.after(*point) for point in points}
    update_region = scenario.dirty_region.expanded(scenario.halo_mm)
    local_after = {
        point: (full_after[point] if update_region.contains(*point) else before[point])
        for point in points
    }

    inside_update = [point for point in points if update_region.contains(*point)]
    dirty_points = [point for point in points if scenario.dirty_region.contains(*point)]
    near_interface = [
        point
        for point in points
        if abs(full_after[point]) <= scenario.interface_band_mm
        or abs(local_after[point]) <= scenario.interface_band_mm
    ]
    changed = [
        point
        for point in points
        if abs(full_after[point] - before[point]) > 1.0e-9
        and (
            abs(full_after[point]) <= scenario.interface_band_mm
            or abs(before[point]) <= scenario.interface_band_mm
        )
    ]

    sign_mismatches = [
        point
        for point in near_interface
        if _sign(full_after[point]) != _sign(local_after[point])
    ]
    unexpected_outside = [
        point
        for point in points
        if not update_region.contains(*point)
        and _sign(full_after[point]) != _sign(before[point])
    ]
    gradient_errors = _gradient_abs_errors(
        scenario.after,
        near_interface,
        scenario.grid_spacing,
    )
    confidence = 1.0
    if sign_mismatches or unexpected_outside:
        confidence = 0.0
    elif gradient_errors:
        confidence = max(0.0, 1.0 - min(1.0, _percentile(gradient_errors, 95)))

    return ConditioningFixtureResult(
        scenario_name=scenario.name,
        dirty_region=scenario.dirty_region,
        update_region=update_region,
        grid_point_count=len(points),
        dirty_block_count=len(dirty_points),
        halo_block_count=max(0, len(inside_update) - len(dirty_points)),
        conditioned_sample_count=len(inside_update),
        changed_sample_count=len(changed),
        max_abs_error_inside_update=_max_abs_error(local_after, full_after, inside_update),
        max_abs_error_near_interface=_max_abs_error(local_after, full_after, near_interface),
        sign_mismatch_count_near_interface=len(sign_mismatches),
        unexpected_sign_flip_count_outside_update=len(unexpected_outside),
        gradient_abs_error_p50=median(gradient_errors) if gradient_errors else 0.0,
        gradient_abs_error_p95=_percentile(gradient_errors, 95),
        gradient_abs_error_max=max(gradient_errors) if gradient_errors else 0.0,
        conditioning_confidence=confidence,
    )


def run_default_conditioning_fixtures() -> list[ConditioningFixtureResult]:
    return [
        run_conditioning_fixture(scenario)
        for scenario in default_conditioning_fixture_scenarios()
    ]


def _circle(cx: float, cy: float, radius: float) -> Sdf2D:
    return lambda x, y: hypot(x - cx, y - cy) - radius


def _box(cx: float, cy: float, half_x: float, half_y: float) -> Sdf2D:
    def sdf(x: float, y: float) -> float:
        qx = abs(x - cx) - half_x
        qy = abs(y - cy) - half_y
        outside = hypot(max(qx, 0.0), max(qy, 0.0))
        inside = min(max(qx, qy), 0.0)
        return outside + inside

    return sdf


def _ellipse(cx: float, cy: float, radius_x: float, radius_y: float) -> Sdf2D:
    def sdf(x: float, y: float) -> float:
        nx = (x - cx) / radius_x
        ny = (y - cy) / radius_y
        scale = min(radius_x, radius_y)
        return (sqrt(nx * nx + ny * ny) - 1.0) * scale

    return sdf


def _annulus(
    cx: float,
    cy: float,
    *,
    outer_radius: float,
    inner_radius: float,
) -> Sdf2D:
    outer = _circle(cx, cy, outer_radius)
    inner = _circle(cx, cy, inner_radius)
    return lambda x, y: max(outer(x, y), -inner(x, y))


def _smooth_union(a: Sdf2D, b: Sdf2D, *, blend: float) -> Sdf2D:
    def sdf(x: float, y: float) -> float:
        av = a(x, y)
        bv = b(x, y)
        h = max(0.0, min(1.0, 0.5 + 0.5 * (bv - av) / blend))
        return (1.0 - h) * bv + h * av - blend * h * (1.0 - h)

    return sdf


def _grid_points(domain: BBox2D, spacing: float) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    x_count = int(round((domain.xmax - domain.xmin) / spacing))
    y_count = int(round((domain.ymax - domain.ymin) / spacing))
    for ix in range(x_count + 1):
        x = domain.xmin + ix * spacing
        for iy in range(y_count + 1):
            y = domain.ymin + iy * spacing
            points.append((x, y))
    return points


def _gradient_abs_errors(
    sdf: Sdf2D,
    points: list[tuple[float, float]],
    spacing: float,
) -> list[float]:
    errors: list[float] = []
    h = spacing
    for x, y in points:
        gx = (sdf(x + h, y) - sdf(x - h, y)) / (2.0 * h)
        gy = (sdf(x, y + h) - sdf(x, y - h)) / (2.0 * h)
        errors.append(abs(hypot(gx, gy) - 1.0))
    return errors


def _max_abs_error(
    local: dict[tuple[float, float], float],
    full: dict[tuple[float, float], float],
    points: list[tuple[float, float]],
) -> float:
    if not points:
        return 0.0
    return max(abs(local[point] - full[point]) for point in points)


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((percentile / 100.0) * (len(ordered) - 1)))
    return ordered[index]


def _sign(value: float) -> int:
    if value < 0.0:
        return -1
    if value > 0.0:
        return 1
    return 0


def _metric(value: object, unit: str | None) -> dict[str, object]:
    return {
        "value": value,
        "unit": unit,
        "confidence": 1.0,
        "source": "geometry_generator_conditioning_fixture",
    }
