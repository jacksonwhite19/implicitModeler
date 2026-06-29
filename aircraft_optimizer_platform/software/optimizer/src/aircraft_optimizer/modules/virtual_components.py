from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.records import CandidateSeed, MetricValue

MODULE_NAME = "virtual_component_mass_properties"
MODULE_VERSION = "0.1.0"

DEFAULT_VIRTUAL_COMPONENT_ASSUMPTIONS: dict[str, Any] = {
    "construction": "mostly_3d_printed_shell_with_integral_ribs_and_bulkheads",
    "printed_shell_areal_density_kg_m2": 0.95,
    "printed_rib_linear_density_kg_m": 0.045,
    "wing_rib_spacing_mm": 95.0,
    "wing_spar_linear_density_kg_m": 0.075,
    "wing_root_reinforcement_kg": 0.055,
    "fuselage_length_mm": 960.0,
    "fuselage_diameter_mm": 115.0,
    "fuselage_inner_diameter_mm": 100.0,
    "fuselage_usable_x_min_mm": 105.0,
    "fuselage_usable_x_max_mm": 745.0,
    "fuselage_bulkhead_count": 5,
    "fuselage_bulkhead_mass_kg": 0.018,
    "fuselage_mount_allowance_kg": 0.055,
    "target_cg_x_mm": 341.05,
    "force_internal_components_on_x_axis": True,
    "component_axial_clearance_mm": 8.0,
    "default_inner_surface_keepout_mm": 2.0,
    "components": {
        "battery": {
            "mass_kg": 0.27,
            "dimensions_mm": [120.0, 38.0, 32.0],
            "inner_surface_keepout_mm": 3.0,
            "x_range_mm": [190.0, 560.0],
            "z_mm": 0.0,
        },
        "payload": {
            "mass_kg": 0.18,
            "dimensions_mm": [85.0, 45.0, 35.0],
            "inner_surface_keepout_mm": 3.0,
            "fixed_x_mm": 365.0,
            "z_mm": 0.0,
        },
        "avionics": {
            "mass_kg": 0.08,
            "dimensions_mm": [70.0, 45.0, 18.0],
            "inner_surface_keepout_mm": 2.0,
            "x_range_mm": [180.0, 620.0],
            "z_mm": 0.0,
        },
        "edf": {
            "mass_kg": 0.16,
            "dimensions_mm": [95.0, 78.0, 78.0],
            "inner_surface_keepout_mm": 2.0,
            "fixed_x_mm": 232.5,
            "z_mm": 0.0,
        },
    },
}


@dataclass(frozen=True)
class VirtualComponent:
    name: str
    role: str
    mass_kg: float
    cg_mm: tuple[float, float, float]
    dimensions_mm: tuple[float, float, float] | None = None
    inner_surface_keepout_mm: float | None = None
    placement_kind: str = "fixed"
    export_geometry: bool = False

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "name": self.name,
            "role": self.role,
            "mass_kg": round(self.mass_kg, 6),
            "cg_mm": {
                "x": round(self.cg_mm[0], 6),
                "y": round(self.cg_mm[1], 6),
                "z": round(self.cg_mm[2], 6),
            },
            "placement_kind": self.placement_kind,
            "export_geometry": self.export_geometry,
        }
        if self.dimensions_mm is not None:
            result["dimensions_mm"] = {
                "length": round(self.dimensions_mm[0], 6),
                "width": round(self.dimensions_mm[1], 6),
                "height": round(self.dimensions_mm[2], 6),
            }
        if self.inner_surface_keepout_mm is not None:
            result["inner_surface_keepout_mm"] = round(self.inner_surface_keepout_mm, 6)
        return result


@dataclass(frozen=True)
class VirtualMassProperties:
    components: list[VirtualComponent]
    total_mass_kg: float
    cg_mm: tuple[float, float, float]
    neutral_point_x_mm: float | None
    static_margin: float | None
    reference_chord_mm: float
    layout_feasible: bool
    target_cg_x_mm: float
    initial_cg_x_mm: float
    cg_error_mm: float
    cg_shift_mm: float
    layout_wiggle_room_mm: float
    layout_issues: list[str]
    assumptions: dict[str, Any]

    def metrics(self) -> dict[str, dict[str, object]]:
        metrics = {
            "mass.estimated_total_kg": _metric(self.total_mass_kg, "kg"),
            "mass.cg_x_mm": _metric(self.cg_mm[0], "mm"),
            "mass.cg_y_mm": _metric(self.cg_mm[1], "mm"),
            "mass.cg_z_mm": _metric(self.cg_mm[2], "mm"),
            "layout.target_cg_x_mm": _metric(self.target_cg_x_mm, "mm"),
            "layout.initial_cg_x_mm": _metric(self.initial_cg_x_mm, "mm"),
            "layout.cg_error_mm": _metric(self.cg_error_mm, "mm"),
            "layout.cg_shift_mm": _metric(self.cg_shift_mm, "mm"),
            "layout.wiggle_room_mm": _metric(self.layout_wiggle_room_mm, "mm"),
            "layout.feasible": _metric(1 if self.layout_feasible else 0, None),
            "geometry.reference_chord_mm": _metric(self.reference_chord_mm, "mm"),
            "mass.virtual_component_count": _metric(len(self.components), None),
        }
        if self.neutral_point_x_mm is not None:
            metrics["stability.neutral_point_x_mm"] = _metric(self.neutral_point_x_mm, "mm")
        if self.static_margin is not None:
            metrics["stability.static_margin"] = _metric(self.static_margin, None)
        else:
            metrics["stability.static_margin_available"] = _metric(0, None)
        return metrics

    def metadata(self) -> dict[str, object]:
        return {
            "component_model": MODULE_NAME,
            "component_model_version": MODULE_VERSION,
            "export_geometry": False,
            "components": [component.to_dict() for component in self.components],
            "layout": {
                "feasible": self.layout_feasible,
                "target_cg_x_mm": round(self.target_cg_x_mm, 6),
                "initial_cg_x_mm": round(self.initial_cg_x_mm, 6),
                "cg_error_mm": round(self.cg_error_mm, 6),
                "cg_shift_mm": round(self.cg_shift_mm, 6),
                "wiggle_room_mm": round(self.layout_wiggle_room_mm, 6),
                "issues": self.layout_issues,
                "static_margin_available": False,
                "static_margin_deferred_until": "cfd_neutral_point_analysis",
                "cg_adjustment_policy": "move_non_fixed_internal_components_within_allowed_x_ranges_toward_target_cg",
            },
            "assumptions": {
                "purpose": "pre-definition mass and CG screening only",
                "not_exported": True,
                "not_structural_analysis": True,
                "construction": self.assumptions["construction"],
                "coordinate_frame": "reference aircraft local mm",
                "config": self.assumptions,
            },
        }


def estimate_virtual_mass_properties(
    candidate: CandidateSeed,
    assumptions: dict[str, Any] | None = None,
) -> VirtualMassProperties:
    return _estimate_virtual_mass_properties(
        candidate,
        assumptions=assumptions,
        target_cg_x_mm=None,
        neutral_point_x_mm=None,
        static_margin=None,
    )


def estimate_virtual_mass_properties_for_static_margin(
    candidate: CandidateSeed,
    *,
    neutral_point_x_mm: float,
    desired_static_margin: float,
    assumptions: dict[str, Any] | None = None,
) -> VirtualMassProperties:
    active_assumptions = _merge_assumptions(assumptions)
    mean_chord_mm = _mean_chord_mm(candidate)
    target_cg_x_mm = float(neutral_point_x_mm) - float(desired_static_margin) * mean_chord_mm
    mass_properties = _estimate_virtual_mass_properties(
        candidate,
        assumptions=active_assumptions,
        target_cg_x_mm=target_cg_x_mm,
        neutral_point_x_mm=float(neutral_point_x_mm),
        static_margin=None,
    )
    final_static_margin = (float(neutral_point_x_mm) - mass_properties.cg_mm[0]) / mean_chord_mm
    return VirtualMassProperties(
        components=mass_properties.components,
        total_mass_kg=mass_properties.total_mass_kg,
        cg_mm=mass_properties.cg_mm,
        neutral_point_x_mm=float(neutral_point_x_mm),
        static_margin=final_static_margin,
        reference_chord_mm=mass_properties.reference_chord_mm,
        layout_feasible=mass_properties.layout_feasible,
        target_cg_x_mm=mass_properties.target_cg_x_mm,
        initial_cg_x_mm=mass_properties.initial_cg_x_mm,
        cg_error_mm=mass_properties.cg_error_mm,
        cg_shift_mm=mass_properties.cg_shift_mm,
        layout_wiggle_room_mm=mass_properties.layout_wiggle_room_mm,
        layout_issues=mass_properties.layout_issues,
        assumptions=mass_properties.assumptions,
    )


def _estimate_virtual_mass_properties(
    candidate: CandidateSeed,
    *,
    assumptions: dict[str, Any] | None,
    target_cg_x_mm: float | None,
    neutral_point_x_mm: float | None,
    static_margin: float | None,
) -> VirtualMassProperties:
    active_assumptions = _merge_assumptions(assumptions)
    variables = candidate.design_variables
    span_mm = float(variables["wing.span_mm"].value)
    root_chord_mm = float(variables["wing.root_chord_mm"].value)
    tip_chord_mm = float(variables["wing.tip_chord_mm"].value)
    sweep_deg = float(variables["wing.sweep_deg"].value)

    mean_chord_mm = (root_chord_mm + tip_chord_mm) / 2.0
    area_m2 = (span_mm / 1000.0) * (mean_chord_mm / 1000.0)
    wetted_wing_area_m2 = area_m2 * 2.08
    reference_chord_mm = mean_chord_mm
    wing_ac_x_mm = 285.0 + 0.25 * root_chord_mm + max(0.0, sweep_deg) * 1.5

    printed_shell_areal_density_kg_m2 = float(active_assumptions["printed_shell_areal_density_kg_m2"])
    printed_rib_linear_density_kg_m = float(active_assumptions["printed_rib_linear_density_kg_m"])
    wing_rib_count = max(6, round(span_mm / float(active_assumptions["wing_rib_spacing_mm"])))
    wing_skin_mass_kg = wetted_wing_area_m2 * printed_shell_areal_density_kg_m2
    wing_rib_mass_kg = wing_rib_count * (mean_chord_mm / 1000.0) * printed_rib_linear_density_kg_m
    wing_spar_mass_kg = (span_mm / 1000.0) * float(active_assumptions["wing_spar_linear_density_kg_m"])
    wing_root_reinforcement_kg = float(active_assumptions["wing_root_reinforcement_kg"])
    wing_mass_kg = (
        wing_skin_mass_kg
        + wing_rib_mass_kg
        + wing_spar_mass_kg
        + wing_root_reinforcement_kg
    )

    fuselage_length_m = float(active_assumptions["fuselage_length_mm"]) / 1000.0
    fuselage_diameter_m = float(active_assumptions["fuselage_diameter_mm"]) / 1000.0
    fuselage_shell_area_m2 = 3.14159 * fuselage_diameter_m * fuselage_length_m
    bulkhead_count = int(active_assumptions["fuselage_bulkhead_count"])
    bulkhead_mass_kg = float(active_assumptions["fuselage_bulkhead_mass_kg"])
    mount_allowance_kg = float(active_assumptions["fuselage_mount_allowance_kg"])
    fuselage_mass_kg = (
        fuselage_shell_area_m2 * printed_shell_areal_density_kg_m2
        + bulkhead_count * bulkhead_mass_kg
        + mount_allowance_kg
    )
    component_assumptions = active_assumptions["components"]
    battery_mass_kg = float(component_assumptions["battery"]["mass_kg"])
    payload_mass_kg = float(component_assumptions["payload"]["mass_kg"])
    avionics_mass_kg = float(component_assumptions["avionics"]["mass_kg"])
    propulsion_mass_kg = float(component_assumptions["edf"]["mass_kg"])

    structural_components = [
        VirtualComponent("printed_wing_shell_ribs_spar", "printed_structure", wing_mass_kg, (wing_ac_x_mm, 0.0, 18.0)),
        VirtualComponent("printed_fuselage_shell_bulkheads", "printed_structure", fuselage_mass_kg, (315.0, 0.0, 0.0)),
    ]
    active_target_cg_x_mm = (
        float(target_cg_x_mm)
        if target_cg_x_mm is not None
        else float(active_assumptions["target_cg_x_mm"])
    )
    layout_components, layout_feasible, wiggle_room_mm, layout_issues, initial_cg_x = _layout_internal_components(
        structural_components=structural_components,
        target_cg_x_mm=active_target_cg_x_mm,
        components=[
            _layout_spec("battery", "energy_storage", battery_mass_kg, component_assumptions["battery"]),
            _layout_spec("payload", "payload", payload_mass_kg, component_assumptions["payload"]),
            _layout_spec("avionics", "systems", avionics_mass_kg, component_assumptions["avionics"]),
            _layout_spec("edf", "propulsion", propulsion_mass_kg, component_assumptions["edf"]),
        ],
        assumptions=active_assumptions,
    )

    components = structural_components + layout_components
    total_mass_kg = sum(component.mass_kg for component in components)
    cg_x = sum(component.mass_kg * component.cg_mm[0] for component in components) / total_mass_kg
    cg_y = sum(component.mass_kg * component.cg_mm[1] for component in components) / total_mass_kg
    cg_z = sum(component.mass_kg * component.cg_mm[2] for component in components) / total_mass_kg
    cg_error_mm = cg_x - active_target_cg_x_mm
    cg_shift_mm = cg_x - initial_cg_x
    return VirtualMassProperties(
        components=components,
        total_mass_kg=total_mass_kg,
        cg_mm=(cg_x, cg_y, cg_z),
        neutral_point_x_mm=neutral_point_x_mm,
        static_margin=static_margin,
        reference_chord_mm=reference_chord_mm,
        layout_feasible=layout_feasible,
        target_cg_x_mm=active_target_cg_x_mm,
        initial_cg_x_mm=initial_cg_x,
        cg_error_mm=cg_error_mm,
        cg_shift_mm=cg_shift_mm,
        layout_wiggle_room_mm=wiggle_room_mm,
        layout_issues=layout_issues,
        assumptions=active_assumptions,
    )


def _mean_chord_mm(candidate: CandidateSeed) -> float:
    variables = candidate.design_variables
    return (
        float(variables["wing.root_chord_mm"].value)
        + float(variables["wing.tip_chord_mm"].value)
    ) / 2.0


def _merge_assumptions(overrides: dict[str, Any] | None) -> dict[str, Any]:
    merged = {
        key: (value.copy() if isinstance(value, dict) else value)
        for key, value in DEFAULT_VIRTUAL_COMPONENT_ASSUMPTIONS.items()
    }
    merged["components"] = {
        key: value.copy()
        for key, value in DEFAULT_VIRTUAL_COMPONENT_ASSUMPTIONS["components"].items()
    }
    if not overrides:
        return merged
    for key, value in overrides.items():
        if key == "components":
            for component_name, component_value in value.items():
                merged["components"].setdefault(component_name, {})
                merged["components"][component_name].update(component_value)
        else:
            merged[key] = value
    return merged


def _layout_spec(
    name: str,
    role: str,
    mass_kg: float,
    spec: dict[str, Any],
) -> dict[str, Any]:
    return {
        "name": name,
        "role": role,
        "mass_kg": mass_kg,
        "dimensions_mm": tuple(float(value) for value in spec["dimensions_mm"]),
        "x_range_mm": tuple(float(value) for value in spec["x_range_mm"]) if "x_range_mm" in spec else None,
        "fixed_x_mm": float(spec["fixed_x_mm"]) if "fixed_x_mm" in spec else None,
        "z_mm": float(spec.get("z_mm", 0.0)),
        "inner_surface_keepout_mm": float(spec.get("inner_surface_keepout_mm", 0.0)),
    }


def _layout_internal_components(
    *,
    structural_components: list[VirtualComponent],
    target_cg_x_mm: float,
    components: list[dict[str, Any]],
    assumptions: dict[str, Any],
) -> tuple[list[VirtualComponent], bool, float, list[str], float]:
    issues: list[str] = []
    inner_diameter_mm = float(assumptions["fuselage_inner_diameter_mm"])
    usable_min = float(assumptions["fuselage_usable_x_min_mm"])
    usable_max = float(assumptions["fuselage_usable_x_max_mm"])
    axial_clearance = float(assumptions["component_axial_clearance_mm"])
    default_keepout = float(assumptions["default_inner_surface_keepout_mm"])
    usable_length = usable_max - usable_min
    required_length = 0.0
    placed: list[dict[str, Any]] = []

    for component in components:
        length, width, height = component["dimensions_mm"]
        keepout = float(component["inner_surface_keepout_mm"] or default_keepout)
        required_length += length + axial_clearance
        if max(width, height) + 2.0 * keepout > inner_diameter_mm:
            issues.append(f"{component['name']}.cross_section_exceeds_fuselage")
        x_range = component["x_range_mm"]
        if x_range is not None:
            min_x = max(usable_min + length / 2.0, x_range[0])
            max_x = min(usable_max - length / 2.0, x_range[1])
            if min_x > max_x:
                issues.append(f"{component['name']}.x_range_too_small")
            x = (min_x + max_x) / 2.0
            placement_kind = "movable"
        else:
            x = float(component["fixed_x_mm"])
            min_x = max_x = x
            placement_kind = "fixed"
            if x - length / 2.0 < usable_min or x + length / 2.0 > usable_max:
                issues.append(f"{component['name']}.fixed_position_outside_usable_fuselage")
        z_mm = 0.0 if bool(assumptions["force_internal_components_on_x_axis"]) else float(component["z_mm"])
        placed.append(component | {"x_mm": x, "min_x_mm": min_x, "max_x_mm": max_x, "placement_kind": placement_kind, "z_mm": z_mm})

    wiggle_room = usable_length - required_length
    if wiggle_room < 0.0:
        issues.append("internal_components.axial_length_exceeds_usable_fuselage")

    fixed_moment = sum(component.mass_kg * component.cg_mm[0] for component in structural_components)
    fixed_mass = sum(component.mass_kg for component in structural_components)
    initial_mass = fixed_mass + sum(item["mass_kg"] for item in placed)
    initial_moment = fixed_moment + sum(item["mass_kg"] * item["x_mm"] for item in placed)
    initial_cg_x = initial_moment / initial_mass
    for _ in range(2):
        for component in sorted(placed, key=lambda item: item["mass_kg"], reverse=True):
            if component["placement_kind"] != "movable":
                continue
            other_mass = fixed_mass + sum(item["mass_kg"] for item in placed if item is not component)
            other_moment = fixed_moment + sum(item["mass_kg"] * item["x_mm"] for item in placed if item is not component)
            required_x = (target_cg_x_mm * (other_mass + component["mass_kg"]) - other_moment) / component["mass_kg"]
            component["x_mm"] = min(component["max_x_mm"], max(component["min_x_mm"], required_x))

    virtual_components = [
        VirtualComponent(
            str(component["name"]),
            str(component["role"]),
            float(component["mass_kg"]),
            (float(component["x_mm"]), 0.0, float(component["z_mm"])),
            dimensions_mm=component["dimensions_mm"],
            inner_surface_keepout_mm=float(component["inner_surface_keepout_mm"] or default_keepout),
            placement_kind=str(component["placement_kind"]),
        )
        for component in placed
    ]
    return virtual_components, not issues, wiggle_room, issues, initial_cg_x


def _metric(value: float | int, unit: str | None) -> dict[str, object]:
    return MetricValue(
        value=round(value, 6) if isinstance(value, float) else value,
        unit=unit,
        confidence=0.25,
        source=MODULE_NAME,
    ).to_dict()
