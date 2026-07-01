import argparse
import json
from pathlib import Path


def fmt_bbox(bbox: dict) -> str:
    return (
        f"x={bbox['xmin']:.1f}..{bbox['xmax']:.1f}, "
        f"z={bbox['zmin']:.1f}..{bbox['zmax']:.1f}"
    )


def classify_context(item: dict) -> str:
    text = json.dumps(item).lower()
    if "inlet" in text or "duct" in text or "mouth" in text:
        return "inlet_flowpath"
    if "tail" in text or "outlet" in text or "aft" in text:
        return "aft_body"
    return "general"


def build_recommendations(report: dict) -> list[dict]:
    recs = []

    for rule in report.get("rules", []):
        if rule.get("pass", True):
            continue
        context = classify_context(rule)
        if rule.get("failure_mode") == "false_closure_or_extra_material":
            base = {
                "rule": rule["name"],
                "priority": "high",
                "location": {
                    "x": rule["x"],
                    "z": rule["z"],
                    "plane_y": report.get("plane_y", 0.0),
                },
                "failure_mode": rule["failure_mode"],
            }
            recs.append({
                **base,
                "action": "add_local_transition_opening",
                "why": "The STL is closed where the SDF is open.",
                "edit_strategy": (
                    "Add a short connector cut that reaches from the inlet mouth opening into the "
                    "start of the duct void, instead of subtracting only a shallow face box."
                ),
            })
            recs.append({
                **base,
                "action": "limit_cut_extent_by_station",
                "why": "A broad subtraction can damage side walls or aft geometry.",
                "edit_strategy": (
                    "Constrain the opening cut to an X-limited transition region around the mouth "
                    "and stop it before the aft duct run."
                ),
            })
            if context == "inlet_flowpath":
                recs.append({
                    **base,
                    "action": "export_opening_specific_oml_patch",
                    "why": "The current OML export likely contains cap/bridge geometry near the inlet lip.",
                    "edit_strategy": (
                        "Build a dedicated aero-export OML SDF for the inlet region that subtracts a "
                        "transition opening volume from `body_outer` / `inlet_outer`, while leaving the "
                        "rest of the aircraft OML untouched."
                    ),
                })
        elif rule.get("failure_mode") == "lost_wall_or_missing_surface":
            recs.append({
                "rule": rule["name"],
                "priority": "high",
                "location": {
                    "x": rule["x"],
                    "z": rule["z"],
                    "plane_y": report.get("plane_y", 0.0),
                },
                "failure_mode": rule["failure_mode"],
                "action": "reduce_opening_cut",
                "why": "The STL removed wall where the SDF is still solid.",
                "edit_strategy": (
                    "Shrink the opening cutter locally in width/height or move it forward so it no "
                    "longer clips adjacent outer shell structure."
                ),
            })
        elif rule.get("failure_mode") == "surface_intrudes_into_required_open_region":
            recs.append({
                "rule": rule["name"],
                "priority": "high",
                "location": {
                    "x": rule["x"],
                    "z": rule["z"],
                    "plane_y": report.get("plane_y", 0.0),
                },
                "failure_mode": rule["failure_mode"],
                "action": "increase_local_clearance",
                "why": "A section segment sits too close to a point that should remain open.",
                "edit_strategy": (
                    "Widen or shift the local opening cut in the immediate mouth/transition region, "
                    "or move the rule probe deeper into the passage if the current point is on the lip."
                ),
            })
        elif rule.get("failure_mode") == "expected_surface_missing_near_probe":
            recs.append({
                "rule": rule["name"],
                "priority": "high",
                "location": {
                    "x": rule["x"],
                    "z": rule["z"],
                    "plane_y": report.get("plane_y", 0.0),
                },
                "failure_mode": rule["failure_mode"],
                "action": "restore_local_surface_presence",
                "why": "No sliced STL surface was found near a point that should lie on a wall.",
                "edit_strategy": (
                    "Restore the local shell or move/trim the export opening so a wall contour remains "
                    "near this station."
                ),
            })

    for region in report.get("actionable_regions", {}).get("stl_only", [])[:5]:
        context = classify_context(region)
        recs.append({
            "priority": "medium",
            "region_type": "stl_only",
            "bbox": region["bbox"],
            "centroid": region["centroid"],
            "action": "inspect_false_closure_region",
            "why": region["meaning"],
            "edit_strategy": (
                "Inspect the export SDF in this bounding box for a cap, overlap, or bridging surface. "
                "If this is near an inlet/duct, add a section rule there and trim the opening subtraction "
                "to just this region."
            ),
            "context": context,
        })

    for region in report.get("actionable_regions", {}).get("sdf_only", [])[:5]:
        context = classify_context(region)
        recs.append({
            "priority": "medium",
            "region_type": "sdf_only",
            "bbox": region["bbox"],
            "centroid": region["centroid"],
            "action": "inspect_lost_surface_region",
            "why": region["meaning"],
            "edit_strategy": (
                "Inspect the export SDF in this bounding box for over-aggressive subtraction or patch "
                "filtering. Restore shell material locally and re-run the section check."
            ),
            "context": context,
        })

    return recs


def main() -> int:
    ap = argparse.ArgumentParser(description="Turn section-check failures into concrete repair suggestions.")
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    report = json.loads(args.report.read_text())
    recommendations = build_recommendations(report)
    output = {
        "source_report": str(args.report),
        "pass": report.get("pass", False),
        "summary": {
            "rule_failures": len([r for r in report.get("rules", []) if not r.get("pass", True)]),
            "recommendation_count": len(recommendations),
        },
        "recommendations": recommendations,
    }

    if args.out:
        args.out.write_text(json.dumps(output, indent=2))
        print(f"saved={args.out}")
    else:
        print(json.dumps(output, indent=2))

    return 0 if report.get("pass", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
