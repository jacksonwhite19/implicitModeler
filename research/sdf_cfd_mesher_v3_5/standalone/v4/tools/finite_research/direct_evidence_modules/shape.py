import numpy as np
from sdf_cfd_mesher_v4.common.authority import array_hash,identity
from finite_research.direct_evidence_modules import driver

def diagnostics(state, joined, policy, arrays, check, publish):
    from finite_support.transition_front_targets import tetrahedron_ratio, triangle_ratio, population_report
    from finite_research.joined_diagnostics import distribution
    from generic_core_sizing import triangle_distances, equal_volume_seed
    from itertools import combinations
    points=joined["points"]; rows=joined["bulk_tets"]; owners=joined["owners"]
    prefix_ids=set(joined["prefix_ids"])
    bulk_byid={cell["id"]:cell for cell in joined["cells"] if cell["id"] not in prefix_ids}
    adjacent=set()
    for face in state["outer"]:
        uses=owners[tuple(sorted(face))]
        old=[owner for owner,cycle in uses if owner in prefix_ids]
        new=[owner for owner,cycle in uses if owner not in prefix_ids]
        driver.require(len(uses)==2 and len(old)==len(new)==1, "actual Prism-tet shape population")
        adjacent.add(new[0])
    reports={}
    selections={"interface_tetrahedra":(np.asarray([bulk_byid[owner]["vertices"] for owner in sorted(adjacent)]), "outer_tetrahedra"),
        "all_bulk_tetrahedra":(rows,"outer_tetrahedra"),"preserved_cap_triangles":(state["outer"],"outer_triangles")}
    for name,(population,kind) in selections.items():
        scores=(triangle_ratio if kind=="outer_triangles" else tetrahedron_ratio)(points,population)
        driver.require(np.isfinite(scores).all() and np.all(scores>0), "positive finite shape scores")
        from finite_research.direct_fill_contract import classify
        report=classify(population_report(scores,policy["quality_targets"][kind]),policy["quality_targets"][kind])
        report["minimum_actual_count"]=int(np.count_nonzero(scores>=report["required_minimum"]))
        report["population_identity"]=array_hash(population)
        report["points_identity"]=array_hash(points)
        report["targets_identity"]=identity(policy["quality_targets"][kind])
        report["selected_ids"]=sorted(adjacent) if name=="interface_tetrahedra" else (
            [cell["id"] for cell in joined["cells"] if cell["id"] not in prefix_ids] if name=="all_bulk_tetrahedra"
            else list(range(len(state["outer"]))))
        reports[name]=report
    from finite_research.direct_fill_contract import binding,acceptance
    publish("SHAPE_COUNTS",dict(populations=reports,acceptance_contract=binding(policy),shape_applicability=acceptance(reports,policy),scientific_acceptance=False,
        role="Mandatory minimum floors and unchanged nonblocking percentage diagnostics on actual bulk/interface/cap populations; no independent transition band exists.",
        graded_outer_transition_gate="NOT_APPLICABLE_NO_TRANSITION_CONSTRUCTOR"))
    check()
    return reports
