"""Seal executable dependency evidence and local wall descriptors."""
import sys
sys.dont_write_bytecode=True
import json
import numpy as np
from first_layer_determinant_trace import OUT,g,production,geometry
from sdf_cfd_mesher_v4.common.authority import file_hash


def run():
    root=OUT.parents[1]
    foundation=root.parent/'sdf_cfd_mesher_v3/short_wing_displacement_contract_2026-09-06/evidence/Foundation_primitiveMeshCheck.C'
    sources=[foundation,root/'tools/local_termination_topology.py',root/'tools/final_shell_incidence.py',
             root/'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_support/prism_feasibility_contract/prospective_commit_adapter.py']
    g.save(OUT/'METRIC_DEPENDENCIES.json',dict(
        sources={str(p):file_hash(p) for p in sources},
        formula='abs(det(sum_f outer(S_f/mean_selected_norm,S_f/mean_selected_norm)))',
        dimension=3,threshold=.0011,
        determinant=dict(face_area_vectors='Current cell coordinates and oriented face connectivity',
            selection='Internal or coupled face role in the declared final domain',
            normalization='Mean magnitude over precisely the selected faces',
            cell_centres=False,face_centres=False,neighbor_centres=False,neighbor_volume=False,
            wall_cap='Uncoupled wall excluded',lateral='Actual adjacent shell cells required',
            top_cap='Declared internal continuation/core interface; exact role closure before commit'),
        pair_metrics=dict(weight='Actual face area/centre and owner/neighbor centres',
            skewness='Actual face vertices, centre, area, and owner/neighbor centres',
            symmetric_volume_ratio='Actual owner/neighbor volumes',
            nonorthogonality='Actual area and owner-to-neighbor vector'),
        unavailable_pair_metrics='No artificial top-cap weight/skewness receipt is issued; new actual pairs are gated on subsequent assembly',
        mixed_transitions='Existing bounded provisional core closure and complete-neighbor validation unchanged',
        authoritative_scope='ProspectiveFirstPrismDeterminant, not CompleteNeighborQuality'))
    results=[]
    for spec in production.cohort()+[s for s in geometry.cohort() if s['name']=='rounded_box__scale_0p25']:
        x,t=geometry.build(spec)
        trace_name='ROUNDED_BOX_TRACE.json' if spec['name']=='rounded_box__scale_0p25' else spec['name']+'_TRACE.json'
        cell=json.loads((OUT/trace_name).read_text())['reproducers'][0]['cell'];i=cell['column'];ids=t[i]
        normal=geometry.Source(spec).evaluate(x[ids]).normal
        normal=normal/np.linalg.norm(normal,axis=1)[:,None]
        angles=[float(np.degrees(np.arccos(np.clip(np.dot(normal[j],normal[(j+1)%3]),-1,1)))) for j in range(3)]
        edges=[float(np.linalg.norm(x[ids[j]]-x[ids[(j+1)%3]])) for j in range(3)]
        valence=[int(np.sum(np.any(t==v,axis=1))) for v in ids]
        results.append(dict(case=spec['name'],column=i,wall_vertex_valence=valence,wall_edges=edges,
            edge_ratio=max(edges)/min(edges),source_normal_angles_degrees=angles))
    from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall
    wall=load_sf2_wall();r=json.loads((OUT/'SF2_TRACE.json').read_text())['reproducers'][0]
    pts=np.asarray(r['coordinates']);directions=pts[3:]-pts[:3]
    directions/=np.linalg.norm(directions,axis=1)[:,None]
    ids=wall.triangles[r['cell']['column']]
    edges=[float(np.linalg.norm(pts[j]-pts[(j+1)%3])) for j in range(3)]
    results.append(dict(case='SF2',column=r['cell']['column'],wall_edges=edges,edge_ratio=max(edges)/min(edges),
        wall_vertex_valence=[int(np.sum(np.any(wall.triangles==v,axis=1))) for v in ids],
        accepted_extrusion_direction_angles_degrees=[float(np.degrees(np.arccos(np.clip(np.dot(directions[j],directions[(j+1)%3]),-1,1)))) for j in range(3)]))
    g.save(OUT/'WALL_DESCRIPTORS.json',results)

if __name__=='__main__':run()
