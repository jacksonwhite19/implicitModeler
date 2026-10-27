"""Compact geometric descriptors from the preserved rejected candidate."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import geometry_family_authority as geometry
from investigate_large_fallback import save


def main():
    root=ROOT/'results/bent_passage_large_case_fallback';nominal=root/'nominal'
    data=json.loads((nominal/'NOMINAL.json').read_text());z=np.load(nominal/'REPRODUCER.npz')
    spec=json.loads((ROOT/'results/production_scale_resume_fast/bent_continuation/GEOMETRY_AUTHORITY.json').read_text())['spec']
    source=geometry.Source(spec);wall=z['wall'];tri=z['triangles'];fronts=z['fronts']
    top=geometry.q.build_topology(tri,len(wall)) if hasattr(geometry,'q') else None
    rows=[]
    for component in data['components']:
        ids=component['source_vertices'];cols=component['source_columns'];faces=tri[cols]
        sample=source.evaluate(fronts[-1][ids]);old=source.evaluate(fronts[-2][ids]);ww=source.evaluate(wall[ids])
        normals=sample.gradient;pair_angles=[];edges=[]
        local={v:i for i,v in enumerate(ids)}
        for face in faces:
            for a,b in zip(face,np.roll(face,-1)):
                i,j=local[int(a)],local[int(b)];den=np.linalg.norm(normals[i])*np.linalg.norm(normals[j])
                pair_angles.append(float(np.arccos(np.clip(np.dot(normals[i],normals[j])/den,-1,1))))
                edges.append(float(np.linalg.norm(fronts[-1][a]-fronts[-1][b])))
        rows.append(dict(cells=component['cells'],columns=cols,bounds=component['bounds'],
            branch_ids=sample.boundary_normal_selection.tolist(),previous_branch_ids=old.boundary_normal_selection.tolist(),
            wall_branch_ids=ww.boundary_normal_selection.tolist(),edge_lengths=edges,normal_angles_radians=pair_angles,
            discrete_normal_variation_per_m=[a/e if e else None for a,e in zip(pair_angles,edges)],
            descriptor_scope='Measured normal variation over edges; not an analytic curvature or new gate',
            sdf_values=sample.phi.tolist(),sdf_bounds=sample.source_error_bound.tolist(),
            heights=component['heights'],quality=component['quality'],exact=component['exact'],faces=component['faces']))
    save(root/'LOCAL_FORENSICS.json',dict(components=rows,scope='Rejected layer 19; no exact collision certificate was reached because hard quality failed',
         exact_fold_evidence=[r for r in data['failures'] if r['kind']=='exact_cell'],
         target_level_set_admission=data['sdf'],coupled_core_failure='Not evaluated; no mixed candidate admitted'))
    print(json.dumps(dict(components=len(rows),branch_changes=sum(a!=b for r in rows for a,b in zip(r['branch_ids'],r['previous_branch_ids'])),
        minimum_edge=min(e for r in rows for e in r['edge_lengths']),maximum_normal_angle=max(a for r in rows for a in r['normal_angles_radians']))))

if __name__=='__main__':main()
