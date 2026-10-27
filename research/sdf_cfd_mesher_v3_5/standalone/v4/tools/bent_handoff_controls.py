"""Bounded read-only controls, ray evidence and upper-tail prism metrics."""
import json,time
from collections import defaultdict
import numpy as np
from scipy.optimize import brentq
from bent_handoff_inputs import OUT,V4
from bent_handoff_study import event
from fixed_core_study import save
from core_handoff_audit import surface,distribution,pre_core_audit
from geometry_family_authority import Source
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash

def ray_forensics():
    m=json.loads((OUT/'MAX_EDGE.json').read_text());sp=json.loads((OUT/'SOURCE.json').read_text())['spec'];s=Source(sp)
    d=sp['components'][0];anchor=np.array(d['ray_anchor']);wx=np.array(m['wall_endpoint_coordinates']);dirs=wx-anchor;dirs/=np.linalg.norm(dirs,axis=1)[:,None]
    records=[]
    for name,direction in [('endpoint0',dirs[0]),('middle_ray',(dirs[0]+dirs[1])/np.linalg.norm(dirs[0]+dirs[1])),('endpoint1',dirs[1])]:
        grid=np.linspace(0,d['ray_bound'],8193);sample=s.evaluate(anchor+grid[:,None]*direction);phi=sample.phi
        idx=np.flatnonzero(phi[:-1]*phi[1:]<0)
        roots=[brentq(lambda t:float(s.evaluate((anchor+t*direction)[None]).phi[0]),grid[j],grid[j+1],xtol=1e-16) for j in idx]
        interval_points=(np.r_[0,roots]+np.r_[roots,d['ray_bound']])/2
        signs=s.evaluate(anchor+interval_points[:,None]*direction)
        records.append(dict(name=name,direction=direction,bracketed_roots=roots,root_coordinates=[anchor+t*direction for t in roots],sample_count=len(grid),
            interval_witnesses=interval_points,interval_phi=signs.phi,interval_source_error=signs.source_error_bound))
    save(OUT/'WALL_RAY_FORENSICS.json',dict(records=records,direction_separation_degrees=float(np.degrees(np.arccos(dirs[0]@dirs[1]))),
        method='All sign-changing brackets on 8193 fixed radial samples; not a proof of root completeness. Alternating interior/exterior witnesses establish multiple crossings by continuity.',
        source=sp,source_identity=s.source_identity,implementation='geometry_family_authority.radial_wall'))
    event('ray_complete')

def controls():
    paths={'capsule':V4/'results/production_scale_resume_fast/capsule_completion',
           'SF2':V4/'results/fixed_interface_bulk_core_repair'}
    result={}
    for key,path in paths.items():
        interface=path/('interface' if key=='capsule' else 'source/sf2_interface')
        r=pre_core_audit(interface);x=np.load(interface/'points.npy');f=np.load(interface/'connectivity.npy').reshape(-1,3);a=surface(x,f)
        if key=='capsule':
            qpath=path/'repaired_core/HARD_QUALITY.json';q=json.loads(qpath.read_text());sp=json.loads((path/'GEOMETRY_AUTHORITY.json').read_text())['spec']
            sealpath=V4/'results/production_scale_resume_fast/capsule_complete_retry/SEAL.json'
            geometry='Exterior capsule and enclosing box; no inward medial-core depth. Unlike the bent inward cavity.'
        else:
            qpath=path/'FINAL_CORE.json';q=json.loads(qpath.read_text());sp=None;sealpath=path/'CONTINUOUS.json'
            geometry='Exterior SF2 wing/support domain; source-bound inward medial depth not applicable. Unlike the bent inward cavity.'
        result[key]=dict(audit=r,source_spec=sp,geometry_difference=geometry,quality=q,quality_path=str(qpath),quality_sha256=file_hash(qpath),
            seal_path=str(sealpath),seal_sha256=file_hash(sealpath),seal=json.loads(sealpath.read_text()),
            edge_depth_comparison='Not measured; no common qualified definition for the exterior controls',
            coordinates_sha256=array_hash(x),connectivity_sha256=array_hash(f),
            triangle={k:distribution(a[k]) for k in ('longest_edge','area','minimum_angle','maximum_angle','radius_ratio')})
    save(OUT/'QUALIFIED_CONTROLS.json',result);event('qualified_controls_complete')

def columns():
    import local_termination_topology as top
    z=np.load(OUT/'SEALED_GEOMETRY.npz');p=z['points'];wall=z['wall'];n=int(wall.max())+1
    a=np.load(OUT/'TERMINAL_FIELDS.npz');globalids=a['levels']*n+a['source'];outer=[tuple(f) for f in globalids[a['faces']]]
    tail=a['edge_length']>=np.quantile(a['edge_length'],.99);selected_edges={tuple(e) for e in a['edges'][tail]}
    faces=[i for i,f in enumerate(a['faces']) if any(tuple(sorted((int(f[j]),int(f[(j+1)%3])))) in selected_edges for j in range(3))]
    columns=set(map(int,a['column'][faces]));cells=json.loads((OUT/'CELL_LINEAGE.json').read_text());selected={c['id'] for c in cells if c['kind']=='prism' and c['column'] in columns}
    owners,ownership=top.ownership(cells,outer,[tuple(reversed(f)) for f in wall]);event('upper_tail_quality_start',columns=len(columns),prisms=len(selected))
    quality=top.quality(p,cells,owners,1,active_cells=selected);assert quality['passed']
    byid={c['id']:c for c in cells};cellrows={r['id']:r for r in quality['cells']};faceby=defaultdict(list)
    for r in quality['faces']:
        for c in r['owners']:faceby[c].append(r)
    records=[]
    for ident in sorted(selected):
        c=byid[ident];q=p[c['vertices']];sides=np.linalg.norm(q[3:]-q[:3],axis=1)
        bottom=surface(q[:3],np.array([[0,1,2]]));up=surface(q[3:],np.array([[0,1,2]]));fs=faceby[ident]
        records.append(dict(**cellrows[ident],layer=c['layer'],source_column=c['column'],side_edges=sides,
            height_normal_projection=abs((q[3:]-q[:3])@bottom['normal'][0]),bottom_edges=bottom['edge_length'],top_edges=up['edge_length'],
            weight_min=min(r['weight'] for r in fs),ratio_min=min(r['symmetric_volume_ratio'] for r in fs),skew_max=max(r['skewness'] for r in fs)))
    save(OUT/'UPPER_TAIL_COLUMNS.json',records)
    save(OUT/'UPPER_TAIL_COLUMN_SUMMARY.json',dict(columns=len(columns),prisms=len(records),selection='Terminal unique-edge p99 tail, including ties; diagnostic population only',passed=True,
        minimum_determinant=min(r['determinant'] for r in records),minimum_weight=min(r['weight_min'] for r in records),minimum_ratio=min(r['ratio_min'] for r in records),
        maximum_skew=max(r['skew_max'] for r in records),maximum_aspect=max(r['aspect'] for r in records),exact_positive=all(r['volume_sign']>0 and r['jacobian_sign']>0 for r in quality['exact']),
        nonorthogonality='Not measured',explanation='Existing selected-face determinant and shell metric gates contain no interface-length/core-depth feasibility predicate.'))
    event('upper_tail_quality_complete')

if __name__=='__main__':
    ray_forensics();controls();columns()
