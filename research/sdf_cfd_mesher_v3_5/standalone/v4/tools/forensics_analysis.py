"""Direct mechanism evidence and deterministic summaries of the forensic run."""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'));sys.dont_write_bytecode=True
import numpy as np
from task2_describe import decode,read as task2read
from forensics_intersection import load_coordinates
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.detailed.metrics import cell_metrics
from sdf_cfd_mesher_v4.runtime.frozen_imports import legacy_modules
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
from sdf_cfd_mesher_v4.forecast.forensics import cell_events,face_events,first_invalid
OUT=ROOT/'results/nominal_forecast_forensics'
def write(name,v):(OUT/name).write_text(json.dumps(plain(v),indent=2,sort_keys=True)+'\n')
def calculate(s,front,top,x):
    q=legacy_modules()['component_quality'];geometry=[]
    # Both false-negative interfaces: actual centers on both sides, no truncation.
    for columns in [(19138,103774),(58145,79052)]:
        edge=int(np.intersect1d(top.triangle_edges[columns[0]],top.triangle_edges[columns[1]])[0]);a,b=top.edges[edge];records=[]
        for k in range(1,21):
            f=top.triangles[list(columns)];p=np.concatenate((x[k-1,f],x[k,f]),axis=1);C,V=q.FG.prism_cell_centres(p)
            face=np.array([[x[k-1,a],x[k-1,b],x[k,b],x[k,a]]]);S,F=q.FG.face_area_centre_quad(face);area=np.linalg.norm(S[0]);da=abs(S[0]@(F[0]-C[0]))/area;db=abs(S[0]@(C[1]-F[0]))/area
            w,sk,vr,no=q.face_metrics(face,S,F,C[:1],C[1:],V[:1],V[1:])
            records.append(dict(layer=k,owner_normal_distance=float(da),neighbor_normal_distance=float(db),weight=float(w[0]),face_area=float(area),owner_volume=float(V[0]),neighbor_volume=float(V[1]),owner_centre=C[0],neighbor_centre=C[1],face_centre=F[0]))
        geometry.append(dict(columns=list(columns),edge=edge,vertices=top.edges[edge],history=records))
    mask_history=[]
    for k in (1,2,3):
        f=top.triangles[58145];p=np.concatenate((x[k-1,f],x[k,f]))[None]
        mask_history.append(dict(column=58145,layer=k,actual_determinant=float(q.determinant(p,k)[0]),all_faces_expression=float(q.PM._foundation_determinant(p,lower_cap_is_uncoupled_wall=False)[0]),wall_excluded_expression=float(q.PM._foundation_determinant(p,lower_cap_is_uncoupled_wall=True)[0]),actual_lower_cap_is_wall=k==1))
    descriptors=['edge_ratio_max','triangle_area_ratio','neighbor_normal_angle_max_degrees','section_curvature_max','successive_direction_change_max_degrees','sdf_residual_abs_max','normal_convergence_max']
    old=task2read('DIRECT_PRISM_ROWS.json');precursors=[];trajectories=[];causes=[]
    mechanisms={
     'component_00':'Positive-span leading-edge cap: mixed cap-normal and section-normal trajectories. Some initial vertical edges have exactly zero wall-normal motion. Adjacent pure-translation and expanding cells develop unequal face-normal center distances and fail interpolation weight at layer 18.',
     'component_01':'Negative-span trailing-edge cap: columns 78681 and 78682 remain exactly in y=-0.25. Degenerate neighbor centers explain the nine unavailable face measures. The two retained side images intersect across layers even though their sampled endpoints meet SDF target distances.',
     'component_02':'Negative-span aft cap: column 78678 remains exactly in y=-0.25 and collapses in volume and reconstruction rank. Neighbor 38589 mixes opposite section-normal directions and cap-normal motion and has exact negative wedge Jacobians through layer 15.',
     'component_03':'Trailing-face/upper-surface junction: uniform +x motion on column 79052 neighbors differential section-normal motion on 58145. Their shared face has increasing center-distance imbalance. Layer-2 determinant uses the correct interior ownership mask and crosses the retained gate while sigma increases.',
     'component_04':'Negative-span leading-edge cap: two vertices move tangent to the wall cap and one moves along its normal. The first wedge has a zero corner Jacobian, while later wedge Jacobians are positive.'}
    for c in s['components']:
        cid=c['component'];cols=set(c['columns']);h=[r for r in s['cell_histories'] if r['component']==cid]
        investigated_events=[dict(kind=kind,layers=[r['layer']]) for r in h for kind in cell_events(r)]
        investigated_events += [dict(kind=kind,layers=[r['layer']] if r['kind']=='side' else [r['layer'],r['layer']+1]) for r in c['face_histories'] for kind in face_events(r)]
        investigated_events += [r for r in c['observed_events'] if r['kind']=='exact_intersection']
        for k in range(1,21):
            values=[r for r in h if r['layer']==k];prior=[r for r in old if r['column'] in cols and r['layer']==k]
            fields={name:dict(min=float(min(r[name] for r in values)),max=float(max(r[name] for r in values))) for name in ('signed_volume','jacobian','determinant','sigma','side_normal_separation_degrees','front_triangle_area')}
            faces=[r for r in c['face_histories'] if r['layer']==k and r['kind']=='side'];valid=[r['weight'] for r in faces if np.isfinite(r['weight'])]
            fields['internal_side_weight']=dict(min=min(valid) if valid else None,undefined=sum(not np.isfinite(r['weight']) for r in faces))
            gains=[r['G_bound'] for r in values if r['G_bound_available']]
            fields['G_bound']=dict(max=max(gains) if gains else None,unavailable=sum(not r['G_bound_available'] for r in values))
            trajectories.append(dict(component=cid,layer=k,metrics=fields))
            if prior:
                precursors.append(dict(component=cid,layer=k,columns=[r['column'] for r in prior],descriptor_ranges={d:[min(r[d] for r in prior),max(r[d] for r in prior)] for d in descriptors}))
        causes.append(dict(component=cid,classification='GenuineNominalForecastHazard',supported_mechanism=mechanisms[cid],
                           evidence=dict(original_metric_replay_exact=True,halo_expansion_invariant=True,coordinates_and_topology_unchanged=True,sdf_local_archive_replay_exact=True,coplanar_columns=c['all_forecast_vertices_coplanar'],first_observed_invalid=c['first_invalid_by_kind'],first_invalid_in_complete_investigated_support=first_invalid(investigated_events)),
                           unresolved=['No medial-axis distance is available or estimated.','No causal claim of general offset-surface singularity; only these discrete nominal lineages are diagnosed.','No feasible correction or required intervention threshold is proved.']))
    return plain(dict(interface_distance_histories=geometry,determinant_ownership_mask_diagnostic=mask_history,component_trajectories=trajectories,descriptor_precursors=precursors,cause_classifications=causes,
                      boundary_scope=dict(classification='BoundaryOrClosureArtifact',scope='Task-2 outer caps have no exterior core-mesh neighbors; 68 entries remain unavailable. These are separate from the nine degenerate-neighbor internal faces.'),
                      implementation_defects=dict(material_geometry_metric_or_certificate_defects=0,reporting_policy_correction='Task-2 stopped on a nominal forecast intersection. Forecast evidence is now reported as PredictedFutureIntersection; original sealed output is unchanged.')))
if __name__=='__main__':
    t=time.perf_counter();s=decode(json.loads((OUT/'SCIENTIFIC_OUTPUTS.json').read_text()));front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));x=load_coordinates(front)
    a=calculate(s,front,top,x);b=calculate(s,front,top,x);assert scientific_hash(a)==scientific_hash(b)
    write('MECHANISM_ANALYSIS.json',a)
    prior=json.loads((OUT/'DETERMINISM.json').read_text());final1=scientific_hash(dict(forensics=s,analysis=a));final2=scientific_hash(dict(forensics=s,analysis=b))
    assert final1==final2;write('FINAL_DETERMINISM.json',dict(forensic_replay=prior,final_hash_run1=final1,final_hash_run2=final2,passed=True,scope='Forensic replay plus all final calculated mechanism evidence and classifications'))
    write('ANALYSIS_TIMING.json',dict(seconds=time.perf_counter()-t));print(json.dumps(dict(final_hash_run1=final1,final_hash_run2=final2)))
