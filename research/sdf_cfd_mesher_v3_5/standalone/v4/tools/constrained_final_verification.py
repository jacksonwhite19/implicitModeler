"""Independent retained arrays, local proof coverage, volume, and input hashes."""
import time,json
from pathlib import Path
import numpy as np
from fractions import Fraction as F
from constrained_cavity_audit import OUT,V4,digest
from fixed_core_study import save,load_core,identity,PATS
from constrained_cavity import exact_volume
from constrained_cavity_qualify import unique_faces

def independent_coverage(x,t,interface,meta,proofs):
    faces=unique_faces(t);index={tuple(sorted(f)):i for i,f in enumerate(faces)}
    active={index[tuple(sorted(f))] for f in unique_faces(t[-meta['replacement_tets']:])}
    coords=list(x);lookup={p.tobytes():i for i,p in enumerate(x)}
    patterns={4:PATS.tolist(),5:[[0,3,2,1],[0,1,4],[1,2,4],[2,3,4],[3,0,4]],6:[[0,2,1],[3,4,5],[0,1,4,3],[1,2,5,4],[2,0,3,5]]}
    for p in sorted(interface.glob('owner_geometry_*.npy')):
        for cell in np.load(p):
            ids=[]
            for v in cell:
                if v.tobytes() not in lookup:lookup[v.tobytes()]=len(coords);coords.append(v)
                ids.append(lookup[v.tobytes()])
            for pat in patterns[cell.shape[0]]:
                f=tuple(ids[j] for j in pat);key=tuple(sorted(f))
                if key not in index:index[key]=len(faces);faces.append(f)
    coords=np.array(coords);lo=np.array([coords[list(f)].min(0) for f in faces]);hi=np.array([coords[list(f)].max(0) for f in faces])
    # Independent direct scan, no production spatial index traversal.
    expected=set()
    for i in active:
        for j in np.flatnonzero(np.all(hi>=lo[i],axis=1)&np.all(lo<=hi[i],axis=1)):
            if i!=j:expected.add(tuple(sorted((i,int(j)))))
    actual=[tuple(r['pair']) for r in proofs]
    return dict(expected=len(expected),actual=len(actual),missing=len(expected-set(actual)),extra=len(set(actual)-expected),duplicate=len(actual)-len(set(actual)),active_faces=len(active),total_faces=len(faces))

def run():
    start=time.perf_counter();src=V4/'results/bent_terminal_shell_fixed_interface_core/final_stars';iface=V4/'results/terminal_capacity_aware_remesh_acceptance/interface'
    x0,t0,ci0,interface=load_core(src,iface);x,t,ci,_=load_core(OUT/'accepted_local',iface)
    artifacts=[]
    for name in ('terminal_capacity_aware_remesh_acceptance','bent_terminal_shell_fixed_interface_core'):
        p=V4/'results'/name;a=json.loads((p/'AUTHORITY.json').read_text());bad=[]
        for r in a['artifacts']:
            f=p/r['path']
            if digest(f)!=r['sha256'] or f.stat().st_size!=r['bytes']:bad.append(r['path'])
        artifacts.append(dict(root=name,verified=len(a['artifacts']),mismatches=bad))
    replay={}
    for name in ('coordinates','tets','node_tags','interface_tags','interface_owner'):
        a=np.load(OUT/f'accepted_local/{name}.npy');b=np.load(OUT/f'replay_local/{name}.npy');replay[name]=bool(np.array_equal(a,b))
    a=np.load(OUT/'accepted_local/METRICS.npz');b=np.load(OUT/'replay_local/METRICS.npz');replay['all_quality_arrays']=all(np.array_equal(a[k],b[k]) for k in a.files)
    a=json.loads((OUT/'accepted_local/LOCAL_CERTIFICATE.json').read_text());b=json.loads((OUT/'replay_local/LOCAL_CERTIFICATE.json').read_text());replay['all_proof_records']=a['proofs']==b['proofs'];replay['proof_hash']=a['proof_hash']==b['proof_hash']
    for original,repeated in [('g0_r3_safe_one/optimized_CANDIDATE.npz','replay_g0/optimized_CANDIDATE.npz'),('g1_r3_safe_multi/optimized_CANDIDATE.npz','replay_g1/optimized_CANDIDATE.npz'),('g1_margin/CANDIDATE.npz','replay_margin/CANDIDATE.npz')]:
        aa=np.load(OUT/original);bb=np.load(OUT/repeated);replay[original]=all(np.array_equal(aa[k],bb[k]) for k in aa.files)
    for original,repeated in [('g0_r3_safe_one','replay_g0'),('g1_r3_safe_multi','replay_g1')]:
        replay[original+'/initial_steiner']=bool(np.array_equal(np.load(OUT/original/'STEINER.npy'),np.load(OUT/repeated/'STEINER.npy')))
    tick=time.perf_counter();v=[exact_volume(q) for q in x[t]];prior=sum(exact_volume(q) for q in x0[t0]);positive=all(z>0 for z in v)
    # Exact oriented closed surface formula, independently of tet summation.
    points=[[F(float(z)) for z in p] for p in interface.x]
    def det(q):return 6*exact_volume(np.vstack((np.zeros(3),q)))
    bv=sum(det(interface.x[f[::-1]])/6 for f in interface.f)
    volume_seconds=time.perf_counter()-tick
    coverage=independent_coverage(x,t,iface,json.loads((OUT/'accepted_local/RESULT.json').read_text()),a['proofs'])
    assert not any(coverage[k] for k in ('missing','extra','duplicate'))
    result=dict(identity=identity(x,t),interface_coordinates_exact=np.array_equal(x[ci].view('u8'),interface.x.view('u8')),interface_connectivity_orientation_exact=len(interface.mapping(x,t,ci))==len(interface.f),interface_vertices=len(ci),interface_faces=len(interface.f),replay=replay,prior_artifacts=artifacts,exact_positive_tets=sum(z>0 for z in v),exact_total_volume=str(sum(v)),exact_boundary_volume=str(bv),exact_total_equal_prior=sum(v)==prior,exact_volume_balance=sum(v)==bv,exact_volume_seconds=volume_seconds,independent_coverage=coverage,local_required_relations=a['required_pairs'],local_duplicate_pairs=len(a['proofs'])-len({tuple(r['pair']) for r in a['proofs']}),local_forbidden=sum(r['classification']=='CertifiedForbiddenIntersection' for r in a['proofs']),local_unresolved=sum(r['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact','CertifiedForbiddenIntersection') for r in a['proofs']),seconds=time.perf_counter()-start)
    assert positive and sum(v)==prior==bv and all(replay.values()) and not any(z['mismatches'] for z in artifacts)
    save(OUT/'FINAL_VERIFICATION.json',result);print(json.dumps(result),flush=True)

if __name__=='__main__':run()
