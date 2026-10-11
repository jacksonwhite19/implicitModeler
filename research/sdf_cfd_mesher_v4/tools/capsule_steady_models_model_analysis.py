"""Canonical diagnostics for an admitted steady model; no solver or tuning."""
import hashlib
import sys
import numpy as np
from scipy.spatial import cKDTree
import capsule_steady_models_sa as sa
import capsule_steady_models_history as history
import capsule_pressure_diagnostic as p
s=sa.s;h=s.h

def collect(name):
    folder=s.ROOT/'model_runs'/name;case=folder/'pilot'
    contract=h.read(folder/'CFD_CONTRACT.json');h.verify_record(contract)
    model=contract['model'];source=sa.mesh_source(contract['mesh'])
    candidate=h.read(source/'CANDIDATE.json');h.verify_record(candidate)
    assert candidate['identity']==contract['candidate']
    assert candidate['frozen']==s.f.prior.frozen()
    for rel,digest in contract['controls'].items():assert h.file_hash(case/rel)==digest
    for rel,digest in contract['mesh_files'].items():assert h.file_hash(case/'constant/polyMesh'/rel)==digest
    initial=h.read(folder/'INITIAL_AUDIT.json');h.verify_record(initial)
    assert initial['contract']==contract['identity'] and initial['passed']
    for rel,digest in initial['files'].items():assert h.file_hash(case/rel)==digest
    assert (folder/'pilot.exit').read_text().strip()=='0'
    text=(folder/'pilot.log').read_text();assert p.l.u.e.warning_audit(text)
    rows=history.parse(text,model);assert len(rows)==500
    h.durable_json(folder/'HISTORY.json',dict(model=model,rows=rows))
    x=np.load(source/'assembly/coordinates.npy');pr=np.load(source/'assembly/prisms.npy');tet=np.load(source/'assembly/tets.npy')
    for key,a in [('coordinates',x),('prisms',pr),('tets',tet)]:assert h.array_hash(a)==candidate['assembly']['hashes'][key]
    centers=np.concatenate([x[pr].mean(1),x[tet].mean(1)]);n=len(centers)
    rankids=[p.l.labels(case/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
    assert np.array_equal(np.sort(np.concatenate(rankids)),np.arange(n))
    for i,ids in enumerate(rankids):assert np.array_equal(ids,p.l.labels(source/'pilot'/f'processor{i}/constant/polyMesh/cellProcAddressing'))
    fields={'U':3,'p':1,'nut':1,**{f:1 for f in history.TRANSPORT[model]}}
    checkpoints={}
    for step in (20,50,100,200,300,400,500):
        arrays={};stats={}
        for field,dim in fields.items():
            a=np.empty((n,dim))
            for i,ids in enumerate(rankids):a[ids]=p.l.u.e.read_internal(case/f'processor{i}'/str(step)/field,dim)
            arrays[field]=a;v=np.linalg.norm(a,axis=1) if dim==3 else a[:,0]
            stats[field]=dict(min=float(v.min()),max=float(v.max()),negative=int((v<0).sum()),nonfinite=int((~np.isfinite(a)).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
        checkpoints[str(step)]=stats
    assert all(v['nonfinite']==0 for c in checkpoints.values() for v in c.values())
    assert all(c[f]['negative']==0 for c in checkpoints.values() for f in ('nut',*history.TRANSPORT[model]))
    import capsule_steady_models_wall as wall
    start,total=wall.patch_layout(case/'constant/polyMesh/boundary');assert total==33786
    wall_values={f:np.full(total,np.nan) for f in ('p','nut',*history.TRANSPORT[model])}
    seen=np.zeros(total,dtype=int)
    for rank in range(6):
        proc=case/f'processor{rank}';first,count=wall.patch_layout(proc/'constant/polyMesh/boundary')
        ids=np.abs(wall.labels(proc/'constant/polyMesh/faceProcAddressing')[first:first+count])-1-start
        assert np.all((ids>=0)&(ids<total));seen[ids]+=1
        for field in wall_values:wall_values[field][ids]=wall.pressure_patch(proc) if field=='p' else wall.scalar_patch(proc/'500'/field,count)
    assert np.all(seen==1) and all(np.isfinite(v).all() for v in wall_values.values())
    if model=='SpalartAllmaras':assert np.all(np.array(wall_values['nuTilda'])==0)
    wall_archive=folder/'WALL_FIELDS.npz';np.savez_compressed(wall_archive,global_faces=np.arange(start,start+total),**wall_values)
    import capsule_gw_wake as wake
    points,per_station=wake.locations();distances,indices=cKDTree(centers).query(points)
    sampled={k:v[indices] for k,v in arrays.items()}
    archive=folder/'WAKE.npz';np.savez_compressed(archive,points=points,cell_ids=indices,distances=distances,**sampled)
    stations={}
    for j,station in enumerate((.5,1,2,4,6,8,10)):
        sl=slice(j*per_station,(j+1)*per_station);q=points[sl];u=sampled['U'][sl]
        r=wake.disk_metrics(q,u);r['sampling_distance']=s.f.statistics(distances[sl])
        r['centerline_U']=u[np.all(q[:,:2]==0,axis=1)][0].tolist()
        r['speed']=s.f.statistics(np.linalg.norm(u,axis=1))
        r['lateral_speed']=s.f.statistics(np.linalg.norm(u[:,:2],axis=1))
        r['sampled_fields']={k:s.f.statistics(v[sl]) for k,v in sampled.items() if k!='U'}
        stations[str(station)]=r
    result=h.seal(dict(schema='AdmittedSteadyModelMeasurements/1',contract=contract['identity'],model=model,mesh=contract['mesh'],iterations=500,qualifying=False,
        history=h.file_hash(folder/'HISTORY.json'),log=h.file_hash(folder/'pilot.log'),checkpoints=checkpoints,windows=history.windows(rows,model),
        wall_fields={f:dict(statistics=s.f.statistics(np.array(v)),sha256=h.array_hash(np.array(v))) for f,v in wall_values.items()},
        wall_archive_sha256=h.file_hash(wall_archive),wall_order='Global canonical boundary-face order; exact faceProcAddressing permutation',
        summary=dict(linear_totals={eq:sum(r['residuals'][eq]['iterations'] for r in rows) for eq in rows[0]['residuals']},bounds=[dict(iteration=r['iteration'],**b) for r in rows for b in r['bounds']],maxiter=[r['iteration'] for r in rows if any(v['iterations']>=5000 for v in r['residuals'].values())],flux_max=max(r['flux']['normalized'] for r in rows),wall_max=max(r['flux']['wall_sumMag'] for r in rows),last=rows[-1]),
        wake=dict(method='Same inherited nearest canonical centroid sampling and positive-deficit disk metrics; no new interpolation rule',point_count=len(points),points_per_station=per_station,archive_sha256=h.file_hash(archive),scientific_arrays={k:h.array_hash(v) for k,v in dict(points=points,cell_ids=indices,distances=distances,**sampled).items()},stations=stations)))
    h.durable_json(folder/'MEASUREMENTS.json',result);print(result['identity'])

if __name__=='__main__':collect(sys.argv[1])
