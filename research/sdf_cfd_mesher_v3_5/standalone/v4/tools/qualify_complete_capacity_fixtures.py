"""Two-layer fixture foundation qualification; no rolling builder changes."""
from pathlib import Path
import sys,os,json,time,hashlib
os.environ['OPENBLAS_NUM_THREADS']='1'
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import psutil
from types import SimpleNamespace
from complete_capacity_fixtures import *
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.geometry.front import build_topology
from sdf_cfd_mesher_v4.common.authority import array_hash
from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer
from sdf_cfd_mesher_v4.advancing_front.local_relaxation import numeric_gates
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_signs
from sdf_cfd_mesher_v4.forecast.capacity import CapacityForecast,first_limit,rational
from sdf_cfd_mesher_v4.forecast.detailed.patch import Patch,require_closure
from sdf_cfd_mesher_v4.certificate.production import surface_arrays
from sdf_cfd_mesher_v4.certificate.efficient import Filter
from sdf_cfd_mesher_v4.certificate.efficient_proof import prove
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
OUT=ROOT/'results/complete_capacity_fixtures'
def plain(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if isinstance(v,dict):return {str(k):plain(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [plain(x) for x in v]
    return v

def save(n,v):(OUT/n).write_text(json.dumps(plain(v),indent=2,sort_keys=True,allow_nan=False)+'\n')

def foundation(x,f,core,sdf,spec):
    top=build_topology(f,len(x));owners=[list(map(int,top.edge_triangles.row(i))) for i in range(len(top.edges))]
    assert all(len(v)==2 for v in owners)
    directed={}
    for tri in f:
        for a,b in zip(tri,np.roll(tri,-1)):
            key=tuple(sorted((int(a),int(b))));directed[key]=directed.get(key,0)+(1 if a<b else -1)
    assert all(v==0 for v in directed.values())
    assert len(np.unique(np.sort(f,axis=1),axis=0))==len(f)
    cross=np.cross(x[f[:,1]]-x[f[:,0]],x[f[:,2]]-x[f[:,0]])
    assert np.isfinite(x).all() and np.all(np.linalg.norm(cross,axis=1)>0)
    assert np.all(np.einsum('ij,ij->i',cross,sdf.evaluate(x[f].mean(1)).normal)>0)
    sample=sdf.evaluate(x);assert np.all(abs(sample.phi)<=sample.source_error_bound)
    if spec['kind']=='spheres' and not spec.get('cavity'):
        for i,c in enumerate(spec['centres']):
            for j in range(i):assert np.linalg.norm(np.array(c)-spec['centres'][j])>spec['radii'][i]+spec['radii'][j]
    edges=np.unique(top.triangle_edges[core]);columns=np.unique(np.r_[core,np.array(owners)[edges].ravel()]);v=np.unique(f[columns]);cv=np.unique(f[core])
    patch=Patch(core,columns,v,cv,edges,top.sha256);require_closure(patch,top)
    return top,patch,dict(passed=True,vertices=len(x),triangles=len(f),edges=len(top.edges),one_owner_edges=0,
        duplicate_triangles=0,zero_area_triangles=0,orientation_conflicts=0,minimum_double_area=float(np.linalg.norm(cross,axis=1).min()),
        source_ownership='Stable component/vertex/triangle IDs; all wall vertices satisfy authoritative fixture SDF',
        wall_residual_max=float(abs(sample.phi).max()),self_intersection_proof='Injective box/wedge/cylinder/frustum construction or radial octahedral subdivision; distinct sphere bounds disjoint',
        coordinate_hash=array_hash(x),topology_hash=top.sha256,sdf_identity=sdf.source_identity,
        geometry_identity=identity(spec),analytic_reference_identity=identity(spec['capacity']),
        patch=patch.record(),halo_vertices=np.setdiff1d(v,cv).tolist(),halo_triangles=np.setdiff1d(columns,core).tolist(),
        closure_only_triangles=np.setdiff1d(np.arange(len(f)),columns).tolist())

def certificate(x,top,patch,prefix):
    q,labels=surface_arrays(x,top.edges,prefix);native=Filter();E=len(top.edges);forest=native.forest(q,E);allpairs=forest.pairs(len(prefix));forest.close()
    aa=(allpairs>>np.uint64(32)).astype(np.int64);bb=(allpairs&np.uint64(0xffffffff)).astype(np.int64)
    # Every new-vs-prefix pair involving a measured core side, including core
    # sides at prior layers. Other geometry is only an interaction halo.
    selected=np.isin(aa%E,patch.side_edges)|np.isin(bb%E,patch.side_edges);pairs=allpairs[selected]
    core_indices=np.concatenate([k*E+patch.side_edges for k in range(len(prefix))]);valid=native.valid(q[core_indices]);invalid=[]
    for i in np.flatnonzero(valid==0):
        j=core_indices[i];ok,why=kernel.Patch.binary(q[j],tuple(map(int,labels[j]))).valid()
        if not ok:invalid.append(dict(side=int(j),reason=why))
    result,perf=prove(q,labels,pairs,native)
    bad=[r for r in result['fallback'] if r['result']['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact')]
    return dict(passed=not bad and not invalid,required_pairs=len(pairs),pair_hash=array_hash(pairs),outcome_hash=array_hash(result['outcomes']),
                code_hash=array_hash(result['codes']),proof_identity=identity(plain(result['fallback'])),exact_calls=perf['exact_calls'],invalid=invalid,failures=bad,
                completeness='All conservative new-vs-prefix candidates touching any core side; AABB-disjoint complement separated',
                scope='Core side continuous certificate, including interactions with complete closure geometry'),perf

def run(persist):
    start=time.perf_counter();s=load_schedule();records=[];timings=[];capacity=CapacityForecast()
    for spec in definitions(s.targets):
        tick=time.perf_counter();x,f,core=build(spec);sdf=FixtureSDF(spec);top,patch,base=foundation(x,f,core,sdf,spec)
        events=[]
        if spec['capacity'] is None and spec['kind'] in ('wedge','frustum'):
            local=spec.get('half_gap',spec.get('radius'));events=[dict(layer=first_limit(s.targets,rational(local)),metric='FixedStationOffsetDemand',station=0,scope='Station is not invariant under tangent motion')]
        if spec['capacity'] is None and spec['kind']=='spheres':
            for i,c in enumerate(spec['centres']):
                for j in range(i):
                    gap=float(np.linalg.norm(np.array(c)-spec['centres'][j])-spec['radii'][i]-spec['radii'][j]);layer=first_limit(s.targets,rational(gap)/2)
                    if layer is not None:events.append(dict(layer=layer,metric='opposing_nominal_front_contact',components=[j,i],spacing=gap))
        forecast=capacity.analyze(spec['name'],s,0,source=spec['capacity'],events=events,geometry_identity=base['coordinate_hash'],topology_identity=top.sha256)
        prefix=[];previous=None;smoke=[];ct=0.
        for layer in (1,2):
            low=x if not prefix else prefix[-1]
            try:
                with np.errstate(all='ignore'):prediction=predict_next_front(low,float(s.targets[layer-1]),sdf)
            except (RuntimeError,ValueError) as exc:
                row=dict(layer=layer,passed=False,projection_failure=str(exc),intended_capacity=forecast['predicted_first_infeasible_layer']==layer)
                if spec['name']=='near_contact':
                    h=spec['half'][0];target=float(s.targets[layer-1]);w=.0008
                    a=np.array([[-h,0,0],[-h,w,0],[-h+target,0,0],[-h+target,w,0]])
                    b=np.array([[h,0,0],[h,w,0],[h-target,0,0],[h-target,w,0]])
                    row['decisive_collision']=CertificateSession().pair(kernel.Patch.binary(a,tuple(range(4))),kernel.Patch.binary(b,tuple(range(4,8))))
                    row['collision_scope']='Two analytic opposing strips continued to impossible target; not an admitted SDF front'
                    assert row['decisive_collision']['classification']=='CertifiedForbiddenIntersection'
                smoke.append(row);break
            high=prediction.coordinates;metric=evaluate_layer(top,low,high,layer,previous)
            sides=patch.side_edges;faces={k:np.r_[v[sides],metric['caps'][k][core] if layer>1 else []] for k,v in metric['sides'].items()}
            ids=[('side',layer,int(e),0,0) for e in sides]+([('cap',layer-1,int(c),0,0) for c in core] if layer>1 else [])
            gate=numeric_gates(dict(cells={k:v[None,core] for k,v in metric['cells'].items()},faces=faces,face_ids=ids))
            exact=[exact_signs(np.concatenate((low[tri],high[tri]))) for tri in f[core]];exact_ok=all(r['jacobian_sign']>0 and r['volume_sign']>0 for r in exact)
            t=time.perf_counter();cert,perf=certificate(x,top,patch,prefix+[high]);ct+=time.perf_counter()-t
            passed=gate['passed'] and exact_ok and cert['passed'];smoke.append(dict(layer=layer,passed=passed,gate=gate,exact_positive=exact_ok,exact_hash=identity(exact),certificate=cert,
                coordinate_hash=array_hash(high),sdf_residual_max=float(abs(prediction.residual).max())))
            if not passed:break
            prefix.append(high);previous=metric['cells']
        if persist:
            np.savez_compressed(OUT/(spec['name']+'.npz'),wall=x,triangles=f,core=core,**{f'layer_{i+1}':v for i,v in enumerate(prefix)})
        records.append(dict(definition=spec,foundation=base,capacity=forecast,smoke=smoke,passed_smoke_layers=len(prefix),
            construction_ids='vertex ordinal, oriented triangle ordinal, sorted endpoint edge ordinal; topology immutable during smoke',
            qualified_scope='All core cells, every incident side with actual two owners, and interlayer caps; closure-only cells not qualified'))
        timings.append(dict(name=spec['name'],seconds=time.perf_counter()-tick,certificate_seconds=ct))
        print(spec['name'],len(f),len(core),len(prefix),flush=True)
    return records,dict(total_seconds=time.perf_counter()-start,fixtures=timings,peak_rss=psutil.Process().memory_info().peak_wset)

if __name__=='__main__':
    a,t=run(True);save('RESULTS.json',a);save('TIMINGS.json',t)
    b,u=run(False);save('REPLAY_TIMINGS.json',u);assert identity(plain(a))==identity(plain(b));save('DETERMINISM.json',dict(equal=True,scientific_hash=identity(plain(a))))
