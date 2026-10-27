"""Task-3E private transactions. No production front or publisher is changed.

The immediate layer has complete numeric, exact-sign and continuous gates.
Future coordinates are diagnostic. A complete future continuous proof is not
an admission dependency. All numerical definitions come from preserved code.
"""
from dataclasses import dataclass, replace
from types import SimpleNamespace
import time
import hashlib,json
import numpy as np
from ..common.authority import array_hash, identity, frozen
from ..forecast.engine import vertex_trajectories
from ..forecast.descriptors import edge_descriptors, triangle_descriptors
from ..forecast.forensics import exact_wedge
from ..forecast.detailed.continuous import SideIndex
from ..certificate.incremental import build_scope, pair_identity
from ..certificate.continuous_prism_certificate.runtime import canonical_bytes
from ..runtime.regions import dependencies
from .local_relaxation import CoordinateOverlay, numeric_gates, no_new_earlier_failure, project_tangent_move

VERSION='Task3EPrivateRolling/1'

def try_tangent_proposal(current, field, distances, sdf, target):
    """Source-defined projection failures reject a proposal, not the study."""
    try:return project_tangent_move(current,field,distances,sdf,target)
    except RuntimeError as error:
        if str(error) not in ('ProjectionIterationCap','ProjectionBinary64Stagnation','NonfiniteCoordinate'):raise
        return None,dict(projection_failed=True,reason=str(error),base=current,field=field,distances=distances,target=target)

@dataclass(frozen=True)
class PrivateState:
    vertex_ids: np.ndarray
    prefix: tuple=()
    proof_hashes: tuple=()

    @property
    def layer(self): return len(self.prefix)

    @property
    def sha256(self):
        return identity(dict(version=VERSION,ids=array_hash(self.vertex_ids),prefix=[array_hash(x) for x in self.prefix],proofs=self.proof_hashes))

    def accept(self, coordinates, decision, proof_hash):
        if not decision.get('admissible'): raise ValueError('PrivateAdmissionRejected')
        a=np.asarray(coordinates)
        if a.shape!=(len(self.vertex_ids),3) or not np.isfinite(a).all(): raise ValueError('PrivateCoordinateShape')
        return PrivateState(self.vertex_ids,self.prefix+(frozen(a,'<f8'),),self.proof_hashes+(proof_hash,))


def immediate_admission(numeric, exact, certificate, earlier, maintained):
    reasons=[]
    if not numeric['passed'] or not exact['passed']: reasons.append('ImmediateHardGate')
    required=('complete','candidate_pairs','completed_pairs','forbidden','unresolved','invalid_surfaces')
    complete=all(k in certificate for k in required) and certificate['complete'] and certificate['candidate_pairs']==certificate['completed_pairs']
    if not complete: reasons.append('ImmediateCertificateIncomplete')
    for k in ('forbidden','unresolved','invalid_surfaces'):
        if certificate.get(k): reasons.append('ImmediateCertificate:'+k)
    if earlier: reasons.append('NewEarlierFutureFailure')
    if not maintained: reasons.append('FutureTrajectoryRegressed')
    return dict(admissible=not reasons,reasons=reasons,production_committed_layer=0,scope='private study only')


def immediate_subset(result, layer):
    """Include the new side faces AND the interface with the prior layer.

    The top exposed face has no physical neighbor. Its interpolation operator
    becomes defined only when the next cell exists. At that time it is gated.
    """
    if not 1<=layer<=20: raise ValueError('LayerRange')
    face_ids=result['face_ids']
    indices=[i for i,(kind,k,*_) in enumerate(face_ids) if (kind=='side' and k==layer) or (kind=='cap' and k==layer-1)]
    return dict(cells={k:v[layer-1:layer] for k,v in result['cells'].items()},
                faces={k:v[indices] for k,v in result['faces'].items()},face_ids=[face_ids[i] for i in indices])


def immediate_numeric(result, layer):
    out=numeric_gates(immediate_subset(result,layer))
    out.update(layer=layer,scope='All verification cells, new internal side faces, and the new interface with the previous private layer')
    return out


def exact_layer(front, top, forecast, patch, layer, *, cache=None):
    from ..forecast.detailed.exact_cache import LOCAL_EXACT
    cache=LOCAL_EXACT if cache is None else cache
    ids=top.triangles[patch.core_columns]
    low=front.coordinates[ids] if layer==1 else forecast.vertex['coordinates'][layer-2,ids]
    high=forecast.vertex['coordinates'][layer-1,ids]
    records=[dict(column=int(c),layer=layer,**cache.evaluate(np.concatenate((a,b)))) for c,a,b in zip(patch.core_columns,low,high)]
    return dict(passed=all(r['jacobian_sign']>0 and r['volume_sign']>0 for r in records),records=records)


def future_failures(result, after):
    """Global layer IDs; cap metric first exists when its upper cell exists."""
    out={};c=result['cells']
    for metric,fn in [('volume',lambda x:x<=0),('signed_volume',lambda x:x<=0),('jacobian',lambda x:x<=0),('determinant',lambda x:x<.0011),('aspect',lambda x:x>1000)]:
        bad=fn(c[metric])|~np.isfinite(c[metric])
        for k,j in zip(*np.where(bad)):
            if k+1<=after: continue
            key=f'cell:{int(c["column"][k,j])}:{metric}';out[key]=min(out.get(key,21),int(k+1))
    for i,(kind,k,entity,*_) in enumerate(result['face_ids']):
        available=k if kind=='side' else k+1
        if available<=after: continue
        for metric,fn in [('weight',lambda x:x<.055),('skewness',lambda x:x>3.63636),('symmetric_volume_ratio',lambda x:x<.01)]:
            x=result['faces'][metric][i]
            if fn(x) or not np.isfinite(x):
                key=f'{kind}:{entity}:{metric}';out[key]=min(out.get(key,21),int(available))
    return out


def target_trajectory(result, columns, metric, after):
    rows=[];c=result['cells'];chosen=np.isin(c['column'][0],columns)
    for layer in range(after+1,21):
        if metric=='weight':
            ii=[i for i,(kind,k,_,a,b) in enumerate(result['face_ids']) if (k if kind=='side' else k+1)==layer and (a in columns or b in columns)]
            values=result['faces']['weight'][ii];bad=(values<.055)|~np.isfinite(values)
        else:
            values=c[metric][layer-1,chosen];bad=(values<=0)|~np.isfinite(values)
        rows.append(dict(layer=layer,minimum=float(np.min(values)),violations=int(bad.sum())))
    return dict(metric=metric,rows=rows,first_invalid=next((r['layer'] for r in rows if r['violations']),None),minimum=min((r['minimum'] for r in rows),default=None))


def maintained_target(before,after):
    a=before['first_invalid'] or 21;b=after['first_invalid'] or 21
    if a!=b: return b>a
    # A removed hazard remains removed even if its positive diagnostic tail
    # changes. Do not turn a previous positive value into a new hard floor.
    if a==21:return True
    # Compare the mechanism's lower tail only when the first invalidity ties.
    # No mixed-unit score or fractional improvement threshold.
    return sorted(r['minimum'] for r in after['rows'])>=sorted(r['minimum'] for r in before['rows'])


def immediate_scope(front, top, forecast, patch, layer):
    prefix=SimpleNamespace(vertex={'coordinates':forecast.vertex['coordinates'][:layer]})
    index=SideIndex(front,prefix,top)
    scope=build_scope(index,patch.side_edges)
    # Every newly created verification side versus all possible new/prior sides.
    # Prior/prior pairs were checked at earlier admissions and cannot change.
    flags=np.array([bool(core) and k==layer for core,(k,e) in zip(scope.core,scope.sides)])
    flags.setflags(write=False)
    return replace(scope,core=flags)


def restore_exact_proofs(scope, certificate, proof_bytes, cache):
    """Restore existing exact records only under identical scientific authority.

    This is persistence plumbing for the qualified Task-3D proof cache. Every
    coordinate bit, ownership label, topology, kernel, pair and proof byte is
    bound. It cannot transfer a proof to different or unexamined geometry.
    """
    identities=[scope.identity(i) for i in range(len(scope.sides))]
    geometry=hashlib.sha256(canonical_bytes(identities)).hexdigest()
    if geometry!=certificate['geometry_sha256'] or scope.topology!=certificate['topology']:
        raise ValueError('ProofGeometryAuthorityMismatch')
    if hashlib.sha256(proof_bytes).hexdigest()!=certificate['records_sha256']:
        raise ValueError('ProofBytesMismatch')
    pairs,_,_=cache.pairs(scope);lines=proof_bytes.splitlines()
    if not certificate['complete'] or len(lines)!=len(pairs) or certificate['completed_pairs']!=len(pairs):
        raise ValueError('ProofCoverageMismatch')
    index={s:i for i,s in enumerate(scope.sides)}
    for line,(a,b) in zip(lines,pairs):
        record=json.loads(line)
        if (index[tuple(record.pop('side_a'))],index[tuple(record.pop('side_b'))])!=(int(a),int(b)):
            raise ValueError('ProofPairIdentityMismatch')
        cache.put(pair_identity(scope,int(a),int(b),identities),record)
    return len(lines)


class RollingForecastCache:
    """Bounded memo of local rebased trajectories with immutable past layers."""
    def __init__(self,front,top,schedule,sdf,coordinates,normals):
        self.front=front;self.top=top;self.schedule=schedule;self.sdf=sdf
        self.coordinates=coordinates;self.normals=normals;self.entries={};self.hits=0;self.misses=0;self.last_stats={}

    def forecast(self,state,candidate):
        started=time.perf_counter();ids=state.vertex_ids;layer=state.layer+1
        if layer>20: raise ValueError('ScheduleExhausted')
        key=identity(dict(version=VERSION,state=state.sha256,candidate=array_hash(candidate),sdf=self.sdf.source_identity,
                          geometry=self.front.geometry_authority,topology=self.top.sha256,remaining=array_hash(self.schedule.targets[layer:]),target=float(self.schedule.targets[layer-1])))
        if key in self.entries:
            self.hits+=1;self.last_stats=dict(hit=True,recomputed_entries=0,seconds=time.perf_counter()-started);return self.entries[key]
        self.misses+=1
        origin=SimpleNamespace(committed_layer=layer,coordinates=np.asarray(candidate),target_distances=np.full(len(ids),self.schedule.targets[layer-1]))
        future,projection=vertex_trajectories(origin,self.schedule,self.sdf,np.arange(len(ids)))
        prefix=np.stack((*state.prefix,np.asarray(candidate)))
        values=np.concatenate((prefix,future['coordinates'])) if layer<20 else prefix
        ns=self.sdf.evaluate(prefix.reshape(-1,3)).normal.reshape(prefix.shape)
        normals=np.concatenate((ns,future['normal'])) if layer<20 else ns
        overlay=CoordinateOverlay(self.coordinates,ids,values);normal_overlay=CoordinateOverlay(self.normals,ids,normals)
        _,edges,triangles=dependencies(self.top,ids);edata={};tdata={};previous=self.front.coordinates
        for k,x in enumerate(overlay):
            for dest,data in ((edata,edge_descriptors(x,previous,self.front.coordinates,normal_overlay[k],self.top.edges[edges])),
                              (tdata,triangle_descriptors(x,previous,self.front.coordinates,self.top.triangles[triangles]))):
                for name,v in data.items():dest.setdefault(name,[]).append(v)
            previous=x
        result=SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':overlay},key=key,
            local=dict(vertices=ids,coordinates=values,normal=normals,remaining_vertex=future,edges=edges,triangles=triangles,
                       edge={k:np.stack(v) for k,v in edata.items()},triangle={k:np.stack(v) for k,v in tdata.items()}))
        n=len(ids)+len(edges)+len(triangles)
        self.last_stats=dict(hit=False,invalidated_vertices=len(ids),invalidated_edges=len(edges),invalidated_triangles=len(triangles),recomputed_entries=n,
            unchanged_entries=len(self.front.coordinates)+len(self.top.edges)+len(self.top.triangles)-n,projection_seconds=projection,seconds=time.perf_counter()-started)
        self.entries[key]=result
        if len(self.entries)>4:self.entries.pop(next(iter(self.entries)))
        return result
