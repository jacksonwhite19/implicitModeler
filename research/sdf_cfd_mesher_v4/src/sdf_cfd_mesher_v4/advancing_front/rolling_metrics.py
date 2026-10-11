"""Whole-front coverage in bounded batches; unchanged frozen metric primitives."""
import time
import numpy as np
from ..forecast.detailed.metrics import cell_metrics
from ..runtime.frozen_imports import legacy_modules
from ..runtime.regions import extract_regions
from .local_relaxation import numeric_gates
from .local_relaxation import no_new_earlier_failure

def sdf_diagnostic_record(sample,target,existing=None):
    """Derive SDF evidence from queried coordinates, without requiring old files."""
    from ..common.authority import array_hash
    record=dict(target=float(target),maximum_residual=float(abs(sample.phi-target).max()),
        phi_hash=array_hash(sample.phi),gradient_hash=array_hash(sample.gradient),
        gradient_norm_min=float(sample.gradient_norm.min()),gradient_norm_max=float(sample.gradient_norm.max()))
    if existing is not None and record!=existing:raise ValueError('SdfDiagnosticReplayMismatch')
    return record

def future_admission(before,after):
    """Reject earlier hard events anywhere in the complete verification scope.

    A regional minimum must not conceal a new failure at a different entity.
    These are existing hard events, not individual conditioning floors.
    """
    earlier=no_new_earlier_failure(before,after)
    return dict(passed=not earlier,new_earlier=earlier)

def validate_admission_history(history):
    """An independently checked record is required before publication."""
    accepted=0
    for row in history:
        if not row.get('accepted'):continue
        if not row['gate']['passed'] or not row['exact']['passed']:raise ValueError('RecordedImmediateHardFailure')
        if row.get('new_earlier') or row.get('new_entity_failures'):raise ValueError('RecordedEarlierFutureFailure')
        if row.get('future_spots',{}).get('new_earlier_pair_families'):raise ValueError('RecordedEarlierExactPairFamily')
        accepted+=1
    return accepted

CELL_LIMITS={'volume':lambda a:a<=0,'signed_volume':lambda a:a<=0,'jacobian':lambda a:a<=0,
             'determinant':lambda a:a<.0011,'aspect':lambda a:a>1000}
FACE_LIMITS={'weight':lambda a:a<.055,'skewness':lambda a:a>3.63636,'symmetric_volume_ratio':lambda a:a<.01}

def evaluate_cells(top,low,high,layer,batch=4096):
    chunks={}
    for begin in range(0,len(top.triangles),batch):
        ids=top.triangles[begin:begin+batch];p=np.concatenate((low[ids],high[ids]),axis=1)
        values=cell_metrics(p,np.full(len(ids),layer))
        for name,a in values.items():chunks.setdefault(name,[]).append(a)
    return {k:np.concatenate(v) for k,v in chunks.items()}

def evaluate_layer(top,low,high,layer,previous=None,batch=4096):
    started=time.perf_counter();cq=legacy_modules()['component_quality'];fg=cq.FG
    cells=evaluate_cells(top,low,high,layer,batch)
    owners=np.array([top.edge_triangles.row(e) for e in range(len(top.edges))])
    if owners.shape!=(len(top.edges),2):raise ValueError('IncompleteSurfaceOwnership')
    sides={k:[] for k in ('weight','skewness','symmetric_volume_ratio','nonorthogonality')}
    for begin in range(0,len(top.edges),batch):
        e=top.edges[begin:begin+batch];a,b=owners[begin:begin+batch].T
        vs=np.stack((low[e[:,0]],low[e[:,1]],high[e[:,1]],high[e[:,0]]),axis=1);S,C=fg.face_area_centre_quad(vs)
        values=cq.face_metrics(vs,S,C,cells['centre'][a],cells['centre'][b],cells['volume'][a],cells['volume'][b])
        for name,x in zip(sides,values):sides[name].append(x)
    sides={k:np.concatenate(v) for k,v in sides.items()};caps={k:np.empty(0) for k in sides}
    if layer>1:
        if previous is None:raise ValueError('MissingPriorCellGeometry')
        chunks={k:[] for k in sides}
        for begin in range(0,len(top.triangles),batch):
            ids=np.arange(begin,min(begin+batch,len(top.triangles)));vs=low[top.triangles[ids]];S,C=fg.face_area_centre_tri(vs)
            values=cq.face_metrics(vs,S,C,previous['centre'][ids],cells['centre'][ids],previous['volume'][ids],cells['volume'][ids])
            for name,x in zip(chunks,values):chunks[name].append(x)
        caps={k:np.concatenate(v) for k,v in chunks.items()}
    face_ids=[('side',layer,int(e),int(a),int(b)) for e,(a,b) in enumerate(owners)]
    if layer>1:face_ids.extend(('cap',layer-1,int(c),int(c),int(c)) for c in range(len(top.triangles)))
    gate=numeric_gates(dict(cells={k:v[None] for k,v in cells.items()},faces={k:np.r_[sides[k],caps[k]] for k in sides},face_ids=face_ids))
    events=[]
    for name,predicate in CELL_LIMITS.items():
        for col in np.flatnonzero(predicate(cells[name])|~np.isfinite(cells[name])):
            events.append(dict(kind='cell',metric=name,layer=layer,entity=int(col),columns=[int(col)],value=float(cells[name][col])))
    for kind,values in [('side',sides),('cap',caps)]:
        for name,predicate in FACE_LIMITS.items():
            for entity in np.flatnonzero(predicate(values[name])|~np.isfinite(values[name])):
                events.append(dict(kind=kind,metric=name,layer=layer,entity=int(entity),columns=owners[entity].tolist() if kind=='side' else [int(entity)],value=float(values[name][entity])))
    return dict(cells=cells,sides=sides,caps=caps,gate=gate,events=events,seconds=time.perf_counter()-started)

def discover_regions(top,events,*,rings=0,max_vertices=2048):
    """Only current direct hard-event supports; never a saved research cohort."""
    mask=np.zeros(len(top.triangles),bool)
    for event in events:mask[event['columns']]=True
    return extract_regions(top,mask,entity='triangle',rings=rings,max_vertices=max_vertices)

def first_events(events,after=0):
    result={}
    for e in events:
        if e['layer']<=after:continue
        key=f'{e["kind"]}:{e["entity"]}:{e["metric"]}'
        result[key]=min(result.get(key,21),e['layer'])
    return result

def detailed_events(result):
    events=[];c=result['cells']
    for metric,predicate in CELL_LIMITS.items():
        for k,j in zip(*np.where(predicate(c[metric])|~np.isfinite(c[metric]))):
            col=int(c['column'][k,j]);events.append(dict(kind='cell',metric=metric,layer=int(c['layer'][k,j]) if 'layer' in c else int(k+1),entity=col,columns=[col],value=float(c[metric][k,j])))
    for metric,predicate in FACE_LIMITS.items():
        for i in np.flatnonzero(predicate(result['faces'][metric])|~np.isfinite(result['faces'][metric])):
            kind,k,e,a,b=result['face_ids'][i]
            events.append(dict(kind=kind,metric=metric,layer=k if kind=='side' else k+1,entity=e,columns=sorted(set((a,b))),value=float(result['faces'][metric][i])))
    return events

def replace_local_events(events,patch,result):
    columns=set(map(int,patch.core_columns));edges=set(map(int,patch.side_edges))
    kept=[e for e in events if (not set(e['columns'])&columns if e['kind']=='pair' else e['entity'] not in (edges if e['kind']=='side' else columns))]
    return sorted(kept+detailed_events(result),key=lambda e:(e['layer'],e['kind'],e['entity'],e['metric']))

def mechanism_tail(result,key,after):
    kind,entity,metric=key.split(':');entity=int(entity);rows=[]
    if kind=='cell':
        j=int(np.flatnonzero(result['cells']['column'][0]==entity)[0])
        rows=[(k+1,float(v)) for k,v in enumerate(result['cells'][metric][:,j]) if k+1>after]
        predicate=CELL_LIMITS[metric]
    else:
        rows=[(k if label=='side' else k+1,float(result['faces'][metric][i])) for i,(label,k,e,*_) in enumerate(result['face_ids'])
              if label==kind and e==entity and (k if label=='side' else k+1)>after]
        predicate=FACE_LIMITS[metric]
    first=min((k for k,v in rows if not np.isfinite(v) or predicate(v)),default=21)
    reverse=metric in ('aspect','skewness')
    return dict(first_invalid=first,all_finite=all(np.isfinite(v) for _,v in rows),
                tail=sorted((-v if reverse else v) for _,v in rows))

def mechanism_improves(before,after):
    if before['first_invalid']!=after['first_invalid']:return after['first_invalid']>before['first_invalid']
    if before['all_finite']!=after['all_finite']:return after['all_finite']
    return before['all_finite'] and tuple(after['tail'])>tuple(before['tail'])
