"""Successful qualification retention only. Never advances a front.

Original authorities remain immutable. A discard record preserves historical
content identities; a native checkpoint remains the restart authority.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import shutil
import subprocess
import time
import psutil
from path_preserving_deduplication import metadata, physical_key, file_hash, durable_json
from qualify_path_preserving_deduplication import TREES, totals, ROOT, V4

L3=V4/'results/production_adaptive_capsule_layer3'
OUT=V4/'results/rolling_successful_layer_retention'
COMMITTED='f289395a543aeecbbba485b8efe50b188f43d32690d4c3c75bfecf57e32b3f23'
POLICY='V4RollingSuccessfulQualificationRetention/1'

def read(p):return json.loads(Path(p).read_text())
def save(name,value):
    OUT.mkdir(exist_ok=True);durable_json(OUT/name,value)
def resources():
    m=psutil.Process().memory_info()
    return dict(peak_rss=m.peak_wset,retained_rss=m.rss)

def descriptors(value):
    if isinstance(value,dict):
        if 'file' in value and 'sha256' in value:yield value
        for v in value.values():yield from descriptors(v)
    elif isinstance(value,list):
        for v in value:yield from descriptors(v)

def checkpoint_closure(folder):
    pointer=read(folder/'CURRENT.json');envelope=read(folder/pointer['checkpoint'])
    refs={str((folder/'CURRENT.json').resolve()):file_hash(folder/'CURRENT.json'),
          str((folder/pointer['checkpoint']).resolve()):pointer['sha256']}
    pending=list(descriptors(envelope))
    while pending:
        row=pending.pop();p=folder/'objects'/row['file'];key=str(p.resolve())
        if key in refs:
            if refs[key]!=row['sha256']:raise ValueError('ConflictingObjectHash')
            continue
        refs[key]=row['sha256']
        # Nested prism backing descriptors are JSON. Ordinary large component
        # JSON has no object references and is bound by its envelope hash.
        if row.get('encoding')=='prism_entity_view':pending.extend(descriptors(read(p)))
    return refs,envelope

def audit():
    start=time.perf_counter();refs=defaultdict(list);active=set();historical=set()
    for tree in [*TREES,L3]:
        authority=tree/'AUTHORITY.json'
        for name,h in read(authority).get('evidence',{}).items():
            refs[str((tree/name).resolve())].append(dict(source=str(authority),sha256=h,kind='authority'))
        manifest=read(tree/'BULK_MANIFEST.json')
        entries=(manifest if isinstance(manifest,list) else manifest['files'] if 'files' in manifest
                 else [dict(path=k,**v) for k,v in manifest.items()])
        for r in entries:
            p=(ROOT/r['path'] if r['path'].startswith('research/') else tree/r['path']).resolve()
            refs[str(p)].append(dict(source=str(tree/'BULK_MANIFEST.json'),sha256=r['sha256'],kind='historical_bulk'))
        for cp in tree.rglob('CURRENT.json'):
            if cp.parent.name!='checkpoints':continue
            closure,_=checkpoint_closure(cp.parent)
            for p,h in closure.items():
                refs[p].append(dict(source=str(cp),sha256=h,kind='restart'))
                (active if tree==L3 and cp.parent.parent.name in ('A','B') else historical).add(p)
    rows=[];cache={}
    for tree in [*TREES,L3]:
        for p in sorted(tree.rglob('*')):
            if not p.is_file():continue
            r=metadata(p);key=physical_key(r)
            if key not in cache:cache[key]=file_hash(p)
            r['sha256']=cache[key];r['references']=refs.get(r['path'],[])
            if any(x['sha256']!=r['sha256'] for x in r['references']):raise ValueError('ReferenceMismatch:'+str(p))
            r['restore_dependency']=r['path'] in active or r['path'] in historical
            r['next_layer_dependency']=r['path'] in active
            rows.append(r)
    missing=set(refs)-{r['path'] for r in rows}
    if missing:raise ValueError('MissingReferences:'+str(missing))
    result=dict(rows=rows,totals=totals(rows),l3_totals=totals([r for r in rows if Path(r['path']).is_relative_to(L3)]),
                free_bytes=shutil.disk_usage(ROOT).free,seconds=time.perf_counter()-start,**resources())
    save('INVENTORY.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2),flush=True)

def deny(*args,**kwargs):raise AssertionError('ForecastOrGenerationForbiddenDuringRetention')

def regenerate(record='REGENERATION.json',export=None):
    """Rebuild complete exports from existing L3 coordinates, never predict."""
    import hashlib
    import numpy as np
    import generalize_local_remesh as g
    from final_shell_incidence import check
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    from production_surface_authority import load_production_wall
    from production_single_layer_cycle import SURFACE
    start=time.perf_counter();state=NativeCheckpointStore(L3/'A/checkpoints').restart()
    if scientific_hash(state)!=COMMITTED:raise ValueError('WrongRegenerationBase')
    wall,tri,_=load_production_wall(SURFACE);n=len(wall)
    fronts=[np.array([state['vertices'][f'layer:{k}:source:{i}'] for i in range(n)]) for k in (1,2,3)]
    del state
    print('regenerate fixed realized L3 exports',flush=True)
    points,cells,outer,checked,times=g.validation(wall,tri,fronts,np.full(n,3),False)
    incidence=check(points,cells,outer);values={r['id']:r['determinant'] for r in incidence['cells']}
    quality=checked['quality'];quality['cells']=[dict(r,determinant=values[r['id']]) for r in quality['cells']]
    quality['failures']=[r for r in quality['failures'] if r['kind']!='cell_quality']
    quality['failures'] += [dict(kind='cell_quality',**r) for r in quality['cells'] if not (r['volume']>0 and r['determinant']>=.0011 and r['aspect']<=1000)]
    quality['passed']=not quality['failures']
    print('quality and incidence regenerated',flush=True)
    volume=g.volume.audit(points,cells,outer,[tuple(map(int,f)) for f in tri])
    result={}
    for name,value in [('QUALITY.json',quality),('INCIDENCE.json',incidence),('VOLUME.json',volume)]:
        payload=(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
        digest=hashlib.sha256(payload).hexdigest()
        expected=next(r['sha256'] for r in read(OUT/'INVENTORY.json')['rows'] if Path(r['path'])==L3/'A'/name)
        result[name]=dict(sha256=digest,expected=expected,bytes=len(payload),exact=digest==expected)
        print(name,result[name],flush=True)
        if digest!=expected:raise ValueError('RegenerationMismatch:'+name)
        if export:
            destination=OUT/export
            if not destination.resolve().is_relative_to(OUT.resolve()):raise ValueError('ExportOutsideStudy')
            destination.mkdir(exist_ok=True);(destination/name).write_bytes(payload)
    save(record,dict(passed=True,complete_exports=result,committed=COMMITTED,
        implementation={str(p.relative_to(V4)):file_hash(p) for p in [Path(__file__),Path(g.__file__),Path(g.t.__file__),Path(g.volume.__file__)]},
        L4_forecast=False,L4_generated=False,seconds=time.perf_counter()-start,**resources()))

def restore(label,phase):
    import numpy as np
    from unittest.mock import patch
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    from sdf_cfd_mesher_v4.common.authority import array_hash,identity
    from production_forecast_admission import ProductionForecastAdapter,BoundProductionForecastCache
    from production_single_layer_cycle import CONFIG,SURFACE,REQUEST
    from production_surface_authority import load_production_wall
    from geometry_family_authority import Source
    from production_layer2_run import ProductionNextLayerProvider
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
    start=time.perf_counter();k=int(label[1]);run=label.split('_')[1]
    folder=([TREES[0],TREES[1],L3][k-1])/run/'checkpoints'
    state=NativeCheckpointStore(folder).restart();digest=scientific_hash(state)
    expected=['5b036bc4cd5180b9e4a7e3e6aafe57fe21c7b2ae1cae82cf3c42e446b1805fd0',CONFIG['committed'],COMMITTED][k-1]
    if digest!=expected:raise ValueError('RestoreIdentityMismatch')
    result=dict(label=label,scientific=digest,passed=True,checkpoint=read(folder/'CURRENT.json'),
                cells=len(state['cells']),vertices=len(state['vertices']),receipts=len(state['certificate']))
    if k==3:
        expected=dict(CONFIG,committed=COMMITTED,layer=3,front_coordinates='88f476ceb3d2d53ddecb547fa86b078a47fc15fdde0784a442d0836f0719d215')
        wall,tri,surface=load_production_wall(SURFACE)
        definition=read(SURFACE.parent/'GEOMETRY.json')['definition']
        with patch.object(BoundProductionForecastCache,'forecast',deny),patch.object(ProductionNextLayerProvider,'geometry',deny),patch.object(ProductionNextLayerProvider,'propose',deny),patch.object(MixedFrontRunner,'step',deny):
            adapter=ProductionForecastAdapter(state,wall,tri,surface,REQUEST,Source(dict(components=[definition])),expected)
        if adapter.binding['next_layer']!=4:raise ValueError('WrongNextLayer')
        increment=float(adapter.remaining_increments[0])
        if increment!=0.00024808369634458504:raise ValueError('WrongIncrement')
        certificate=state['certificate'];segments=certificate.proof_segments
        if sum(s['count'] for s in segments)!=len(certificate):raise ValueError('ProofPopulationMismatch')
        result['readiness']=dict(binding=adapter.binding,binding_identity=adapter.identity,
            increment=increment,increment_hex=increment.hex(),certificate_count=len(certificate),
            quality_count=len(state['quality']),proof_segments=segments,
            qualified_context=certificate.qualified_context,atlas_identity=identity(state['atlas']),
            atlas_entries=len(state['atlas']),transaction_base=digest,current_front_count=len(state['prism_front']),
            L4_forecast=False,L4_generated=False,guards=['forecast','geometry','propose','step'])
    result.update(seconds=time.perf_counter()-start,**resources());save(phase+'_'+label+'.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='readiness'},indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['audit','restore','regenerate']);p.add_argument('--label',default='L3_A');p.add_argument('--phase',default='PRE');p.add_argument('--record',default='REGENERATION.json');p.add_argument('--export');a=p.parse_args()
    if a.mode=='restore':restore(a.label,a.phase)
    elif a.mode=='regenerate':regenerate(a.record,a.export)
    else:audit()
