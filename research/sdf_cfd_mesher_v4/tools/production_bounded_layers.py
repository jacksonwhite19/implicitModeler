"""Bounded orchestration of the qualified generic production layer cycle.

No meshing mathematics is implemented here. L4 and L5 have separate stores,
admission decisions, replay barriers, certificates and retention milestones.
"""
import argparse,json,shutil,subprocess,time
from pathlib import Path
import numpy as np
import production_single_layer_cycle as cycle
from path_preserving_deduplication import metadata,physical_key,file_hash,durable_json
from qualify_path_preserving_deduplication import TREES,totals,ROOT
from rolling_layer_retention import checkpoint_closure
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash

V4=cycle.V4
OUT=V4/'results/production_adaptive_capsule_layers4_to5'
L3=V4/'results/production_adaptive_capsule_layer3'
RET=V4/'results/rolling_successful_layer_retention'
START='a4ba7f4b5485e152c7bb1ece06093efbfb659d5a'
INITIAL='f289395a543aeecbbba485b8efe50b188f43d32690d4c3c75bfecf57e32b3f23'
INCREMENTS={4:'0x1.042275ccae2b2p-12',5:'0x1.37351a26971d3p-12'}
ORIGINAL_SAVE=cycle.save
ORIGINAL_SEED=cycle.seed

def read(path):return json.loads(Path(path).read_text())
def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);durable_json(path,value)

def prior(layer):return L3 if layer==4 else OUT/'L4'

def configure(layer):
    if layer not in INCREMENTS:raise ValueError('BoundedLayers4To5Only')
    base=prior(layer)
    if layer==5:
        qualified=read(base/'QUALIFIED.json')
        if not qualified['passed'] or not qualified['retention_verified']:raise ValueError('L4MilestoneRequired')
    final=read(base/'A/FINAL.json')
    if layer==4 and final['committed']!=INITIAL:raise ValueError('WrongL3')
    cycle.OUT=OUT/f'L{layer}';cycle.BASE=base/'A/checkpoints'
    cycle.CONFIG=dict(cycle.CONFIG,layer=layer-1,committed=final['committed'],front_coordinates=final['coordinates'])
    cycle.INCREMENT_HEX=INCREMENTS[layer]
    def prepare_base():
        dst=cycle.OUT/'BASE/checkpoints';pointer=read(cycle.BASE/'CURRENT.json')
        if pointer['scientific']!=cycle.CONFIG['committed']:raise ValueError('BasePointerMismatch')
        if file_hash(cycle.BASE/pointer['checkpoint'])!=pointer['sha256']:raise ValueError('BaseCheckpointMismatch')
        if (dst/'CURRENT.json').exists():
            if read(dst/'CURRENT.json')!=pointer:raise ValueError('StaleIsolatedBase')
            return
        rows=cycle.seed(cycle.BASE,dst)
        shutil.copyfile(cycle.BASE/pointer['checkpoint'],dst/pointer['checkpoint']);shutil.copyfile(cycle.BASE/'CURRENT.json',dst/'CURRENT.json')
        save(cycle.OUT/'BASE_SHARING.json',rows)
    cycle.prepare_base=prepare_base
    def scoped_save(name,value):
        if isinstance(value,dict) and 'scope' in value and name.endswith('FINAL.json'):
            value=dict(value,scope=dict(committed_layer=layer,next_layer_generated=False,post_commit_next_layer_forecast=False,core_generated=False))
        ORIGINAL_SAVE(name,value)
    cycle.save=scoped_save
    def shared_seed(source,destination):
        # Parallel independent transactions have no accepted shared output
        # until READY equality authorizes publication. Do not read A's active
        # scratch objects when starting B.
        if source==cycle.OUT/'A/checkpoints':
            if not (cycle.OUT/'REPLAY_AUTHORIZATION.json').exists():return []
            from path_preserving_deduplication import replace_duplicate
            objects=destination/'objects';objects.mkdir(parents=True,exist_ok=True)
            (cycle.OUT/'REPLAY_DEDUP').mkdir(exist_ok=True)
            for p in sorted((source/'objects').iterdir()):
                q=objects/p.name
                if q.exists() and not p.samefile(q):
                    a=metadata(p,content=True);b=metadata(q,content=True)
                    if a['sha256']!=b['sha256']:raise ValueError('ReplayContainerMismatch')
                    for row in (a,b):row.update(immutable=True,references=[dict(kind='equal_replay_write_once_object',sha256=row['sha256'])])
                    replace_duplicate(b,a,cycle.OUT/'REPLAY_DEDUP'/f'{p.name}.json')
        return ORIGINAL_SEED(source,destination)
    cycle.seed=shared_seed
    return cycle.OUT

def inventory():
    rows=[];hashes={}
    for tree in [*TREES,L3,OUT]:
        for p in sorted(tree.rglob('*')):
            if not p.is_file() or p.suffix=='.log':continue
            r=metadata(p);key=physical_key(r)
            if key not in hashes:hashes[key]=file_hash(p)
            r['sha256']=hashes[key];rows.append(r)
    return rows

def preflight(layer):
    folder=configure(layer);folder.mkdir(parents=True,exist_ok=True);started=time.perf_counter()
    if layer==4:
        actual=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
        if actual!=START:raise ValueError('StartingHeadMismatch')
        status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
        if not (OUT/'BASELINE.json').exists():save(OUT/'BASELINE.json',dict(head=actual,status_hex=status.hex()))
        for name,expected in [('AUTHORITY.json','1c3dc9430858f66efb6c0a5725e141b1139db79057ebd7c36421bcec02dc30da'),('COMPACT_L3.json','7d9ebcd10f257507c426efe65b197add13581f9ed3faed77b384a19df506bdda')]:
            if file_hash(RET/name)!=expected:raise ValueError('RetentionAuthorityMismatch')
        for name,h in read(RET/'AUTHORITY.json')['evidence'].items():
            if file_hash(RET/name)!=h:raise ValueError('RetentionEvidenceMismatch:'+name)
        for name,h in [('AUTHORITY.json','b14ee357feda1394980a48902c61f90e83a428ec8ecaad085d4069b6bc037fbe'),('report.md','3c8aabf822c1538e69642a5373f75dc3d1949fc3137fc48fb1e5d36cce4dc7a6')]:
            if file_hash(L3/name)!=h:raise ValueError('L3AuthorityMismatch')
        for p,h in read(L3/'AUTHORITY.json')['scientific']['implementation'].items():
            if file_hash(V4/p)!=h:raise ValueError('QualifiedImplementationChanged:'+p)
    rows=inventory();free=shutil.disk_usage(ROOT).free;measured=read(L3/'STORAGE_FINAL.json')['observed_peak_task_allocated']
    if free<measured:raise ValueError('CapacityBelowMeasuredPriorTaskFootprint')
    closure,_=checkpoint_closure(cycle.BASE);active={physical_key(r):r for r in rows if r['path'] in closure}
    save(folder/'DISK_START.json',dict(rows=rows,totals=totals(rows),free_bytes=free,
        active_allocated=sum(r['cluster_bytes'] for r in active.values()),measured_prior_peak_task_bytes=measured,
        note='Prior observed footprint is evidence, not an extrapolated requirement or reserve. Monitor actual allocation during this layer.',
        seconds=time.perf_counter()-started,**cycle.resources()))
    cycle.prepare_base();print('PREFLIGHT',layer,free,measured,flush=True)

def restore(layer,label,phase='RESTORE'):
    folder=configure(layer);start=time.perf_counter();state=NativeCheckpointStore(folder/label/'checkpoints').restart()
    ready=read(folder/label/'READY.json');digest=scientific_hash(state)
    if digest!=ready['committed'] or {v['last_prism_layer'] for v in state['columns'].values()}!={layer}:raise ValueError('RestoreStateMismatch')
    n=16895;coords=[array_hash(np.array([state['vertices'][f'layer:{k}:source:{i}'] for i in range(n)])) for k in range(layer+1)]
    old=read(prior(layer)/'A/RESTORE.json')['coordinates']
    if coords[:-1]!=old or coords[-1]!=ready['coordinates']:raise ValueError('PrefixChanged')
    wall,tri,surface=cycle.load_production_wall(cycle.SURFACE)
    expected=dict(cycle.CONFIG,layer=layer,committed=digest,front_coordinates=ready['coordinates'])
    definition=read(cycle.SURFACE.parent/'GEOMETRY.json')['definition']
    # Constructor binds only. It neither invokes a predictor nor forecasts.
    from unittest.mock import patch
    def denied(*args,**kwargs):raise AssertionError('NoPostCommitForecastOrGeneration')
    with patch.object(cycle.BoundProductionForecastCache,'forecast',denied),patch.object(cycle.ProductionNextLayerProvider,'propose',denied),patch.object(cycle.MixedFrontRunner,'step',denied):
        adapter=cycle.ProductionForecastAdapter(state,wall,tri,surface,cycle.REQUEST,cycle.Source(dict(components=[definition])),expected)
    save(folder/label/(phase+'.json'),dict(passed=True,scientific=digest,coordinates=coords,cells=len(state['cells']),vertices=len(state['vertices']),receipts=len(state['certificate']),
        readiness=dict(binding=adapter.binding,increment=float(adapter.remaining_increments[0]),increment_hex=float(adapter.remaining_increments[0]).hex(),
            atlas_entries=len(state['atlas']),quality_count=len(state['quality']),proof_segments=state['certificate'].proof_segments,forecast=False,generation=False),
        seconds=time.perf_counter()-start,**cycle.resources()))
    print('RESTORED',layer,label,digest,flush=True)

def diagnostics(layer):
    folder=configure(layer);start=time.perf_counter()
    from production_next_layer_diagnostics import analyze
    wall,tri,surface=cycle.load_production_wall(cycle.SURFACE)
    prefix=[wall]
    roots=[V4/'results/production_adaptive_capsule_first_layer',V4/'results/production_adaptive_capsule_layer2',L3,OUT/'L4']
    for k in range(1,layer):
        with np.load(roots[k-1]/'A/CANDIDATE.npz') as z:prefix.append(z['front'].copy())
    with np.load(folder/'A/CANDIDATE.npz') as z:high=z['front'].copy()
    if array_hash(prefix[-1])!=cycle.CONFIG['front_coordinates']:raise ValueError('DiagnosticBaseMismatch')
    analyze(folder/'A',prefix,high,tri,read(cycle.SURFACE.parent/'GEOMETRY.json')['definition'],float.fromhex(INCREMENTS[layer]),prior(layer)/'A/QUALITY.json',folder/'SCREEN_ARRAYS.npz')
    save(folder/'DIAGNOSTIC_RESOURCES.json',dict(seconds=time.perf_counter()-start,**cycle.resources()))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['preflight','admit','generate','authorize','restore','diagnostics']);p.add_argument('--layer',type=int,required=True);p.add_argument('--label',default='A',choices=['A','B']);p.add_argument('--phase',default='RESTORE');a=p.parse_args()
    if a.mode=='preflight':preflight(a.layer)
    elif a.mode=='restore':restore(a.layer,a.label,a.phase)
    elif a.mode=='diagnostics':diagnostics(a.layer)
    else:
        configure(a.layer)
        if a.mode=='generate':cycle.generate(a.label)
        else:getattr(cycle,a.mode)()
