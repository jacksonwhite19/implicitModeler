"""Trusted local, hash-verified ProductionFast milestone; no mesh advancement.

This storage receipt binds a private live-state snapshot. It is not a final
scientific seal. Pickle is only read after verification against our published
manifest, never as an external CAD/geometry input.
"""
import json,os,pickle,time,gzip,shutil
from pathlib import Path
import numpy as np
from production_fast_layer import FastLayer,MemorySegment,native_append
from incremental_layer_validation import Topology,Quality,ExactVolume,key,t
from incremental_degree import DegreeState
from run_geometry_family import AnalyticProvider
from native_proof_segments import LoadedSegment,SegmentProofRows
from compact_proof_rows import CompactProofRows
from sdf_cfd_mesher_v4.common.authority import file_hash,array_hash,identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key,rebuild_dependencies,geometry_inputs
from sdf_cfd_mesher_v4.advancing_front.execution_profile import ExecutionProfile,MemoryTransaction

SCHEMA='V4ProductionFastLocalMilestone/2'

def ensure_proofs(run):
    if run.proofs is not None:return
    n=len(run.provider.x)
    def source(v):return f'layer:{v//n}:source:{v%n}'
    faces=[run.topology.owners[k][0][1] for k in sorted(run.topology.owners)]
    run.face_keys=[face_key(tuple(source(v) for v in f)) for f in faces]
    indices={k:i for i,k in enumerate(run.face_keys)};base=run.tx.current['certificate']
    segments=[LoadedSegment(s,base.proof_objects,indices,True) for s in base.proof_segments]
    if sum(s.descriptor['count'] for s in segments)!=len(base):raise ValueError('IncompleteBootstrapProofs')
    run.proofs=SegmentProofRows(segments,CompactProofRows())

def signature(run):
    ensure_proofs(run)
    return dict(geometry={k:scientific_digest(run.tx.current[k]) for k in ('wall','first_layer','vertices','cells','faces','columns','outer','prism_front','versions','atlas','forecast','authority','events')},
        degree=run.degree_state.root,points=array_hash(run.points),relations=len(run.proofs),
        segments=[dict(faces=identity(s.faces),pairs=array_hash(s.pairs),codes=array_hash(s.codes),payloads=identity(s.payloads),mapping=array_hash(s.mapping)) for s in run.proofs.segments],
        fresh=dict(pairs=array_hash(np.frombuffer(run.proofs.fresh.pairs,dtype=np.uint64)),codes=array_hash(np.frombuffer(run.proofs.fresh.codes,dtype=np.uint64)),payloads=identity(run.proofs.fresh.payloads)))

def save(run,path):
    started=time.perf_counter();path=Path(path);path.mkdir(parents=True,exist_ok=False);ensure_proofs(run)
    state=dict(run.tx.current,quality={},certificate={});state['_production_unsealed']=True
    data=dict(state=state,spec=run.spec,generation=run.tx.generation,
        attributes={k:getattr(run,k) for k in ('points','outer','wall','topology','quality','volume','degree_state','inputs','vertex_incidence','face_keys')},
        segments=[{k:getattr(s,k) for k in ('faces','pairs','codes','payloads','mapping','descriptor')} for s in run.proofs.segments],fresh=vars(run.proofs.fresh))
    # Streaming compression avoids large uncompressed checkpoint copies. The
    # manifest authenticates stored bytes; scientific state is unchanged.
    if shutil.disk_usage(path).free<64*1024*1024:raise OSError('MilestoneStorageReserveUnavailable')
    payload=path/'STATE.pickle.gz';temporary=path/'STATE.pending'
    with temporary.open('wb') as f:
        with gzip.GzipFile(fileobj=f,mode='wb',compresslevel=1,mtime=0) as z:pickle.dump(data,z,protocol=5)
        f.flush();os.fsync(f.fileno())
    os.replace(temporary,payload)
    record=dict(schema=SCHEMA,payload=payload.name,sha256=file_hash(payload),bytes=payload.stat().st_size,
        signature=signature(run),implementation={p.name:file_hash(p) for p in (Path(__file__),Path(__file__).with_name('production_fast_layer.py'),Path(__file__).with_name('incremental_degree.py'))},
        layer=max(c['layer'] for c in run.topology.cells.values()),seconds=time.perf_counter()-started,final_seal_required=True)
    temporary=path/'CURRENT.pending';temporary.write_text(json.dumps(record,indent=2));os.replace(temporary,path/'CURRENT.json')
    return record

def load(path,folder):
    path=Path(path);record=json.loads((path/'CURRENT.json').read_text())
    if record['schema']!=SCHEMA or Path(record['payload']).name!=record['payload']:raise ValueError('MilestoneSchema')
    payload=path/record['payload']
    if file_hash(payload)!=record['sha256']:raise ValueError('MilestoneByteCorruption')
    for name,h in record['implementation'].items():
        if file_hash(Path(__file__).with_name(name))!=h:raise ValueError('MilestoneImplementationChanged')
    with gzip.open(payload,'rb') as f:data=pickle.load(f)
    run=object.__new__(FastLayer);run.spec=data['spec'];run.folder=Path(folder);run.profile=ExecutionProfile.ProductionFast
    run.provider=AnalyticProvider(run.spec,run.folder);run.tx=MemoryTransaction(data['state'],data['generation'])
    if any(run.tx.current['authority'].get(k)!=v for k,v in run.provider.authority.items()):raise ValueError('MilestoneScientificAuthorityChanged')
    for k,v in data['attributes'].items():setattr(run,k,v)
    segments=[]
    for values in data['segments']:
        s=object.__new__(MemorySegment)
        for k,v in values.items():setattr(s,k,v)
        segments.append(s)
    fresh=CompactProofRows();fresh.__dict__.update(data['fresh']);run.proofs=SegmentProofRows(segments,fresh)
    if signature(run)!=record['signature']:raise ValueError('MilestoneScientificMismatch')
    return run

def from_sealed_append(source,seal_folder,spec,folder):
    """Import saved accepted coordinates; never regenerate an accepted layer."""
    seal_folder=Path(seal_folder);seal=json.loads((seal_folder/'SEAL.json').read_text());z=np.load(seal_folder/'SEALED_GEOMETRY.npz')
    representation=json.loads((seal_folder/'FINAL_REPRESENTATION_AUDIT.json').read_text()) if 'geometry_sha256' not in seal else seal
    if file_hash(seal_folder/'SEALED_GEOMETRY.npz')!=representation['geometry_sha256']:raise ValueError('SealedGeometryCorrupt')
    before=NativeCheckpointStore(source).restart();points=z['points'];outer=[tuple(map(int,f)) for f in z['outer']]
    cells=[t.cell(v,int(c),int(k)) for v,c,k in zip(z['cell_vertices'],z['source_columns'],z['layers'])]
    additions=[c for c in cells if str(c['id']) not in before['cells']]
    state=native_append(before,cells,points,outer,additions);rebuild_dependencies(before,state,z['schedule'].tolist())
    for name,h in seal['geometry_components'].items():
        if scientific_digest(state[name])!=h:raise ValueError('SavedGeometryMismatch:'+name)
    provider=AnalyticProvider(spec,folder);wall=[tuple(reversed(f)) for f in provider.tri]
    if any(state['authority'].get(k)!=v for k,v in provider.authority.items()):raise ValueError('ChangedScientificImplementation')
    owners,check=t.ownership(cells,outer,wall)
    if not check['passed']:raise ValueError('SavedOwnershipFailure')
    q=t.quality(points,cells,owners,1)
    from final_shell_incidence import check as incidence
    inc=incidence(points,cells,outer)
    if scientific_digest(q)!=seal['quality_digest'] or scientific_digest(inc)!=seal['prospective_digest']:raise ValueError('SavedQualityMismatch')
    run=object.__new__(FastLayer);run.spec=spec;run.folder=Path(folder);run.provider=provider;run.profile=ExecutionProfile.ProductionFast
    run.tx=MemoryTransaction(state);run.points=points;run.outer=outer;run.wall=wall;run.topology=Topology(cells,owners)
    run.quality=Quality({r['id']:r for r in q['cells']},{key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in q['faces']},{r['id']:r for r in q['exact']},{r['id']:r for r in inc['cells']})
    run.quality.bind_masks(run.topology,outer);run.volume=ExactVolume(points,cells,outer,provider.tri,run.quality.exact)
    run.degree_state=DegreeState(points,wall+outer,run.volume.witness);run.proofs=None;ensure_proofs(run)
    indices={k:i for i,k in enumerate(run.face_keys)}
    run.proofs.segments.append(LoadedSegment(seal['delta_segment'],seal_folder/'seal_objects',indices,True))
    if len(run.proofs)!=seal['coverage']['active']:raise ValueError('SealedRelationCountMismatch')
    run.inputs=geometry_inputs(state);run.vertex_incidence={}
    for k,row in state['faces'].items():
        for v in row['vertices']:run.vertex_incidence.setdefault(v,set()).add(k)
    # Old raw receipt tables are unnecessary after compact proof/cache import.
    run.tx.current=dict(state,quality={},certificate={})
    return run
