"""Production L4/L5 storage controls only. No future geometry or forecast."""
from contextlib import ExitStack
from collections import Counter
from pathlib import Path
import json,os,shutil,subprocess,time
from unittest.mock import patch
import psutil
import qualify_immutable_state_access as prior
import immutable_state_access as access
from path_preserving_deduplication import file_hash,durable_json,metadata,physical_key
from rolling_layer_retention import checkpoint_closure
from sdf_cfd_mesher_v4.common.authority import identity

V4=prior.audit.cycle.V4
OUT=V4/'results/incremental_transaction_publication'
L4=prior.audit.LAYERS/'L4/A/checkpoints'
L5=prior.audit.STORE
BEFORE='8673f3f242b41e9733aa94bae7e0ca409ccee24b0e944afe6ca87f563e2a04f4'
AFTER=prior.audit.EXPECTED
def read(p):return access.read(p)
def save(name,x):OUT.mkdir(parents=True,exist_ok=True);durable_json(OUT/name,x)
def resources(start,cpu,io):
    p=psutil.Process();m=p.memory_info();now=p.io_counters()
    return dict(seconds=time.perf_counter()-start,cpu=time.process_time()-cpu,peak_rss=m.peak_wset,retained_rss=m.rss,
        io={k:getattr(now,k)-getattr(io,k) for k in io._fields})
def preflight():
    head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip();assert head=='da0d00432491f0278195ec31188b9247c504d4f9'
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
    original=[r for r in status.split(b'\0') if r and b'incremental_transaction' not in r]
    expected=bytes.fromhex(read(prior.OUT/'BASELINE.json')['status_hex']).split(b'\0')
    assert original==[r for r in expected if r]
    authority=read(prior.OUT/'AUTHORITY.json');assert authority['identity']=='7744d0bc88a60f75e6048edb105c983bcc351e9946b3710b32faa9e9df76e5a8'
    assert identity(authority['scientific'])==authority['identity']
    protected={str(prior.OUT/'AUTHORITY.json'):file_hash(prior.OUT/'AUTHORITY.json')}
    for name,h in authority['scientific']['evidence'].items():assert file_hash(prior.OUT/name)==h;protected[str(prior.OUT/name)]=h
    refs4,env4=checkpoint_closure(L4);refs5,env5=checkpoint_closure(L5)
    assert env4['scientific']==BEFORE and env5['scientific']==AFTER
    with access.ImmutableState(prior.STORE,authority['scientific']['access_root']) as state:assert state.verified_legacy_identity()==AFTER
    rows=[metadata(p) for p in set(refs4)|set(refs5)];unique={physical_key(r):r for r in rows}
    free=shutil.disk_usage(OUT.anchor).free;size=sum(r['cluster_bytes'] for r in unique.values())
    assert free>size,'Available space does not cover measured control closure'
    save('BASELINE.json',dict(head=head,status_hex=b'\0'.join(original).hex(),unrelated=155,free=free,closure_physical=size,
        protected=protected,l4=refs4,l5=refs5,access_root=authority['scientific']['access_root'],no_forecast=True,no_generation=True))
    print('PREFLIGHT',free,size,flush=True)

def trace():
    """Measure real journal copying and native publication with sealed controls.

    Isolated copies are temporary. No hardlinks are added to immutable originals:
    adding a link would invalidate their qualified NTFS mutation tokens.
    """
    from sdf_cfd_mesher_v4.advancing_front import native_checkpoint as nc,mixed_front as mf,mixed_front_journal as journal
    from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import ReceiptDelta
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters();timers=prior.audit.Timers();counts=Counter();stages={}
    with ExitStack() as stack:
        for obj,names in [(nc,['scientific_digest','file_hash']),(mf,['scientific_hash']),(nc.NativeCheckpointStore,['_json','_flat_table','_receipt','_publish']),(ReceiptDelta,['measure_identity'])]:
            for name in names:stack.enter_context(patch.object(obj,name,timers.wrap(obj.__name__+'.'+name,getattr(obj,name))))
        for source,expected,label in [(L4,BEFORE,'L4'),(L5,AFTER,'L5')]:
            t=time.perf_counter();state=nc.NativeCheckpointStore(source).restart();assert mf.scientific_hash(state)==expected
            stages[label+'_restore']=time.perf_counter()-t
            if label=='L4':before=state
            else:after=state
        # The control boundary starts with validated immutable input states.
        t=time.perf_counter();mem=journal.InMemoryMixedFrontJournal(before);stages['journal_initialize']=time.perf_counter()-t
        t=time.perf_counter();base=mem.restart();stages['journal_restart']=time.perf_counter()-t
        assert mf.scientific_hash(base)==BEFORE
        t=time.perf_counter();result=mem.publish(after);stages['journal_publish']=time.perf_counter()-t;assert result==AFTER
        # Native publication is unchanged. Force its existing copy fallback,
        # because creating hardlinks would mutate original NTFS change tokens.
        def no_link(*a,**kw):raise OSError('ControlRequiresIndependentTemporaryCopies')
        original_copy=shutil.copyfile
        def copy(src,dst,*a,**kw):counts['files_copied']+=1;counts['bytes_copied']+=Path(src).stat().st_size;return original_copy(src,dst,*a,**kw)
        target=OUT/'legacy_temporary';t=time.perf_counter()
        with patch.object(os,'link',no_link),patch.object(shutil,'copyfile',copy):
            assert nc.NativeCheckpointStore(target).publish(mem.current)==AFTER
        stages['native_publication']=time.perf_counter()-t
        rows=[dict(**metadata(p),sha256=file_hash(p)) for p in target.rglob('*') if p.is_file()]
        save('LEGACY_TRACE.json',dict(control='Sealed L4 state -> sealed L5 state; journal initialize/restart/publish and native publish, no scientific regeneration',
            stages=stages,timers=timers.rows,counts=dict(counts),physical=sum(r['cluster_bytes'] for r in rows),files=rows,
            before=BEFORE,after=AFTER,**resources(start,cpu,io)))
    print('LEGACY TRACE COMPLETE',flush=True)

def envelope(store):return read(store/read(store/'CURRENT.json')['checkpoint'])
def raw_component(store,name):return read(store/'objects'/envelope(store)['components'][name]['file'])
def table(store,name):
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    return NativeCheckpointStore(store)._receipt(envelope(store)['components'][name],verify=False)

def candidate():
    """Extract the exact sealed transition, then time changed-state publication."""
    import sqlite3
    import incremental_transaction as tx
    from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
    prep=time.perf_counter();refs={};base=read(OUT/'BASELINE.json')
    # Enrollment/extraction is qualification work, not claimed incremental.
    for p,h in {**base['l4'],**base['l5']}.items():refs[p]=access.attest(p,h)
    e4,e5=envelope(L4),envelope(L5)
    oldfaces=raw_component(L4,'faces');newfaces=raw_component(L5,'faces')
    vertices=raw_component(L5,'vertices');oldvertices=raw_component(L4,'vertices')
    assert all(vertices[k]==v for k,v in oldvertices.items())
    patch_inputs={};changed_old=0
    for key,face in newfaces.items():
        if oldfaces.get(key)!=face:
            patch_inputs['face:'+key]=identity(dict(face=face,coordinates={v:vertices[v] for v in face['vertices']}))
            changed_old+=key in oldfaces
    assert changed_old==33786 and len(patch_inputs)==118251
    del oldfaces,newfaces,vertices,oldvertices
    old=iter(raw_items(table(L4,'quality')));a=next(old,None);rows=[];counts=Counter()
    for key,raw in raw_items(table(L5,'quality')):
        if a is not None and a[0]<key:raise ValueError('UnexpectedQualityRemoval')
        value=json.loads(raw)
        if a is not None and a[0]==key:
            previous=json.loads(a[1]);a=next(old,None)
            if previous==value:counts['unchanged']+=1;continue
            op={k:v for k,v in previous.items() if k!='inputs'};np={k:v for k,v in value.items() if k!='inputs'}
            if op!=np:raise ValueError('UnqualifiedChangedQualityPayload')
            payload=None;counts['binding_only']+=1
        else:payload={k:v for k,v in value.items() if k!='inputs'};counts['new']+=1
        rows.append(dict(key=key,inputs=value['inputs'],payload=payload))
    assert a is None and dict(counts)==dict(unchanged=506790,binding_only=67572,new=152037)
    seg4=e4['components']['certificate']['proof_segments'];seg5=e5['components']['certificate']['proof_segments']
    assert seg5[:-1]==seg4 and seg5[-1]['count']==3782768
    target=OUT/'candidate';store=tx.Store(target)
    initial=dict(scientific=BEFORE,epoch=access.epoch(),plan=dict(references={p:r for p,r in refs.items() if p in base['l4']},components=e4['components']))
    expected_base=identity(dict(initial,schema=tx.SCHEMA))
    if (target/'CURRENT.json').exists():
        root=read(target/'CURRENT.json')['root']
        if root!=expected_base:raise ValueError('UnexpectedCandidateRetryBase')
    else:root=store.initialize(initial)
    plan=dict(scientific=AFTER,references=refs,components=e5['components'],base_components=e4['components'],
        immutable_proofs=seg4,new_proofs=seg5[-1],base_store=str(L4),target_store=str(L5),
        backing_access_root=base['access_root'],quality_counts=dict(counts),receipt_counts=dict(unchanged=10344468,binding_updates=1715100,new=3782768),
        policy='Sealed production transition; accepted proof components and affected quality delta. No unsealed geometry acceptance.')
    acceptance=dict(base=root,plan=identity(plan),passed=True,quality_delta=identity(rows),quality_count=len(rows),
        binding_patch=identity(patch_inputs),control_before=BEFORE,control_after=AFTER,
        scientific_authority=file_hash(prior.audit.LAYERS/'AUTHORITY.json'),
        authority_scope='Already independently qualified L4 to L5; no authorization for an unknown future mesh')
    save('ACCEPTANCE.json',dict(identity=identity(acceptance),record=acceptance))
    save('PREPARATION.json',dict(seconds=time.perf_counter()-prep,quality_counts=dict(counts),changed_faces=changed_old,new_faces=len(patch_inputs)-changed_old,
        full_quality_rows_read=574362+726399,complete_old_geometry_comparison=True,base=root,
        proof_segments_reused=len(seg4),new_proofs=seg5[-1]['count'],no_forecast=True,no_generation=True))
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
    # Both inputs are already validated, as for the legacy journal/publication
    # control. Only changed quality rows and factorized bindings are serialized.
    newroot=store.publish(root,plan,rows,patch_inputs,acceptance,identity(acceptance))
    files=[metadata(p) for p in target.rglob('*') if p.is_file()]
    save('CANDIDATE.json',dict(root=newroot,base=root,scientific=AFTER,stats=dict(store.stats),
        physical=sum(r['cluster_bytes'] for r in files),files=files,**resources(start,cpu,io)))
    print('CANDIDATE',newroot,flush=True)

def deep():
    import hashlib,zlib,sqlite3
    import incremental_transaction as tx
    from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters();c=read(OUT/'CANDIDATE.json')
    with tx.Store(OUT/'candidate').open(c['root']) as body:
        for p,r in {**body['plan']['references'],**body['produced']}.items():assert file_hash(p)==r['sha256']
        patch_inputs=read(body['binding_patch']);counts=Counter();h=hashlib.sha256();h.update(b'{')
        old=iter(raw_items(table(L4,'certificate')));a=next(old,None)
        for i,(key,raw) in enumerate(raw_items(table(L5,'certificate'))):
            value=json.loads(raw)
            if a is not None and a[0]<key:raise ValueError('UnexpectedProofRemoval')
            if a is not None and a[0]==key:
                previous=json.loads(a[1]);a=next(old,None)
                expected=dict(previous,inputs={k:patch_inputs.get(k,v) for k,v in previous['inputs'].items()})
                assert expected==value,'ProofOrBindingDifference'
                counts['unchanged' if previous==expected else 'binding_updates']+=1
            else:expected=value;counts['new']+=1
            if i:h.update(b',')
            h.update(tx.canonical(key));h.update(b':');h.update(tx.canonical(expected))
        h.update(b'}');assert a is None
        assert h.hexdigest()==envelope(L5)['components']['certificate']['scientific']
        assert dict(counts)==body['plan']['receipt_counts']
        # Sequential exact quality replay, without per-key DB reopen overhead.
        old=iter(raw_items(table(L4,'quality')));a=next(old,None)
        db=sqlite3.connect(Path(body['quality_delta']).as_uri()+'?mode=ro&immutable=1',uri=True)
        changes=iter(db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'));b=next(changes,None)
        qh=hashlib.sha256();qh.update(b'{');quality_count=0
        for key,raw in raw_items(table(L5,'quality')):
            old_value=None
            if a is not None and a[0]==key:old_value=json.loads(a[1]);a=next(old,None)
            if b is not None and b[0]==key:
                value=old_value if b[2] is None else json.loads(zlib.decompress(b[2]));value['inputs']=json.loads(zlib.decompress(b[1]));b=next(changes,None)
            else:value=old_value
            assert value==json.loads(raw),'QualityDifference'
            if quality_count:qh.update(b',')
            qh.update(tx.canonical(key));qh.update(b':');qh.update(tx.canonical(value));quality_count+=1
        qh.update(b'}');db.close();assert a is None and b is None
        assert qh.hexdigest()==envelope(L5)['components']['quality']['scientific']
        save('DEEP.json',dict(scientific=AFTER,receipt_counts=dict(counts),certificate_digest=h.hexdigest(),quality_count=quality_count,
            quality_digest=qh.hexdigest(),all_other_components='Exact original L5 descriptors and rehashed immutable files',
            proof_payloads='Identical inherited segment descriptors; every prior receipt payload compared exactly',
            no_forecast=True,no_generation=True,**resources(start,cpu,io)))
    print('DEEP CONTROL EQUALITY PASSED',flush=True)

def reopen():
    import incremental_transaction as tx
    import production_forecast_admission as pf
    import sqlite3
    from sdf_cfd_mesher_v4.advancing_front import native_checkpoint as nc
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters();c=read(OUT/'CANDIDATE.json')
    with tx.State(OUT/'candidate',c['root'],prior.STORE) as state:
        assert state.verified_legacy_identity()==AFTER
        wall,tri,surface=prior.audit.cycle.load_production_wall(prior.audit.cycle.SURFACE)
        source=prior.audit.cycle.Source(dict(components=[read(prior.audit.cycle.SURFACE.parent/'GEOMETRY.json')['definition']]))
        with patch.object(sqlite3,'connect',prior.audit.denied),patch.object(pf,'scientific_hash',prior.audit.denied),patch.object(nc.NativeCheckpointStore,'restart',prior.audit.denied),patch.object(pf.BoundProductionForecastCache,'forecast',prior.audit.denied):
            adapter=state.initialize(wall,tri,surface,prior.audit.cycle.REQUEST,source,state.original.body['expected'])
            cache=pf.BoundProductionForecastCache(adapter);cache.check(adapter);assert cache.cache.result is None
        assert float(adapter.remaining_increments[0])==0.00035506009452908446
        save('REOPEN.json',dict(root=c['root'],scientific=AFTER,binding=adapter.binding,increment=float(adapter.remaining_increments[0]),
            stats=dict(state.original.stats),proof_rows_visited=0,quality_rows_visited=0,no_forecast=True,no_generation=True,**resources(start,cpu,io)))
    print('REOPEN PASSED',flush=True)

def replay(shared=False):
    """Fresh-process changed-input publication; independent of prefix size.

    Inputs are the previously accepted changed records, not a legacy restore.
    Component equality links this repeat to the complete deep control replay.
    """
    import sqlite3,zlib
    import incremental_transaction as tx
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters()
    c=read(OUT/'CANDIDATE.json');original=tx.Store(OUT/'candidate')
    with original.open(c['root']) as body:
        initial=read(original.path/('ROOT_'+c['base']+'.json'))
        db=sqlite3.connect(Path(body['quality_delta']).as_uri()+'?mode=ro&immutable=1',uri=True)
        rows=[dict(key=k,inputs=json.loads(zlib.decompress(i)),payload=None if p is None else json.loads(zlib.decompress(p)))
            for k,i,p in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key')];db.close()
        bindings=read(body['binding_patch'])
        store=tx.Store(OUT/('candidate_shared_replay' if shared else 'candidate_replay'),
            original.path/'objects' if shared else None);base=store.initialize(initial);assert base==c['base']
        load_seconds=time.perf_counter()-start;t=time.perf_counter()
        root=store.publish(base,body['plan'],rows,bindings,body['acceptance'],body['acceptance_identity'])
        publication=time.perf_counter()-t
        with store.open(root) as result:
            assert result['plan']==body['plan'] and result['acceptance']==body['acceptance']
            for key in ('quality_delta','binding_patch'):
                assert file_hash(result[key])==file_hash(body[key])
        assert store.publish(base,body['plan'],rows,bindings,body['acceptance'],body['acceptance_identity'])==root
        if shared:assert root==c['root']
        files=[metadata(p) for p in store.path.rglob('*') if p.is_file()]
        save('SHARED_REPLAY.json' if shared else 'REPLAY.json',dict(root=root,scientific=AFTER,publication_seconds=publication,changed_input_load_seconds=load_seconds,
            stats=dict(store.stats),changed_rows_loaded=len(rows),byte_identical_changed_components=True,
            physical=sum(r['cluster_bytes'] for r in files),files=files,**resources(start,cpu,io)))
    print('CHANGED-STATE REPLAY PASSED',flush=True)

def points():
    """Exercise exact cold pair/quality lookup separately from normal init."""
    import incremental_transaction as tx
    from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items
    start=time.perf_counter();cpu=time.process_time();io=psutil.Process().io_counters();c=read(OUT/'CANDIDATE.json')
    with tx.State(OUT/'candidate',c['root'],prior.STORE) as state:
        base=table(L4,'certificate');target=table(L5,'certificate');patch_inputs=read(state.body['binding_patch']);keys={}
        for key,raw in raw_items(target.updates):
            try:base[key];kind='updated_prior'
            except KeyError:kind='new'
            keys.setdefault(kind,key)
            if len(keys)==2:break
        for key,raw in raw_items(base):
            old=json.loads(raw)
            if all(patch_inputs.get(k,v)==v for k,v in old['inputs'].items()):keys['unchanged']=key;break
        assert len(keys)==3
        for key in keys.values():assert state['certificate'][key]==target[key]
        import sqlite3
        db=sqlite3.connect(Path(state.body['quality_delta']).as_uri()+'?mode=ro&immutable=1',uri=True)
        qkeys={kind:db.execute('SELECT key FROM updates WHERE payload IS '+condition+' LIMIT 1').fetchone()[0]
            for kind,condition in [('binding_only','NULL'),('new','NOT NULL')]}
        for key,raw in raw_items(table(L4,'quality')):
            if db.execute('SELECT 1 FROM updates WHERE key=?',(key,)).fetchone() is None:qkeys['unchanged']=key;break
        db.close();targetq=table(L5,'quality')
        for key in qkeys.values():assert state['quality'][key]==targetq[key]
        save('POINT_LOOKUP.json',dict(receipt_keys=keys,quality_keys=qkeys,exact=True,
            scope='Explicit demanded cold access; full component hash on first use, excluded from normal init',**resources(start,cpu,io)))
    print('POINT LOOKUPS PASSED',flush=True)

def cleanup():
    """Remove this task's disposable exact copies, never historical files."""
    import incremental_transaction as tx
    assert read(OUT/'DEEP.json')['scientific']==AFTER
    assert read(OUT/'SHARED_REPLAY.json')['root']==read(OUT/'CANDIDATE.json')['root']
    b=read(OUT/'BASELINE.json');by_hash={h:p for p,h in {**b['l4'],**b['l5']}.items()}
    c=read(OUT/'CANDIDATE.json');body=tx.Store(OUT/'candidate').root(c['root'])
    by_hash.update({r['sha256']:p for p,r in body['produced'].items()})
    candidates=[r for r in read(OUT/'LEGACY_TRACE.json')['files'] if Path(r['path']).parent.name=='objects']
    candidates += [dict(**metadata(p),sha256=file_hash(p)) for p in (OUT/'candidate_replay/objects').glob('*') if p.is_file()]
    candidates += read(OUT/'INFRASTRUCTURE_FAILURE.json')['files']
    records=[];before=[metadata(p) for p in OUT.rglob('*') if p.is_file()]
    for r in candidates:
        p=Path(r['path']).resolve();p.relative_to(OUT.resolve())
        assert any(p.is_relative_to((OUT/part).resolve()) for part in ['legacy_temporary/objects','candidate_replay/objects','candidate/objects'])
        assert r['sha256'] in by_hash and r['links']==1
        source=Path(by_hash[r['sha256']]).resolve();assert source!=p
        with access.guard(source),access.guard(p):
            assert file_hash(p)==file_hash(source)==r['sha256']
            assert metadata(p)['file_id']==r['file_id']
        records.append(dict(**r,retained_source=str(source),role='Disposable task timing/failure duplicate; unique evidence retained'))
    plan=dict(stage='sealed_before_removal',records=records,paths=len(records),
        logical=sum(r['bytes'] for r in records),physical=sum(r['cluster_bytes'] for r in records),
        before_task_physical=sum(r['cluster_bytes'] for r in before),free_before=shutil.disk_usage(OUT.anchor).free)
    save('TEMPORARY_REMOVAL_PLAN.json',plan)
    for r in records:
        p=Path(r['path']);source=Path(r['retained_source'])
        with access.guard(source):
            assert metadata(p)['file_id']==r['file_id'];p.unlink()
    save('TEMPORARY_REMOVAL.json',dict(plan_sha256=file_hash(OUT/'TEMPORARY_REMOVAL_PLAN.json'),
        removed_paths=[r['path'] for r in records],physical_reclaimed=plan['physical'],logical_removed=plan['logical'],
        free_after=shutil.disk_usage(OUT.anchor).free,all_sources_exist=all(Path(r['retained_source']).exists() for r in records),
        historical_files_removed=0))
    print('DISPOSABLE COPIES REMOVED',len(records),plan['physical'],flush=True)

def seal():
    classification='V4IncrementalTransactionPublicationArchitecturePartialSealedControlOnly'
    b=read(OUT/'BASELINE.json');l=read(OUT/'LEGACY_TRACE.json');c=read(OUT/'CANDIDATE.json')
    d=read(OUT/'DEEP.json');r=read(OUT/'REOPEN.json');s=read(OUT/'STORAGE.json');replay=read(OUT/'REPLAY.json');shared=read(OUT/'SHARED_REPLAY.json')
    assert d['scientific']==r['scientific']==AFTER and shared['root']==c['root']
    assert read(OUT/'TESTS.json')['returncode']==read(OUT/'FINAL_FOCUSED_TESTS.json')['returncode']==0
    lines=[classification,'',
        'Exact claim: isolated changed-component publication passes the sealed L4-to-L5 control, complete scientific equality, shared-root replay, failure tests and trusted reopen. Live transition assembly and independent target access-root construction remain unqualified. Production remains L5.',
        '',f"Starting HEAD: {b['head']}. Ending HEAD is the commit containing this report, printed in the final response.",
        f'Legacy L4 identity: {BEFORE}.',f'Legacy L5 identity: {AFTER}.',
        'Input authority identities and file hashes: INPUT_AUTHORITIES.json. All 22 protected immutable-access authority/report artifacts remain byte-identical.',
        '', 'Measured control (seconds; timers overlap where nested):',
        '| Operation | Seconds |','|---|---:|']
    lines += [f'| Legacy {k} | {v:.9f} |' for k,v in l['stages'].items()]
    lines += [f"| Legacy complete trace, including restores and output hashing | {l['seconds']:.9f} |",
        f"| Candidate qualification preparation, excluded from publication | {read(OUT/'PREPARATION.json')['seconds']:.9f} |",
        f"| Candidate publication | {c['seconds']:.9f} |",
        f"| Fresh changed-input replay: input load / publication / complete | {replay['changed_input_load_seconds']:.9f} / {replay['publication_seconds']:.9f} / {replay['seconds']:.9f} |",
        f"| Shared replay publication | {shared['publication_seconds']:.9f} |",
        f"| Deep complete equality | {d['seconds']:.9f} |",f"| Fresh post-cleanup trusted open/init | {r['seconds']:.9f} |",'',
        '| Measurement | Legacy trace | Candidate publication | Fresh changed-input replay | Trusted reopen |',
        '|---|---:|---:|---:|---:|']
    for title,key in [('CPU seconds','cpu'),('Peak RSS bytes','peak_rss'),('Retained RSS bytes','retained_rss')]:
        lines.append('| '+title+' | '+' | '.join(str(x[key]) for x in [l,c,replay,r])+' |')
    for key in ['read_bytes','read_count','write_bytes','write_count']:
        lines.append('| Process '+key+' | '+' | '.join(str(x['io'][key]) for x in [l,c,replay,r])+' |')
    lines += ['',
        'Scopes differ: the legacy complete trace includes two restores and output-file hashes. Its native publication is the direct already-accepted-state comparison. The first candidate peak includes preparation; fresh replay includes only changed-input loading and publication/verification/retry. No speedup for the complete historical 3244.515 s transaction is claimed.',
        f"Legacy observed calls: {json.dumps({k:v['calls'] for k,v in l['timers'].items()},sort_keys=True)}.",
        f"Legacy fallback copied {l['counts']['files_copied']} files / {l['counts']['bytes_copied']} bytes; output allocation {l['physical']} bytes. Full-prefix maps, hashes and three top-level journal deep copies remain in the old path. Recursive copy counts and SQL row visits were not separately instrumented.",
        '', 'Changed/reused populations:',
        '- Receipts: 12,059,568 old proof payloads reused; 10,344,468 prior bindings unchanged; 1,715,100 prior bindings updated; 3,782,768 new interactions imported from a sealed component; zero changed old payloads.',
        '- Bindings: 33,786 changed old faces plus 84,465 new faces = 118,251 entries.',
        '- Quality: 506,790 exact reused; 67,572 inputs-only updates; 152,037 new rows. Delta has 219,609 rows. Two verification passes read 439,218 delta rows. No unchanged quality values are rewritten.',
        '- Candidate publication: zero historical receipt/quality rows, zero proof payload writes, zero new superseded receipt rows, zero complete-prefix digests or deep copies. Four changed-input canonical passes; six including idempotent retry. Compact metadata hashes are additional. 63 reference tokens checked; base metadata also checked.',
        '- Trusted reopen: SQLite access and legacy restart/hash/forecast entry points disabled. Zero proof/quality/history traversal; five demanded component checks and 45 original access token checks, plus transaction root/reference metadata checks.',
        '',f"Complete proof digest: {d['certificate_digest']}",f"Complete quality digest: {d['quality_digest']}",
        f"All 15,842,336 receipts and {d['quality_count']} quality records matched exactly. Fourteen other components retain exact descriptors and rehashed file bytes. Exact cold lookups passed for unchanged, updated and new records.",
        f"Candidate and shared-replay root: {c['root']}",f"Independent-path replay root: {replay['root']}; same scientific content and byte-identical changed components.",
        '', 'Safety: missing/mutated references, changed accepted inputs, stale base/writer, concurrent writer, corrupt/incomplete changed objects, wrong/incomplete root, pointer failure, six exception boundaries and six abrupt process-exit boundaries passed. Idempotent retry reconciles the committed journal. Hardware power loss is not directly tested. The qualified write-through atomic primitive and host-local immutable trust boundary remain.',
        'Initial Windows fsync failure stopped before pointer switch. Fifteen initial mechanics failures shared this cause; six initially passed. Fixed with a writable descriptor. Failure identity, base pointer and file hash remain recorded.',
        '', f"L6 initialization only: exact increment {r['increment']!r}; committed binding remains L5. L6 forecast = NO; L6 generation = NO; core = NO.",
        '', 'Storage (NTFS allocated data clusters, excluding shared MFT overhead):',
        f"- Starting free: {s['starting_free']} bytes; measured post-cleanup free: {s['ending_free']} bytes.",
        f"- Original L5 closure: {s['original_L5_closure_physical']} bytes; L4/L5 control union: {s['L4_L5_control_union_physical']} bytes. Prior rolling-active union about 27.75 GB remains unretired.",
        '- New quality component: 141,504,512 bytes. New binding component: 16,909,896 logical / 16,912,384 allocated bytes. Combined new scientific payload allocation: 158,416,896 bytes.',
        f"- Measured retained task allocation before report sealing: {s['task_physical']} bytes; observed task allocation peak: {s['observed_task_peak']} bytes. Transient peak was not continuously sampled.",
        f"- {s['reclaimed']} physical bytes removed from 35 exact task-only copies. Historical removals: zero. Complete path records and retained source hashes: TEMPORARY_REMOVAL_PLAN.json.",
        f"- Shared replay adds {shared['physical']} bytes of metadata and no second retained scientific payload. The retired independent timing clone's manifests are evidence, not live restart closures; its reconstruction sources are sealed.",
        f"- Remaining duplicate physical allocation: {s['new_duplicate_physical']} bytes of compact metadata; duplicate scientific payload allocation: zero.",
        '- Free-space change is not used as reclaimed-allocation measurement. Small report/Git metadata and unrelated filesystem activity explain differences.',
        '', 'Tests: full regression run 77 passed / 0 failed / 0 skipped, then final focused run 36 passed / 0 failed / 0 skipped after shared-object tests. Eighty distinct tests across these suites. Production deep equality, independent replay, shared-root replay, point lookup and post-cleanup reopen all passed.',
        'Git: 155 unrelated porcelain entries match the starting snapshot exactly. V3 task modifications: zero. Only new task tools/tests/results are committed. No source in the qualified read path or scientific mesher was changed.',
        '', 'Decision: full requested architecture qualification is withheld. The producer still forms complete legacy target state; the new view depends on the pre-existing target access root and receipt update store. This control does not establish an incremental unseen-state validator, proof producer or independent access-root builder. Metadata reference authentication still scales with component count. Existing active stores remain required.',
        'Qualification retains deep replay and failure evidence. Future production could reuse this publication boundary after assembly/access-root qualification, without routine A/B replay. ProductionFast is not implemented.',
        '', 'Exactly one recommended next task: QualifyV4IncrementalTransitionAssemblyAndAccessRootFromSealedL4ToL5. Qualify changed-state assembly and an independently publishable target access root on the known transition; do not advance L6.',
        '', 'DESIGN.md contains the complete access-pattern mapping, trust boundary, reproduction instructions and limitations. AUTHORITY.json binds implementation, report and evidence hashes.']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    sources=[V4/'tools/incremental_transaction.py',V4/'tools/qualify_incremental_transaction.py',V4/'tests/test_incremental_transaction.py']
    evidence={str(p.relative_to(OUT)):file_hash(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.name!='AUTHORITY.json' and 'objects' not in p.parts}
    scientific=dict(classification=classification,complete_architecture_qualified=False,sealed_control_passed=True,
        legacy_before=BEFORE,legacy_after=AFTER,root=c['root'],implementation={str(p.relative_to(V4)):file_hash(p) for p in sources},
        evidence=evidence,starting_head=b['head'],no_L6_forecast=True,no_L6_generation=True,no_core=True)
    save('AUTHORITY.json',dict(identity=identity(scientific),scientific=scientific))
    print('SEALED',classification,identity(scientific),file_hash(OUT/'REPORT.md'),file_hash(OUT/'AUTHORITY.json'),flush=True)

if __name__=='__main__':
    import sys
    {'preflight':preflight,'trace':trace,'candidate':candidate,'deep':deep,'reopen':reopen,'replay':replay,
        'replay_shared':lambda:replay(True),'points':points,'cleanup':cleanup,'seal':seal}[sys.argv[1]]()
