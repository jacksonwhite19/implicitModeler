"""Bounded retained-input storage checks. Never generates or publishes a front."""
import argparse
import json
import mmap
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
OUT = V4/'results/single_quality_storage'


def baseline():
    from path_preserving_deduplication import durable_json, file_hash
    from check_current_front_encoding import allocation
    status = subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT,text=True,encoding='utf-8').splitlines()
    status = [s for s in status if s != '?? research/sdf_cfd_mesher_v4/tools/check_single_quality_storage.py']
    sources = ['critical_transaction.py','critical_prepare.py','critical_assembly.py','composable_publication.py',
               'composable_state.py','incremental_transaction.py','critical_quality_codec.py',
               'incremental_pair_storage.py','current_front_dependencies.py','recursive_replay_retention.py']
    files = [V4/'tools'/n for n in sources]
    files += [ROOT/'src/sdf/aerospace'/n for n in ('wing.rs','airfoil.rs','naca_spline.rs')]
    roots = [V4/'results'/n for n in ('wing_receipt_resume/A','current_front_transition_round_trip','wing_continuation_layers6to8')]
    payloads = [p for root in roots for p in root.rglob('*') if p.is_file()]
    census = allocation(payloads)
    OUT.mkdir(exist_ok=False)
    durable_json(OUT/'BASELINE.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),status=status,
        files=[dict(path=str(p.relative_to(ROOT)),sha256=file_hash(p),bytes=p.stat().st_size) for p in files if p.exists()],
        census=census,initial_C=shutil.disk_usage(ROOT).free,initial_Z=shutil.disk_usage('Z:/').free,start=time.time()))
    print(json.dumps({k:v for k,v in census.items() if k!='records'}),flush=True)


class RetainedRows:
    """Existing canonical top-level framing, adapted from retained_rows.

    Diagnostic only; full reconstructed byte hash must equal the original file.
    Never installs an unverified file as a production authority.
    """
    def __init__(self,path):self.path=Path(path)
    def __iter__(self):
        decoder=json.JSONDecoder()
        with self.path.open('rb') as stream, mmap.mmap(stream.fileno(),0,access=mmap.ACCESS_READ) as data:
            matches=re.finditer(rb'(?m)^  \{',data);current=next(matches,None)
            while current is not None:
                following=next(matches,None);end=following.start() if following else len(data)-2
                text=data[current.end()-1:end].decode('utf-8').strip()
                row,finish=decoder.raw_decode(text)
                if text[finish:].strip() not in ('',','):raise ValueError('UnsupportedLegacyQualityFraming')
                yield row;current=following


def verify(*, recheck=False):
    from contextlib import ExitStack
    import psutil
    import single_quality_component as codec
    from check_current_front_encoding import allocation
    from path_preserving_deduplication import durable_json, file_hash, metadata
    from sdf_cfd_mesher_v4.common.authority import identity
    source=V4/'results/wing_continuation_layers6to8/A/L6/QUALITY_DELTA.json'
    plan=json.loads((source.parent/'CANDIDATE_PLAN.json').read_text(encoding='utf-8'))
    acceptance=json.loads((source.parent/'CANDIDATE_ACCEPTANCE.json').read_text(encoding='utf-8'))
    failed=json.loads((V4/'results/wing_continuation_layers6to8/FAILED_TRANSACTION.json').read_text(encoding='utf-8'))
    old_db=failed['pending'][0]
    run=OUT/('retained_L6_rechecked' if recheck else 'retained_L6');run.mkdir(exist_ok=False);path=run/'QUALITY_DELTA.json'
    start=time.perf_counter();times={};peak_alloc=0
    # Bounded diagnostic allowance is the two existing representations, not an
    # increased production cap. Reuse the existing minimum-free requirement.
    resources=json.loads((V4/'results/wing_continuation_layers6to8/RESOURCE_PREFLIGHT.json').read_text(encoding='utf-8'))['resources']
    budget=source.stat().st_size+old_db['bytes']
    def check():
        nonlocal peak_alloc
        size=sum(p.stat().st_size for p in run.rglob('*') if p.is_file());peak_alloc=max(peak_alloc,size)
        if size>budget or shutil.disk_usage(ROOT).free<resources['minimum_free_bytes']:
            raise ValueError('BoundedStorageCheckResourceLimit')
    with codec.access.guard(source):
        original=file_hash(source)
        if original!=plan['components']['quality']['delta']:raise ValueError('RetainedQualityIdentityMismatch')
        if codec.access.token(source)!=plan['references'][str(source.resolve())]['token']:
            raise ValueError('RetainedQualityTokenMismatch')
        rows=RetainedRows(source);tick=time.perf_counter();expected=codec.hashes(iter(rows))
        times['direct_hash']=time.perf_counter()-tick
        if expected['sha256']!=original or expected['identity']!=acceptance['quality_delta'] or expected['count']!=acceptance['quality_count']:
            raise ValueError('RetainedQualityLogicalMismatch')
        print('retained hashes verified',expected['count'],flush=True)
        tick=time.perf_counter()
        revalidation=None
        if recheck:
            previous=OUT/'retained_L6/QUALITY_DELTA.json'
            body=json.loads(previous.read_text(encoding='utf-8'));existing=Path(body['payload']['path'])
            with codec.access.guard(previous),codec.access.guard(existing):
                if codec.access.token(existing)!=body['payload']['token'] or file_hash(existing)!=body['payload']['sha256']:
                    raise ValueError('ChangedDiagnosticPayload')
                if body['logical']!=expected:raise ValueError('DiagnosticLogicalMismatch')
                from contextlib import closing
                with closing(codec.sqlite3.connect(existing.as_uri()+'?mode=ro&immutable=1',uri=True)) as db:
                    codec._verify(db,expected)
                revalidation=dict(previous=str(previous),previous_sha256=file_hash(previous),
                    previous_encoder=body['encoder_sha256'],current_encoder=file_hash(codec.__file__),
                    scope='NEW diagnostic envelope only after full physical/ordinal/canonical/logical revalidation. No sealed-state source lock replaced.')
                body['encoder_sha256']=revalidation['current_encoder'];durable_json(path,body)
                written=dict(**expected,physical_bytes=existing.stat().st_size+path.stat().st_size,payload_reused=True)
        else:written=codec.write(path,rows,progress=check)
        times['write_and_readbacks']=time.perf_counter()-tick
        with codec.snapshot(path) as snap:
            tick=time.perf_counter();compared=0
            for direct,bound in zip(rows,snap['rows'],strict=True):
                if codec.canonical(direct)!=codec.canonical(bound):raise ValueError('RetainedQualityRowMismatch')
                compared+=1
            times['all_row_comparison']=time.perf_counter()-tick
            if snap['semantic']!=original or codec.quality_identity(snap['rows'])!=acceptance['quality_delta']:
                raise ValueError('BoundQualityIdentityMismatch')
            payload=metadata(snap['rows'].path,content=True)
        command=[sys.executable,'-B','-c',
            'import sys,json; from single_quality_component import snapshot,quality_identity; '
            'exec("with snapshot(sys.argv[1]) as s:\\n print(json.dumps(dict(count=len(s[\\\"rows\\\"]),identity=quality_identity(s[\\\"rows\\\"]),semantic=s[\\\"semantic\\\"])))")',str(path)]
        tick=time.perf_counter();fresh=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',check=True)
        times['fresh_process']=time.perf_counter()-tick
        fresh_record=json.loads(fresh.stdout)
        if fresh_record!=dict(count=expected['count'],identity=expected['identity'],semantic=original):
            raise ValueError('FreshQualityReconstructionMismatch')
        if file_hash(source)!=original:raise ValueError('RetainedQualityChanged')
    measured=allocation([path,Path(payload['path'])]);measured.pop('records')
    old_total=source.stat().st_size+old_db['bytes']
    result=dict(passed=True,source=str(source),source_sha256=original,source_bytes=source.stat().st_size,
        old_sqlite_bytes=old_db['bytes'],old_sqlite_status='Preserved interrupted publication; not a qualified authority.',
        old_combined_bytes=old_total,logical=expected,written=written,all_rows_compared=compared,fresh_process=fresh_record,
        candidate_scientific=plan['scientific'],component_identity_unchanged=True,payload=payload,retained=measured,
        avoided_logical_bytes=old_total-measured['logical_bytes'],diagnostic_storage_allowance=budget,
        sampled_logical_peak=peak_alloc,revalidation=revalidation,times=times,elapsed_s=time.perf_counter()-start,
        peak_process_working_set=psutil.Process().memory_info().peak_wset,
        final_C=shutil.disk_usage(ROOT).free,final_Z=shutil.disk_usage('Z:/').free,
        new_layer=False,production_publication=False,sealed_state_migration=False,reclaimed_bytes=0)
    durable_json(OUT/'VERIFIED.json',result);print(json.dumps(result),flush=True)


def capsule():
    """Bounded actual capsule rows through old and current legacy transactions."""
    from collections import Counter
    import critical_transaction as current
    import incremental_transaction as legacy
    from path_preserving_deduplication import durable_json, file_hash
    from sdf_cfd_mesher_v4.common.authority import identity
    source=V4/'results/recursive_layers9to11/L11/QUALITY_DELTA.json'
    plan_path=source.parent/'CANDIDATE_PLAN.json'
    plan=json.loads(plan_path.read_text(encoding='utf-8'))
    if identity(plan['content'])!=plan['scientific'] or plan['scientific']!='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870':
        raise ValueError('CapsuleControlContentMismatch')
    expected=plan['components']['quality']['delta']
    capsule_scientific=plan['scientific']
    selected=[];counts=Counter();start=time.perf_counter();run=OUT/'capsule_control_verified';run.mkdir(exist_ok=False)
    with legacy.access.guard(source):
        if file_hash(source)!=expected:raise ValueError('CapsuleRetainedQualityMismatch')
        for row in RetainedRows(source):
            kind=(row['key'].split(':',1)[0],row['payload'] is None)
            if counts[kind]<8:selected.append(row);counts[kind]+=1
    digests=[];stats=[]
    for name,cls in (('original',legacy.Store),('current_legacy',current.Store)):
        store=cls(run/name);base=store.initialize(dict(scientific='storage-control',epoch=legacy.access.epoch(),plan={'references':{}}))
        plan=dict(scientific='storage-control-child',references={});bindings={}
        accepted=dict(base=base,plan=identity(plan),passed=True,quality_delta=identity(selected),quality_count=len(selected),binding_patch=identity(bindings))
        root=store.publish(base,plan,selected,bindings,accepted,identity(accepted))
        with store.open(root) as body:digests.append(file_hash(body['quality_delta']))
        stats.append(dict(store.stats))
    if digests[0]!=digests[1]:raise ValueError('CapsuleLegacyQualityByteMismatch')
    result=dict(passed=True,source=str(source),source_sha256=expected,plan_sha256=file_hash(plan_path),
        scientific=capsule_scientific,rows=len(selected),
        categories=[dict(prefix=k[0],input_only=k[1],count=v) for k,v in counts.items()],
        database_sha256=digests[0],legacy_database_byte_equal=True,stats=stats,
        elapsed_s=time.perf_counter()-start,scope='Actual retained quality rows; synthetic transaction metadata only. No capsule layer, publication authority or deep recovery.')
    durable_json(OUT/'CAPSULE_VERIFIED.json',result);print(json.dumps(result),flush=True)


def finish():
    """Proportionate status, source, scientific-input and allocation check."""
    from path_preserving_deduplication import durable_json, file_hash
    from check_current_front_encoding import allocation
    from record_receipt_resume import disk
    baseline=json.loads((OUT/'BASELINE.json').read_text(encoding='utf-8'))
    owned=['research/sdf_cfd_mesher_v4/'+n for n in ('CURRENT_TASK.md','README.md','SINGLE_QUALITY_STORAGE_REPORT.md',
        'tools/check_single_quality_storage.py','tools/single_quality_component.py','tools/critical_transaction.py',
        'tools/critical_prepare.py','tools/critical_assembly.py','tools/composable_publication.py',
        'tools/recursive_replay_retention.py','tests/test_single_quality_component.py','results/single_quality_storage/')]
    status=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT,text=True,encoding='utf-8').splitlines()
    untouched=[s for s in status if not any(s[3:]==p or s[3:].startswith(p) and p.endswith('/') for p in owned)]
    if untouched!=baseline['status']:raise ValueError('BaselineStatusChanged')
    changed={'critical_transaction.py','critical_prepare.py','critical_assembly.py','composable_publication.py','recursive_replay_retention.py'}
    sources=[]
    for row in baseline['files']:
        current=file_hash(ROOT/row['path']);intentional=Path(row['path']).name in changed
        if current!=row['sha256'] and not intentional:raise ValueError('ProtectedSourceChanged:'+row['path'])
        sources.append(dict(row,after=current,intentional=intentional))
    previous=json.loads((V4/'results/wing_continuation_layers6to8/BASELINE.json').read_text(encoding='utf-8'))
    kernel=[]
    for row in previous['protected']:
        if row['path'].replace('\\','/').startswith('src/sdf/'):
            if file_hash(ROOT/row['path'])!=row['sha256']:raise ValueError('DirtyKernelSourceChanged')
            kernel.append(row)
    prior_records=baseline['census']['records'];match_fields=['device','file_id','bytes','mtime_ns','cluster_bytes','links','security','streams']
    for row in prior_records:
        from path_preserving_deduplication import metadata
        current=metadata(row['path'])
        if any(current[k]!=row[k] for k in match_fields):raise ValueError('RetainedCampaignFileChanged:'+row['path'])
    failed=json.loads((V4/'results/wing_continuation_layers6to8/FAILED_TRANSACTION.json').read_text(encoding='utf-8'))
    for row in failed['pending']:
        if file_hash(row['path'])!=row['sha256']:raise ValueError('OriginalFailurePayloadChanged')
    retained=allocation([p for p in OUT.rglob('*') if p.is_file()]);retained.pop('records')
    roots=[V4/'results'/n for n in ('wing_receipt_resume/A','current_front_transition_round_trip','wing_continuation_layers6to8')]
    campaign=allocation([p for root in roots for p in root.rglob('*') if p.is_file()]);campaign.pop('records')
    evidence=dict(passed=True,starting_head=baseline['head'],branch=baseline['branch'],unrelated_entries=len(untouched),
        exact_status_preserved=True,selected_sources=sources,dirty_kernel_sources=kernel,retained_campaign_files=len(prior_records),
        metadata_scope=match_fields,pending_failure_hashes_preserved=True,retained=retained,campaign=campaign,
        disk=disk(),initial_C=baseline['initial_C'],initial_Z_free_not_available=baseline['initial_Z'],
        aliases_not_added=True,reclaimed_bytes=0,elapsed_s=time.time()-baseline['start'],
        source_artifact_hashes={str(p.relative_to(ROOT)):file_hash(p) for p in [ROOT/n for n in owned[:-1]] if p.is_file()},
        qualified_last_layer=5,production_resume=False,
        reproduction='baseline once; verify for a fresh full payload; recheck only for task-local final-source diagnostic revalidation; capsule; finish. Existing output locations are never overwritten.')
    durable_json(OUT/'EVIDENCE.json',evidence);print(json.dumps({k:v for k,v in evidence.items() if k not in ('source_artifact_hashes','selected_sources','dirty_kernel_sources')}),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline','verify','recheck','capsule','finish'])
    args=parser.parse_args()
    if args.mode=='baseline':baseline()
    elif args.mode=='capsule':capsule()
    elif args.mode=='finish':finish()
    else:verify(recheck=args.mode=='recheck')
