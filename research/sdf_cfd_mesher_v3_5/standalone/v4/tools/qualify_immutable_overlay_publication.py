"""Sealed L2 persistence replay. Never forecasts, constructs, or certifies geometry.

Run prepare once, replay baseline/candidate_A/candidate_B in separate processes,
then restore each replay and historical checkpoints in fresh processes.
"""
import argparse
import ast
import collections
from contextlib import closing
import gc
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import time
import numpy as np
import psutil
from sdf_cfd_mesher_v4.advancing_front import native_checkpoint as nc
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import ReceiptDelta
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.common.authority import file_hash, array_hash, identity

V4=Path(__file__).resolve().parents[1]
ROOT=V4.parents[1]
OUT=V4/'results/immutable_receipt_overlay_delta_publication'
L1=V4/'results/production_adaptive_capsule_first_layer'
L2=V4/'results/production_adaptive_capsule_layer2'
AUDIT=V4/'results/production_qualification_storage_audit'
HEAD='e5b3ba76a435345df68d7199e0d740bea00474e3'
L1_ID='5b036bc4cd5180b9e4a7e3e6aafe57fe21c7b2ae1cae82cf3c42e446b1805fd0'
L2_ID='707d9726da9f6a6f7dedc688f4437777aeb2e8fa5cf0958ca6aed83f8f7bfa44'
UPDATE='56cff164272e65e16987ed0afe874590b6efb60d3bee5de8e9b89e987c9ff16a.sqlite'
SEALS={
    L2/'AUTHORITY.json':'1b182d7f872ea80c0a8a43b959d7f7fa5252be0c9aeff40cabb863133e555ae5',
    L2/'report.md':'920871bc44e6ff6cce08e54f57544264a60da4876a08fb5330fde37e7287cad1',
    AUDIT/'AUTHORITY.json':'404c8d7f26de536df836d5af03d18b9bde47a57eea041e51b471a86f6479bf39',
    AUDIT/'report.md':'f695894f737a7f922abd4dcc9dcdd9fe7d563322c5c5d147eb545e8926fd5110',
    L1/'AUTHORITY.json':'ec4c450daf520b903c289d02a35a7493a6bcf0a05191ef184f8f68c916aabab8',
    V4/'results/qualification_prism_progression_convention/REQUEST.json':'37375760bf77a7df2ff6e34ca0a87b6d437dcee604a5d8bf8ddb3c3483c40d80',
}


def save(name,value):
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/name).write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()


def unrelated():
    return [s for s in subprocess.check_output(['git','status','--porcelain=v1'],cwd=ROOT,text=True).splitlines()
            if 'research/sdf_cfd_mesher_v4/' not in s]


def inventory(path,hashes=False,tolerate_inaccessible=False):
    rows=[]
    for p in sorted(path.rglob('*')):
        try:is_file=p.is_file()
        except OSError as exc:
            if not tolerate_inaccessible:raise
            rows.append(dict(path=p.relative_to(path).as_posix(),inaccessible=str(exc)))
            continue
        if is_file:
            st=p.stat();row=dict(path=p.relative_to(path).as_posix(),bytes=st.st_size,
                device=st.st_dev,inode=st.st_ino,mtime_ns=st.st_mtime_ns,links=st.st_nlink)
            if hashes:row['sha256']=file_hash(p)
            rows.append(row)
    return rows


def envelope(path):
    pointer=json.loads((path/'CURRENT.json').read_text())
    assert file_hash(path/pointer['checkpoint'])==pointer['sha256']
    return pointer,json.loads((path/pointer['checkpoint']).read_text())


def prepare():
    assert git('rev-parse','HEAD')==HEAD
    assert len(unrelated())==155
    assert not (OUT/'PREPARED.json').exists(),'Do not overwrite a completed preparation'
    if (OUT/'BASELINE.json').exists():
        assert json.loads((OUT/'BASELINE.json').read_text())['unrelated']==unrelated()
    else:
        save('BASELINE.json',dict(head=HEAD,branch=git('branch','--show-current'),
            unrelated=unrelated(),disk_free=shutil.disk_usage(ROOT).free))
    print('verify sealed authorities and all audited L1/L2 files',flush=True)
    for p,h in SEALS.items():assert file_hash(p)==h,str(p)
    audit=json.loads((AUDIT/'AUTHORITY.json').read_text())
    assert file_hash(AUDIT/'FILES.json')==audit['evidence']['FILES.json']
    files=json.loads((AUDIT/'FILES.json').read_text())
    for i,row in enumerate(files):
        assert file_hash(ROOT/row['path'])==row['sha256'],row['path']
        if i%40==0:print('verified',i,'/',len(files),flush=True)
    from production_surface_authority import load_production_wall
    _,_,surface=load_production_wall(V4/'results/production_adaptive_capsule_surface_foundation/fresh')
    assert surface['surface_identity']=='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
    binding=json.loads((V4/'results/production_layer2_readonly_admission/AUTHORITY.json').read_text())['binding']
    assert binding['request_file']==SEALS[V4/'results/qualification_prism_progression_convention/REQUEST.json']
    # Seal both historical trees and record V3 metadata without changing either.
    save('INPUTS.json',dict(seals={p.relative_to(ROOT).as_posix():h for p,h in SEALS.items()},
        verified_audited_files=len(files),surface=surface,binding=binding,
        L1=inventory(L1),L2=inventory(L2),V3=inventory(V4.parent/'sdf_cfd_mesher_v3',tolerate_inaccessible=True)))
    source=subprocess.check_output(['git','show',HEAD+':research/sdf_cfd_mesher_v4/src/sdf_cfd_mesher_v4/advancing_front/native_checkpoint.py'],cwd=ROOT)
    (OUT/'baseline_native_checkpoint.py.txt').write_bytes(source)
    save('PREPARED.json',dict(passed=True,baseline_source_sha256=hashlib.sha256(source).hexdigest(),bytes_deleted=0))
    print('prepared',flush=True)


def baseline_table():
    source=(OUT/'baseline_native_checkpoint.py.txt').read_text()
    module=ast.parse(source)
    cls=next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='NativeCheckpointStore')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_table')
    namespace=dict(vars(nc));exec(compile(ast.Module(body=[method],type_ignores=[]),'<sealed baseline _table>','exec'),namespace)
    return namespace['_table']


def replay(label):
    baseline=label=='baseline'
    target=OUT/label/'checkpoints';assert not target.exists(),'Never overwrite replay'
    objects=target/'objects';objects.mkdir(parents=True)
    # The production transaction creates this update inside the destination.
    # Bind that staged path, not an additional external alias of the same file.
    os.link(L2/'A/checkpoints/objects'/UPDATE,objects/UPDATE)
    print(label,'restore original L2',flush=True)
    state=nc.NativeCheckpointStore(L2/'A/checkpoints').restart()
    assert scientific_hash(state)==L2_ID
    print(label,'original L2 state verified',flush=True)
    l1store=nc.NativeCheckpointStore(L1/'A/checkpoints')
    p,e=envelope(l1store.path);assert p['scientific']==L1_ID
    base=l1store._receipt(e['components']['certificate'])
    print(label,'L1 base verified',flush=True)
    update_path=objects/UPDATE
    with closing(sqlite3.connect(update_path.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        count=db.execute('SELECT COUNT(*) FROM receipts').fetchone()[0]
    assert count==3856405
    updates=nc.ReceiptTable(update_path,count,UPDATE.removesuffix('.sqlite'))
    assert nc.scientific_digest(updates)==updates.scientific
    delta=ReceiptDelta(base,updates)
    assert delta.scientific==state['certificate'].scientific and len(delta)==4839083
    print(label,'delta identity equals original certificate',flush=True)
    # Reconstruct only the sealed map representation. No transaction callbacks,
    # events, geometry, proof payloads, or dependency rows are regenerated.
    state['certificate']=nc.ReceiptOverlay(delta)
    state['quality']=nc.ReceiptOverlay(state['quality'])
    # The only substitutions above are untouched wrappers and the delta whose
    # exact count/digest was checked against the original. Publication below
    # and a fresh-process restart still verify the full required L2 identity.
    table_method=baseline_table() if baseline else nc.NativeCheckpointStore._table
    class MeasuredStore(nc.NativeCheckpointStore):
        def _table(self,value,content_only=False):
            tick=time.perf_counter()
            result=table_method(self,value,content_only=content_only)
            if value is state['certificate']:self.certificate_seconds=time.perf_counter()-tick
            if value is state['quality']:self.quality_seconds=time.perf_counter()-tick
            return result
        def _flat_table(self,value):
            result=super()._flat_table(value)
            if isinstance(value,nc.ReceiptOverlay):
                self.materialized.append(dict(rows=len(value),bytes=(self.objects/result['file']).stat().st_size,
                    role='certificate' if value is state['certificate'] else 'quality',file=result['file']))
            return result
    store=MeasuredStore(target);store.materialized=[]
    # Both controls begin with the already-created intermediate update object.
    # Same-volume hardlink: no receipt rows or container bytes copied here.
    before=inventory(target);tick=time.perf_counter();process=psutil.Process();io=process.io_counters()
    rss_before=process.memory_info().rss
    print(label,'publication start',flush=True)
    published=store.publish(state)
    seconds=time.perf_counter()-tick;io_after=process.io_counters();mem=process.memory_info()
    assert published==L2_ID
    pointer,env=envelope(target);after=inventory(target,True)
    before_ids={(r['device'],r['inode']) for r in before}
    new=[r for r in after if (r['device'],r['inode']) not in before_ids]
    # Hardlinks have a new path but no new payload allocation/write.
    new_payload=[r for r in new if r['links']==1]
    logical=sum(r['bytes'] for r in after)
    by_content=collections.defaultdict(list)
    for r in after:by_content[(r['bytes'],r['sha256'])].append(r)
    duplicate=sum(size*(len(rows)-1) for (size,_),rows in by_content.items())
    result=dict(label=label,scientific=published,pointer=pointer,certificate=env['components']['certificate'],
        publication_seconds=seconds,receipt_publication_seconds=store.certificate_seconds,
        quality_publication_seconds=store.quality_seconds,rss_before_publication=rss_before,
        peak_rss=mem.peak_wset,retained_rss=mem.rss,publication_io_write_bytes=io_after.write_bytes-io.write_bytes,
        materialized=store.materialized,receipt_bytes_written=sum(r['bytes'] for r in store.materialized),
        checkpoint_new_payload_bytes=sum(r['bytes'] for r in new_payload),retained_logical_bytes=logical,
        seeded_logical_bytes=sum(r['bytes'] for r in before),retained_duplicate_logical_bytes=duplicate,
        receipt_counts=dict(base_population=len(base),base_reused=982678,delta=len(updates),
            dependency_binding_updates=957448,new_interactions=2898957,final=len(delta),
            newly_materialized_certificate=sum(r['rows'] for r in store.materialized if r['role']=='certificate')),
        full_flattening_avoided=env['components']['certificate']['encoding']=='receipt_delta',
        inventory=after,bytes_deleted=0)
    save(label+'_PUBLICATION.json',result)
    print(label,'published',seconds,'seconds;',logical,'retained logical bytes',flush=True)


def restore(label):
    path={'historical_L1':L1/'A/checkpoints','historical_L2_A':L2/'A/checkpoints',
          'historical_L2_B':L2/'B/checkpoints'}.get(label,OUT/label/'checkpoints')
    assert path.exists()
    print(label,'cold-process restore start',flush=True)
    tick=time.perf_counter();state=nc.NativeCheckpointStore(path).restart();elapsed=time.perf_counter()-tick
    expected=L1_ID if label=='historical_L1' else L2_ID
    assert scientific_hash(state)==expected
    n=16895;layers=2 if label=='historical_L1' else 3
    coords=[array_hash(np.array([state['vertices'][f'layer:{k}:source:{i}'] for i in range(n)])) for k in range(layers)]
    assert coords==['2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c',
        '34ca5df3c122a7436caf8a475c1c8cf0fbe629a6afb26fe1387862ef2f884cf7',
        'f57dc72c0ee93cb95529539f5986a206ee315ba59ca886eb284408deb86896cd'][:layers]
    assert {c['layer'] for c in state['cells'].values()}==set(range(1,layers))
    assert len(state['cells'])==33786*(layers-1) and len(state['vertices'])==n*layers
    assert len(state['certificate'])==(1940126 if layers==2 else 4839083)
    # restart already verified object bytes, receipt identities and the complete
    # canonical state. Compare its bound component descriptors without another
    # redundant full rendering of millions of receipt rows.
    _,verified_envelope=envelope(path)
    component_digests={k:v['scientific'] for k,v in verified_envelope['components'].items()}
    result=dict(label=label,scientific=expected,restore_seconds=elapsed,coordinates=coords,
        cells=len(state['cells']),vertices=len(state['vertices']),faces=len(state['faces']),
        certificate_rows=len(state['certificate']),quality_rows=len(state['quality']),component_digests=component_digests,
        peak_rss=psutil.Process().memory_info().peak_wset,retained_rss=psutil.Process().memory_info().rss,
        passed=True)
    save(label+'_RESTORE.json',result);print(label,'restored',elapsed,'seconds',flush=True)


def compare():
    from analyze_qualification_storage import compare_rows
    _,e1=envelope(L1/'A/checkpoints');_,e2=envelope(L2/'A/checkpoints')
    base=L1/'A/checkpoints/objects'/e1['components']['certificate']['file']
    final=L2/'A/checkpoints/objects'/e2['components']['certificate']['file']
    tick=time.perf_counter()
    delta=compare_rows(L2/'A/checkpoints/objects'/UPDATE,final)
    prior=compare_rows(base,final)
    assert delta==dict(equal=3856405,changed=0,left_only=0,right_only=982678,changed_inputs_only=0,changed_other_fields=0)
    assert prior==dict(equal=982678,changed=957448,left_only=0,right_only=2898957,changed_inputs_only=957448,changed_other_fields=0)
    save('ROW_COMPARISON.json',dict(delta_to_final=delta,L1_to_final=prior,seconds=time.perf_counter()-tick,passed=True))
    print('exact row and dependency comparisons passed',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['prepare','replay','restore','compare'])
    parser.add_argument('--label');args=parser.parse_args()
    if args.mode=='prepare':prepare()
    elif args.mode=='replay':replay(args.label)
    elif args.mode=='restore':restore(args.label)
    else:compare()
