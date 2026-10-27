"""Journaled removal of an explicit successful-only retention allowlist.

Recovery hardlinks retain the original allocations until post-cleanup restore
verification succeeds. The final purge alone releases their data clusters.
"""
import hashlib
import json
import os
from pathlib import Path
from path_preserving_deduplication import metadata,physical_key,file_hash,durable_json,locked,handle_hash

def validate_plan(plan,root):
    root=Path(root).resolve()
    if plan.get('successful') is not True:raise ValueError('SuccessfulLayerRequired')
    if not plan.get('compact_sha256') or not plan.get('inventory_sha256'):raise ValueError('UnsealedPlan')
    seen=set()
    for r in plan['actions']:
        p=Path(r['path']).resolve()
        if not p.is_relative_to(root) or p==root or 'checkpoints' in p.parts:raise ValueError('ProtectedPath')
        if str(p) in seen:raise ValueError('DuplicateAction')
        seen.add(str(p))
        if r['classification'] not in ('C','E') or r.get('restore_dependency') or r.get('next_layer_dependency') or r.get('failure_evidence'):
            raise ValueError('ProtectedObject')
        proof=r.get('removal_proof',{})
        if proof.get('sha256')!=r['sha256'] or proof.get('kind') not in ('complete_regeneration','exact_retained_duplicate'):
            raise ValueError('UnprovenRemoval')
    for r in plan['actions']:
        proof=r['removal_proof']
        if proof['kind']=='exact_retained_duplicate' and str(Path(proof['canonical']).resolve()) in seen:
            raise ValueError('CanonicalAlsoRemoved')

def prepare(plan,root,recovery,interrupt=lambda stage:None):
    validate_plan(plan,root);recovery=Path(recovery).resolve();recovery.mkdir(parents=True,exist_ok=True)
    digest=hashlib.sha256(json.dumps(plan,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    records=[]
    for i,r in enumerate(plan['actions']):
        p=Path(r['path']);backup=recovery/f'{i:04d}.payload';journal=recovery/f'{i:04d}.json'
        if journal.exists():
            record=json.loads(journal.read_text())
            if record['plan']!=digest:raise ValueError('ForeignRecoveryJournal')
        else:
            record=dict(plan=digest,original=r,backup=str(backup),stage='prepared');durable_json(journal,record)
        if not p.exists():
            if not backup.exists() or file_hash(backup)!=r['sha256']:raise ValueError('MissingRecoveryPayload')
            records.append(record);continue
        proof=r['removal_proof']
        if proof['kind']=='exact_retained_duplicate' and file_hash(proof['canonical'])!=r['sha256']:raise ValueError('CanonicalMismatch')
        with locked(p) as h:
            now=metadata(p)
            if physical_key(now)!=physical_key(r) or now['bytes']!=r['bytes'] or handle_hash(h)!=r['sha256']:raise ValueError('StaleRemovalObject')
            if backup.exists():
                if not backup.samefile(p):raise ValueError('ForeignRecoveryPayload')
            else:os.link(p,backup)
            record['stage']='recoverable';durable_json(journal,record);interrupt('recoverable')
            p.unlink();record['stage']='removed_pending_verification';durable_json(journal,record);interrupt('removed')
        records.append(record)
    return records

def recover(recovery):
    records=[]
    for j in sorted(Path(recovery).glob('*.json')):
        r=json.loads(j.read_text());p=Path(r['original']['path']);b=Path(r['backup'])
        if not b.exists():
            if r['stage']=='purged':raise ValueError('RecoveryAlreadyCommitted')
            if p.exists() and file_hash(p)==r['original']['sha256']:continue
            raise ValueError('RecoveryPayloadMissing')
        if file_hash(b)!=r['original']['sha256']:raise ValueError('RecoveryPayloadCorrupt')
        if p.exists():
            if file_hash(p)!=r['original']['sha256']:raise ValueError('RecoveryDestinationChanged')
        else:os.link(b,p)
        r['stage']='recovered';durable_json(j,r);records.append(r)
    return records

def purge(recovery,verification,interrupt=lambda stage:None):
    required=('L1_restore','L2_restore','L3_restore','L4_binding_only','protected_hashes','regeneration')
    if any(verification.get(k) is not True for k in required):raise ValueError('PostVerificationRequired')
    records=[]
    for j in sorted(Path(recovery).glob('*.json')):
        r=json.loads(j.read_text());b=Path(r['backup'])
        if Path(r['original']['path']).exists():raise ValueError('RemovalNotApplied')
        if b.exists():
            if file_hash(b)!=r['original']['sha256']:raise ValueError('RecoveryPayloadCorrupt')
            r['stage']='purge_authorized';r['verification']=verification;durable_json(j,r)
            interrupt('purge_authorized');b.unlink()
        elif r['stage'] not in ('purge_authorized','purged'):raise ValueError('MissingRecoveryPayload')
        r['stage']='purged';durable_json(j,r);records.append(r)
    return records
