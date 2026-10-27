"""Hash-verified immutable handoff of a retained generated transition."""
from pathlib import Path
import json, os
import immutable_state_access as access
from path_preserving_deduplication import file_hash, durable_json


def stage(source, destination, manifest, manifest_hash, base, adapter, held):
    source=Path(source).resolve(); manifest=Path(manifest).resolve()
    held.enter_context(access.guard(manifest))
    if file_hash(manifest)!=manifest_hash:raise ValueError('RetainedManifestMismatch')
    root=Path(__file__).resolve().parents[3]
    rows=json.loads(manifest.read_text(encoding='utf-8'))['files']
    recorded={str((root/r['path']).resolve()):r for r in rows}
    verified=[]
    for p in sorted(source.rglob('*')):
        if not p.is_file():continue
        r=recorded.get(str(p.resolve()))
        if r is None:raise ValueError('UnrecordedRetainedInput:'+str(p))
        held.enter_context(access.guard(p))
        if p.stat().st_size!=r['bytes'] or file_hash(p)!=r['sha256']:
            raise ValueError('RetainedInputMismatch:'+str(p))
        verified.append(r)
    read=lambda n:json.loads((source/n).read_text(encoding='utf-8'))
    from accepted_production_contract import plan_transition
    if read('GENERATION.json')['transition_plan']!=plan_transition(base,adapter):raise ValueError('RetainedTransitionPlanMismatch')
    if read('ADMISSION.json')['evidence']['binding']!=adapter.binding:raise ValueError('RetainedAdmissionBindingMismatch')
    names=('ADMISSION.json','DIRECT.json','LOCALIZATION.json','SCREEN_ARRAYS.npz','GENERATION.json',
           'FRESH_CERTIFICATE.json','GEOMETRY_DELTA.json','CHANGED_INPUTS.json','NEW_QUALITY.json',
           'AFFECTED_INCIDENCE.json','VOLUME_DELTA.json','GENERATED_NEXT.npz','PERSISTENT_GEOMETRY.json')
    paths=[source/n for n in names]+[p for p in (source/'objects').iterdir() if p.name!='new-proof-key-index.sqlite']
    for p in paths:
        if str(p.resolve()) not in recorded:raise ValueError('UnrecordedHandoff')
        dest=destination/p.relative_to(source);dest.parent.mkdir(parents=True,exist_ok=True)
        os.link(p,dest)  # Read-only inputs only; output names are never linked.
        held.enter_context(access.guard(dest))
    durable_json(destination/'RETAINED_INPUT.json',dict(manifest=str(manifest),manifest_sha256=manifest_hash,
        source=str(source),parent=base.scientific,verified_files=verified,linked_files=len(paths),
        geometry_regenerated=False,proof_payloads_copied=False,derived_index_rebuilt=True))
