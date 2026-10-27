"""Inventory referenced logical bytes without decoding scientific populations."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'results'
OUT=ROOT/'canonical_block_authenticated_storage'

def inspect(path):
    pointer=json.loads((path/'CURRENT.json').read_text());envelope=path/pointer['checkpoint']
    manifest=json.loads(envelope.read_text());references={}
    def walk(value):
        if isinstance(value,dict):
            if 'file' in value and 'sha256' in value:
                name=value['file'];assert Path(name).name==name
                old=references.setdefault(name,value['sha256']);assert old==value['sha256']
            for item in value.values():walk(item)
        elif isinstance(value,list):
            for item in value:walk(item)
    walk(manifest['components'])
    auth=manifest['components']['certificate']['authenticated']
    sizes={name:(path/'objects'/name).stat().st_size for name in references}
    return dict(path=str(path),scientific=pointer['scientific'],root=auth['root'],
                referenced_object_count=len(sizes),referenced_object_logical_bytes=sum(sizes.values()),
                authenticated_map_bytes=sum(sizes[r['file']] for r in auth['files']),
                raw_receipt_sqlite_bytes=sum(n for k,n in sizes.items() if k.endswith('.sqlite') and not k.endswith('.merkle.sqlite')),
                compact_array_bytes=sum(n for k,n in sizes.items() if k.endswith('.npy')),
                checkpoint_envelope_bytes=envelope.stat().st_size,
                scope='Logical referenced component bytes; shared hard links counted once per checkpoint, not physical allocation; no historical unreferenced files included.')

def run():
    output=OUT/'CHECKPOINT_STORAGE_INVENTORY.json'
    saved={x['path']:x for x in json.loads(output.read_text())} if output.exists() else {}
    sources=[ROOT/'incremental_certificate_receipt_identity/capsule_production_control/final_8_9/checkpoints',
             ROOT/'incremental_certificate_receipt_identity/capsule_production_control/continuation_10/checkpoints',
             OUT/'capsule_production_control/capsule_7_9_retry/checkpoints',
             OUT/'capsule_production_control/capsule_10_migration/checkpoints']
    sources.extend(OUT/f'capsule_production_control/capsule_{layer}{suffix}/checkpoints' for layer in (11,12) for suffix in ('','_retry','_final'))
    for path in sources:
        if (path/'CURRENT.json').exists():saved[str(path)]=inspect(path)
        elif str(path) in saved and (path/'RELEASED_CURRENT.json').exists():
            saved[str(path)].update(measured_before_working_root_release=True,active_map_bytes_after_release=0)
    rows=list(saved.values())
    output.write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps(rows,indent=2))

if __name__=='__main__':run()
