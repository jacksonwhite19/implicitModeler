"""Seal bounded policy evidence after tests. No geometry operations."""
import json
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from audit_initial_boundary_layer_policy import OUT, V4, REPO, save
from initial_boundary_layer_policy import policy_identity
from sdf_cfd_mesher_v4.common.authority import file_hash, identity


def main():
    pre=json.loads((OUT/'PREFLIGHT.json').read_text())
    status=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
    owned=['research/sdf_cfd_mesher_v4/'+p for p in (
        'INITIAL_BOUNDARY_LAYER_SIZING_POLICY.md','tools/initial_boundary_layer_policy.py',
        'tools/audit_initial_boundary_layer_policy.py','tools/seal_initial_boundary_layer_policy.py',
        'tests/test_initial_boundary_layer_policy.py','results/geometry_independent_initial_boundary_layer_sizing/')]
    unrelated=[row for row in status if not any(row[3:].replace('\\','/').startswith(p) for p in owned)]
    assert unrelated==pre['starting_dirty'], 'UnrelatedDirtyStateChanged'
    tests=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite').attrib
    assert int(tests['failures'])==0 and int(tests['errors'])==0
    save('RESOURCES.json',dict(preflight_to_seal_seconds=time.time()-pre['start_epoch'],
        starting_free_bytes=pre['starting_free_bytes'],ending_free_bytes=shutil.disk_usage(REPO).free,
        retained_task_bytes_before_metadata=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),
        bytes_deleted=0,deleted_paths=[],cleanup='No cleanup required. All old evidence and crash dumps untouched.',
        whole_task_peak_rss='NotMeasured',runtime_scope='Elapsed from preflight start; earlier research time not instrumented'))
    files=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json'):
            files.append(dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p),bytes=p.stat().st_size))
    for path in owned[:-1]:
        p=REPO/path;files.append(dict(path=path,sha256=file_hash(p),bytes=p.stat().st_size))
    authority=dict(classification='V4GeometryIndependentInitialBoundaryLayerSizingPolicyQualifiedParameterized',
        supported_claim='Parameterized model arithmetic, explicit provenance and binary64 progression arithmetic qualified; no arbitrary-body physical bound',
        policy_identity=policy_identity(),capsule_physical_request='Unbound',
        automatic_scale_policy='NotEstablished',progression_selection_policy='NotEstablished',
        true_yplus_guarantee=False,geometry_generated=False,prisms_generated=0,
        starting_head=pre['starting_head'],branch=pre['branch'],starting_dirty_count=155,
        unrelated_dirty_count=len(unrelated),unrelated_dirty_preserved=True,v3_task_modifications=0,
        tests=tests,files=files,evidence_identity=identity(files),
        next_task='QualifyV4InitialSurrogateScaleAndProgressionProfile')
    save('AUTHORITY.json',authority)
    save('HASHES.json',{name:file_hash(OUT/name) for name in ('report.md','AUTHORITY.json')})
    print(json.dumps(dict(hashes=json.loads((OUT/'HASHES.json').read_text()),
        policy_identity=policy_identity(),tests=tests,resources=json.loads((OUT/'RESOURCES.json').read_text())),indent=2))


if __name__=='__main__':main()
