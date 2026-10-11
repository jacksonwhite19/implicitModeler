"""Curate reviewable evidence; preserve raw native objects on the local volume."""
import json,shutil
from pathlib import Path
from run_geometry_family import OUT
from sdf_cfd_mesher_v4.common.authority import file_hash


def run():
    dest=OUT/'sealed';dest.mkdir(exist_ok=True);index=[]
    names={'SHELL_RESULT.json','PARTIAL_SHELL_RESULT.json','GEOMETRY_AUTHORITY.json','TRACE.json','FAILURE.json','PERFORMANCE.json',
        'CORE_ATTEMPTS.json','TRIALS.json','CORE_SELECTED.json','COMPLETE_PERFORMANCE.json','ASSEMBLY.json','RESTART_INPUT.json','REPLAY_INPUT.json',
        'EVENT_REPLAY.json','MEMORY_TRACE.json','REGION_INTERACTIONS.json','LOCALITY.json','REPAIR_SCOPE.json','DETERMINISM.json','CORE_DETERMINISM.json','RESULT.json','BOUNDED_REPLAY.json','REPAIR.json','GENERATION.json','HARD_QUALITY.json','CORE_INPUT.json'}
    for case in sorted(OUT.iterdir()):
        if not case.is_dir() or case.name=='sealed':continue
        for p in sorted(case.rglob('*.json')):
            # SQLite/native object populations already carry their own exact
            # checksums. Do not expand or duplicate them into a review bundle.
            if 'objects' in p.parts:continue
            selected=p.name in names or p.name=='CURRENT.json'
            indexed=selected or p.name=='OPERATIONS.json' or p.name.startswith(('PROPOSAL_','FIXED_SUPPORT_','BranchAwareCorrection_','ContinuationAttempt_','DEPENDENCY_GROUPS_','FINAL_INCIDENCE_','CORRECTION_','FIRST_LAYER','DISCOVERY_','COUPLED_','JOURNAL_','EXECUTION_SOURCE_','REJECTED_PROOF_LIFETIME_'))
            if not indexed:continue
            relative=p.relative_to(OUT);entry=dict(path=str(relative),bytes=p.stat().st_size,sha256=file_hash(p))
            target=dest/relative
            if selected:
                target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
            else:entry['storage']='Local raw evidence; content hash sealed here'
            if p.name=='CURRENT.json':
                pointer=json.loads(p.read_text());envelope=p.parent/pointer['checkpoint']
                shutil.copyfile(envelope,target.parent/envelope.name)
                entry['checkpoint']=dict(path=str(envelope.relative_to(OUT)),sha256=file_hash(envelope),
                    components=json.loads(envelope.read_text())['components'])
            index.append(entry)
    (OUT/'ARTIFACT_INDEX.json').write_text(json.dumps(dict(storage='Raw mesh arrays and immutable native objects remain local; sealed contains review evidence and exact object references.',files=index),indent=2,sort_keys=True)+'\n',newline='\n')


if __name__=='__main__':run()
