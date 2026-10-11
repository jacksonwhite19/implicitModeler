"""Seal task evidence and preserve the prior authority and unrelated work."""
import sys
sys.dont_write_bytecode=True
import json,subprocess
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import file_hash
ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parents[1]
OUT=ROOT/'results/coupled_shell_remesh_determinant'
BASE='488073f41667cfa95e35c34a4bf71974303d6b87'


def git(*args):return subprocess.check_output(['git',*args],cwd=REPO).decode().splitlines()
def write(p,v):p.write_text(json.dumps(v,sort_keys=True,indent=2)+'\n',encoding='utf8')


def run():
    summary=json.loads((OUT/'SUMMARY.json').read_text());assert summary['checks']['passed']
    assert git('rev-parse','HEAD')==[BASE]
    original=(OUT/'INITIAL_STATUS.txt').read_text(encoding='utf-8-sig').splitlines()
    # Match the initial --short format: --porcelain normalizes dirty nested
    # checkout markers from 'm' to 'M' even when their actual state is unchanged.
    current=git('status','--short')
    unrelated=lambda rows:sorted(r for r in rows if 'research/sdf_cfd_mesher_v4/' not in r)
    assert unrelated(original)==unrelated(current),'UnrelatedDirtyStateChanged'
    v3=git('diff','--name-only','HEAD','--','research/sdf_cfd_mesher_v3');assert not v3
    write(OUT/'GIT_HANDLING.json',dict(base_head=BASE,branch=git('branch','--show-current')[0],
        unrelated_dirty_entries=len(unrelated(current)),unrelated_dirty_state_exact=True,
        v3_tracked_modifications=len(v3),v3_status=[r for r in current if 'research/sdf_cfd_mesher_v3' in r],
        scope='Pre-existing untracked V3 tree remains untracked. No V3 source edit was made. Only listed V4 task paths are staged.'))
    changed=git('diff','--name-only','HEAD','--','research/sdf_cfd_mesher_v4')
    added=git('ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4')
    paths=set(changed+added)
    for name in ('COMMITTED_PATHS.txt','AUTHORITY.json','SHA256SUMS.txt'):paths.add((OUT/name).relative_to(REPO).as_posix())
    (OUT/'COMMITTED_PATHS.txt').write_text('\n'.join(sorted(paths))+'\n',encoding='utf8')
    archives=json.loads((OUT/'archives/general_asymmetric_passage.json').read_text())
    assert archives['lossless']
    for part in archives['parts']:assert file_hash(OUT/part['path'])==part['sha256']
    evidence={}
    for path in sorted(OUT.rglob('*')):
        relative=path.relative_to(OUT)
        if not path.is_file() or {'checkpoints','assembly'}&set(relative.parts):continue
        if path.name in ('AUTHORITY.json','SHA256SUMS.txt') or path.suffix=='.zip':continue
        evidence[relative.as_posix()]=file_hash(path)
    sources={p:file_hash(REPO/p) for p in sorted(paths) if '/results/' not in p}
    authority=dict(classification=summary['classification'],base_head=BASE,summary=summary,
        report_sha256=file_hash(OUT/'report.md'),evidence_sha256=evidence,source_sha256=sources,
        archived_raw_evidence='archives/general_asymmetric_passage.json',
        final_scientific_runs=['general_asymmetric_passage/D','general_asymmetric_passage/E'],
        complete_continuous_proof_origin='general_asymmetric_passage/A/assembly/NEW_CERTIFICATE.json',
        unchanged_requirements=['wall','first layer','hard gates','certificate mathematics','native production authority','automatic remesh default'],
        broader_cohort_fully_qualified=False)
    write(OUT/'AUTHORITY.json',authority)
    (OUT/'SHA256SUMS.txt').write_text('\n'.join(file_hash(OUT/name)+'  '+name for name in ('report.md','AUTHORITY.json'))+'\n',encoding='utf8')


if __name__=='__main__':run()
