"""Explicitly release an unpinned, completed study working root.

Scientific components, checkpoint envelope, and former CURRENT pointer remain.
Only derived map objects become unreachable. This copy no longer promises
restart; preserved external inputs and qualified successor copies remain.
Never use on a sealed result or an in-flight/replay-pinned working store.
"""
import argparse,json,os
from pathlib import Path
from sdf_cfd_mesher_v4.advancing_front import block_retention as retention

ROOT=Path(__file__).resolve().parents[1]/'results/canonical_block_authenticated_storage'

def run(path,expected_root,evidence):
    path=Path(path).resolve();evidence=Path(evidence).resolve()
    if not path.is_relative_to(ROOT.resolve()) or path.name!='checkpoints':raise ValueError('WorkingReleaseScope')
    if (ROOT/'AUTHORITY.json').exists():raise ValueError('StudyAlreadySealed')
    if not evidence.is_file():raise ValueError('MissingReleaseEvidence')
    if json.loads(evidence.read_text()).get('passed') is not True:raise ValueError('ReleaseEvidenceNotQualified')
    with retention.locked(path):
        if (path/'BLOCK_TRANSACTION.json').exists() or retention.read(path/'BLOCK_PINS.json',{}):raise ValueError('WorkingRootStillPinned')
        current=retention.current(path)
        if current['root']!=expected_root:raise ValueError('UnexpectedWorkingRoot')
        former=path/'RELEASED_CURRENT.json'
        if former.exists():raise ValueError('AlreadyReleasedWorkingRoot')
        note=dict(root=current['root'],count=current['count'],reason='Completed disposable comparison copy; no active execution or replay pin',
                  evidence=str(evidence),scope='No raw scientific component deleted. Derived map files only. Former CURRENT retained; this unpinned working copy is inactive.')
        retention.write(path/'BLOCK_RELEASE.json',note)
        os.replace(path/'CURRENT.json',former)
        result=retention.collect(path)
        (path.parent/'WORKING_ROOT_RELEASE.json').write_text(json.dumps(dict(note=note,collection=result),indent=2)+'\n')
        print(json.dumps(result))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path',type=Path);p.add_argument('root');p.add_argument('evidence',type=Path)
    a=p.parse_args();run(a.path,a.root,a.evidence)
