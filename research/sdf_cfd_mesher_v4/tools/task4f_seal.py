"""Seal the bounded audit; no scientific computation or Git mutation."""
from task4f_common import *
import ast,shutil,psutil

def run():
    assert elapsed()<3600,'TaskWallTimeExceeded'
    assert 'Ran 25 tests' in (OUT/'TESTS.txt').read_text() and (OUT/'TESTS.txt').read_text().rstrip().endswith('OK')
    assert git('rev-parse','HEAD')==read(OUT/'CLOCK.json')['head']
    assert not git('diff','--cached','--name-only')
    for process in psutil.process_iter(['pid','name','cmdline']):
        if process.pid==os.getpid():continue
        if not (process.info['name'] or '').lower().startswith('python'):continue
        command=' '.join(process.info['cmdline'] or [])
        assert not ('task4f_' in command and str(ROOT) in command),command
    sources=sorted([*ROOT.glob('tools/task4f_*.py'),ROOT/'tests/test_task4f_audit.py'])
    for p in sources:ast.parse(p.read_text())
    repairs=read(OUT/'AUDIT_REPAIRS.json');repairs['study_instrumentation_repairs'].append(dict(reproducer='Generic len(mapping) reported13 operator dictionary fields instead of62180 cells',fix='Count len(mapping[N]) for the gains timer; rerun isolated profile',regression='test_operator_entity_counter'))
    save('AUDIT_REPAIRS.json',repairs)
    ignored={'AUTHORITY.json','SEAL.json','SOURCE_AND_ARTIFACTS.json'}
    evidence=sorted(p for p in OUT.iterdir() if p.is_file() and p.name not in ignored)
    rel=lambda p:p.relative_to(ROOT).as_posix()
    paths=[rel(p) for p in sources+evidence]+['results/task4f/'+n for n in sorted(ignored)]
    save('SOURCE_AND_ARTIFACTS.json',dict(paths=sorted(paths),scope='V4 Task4F audit only; no production source changes',total_bytes_before_seal=sum(p.stat().st_size for p in sources+evidence),free_disk_bytes=shutil.disk_usage(ROOT).free))
    evidence.append(OUT/'SOURCE_AND_ARTIFACTS.json')
    summary=read(OUT/'SUMMARY.json');clock=read(OUT/'CLOCK.json')
    authority=dict(task='AuditV4RuntimeWorkAndBruteForcePaths',classification=summary['classification'],starting_head=clock['head'],branch=git('branch','--show-current'),
        prior_Task4E_authority=file_hash(E/'AUTHORITY.json'),report_sha256=file_hash(OUT/'report.md'),
        source_sha256={rel(p):file_hash(p) for p in sources},evidence_sha256={rel(p):file_hash(p) for p in evidence},
        scientific_accounting=read(OUT/'DETERMINISM.json'),preservation=read(OUT/'PRESERVATION.json'),tests=summary['tests'],
        elapsed_at_authority_seal=elapsed(),scientific_cutoff=clock['scientific_cutoff'],task_deadline=clock['task_deadline'],
        production_optimization_integrated=False,full_build_performed=False,limitations=['Bounded per-kernel timings are not a reconstructed allocation of all historical seconds.','The ten whole-patch cache evictions are proved; sub-entity reuse is an audit prototype.','Exact-input unchanged counts are relative to each trial baseline; reuse still needs a prior proof/result receipt.','No full end-to-end speedup is claimed.'])
    save('AUTHORITY.json',authority)
    save('SEAL.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),task_elapsed_seconds=elapsed(),report_sha256=file_hash(OUT/'report.md'),authority_sha256=file_hash(OUT/'AUTHORITY.json'),scientific_workers_remaining=0,within_task_limit=True,git_commit_follows_sealing=True))
    print(json.dumps(read(OUT/'SEAL.json'),indent=2))

if __name__=='__main__':run()
