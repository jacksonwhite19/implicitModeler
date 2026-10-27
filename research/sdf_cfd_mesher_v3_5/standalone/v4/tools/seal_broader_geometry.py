"""Seal compact cohort evidence and lossless archive parts; exclude self-hashes."""
from pathlib import Path
import hashlib,json,subprocess
from summarize_broader_geometry import OUT,ROOT,read,save
from broader_geometry_cohort import sha


def run():
    result=read(OUT/'RESULTS.json')
    authority_inputs=read(OUT/'COHORT_MANIFEST.json')['cases']
    for case in authority_inputs:
        source=case['source']
        assert sha(ROOT/source['path'])==source['sha256']
        assert sha(ROOT/source['authority'])==source['authority_sha256']
    for path in sorted((OUT/'archives').glob('*.json')):
        index=read(path)
        assert index['lossless']
        h=hashlib.sha256()
        for part in index['parts']:
            p=OUT/part['path'];assert sha(p)==part['sha256']
            with p.open('rb') as stream:
                for b in iter(lambda:stream.read(4*1024*1024),b''):h.update(b)
        assert h.hexdigest()==index['archive_sha256']
    excluded={'AUTHORITY.json','SHA256SUMS.txt'}
    evidence={}
    for p in sorted(OUT.rglob('*')):
        if not p.is_file() or p.name in excluded:continue
        relative=p.relative_to(OUT)
        if 'checkpoints' in relative.parts or 'assembly' in relative.parts or p.suffix=='.zip':continue
        evidence[relative.as_posix()]=sha(p)
    tools=['qualify_broader_geometry.py','broader_geometry_core.py','broader_geometry_assembly.py',
           'broader_geometry_cohort.py','broader_geometry_forensics.py','broader_geometry_replay.py',
           'broader_sf2_control.py','summarize_broader_geometry.py','report_broader_geometry.py',
           'archive_broader_geometry.py','seal_broader_geometry.py','repair_bulk_core.py']
    sources={f'tools/{name}':sha(ROOT/'tools'/name) for name in tools}
    sources['tests/test_broader_geometry.py']=sha(ROOT/'tests/test_broader_geometry.py')
    save(OUT/'AUTHORITY.json',dict(classification=result['classification'],
        base_commit='e63b35cfb67465ae209b844526d107a4fc3d1ff5',
        report_sha256=sha(OUT/'report.md'),evidence=evidence,implementation=sources,
        scientific_limits=dict(general_SDF_complete_gate='failed frozen shell determinant',
            new_numerical_cases=0,full_production_domains=1,compression_only_complete_control=False),
        production_authority_changed=False,automatic_remesh_default_changed=False,
        source_authorities_unchanged=True,V3_modifications=0))
    (OUT/'SHA256SUMS.txt').write_text(''.join(f'{sha(OUT/name)}  {name}\n' for name in ('report.md','AUTHORITY.json')))
    print((OUT/'SHA256SUMS.txt').read_text())


if __name__=='__main__':run()
