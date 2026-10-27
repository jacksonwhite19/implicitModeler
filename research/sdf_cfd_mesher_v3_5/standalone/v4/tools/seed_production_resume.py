"""Import a sealed native checkpoint using immutable shared backing files."""
import sys
sys.dont_write_bytecode=True
import json,os,time
from pathlib import Path
from run_production_scale_resume import OUT,native
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json


def verify_stopped_source(folder,pointer,rows):
    assert (folder/'SESSION.json').exists(), 'SourceWorkerMustBeStopped'
    published=[r for r in rows if r['event']=='checkpoint_published']
    if published:
        assert published[-1]['scientific']==pointer['scientific']
    else:
        # A stopped attempt may still own its unchanged imported checkpoint.
        # Component hashes and scientific authority are reverified by seed().
        imported=json.loads((folder/'SEED_AUTHORITY.json').read_text())
        assert imported['scientific']==pointer['scientific']
        assert imported['checkpoint_sha256']==pointer['sha256']


def seed(name,label,source_label=None):
    tick=time.monotonic()
    source=native.ROOT/'results/first_layer_determinant_applicability'/name/('B' if name=='separated_regions_production' else 'A')/'checkpoints'
    if source_label is not None:source=OUT/name/source_label/'checkpoints'
    target=OUT/name/label/'checkpoints'
    if (target/'CURRENT.json').exists():raise ValueError('SeedDestinationAlreadyPublished')
    pointer=json.loads((source/'CURRENT.json').read_text());manifest=source/pointer['checkpoint']
    assert file_hash(manifest)==pointer['sha256']
    envelope=json.loads(manifest.read_text())
    assert envelope['scientific']==pointer['scientific']
    if source_label is None:
        index=json.loads((native.ROOT/'results/first_layer_determinant_applicability/EVIDENCE_INDEX.json').read_text())
        for item in (source/'CURRENT.json',manifest):
            relative=item.relative_to(native.ROOT/'results/first_layer_determinant_applicability').as_posix()
            assert file_hash(item)==index[relative]['sha256']
    else:
        rows=(json.loads(line) for line in (source.parent/'PROGRESS.jsonl').read_text().splitlines())
        verify_stopped_source(source.parent,pointer,rows)
    authority_row=envelope['components']['authority']
    old_authority=json.loads((source/'objects'/authority_row['file']).read_text())
    implementation_path=native.ROOT/'tools/first_layer_metric_state.py'
    metric_hash=file_hash(implementation_path)
    import geometry_family_authority as geometry
    from production_scale_authority import cohort
    from native_mixed_face_certificate import SOURCE
    spec=next(s for s in cohort() if s['name']==name)
    expected=dict(schedule=native.g.q.array_hash(native.g.q.load_schedule().targets),
        quality=file_hash(Path(native.g.t.__file__)),certificate=file_hash(Path(native.g.s.cert.__file__)),
        certificate_filter=file_hash(SOURCE),sdf=geometry.Source(spec).source_identity)
    assert all(old_authority.get(k)==v for k,v in expected.items()), 'SeedScientificAuthorityMismatch'
    version_binding=dict(source_authority=old_authority,current_first_prism_metric=metric_hash)
    if old_authority.get('first_prism_metric')!=metric_hash:
        prior=native.ROOT/'results/first_layer_determinant_applicability'
        sealed=json.loads((prior/'AUTHORITY.json').read_text())
        assert sealed['sources']['research/sdf_cfd_mesher_v4/tools/first_layer_metric_state.py']==metric_hash
        evidence=prior/'CURRENT_NUMERIC_REPLAY.json'
        evidence_index=json.loads((prior/'EVIDENCE_INDEX.json').read_text())
        assert file_hash(evidence)==evidence_index['CURRENT_NUMERIC_REPLAY.json']['sha256']
        replay=next(r for r in json.loads(evidence.read_text()) if r['case']==name)
        assert replay['all_quality_rows_exact'] and replay['exposed_baseline_rows_exact']
        version_binding.update(reason='Qualified performance-only first-layer evaluator revision; stored checkpoint remains unchanged',
            exact_current_numeric_replay=replay,evidence_sha256=file_hash(evidence))
    objects=target/'objects';objects.mkdir(parents=True,exist_ok=True)
    bindings=[]
    for row in envelope['components'].values():
        old=source/'objects'/row['file'];new=objects/row['file']
        assert file_hash(old)==row['sha256']
        if not new.exists():os.link(old,new)
        bindings.append(dict(file=row['file'],sha256=row['sha256'],bytes=old.stat().st_size))
    (target/pointer['checkpoint']).write_bytes(manifest.read_bytes())
    atomic_json(target/'CURRENT.json',pointer)
    native.g.save(target.parent/'SEED_AUTHORITY.json',dict(source=str(source),scientific=pointer['scientific'],
        checkpoint_sha256=pointer['sha256'],implementation_binding=version_binding,objects=bindings,seconds=time.monotonic()-tick,
        storage='Immutable content-addressed native objects, shared hard links; new results use a new CURRENT pointer',
        cold_from_layer0=False))
    return pointer


if __name__=='__main__':seed(sys.argv[1],sys.argv[2],sys.argv[3] if len(sys.argv)>3 else None)
