"""Bounded source preservation and setup metadata; no scientific execution."""
from prepare_source_fork import ROOT, REPO, PKG, sha, put
import datetime, json, shutil, subprocess, sys

def read(name):return json.loads((ROOT/'authorities'/name).read_text())

def preservation():
    files=read('V3_SOURCE_FORK_MANIFEST.json')['files']
    records=[dict(path=r['original_path'],start_sha256=r['original_sha256'],end_sha256=sha(REPO/r['original_path']),source_owner=r['source_owner']) for r in files]
    mismatches=[r for r in records if r['start_sha256']!=r['end_sha256']]
    assert not mismatches,mismatches
    put(ROOT/'authorities/V3_PRESERVATION.json',dict(scope='Only selected copy sources; no repository-wide campaign',records=records,checked=len(records),v3_checked=sum(r['source_owner']=='V3' for r in records),mismatches=mismatches,v3_modification_count=0))
    return files

def seal():
    files=preservation()
    data=read('DATA_AUTHORITIES.json')
    plan=read('SOURCE_COPY_PLAN.json')
    plan['references']=[dict(original_path=r['path'],source_sha256=r['sha256'],size_bytes=r['size_bytes'],role=r['role'],destination=None,decision='reference',reason='External authority retained in V3; not duplicated') for r in data['references']]
    put(ROOT/'authorities/SOURCE_COPY_PLAN.json',plan)
    put(ROOT/'authorities/V4_SETUP_AUTHORITY.json',dict(schema='V4.SetupAuthority/1',branch_name='feature/sdf-cfd-mesher-v4-advancing-front',git_base_head=read('START_STATE.json')['base_head'],setup_timestamp=datetime.datetime.now(datetime.timezone.utc).isoformat(),v4_root=ROOT.relative_to(REPO).as_posix(),architecture_name='SDF CFD Mesher V4 — SDF-Guided Relaxed Advancing Front',manifest_hashes={n:sha(ROOT/'authorities'/n) for n in ['V3_SOURCE_FORK_MANIFEST.json','DATA_AUTHORITIES.json','SOURCE_COPY_PLAN.json']},source_file_count=len(files),v3_source_file_count=sum(r['source_owner']=='V3' for r in files),copied_bytes=sum(r['size_bytes'] for r in files),referenced_large_data_bytes=sum(r['size_bytes'] for r in data['references'] if r['size_bytes']>100_000_000),referenced_authority_bytes=sum(r['size_bytes'] for r in data['references']),v3_modification_count=0,scientific_execution=False,commit_created=False,scope='source scaffold only',source_count_includes='code, certificate manifest/README/tests and one production SDF dependency'))
    print('Sealed setup authority; all selected originals unchanged')

def report():
    files=preservation();tests=read('SETUP_TEST_RESULTS.json');a=read('V4_SETUP_AUTHORITY.json');start=read('START_STATE.json');data=read('DATA_AUTHORITIES.json')
    assert tests['passed'],tests
    status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=REPO,text=True)
    assert status==start['status_porcelain']
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    put(ROOT/'authorities/END_STATE.json',dict(branch=a['branch_name'],head=head,status_porcelain=status,equals_starting_status=True,commit_created=False))
    elapsed=(datetime.datetime.now(datetime.timezone.utc)-datetime.datetime.fromisoformat(start['setup_started_at'])).total_seconds()
    free=shutil.disk_usage(REPO).free
    ledger=dict(started_at=start['setup_started_at'],finished_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),allowance_seconds=1800,elapsed_seconds=round(elapsed,3),heavy_workers=0,scientific_processes=0,scope='source setup and import tests')
    put(ROOT/'authorities/SETUP_LEDGER.json',ledger)
    largest=max(files,key=lambda r:r['size_bytes'])
    above=[dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size) for p in ROOT.rglob('*') if p.is_file() and p.stat().st_size>100_000_000]
    large=[r for r in data['references'] if r['size_bytes']>100_000_000]
    hashes={n:sha(ROOT/'authorities'/n) for n in ['V3_SOURCE_FORK_MANIFEST.json','DATA_AUTHORITIES.json','V4_SETUP_AUTHORITY.json']}
    def content(total):
        rows=[
        ('1. Setup classification','SdfCfdMesherV4SetupComplete'),
        ('2. Branch',a['branch_name']),('3. Base Git HEAD',a['git_base_head']),
        ('4. Final Git HEAD',head+' (unchanged; no commit)'),
        ('5. Starting worktree','Dirty: 40 tracked-status entries and 86 untracked entries; exact porcelain saved in authorities/START_STATE.json'),
        ('6. Ending worktree','Same porcelain status as start; V4 is within the already untracked research/ tree. Exact status saved in authorities/END_STATE.json'),
        ('7. V4 root',str(ROOT)),('8. Architecture',a['architecture_name']),
        ('9. Copied source/support files',str(len(files))+' total; '+str(a['v3_source_file_count'])+' V3 files plus 1 production SDF source dependency'),
        ('10. Copied bytes',f"{a['copied_bytes']:,} total; {sum(r['size_bytes'] for r in files if r['source_owner']=='V3'):,} from V3"),
        ('11. Referenced authorities',f"{len(data['references'])} entries, {a['referenced_authority_bytes']:,} measured bytes; {len(large)} authority above 100 MB ({a['referenced_large_data_bytes']:,} bytes)"),
        ('12. Largest copied file',largest['original_path']+f" ({largest['size_bytes']:,} bytes)"),
        ('13. V4 files above 100 MB',str(len(above))+'; no exceptions'),
        ('14. Certificate source copy','PASS: complete package and 11 qualified mathematical vendor files; all byte-identical. No qualification rerun.'),
        ('15. Conditioning source copy','PASS: weighted sigma, singular spectrum, condition and G_bound source; no new health thresholds.'),
        ('16. Quality source copy','PASS: frozen Foundation geometry, predicates and component quality; thresholds unchanged in source.'),
        ('17. Component graph copy','PASS: deterministic ownership, topology supports and immutable-context checks.'),
        ('18. Compression source copy','PASS: component compression and symbolic dependencies copied; no compression executed.'),
        ('19. Legacy planner copy','PASS: supplied c3813b15... two-way planner and dependencies, explicitly legacy lookahead/reference only.'),
        ('20. Package/import result','PASS: V4, certificate, conditioning, quality, legacy planner, graph and compression imports; no live V3 import.'),
        ('21. Setup tests',f"{tests['tests_run']} passed; {tests['failures']} failures; {tests['errors']} errors; {tests['skipped']} skipped; {tests['seconds']:.3f} s"),
        ('22. V3 files modified','0 by this task; all selected copy sources rehashed unchanged'),
        ('23. Source hash mismatches','0; bounded preservation covers all 45 copied originals, including 44 V3 files'),
        ('24. Source fork manifest SHA-256',hashes['V3_SOURCE_FORK_MANIFEST.json']),
        ('25. Data authority manifest SHA-256',hashes['DATA_AUTHORITIES.json']),
        ('26. Setup authority SHA-256',hashes['V4_SETUP_AUTHORITY.json']),
        ('27. README','Created'),('28. Architecture document','Created; full forecast/commit, authority, cache and certification contracts'),
        ('29. Interface scaffold','Created: typed dataclasses and protocols only; no advancing-front implementation'),
        ('30. Commit created','No. Starting worktree was dirty; nothing staged.'),('31. Commit hash','Not applicable'),
        ('32. Task runtime',f"{elapsed:.3f} s at report seal; allowance 1,800 s"),('33. V4 total bytes',f'{total:,} logical bytes'),
        ('34. Repository free bytes',f'{free:,} at report seal'),('35. Setup blocker','None'),
        ('36. Recommended first implementation task','ImplementV4CommittedFrontAndCheapSdfForecastFoundation — not started')]
        counts=start['status_porcelain'].splitlines()
        rows[4]=(rows[4][0],f"Initial branch main. Dirty: {sum(not x.startswith('??') for x in counts)} tracked-status entries and {sum(x.startswith('??') for x in counts)} untracked entries; exact porcelain saved in authorities/START_STATE.json")
        text='# V4 setup report\n\n| Item | Result |\n|---|---|\n'+'\n'.join('| '+k+' | '+v.replace('|','/')+' |' for k,v in rows)+'\n'
        text+='''
## Source inventory and storage

The full per-file inventory, paths, roles, sizes and hashes are in
authorities/V3_SOURCE_FORK_MANIFEST.json. Copied groups are:

- Certificate: 8 package modules (including tests), README, mathematical manifest and 11 vendor modules.
- Legacy planner: common, accepted_inputs, component_graph, component_quality,
  component_compression, component_planner, diagnostic_convergence, linear_solve,
  trust_region and two_way_planner.
- Support: foundation_geometry, prospective_commit_adapter, accepted_front_state,
  continuation_model, symbolic_prism, placement geometry and sigma-derivative source.
- Conditioning: operator_common and operator_model.
- SDF: earlier and current SF2 smooth_authority, procedural_authorities,
  spline_bridge.rs and production naca_spline.rs.

Large mesh data remain at the explicit V3 paths in DATA_AUTHORITIES.json.
No mesh, history, diagnostic NPZ tree, proof stream or OpenFOAM case was copied.
The 100 MB authority is the accepted polyMesh member set. Its canonical identity
was checked against its member-hash record, and member paths exist. The large
mesh files were not rehashed or scientifically validated during setup. File
authorities were SHA-256 checked. No symlinks, junctions or reparse mirrors were
created. All new artifacts are under V4.

## Setup checks and limits

Exact check: `python -B research/sdf_cfd_mesher_v4/tools/run_setup_tests.py`.
The detailed unittest log is in authorities/SETUP_TEST_RESULTS.json. The first
package import probe also passed. Source copies were hash-checked before copy,
after copy and at the final bounded preservation check. Git status and HEAD
were compared with the saved starting state. No unrelated files were staged.

No scientific test was run. Copied certificate tests were retained as source,
not rerun. No planner study, prism generation, mesh generation, full-shell
certificate, OpenFOAM, potential flow, CFD or RANS ran. Native SDF source was
not compiled. Import success is setup evidence, not scientific qualification.

The loader redirects legacy source imports into V4 and restores temporary
module aliases, sys.path and thread environment values. Dormant legacy study
loaders require explicit input adapters before future use. The native SDF
binding and all forecast, growth, relaxation, projection and cache algorithms
remain unimplemented. The requested package layout was retained, with small
_frozen and _support subdirectories to preserve original scientific bytes.

## Authority notes

The supplied two-way planner hash matches. A newer unpromoted planner was found
and referenced, but did not replace the explicitly requested source. Retired
floors remain only in byte-identical legacy source and are not V4 planner logic.

The request states nominal growth `163359674`; accepted V3 authority states
`1.163359674`. Both are recorded. No growth runs and no inferred configuration
default were introduced. Frozen V3 quality code uses skew `4/1.1`, while the
requested retained hard limit is `3.63636`; that difference is documented without
changing source. The older low-level determinant default `0.001` is not a V4
gate; applicable component quality and the V4 contract retain `0.0011`.

The next task is only ImplementV4CommittedFrontAndCheapSdfForecastFoundation:
establish actual committed-front state, SDF distance/normal access and initial
cheap lookahead, with no optimization. It has not begun.
'''
        return text
    report_path=ROOT/'SETUP_REPORT.md'
    for _ in range(5):
        total=sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file())
        text=content(total)
        report_path.write_text(text,encoding='utf-8')
        if total==sum(p.stat().st_size for p in ROOT.rglob('*') if p.is_file()):break
    print(json.dumps(dict(classification='SdfCfdMesherV4SetupComplete',copied_bytes=a['copied_bytes'],v4_total_bytes=total,free_bytes=free,runtime_seconds=elapsed,hashes=hashes),indent=2))

if __name__=='__main__':
    if sys.argv[1]=='seal':seal()
    elif sys.argv[1]=='report':report()
    else:raise SystemExit('Use seal or report')
