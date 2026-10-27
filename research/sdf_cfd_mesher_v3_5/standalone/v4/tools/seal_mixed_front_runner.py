"""Seal a candid bounded result; this does not enable production remeshing."""
from pathlib import Path
import hashlib,json,subprocess
ROOT=Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
OUT=ROOT/'results/purpose_built_mixed_front_runner'
BASE='f6aaa2872bffc861c8001c92a806276cf826e73a'

def digest(p):
    with p.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def load(p):return json.loads(p.read_text())
def save(p,value):p.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')

def run():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==BASE
    prior={
        'production_automatic_termination_remesh':('d83d694b29cf9032ddaac81402e46ea91b7574daee1a78bd270337b972e500f0','1982301585672a303019fcf825d9ce8d63b5b6fb9fdf914957595908c077aff0'),
        'local_termination_remesh_generalization':('cc79ca134b7c648303e68715d27442acb41428ec203c96a1f036a4e80ea5f734','c8d249f635b7e0fdee4dd1b8b084a97522f1720a5face5081b707d8c6c0400ba'),
        'production_termination_trigger_policy':('fce53f81d4fd8c10706325e0cf043c88fea14f9ed1a2a4313db95c02acacd203','e060bcfe06009beab135f587257c056130c0c206d43bbfe6f586fd6da35a62bc')}
    verified={}
    for folder,hashes in prior.items():
        for name,h in zip(('report.md','AUTHORITY.json'),hashes):
            p=ROOT/'results'/folder/name;assert digest(p)==h;verified[str(p.relative_to(ROOT))]=h
    # Raw checkpoint bytes live in verified archives. Normalize review text
    # only; checkpoint payloads and their pointer digests are not rewritten.
    for p in OUT.rglob('*'):
        if p.is_file() and p.suffix in ('.txt','.json') and not p.name.startswith('STATE_') and p.name!='seal.txt':
            text=p.read_text(encoding='utf8').replace('\r\n','\n').rstrip()
            p.write_text(text+'\n' if text else '',encoding='utf8',newline='\n')
    summary=load(OUT/'SUMMARY.json');cases=summary['cases'];wall=load(OUT/'WALL_FORENSICS.json')
    tests=(OUT/'TESTS.txt').read_text().strip().splitlines()[-1]
    capacity_tests=(OUT/'CAPACITY_TESTS.txt').read_text().strip().splitlines()[-1]
    a=cases['clean_curved_passage'];b=cases['analytic_cavity_with_control']
    initial=[r for r in (OUT/'INITIAL_STATUS.txt').read_text().splitlines() if 'research/sdf_cfd_mesher_v4/' not in r]
    current=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    outside=[r for r in current if 'research/sdf_cfd_mesher_v4/' not in r]
    assert initial==outside,'UnrelatedDirtyStateChanged'
    save(OUT/'GIT_AUDIT.json',dict(base=BASE,unrelated_entries=len(outside),unrelated_unchanged=True,V3_task_modifications=0))
    report=f'''## Decision question

Can V4 replace its prism-only committed-front assumptions with a purpose-built mixed-front production state that supports automatic local rollback/remeshing, deterministic restart, and continued rolling advancement without changing wall identity or ordinary SF2 behavior?

**Result: V4PurposeBuiltMixedFrontRunnerPromising. The requested full production migration is incomplete. Automatic remeshing remains disabled by default.**

### A. Authority verification

Base HEAD: `{BASE}`. The three prior report/authority pairs retain their sealed hashes. V3 task modifications: zero. All {len(outside)} unrelated dirty entries remain unchanged. Only this task's V4 files are committed. No full SF2 build ran.

### B. Previous integration failure

The previous result was V4ProductionAutomaticTerminationRemeshInsufficient. It had private transactions but no native mixed-front rolling state. This task adds `MixedFrontRunner` and `task4l_run.run_mixed_front`. The historical default SF2 subprocess route still uses its legacy adapters. The new route is explicit and bounded; it is not default-enabled.

### C. Wall-coordinate identity mismatch

The mismatch occurs during sphere source generation, before remesh, IDs or serialization. Three-term midpoint-norm reduction order changes binary64 coordinates. Explicit `sqrt((x*x+z*z)+y*y)` reproduces the sealed curved wall hash `{wall['explicit_order_hash']}` and first-layer hash `{wall['first_layer_hash']}`. The alternative x-y-z order reproduces the old failing hash `{wall['legacy_hash']}`. The historical BLAS binary/reduction implementation was not recorded. No tolerance was added.

The same explicit source arithmetic restores the general-SDF wall hash `35ffda61c1f9fc28dbbe274dffa16d097456bb27d7b400ca5cd7a4b617479eb7`. The stored compressed seed revalidates with exact equality of its accepted gate record. Fresh re-projection of that seed still differs and remains a separately recorded replay failure.

### D. Purpose-built mixed-front architecture

`mixed_front.py` stores source wall authority, mesh vertices, typed cells, oriented faces and ownership, column state, exposed outer interface, prism advance front, local versions, receipts, dependency state and event history. It accepts a prism-only mesh as a special case. The model does not require one prism per source triangle at every layer.

### E. Source and local identity model

Wall source IDs are separate from mesh vertex IDs such as `layer:4:source:18`. Fronts reference scientific face IDs. Wall identity binds source-keyed coordinates and oriented source triangles. Explicit source-to-local index maps support reordered mesh arrays. Canonical face cycles preserve orientation; sorted vertex sets are incidence lookup keys only. A sorted-quad receipt mapping defect was reproduced and repaired.

### F. Column states

ActivePrismColumn and TerminatedPrismColumn are explicit. The final curved state has {a['active_columns']} active columns; the final spherical state has {b['active_columns']}. Exposed terminal caps are not sufficient evidence of advancement eligibility. An adapter defect initially reactivated these caps. The corrected adapter uses the remesh map and preserves prior eligibility during ordinary continuation. The old runs remain provisional evidence, not accepted state qualification.

### G. Active prism front

Only eligible exposed prism tops enter the prism front. Terminal and transition faces remain in the outer interface. The corrected runs commit one subsequent prism step from their rebuilt state. They do not reload an accepted remesh as a construction input.

### H. Committed outer interface

The curved interface has {a['outer_faces']} faces; the spherical interface has {b['outer_faces']}. Existing ownership, vertex-link, volume and continuous geometry checks pass. Bounded Gmsh ingestion preserves coordinates/connectivity. No core mesh was built.

### I. Topology and geometry versioning

Versions are local to scientific cell/face inputs. Coordinate-only edits change geometry versions. Connectivity/ownership edits also change topology versions. Unchanged local dependencies retain their versions. Tests cover coordinate edits and source/local reordering.

### J. Risk-atlas integration

Native dependency regions use exposed face ancestry and incident geometry. The continuation steps retain {a['rebuilds'][1]['dependencies']['atlas_retained']} curved and {b['rebuilds'][1]['dependencies']['atlas_retained']} spherical region identities. This is a dependency adapter. Task-4C detailed risk behavior and its existing persistent planner are not yet migrated.

### K. Forecast integration

Forecast entries contain active source columns, current local layers and the remaining stored nominal schedule. Terminated columns have no entries. Continuation retains {a['rebuilds'][1]['dependencies']['forecast_retained']} curved and {b['rebuilds'][1]['dependencies']['forecast_retained']} spherical entries. General geometric forecast samples, correction-path adaptation and adjusted local schedules are incomplete. This is not full Task-4 forecast qualification.

### L. Quality receipts

Receipts bind cell type, geometry, ordered face dependencies and implementation identity. Exact cell-validity records are retained. Prism Jacobians remain prism-specific; pyramids/tetrahedra use the existing general-cell validity and quality definitions. The final manifests contain {a['quality_receipts']} curved and {b['quality_receipts']} spherical receipts. Current dependency audits find no stale receipts.

### M. Certificate manifest

Proof IDs use scientific face pairs, independent of temporary array positions. Current complete local populations contain {a['required_pairs']:,} curved and {b['required_pairs']:,} spherical relations: zero forbidden and zero unresolved. Changed receipts are retired; current manifests contain only newly bound or unchanged receipts. The remesh steps retain {a['rebuilds'][0]['retained']['certificate']:,} and {b['rebuilds'][0]['retained']['certificate']:,} proofs respectively. Migration into the legacy whole-shell certificate manifest is incomplete.

### N. Checkpoint schema

V4MixedFront/1 stores the complete native state in content-addressed snapshots. Scientific identity excludes journal storage and timing. Source/local array ordering does not change identity. Native checkpoint restart passes. Automatic migration of historical prism-only checkpoints and their aggregate evidence into this schema is incomplete.

### O. Transaction journal and atomicity

Stages are Prepared, RollbackApplied, RemeshConstructed, RemeshValidated, StateRebuilt and Committed. CURRENT is the only publication point. Pre-commit failures retain the previous snapshot. Rebuild cannot change validated geometry. Ordinary advancement cannot remove or move committed cells. Both live fixtures remove zero already committed cells; they replace prospective continuation. They do not qualify physical rollback of the historical general-SDF upper stack in this runner.

### P. Cache invalidation matrix

The machine-readable matrix covers six operations and eight cache families. Dependencies are explicit. Native receipt and front-dependency invalidation is tested. The matrix does not claim that every legacy reconstruction, planner, pair-selection or detailed-metric cache has been migrated.

### Q. Curved live integration

Native entry point: automatic exact-collision eligibility, generic cavity discovery, remesh validation, state commit, then a prism continuation commit. Final cells: {a['cells']} ({dict(a['cell_types'])}). Final last-prism map: {dict(a['layer_map'])}. All accepted geometry checks pass. Terminal-cap eligibility was repaired before final state replay. Geometry is exactly unchanged by that eligibility repair.

### R. Spherical live integration

Generic analytic capacity evidence dispatches remesh. The small component terminates at layer 12; the control advances to 13 and then one local column reaches 14. Final cells: {b['cells']} ({dict(b['cell_types'])}). Final last-prism map: {dict(b['layer_map'])}. All accepted geometry checks pass. No near-zero-height workaround is used.

### S. General-SDF integration/replay

The hash-verified stored layer-4 seed passes fresh exact validation with identical accepted gate identity. This reproduces the historical seed gate; it is not a new proof that its exposed terminal caps are nonoverlapping. Its fresh projection replay fails exact coordinate equality. Native integration of its adjusted schedule and rollback of already committed upper cells is not complete. No general-SDF native-runner qualification or prevention claim is made.

### T. Failure-injection recovery

Unit injections cover all journal stages and the point before commit. They verify restoration, idempotence, wall protection and rejection of geometry mutation during rebuild. These are transaction tests, not substitutes for live numerical validation. Separate live interruptions after quality and certificate generation remain unqualified.

### U. Neighbor continuation

The native entry point publishes a subsequent ordinary prism step in both cases. Complete immediate geometry/volume/certificate checks pass again. The resulting layer maps and front memberships are persisted and replay exactly.

### V. SF2 regression

All 63 preserved negative policy snapshots have zero remesh triggers. Read-only checkpoint checks cover layers 1, 11, 12 and 20: coordinate bytes, metadata identities, stored corrections and certificate evidence are preserved. No geometry, correction or layer-count changes occur. These checks do not qualify native mixed-schema SF2 advancement; that adapter remains incomplete. No full SF2 build ran.

### W. Wall identity regression

Wall and first-layer identity hold before remesh, after commit, on restart and after continuation in both native cases. The shared source-generation arithmetic restores the sealed curved and general-SDF wall identities. No wall vertex or wall triangle is remeshed.

### X. Runtime and locality

Corrected runs, including initialization and two steps, took {a['runtime']['total']:.6f} s curved and {b['runtime']['total']:.6f} s spherical. Each used five exact-input certificate-cache hits; these are warm-cache bounded timings, not isolated cold production-event costs. Cold provisional geometry runs took 359.721258 s and 573.778259 s but had the eligibility defect and are not final runner qualification timings. Stage clocks in the current record describe the last invocation, not a complete accumulated event breakdown.

Largest validated state: {max(a['cells'],b['cells'])} cells. Peak process RSS in the corrected runs: {a['runtime']['peak_rss']:,} bytes curved and {b['runtime']['peak_rss']:,} bytes spherical. The remesh seeds have {len(a['cavity_seed']['vertices'])} and {len(b['cavity_seed']['vertices'])} vertices. No global remeshing was used. Native snapshot storage and rebuild remain bounded-case implementations; full-shell scaling is not qualified.

### Y. Determinism

Fresh construction replay matches the complete corrected state and operation trace in both cases. Exact certificate caches are used only after matching points, ordered faces, active population and every bound proof dependency. Unknown inputs use the ordinary exact path. Replay time and RSS are separate. Curved state: `{a['final_hash']}`. Spherical state: `{b['final_hash']}`.

### Z. Tests

`python tools/test_mixed_front_runner.py`: {tests}. This includes retained rolling, policy, remesh and transaction tests. The five capacity/branch suites also passed: {capacity_tests}. The stdin-based Windows multiprocessing launcher failed once; the file-based launcher passes. No certificate mathematics changed. Large evidence files are archived losslessly with per-member hashes using the existing WSL zstd tool; `archive_mixed_front_evidence.py --restore` restores them without overwriting mismatched files.

### AA. Classification

**V4PurposeBuiltMixedFrontRunnerPromising.** Native mixed state, source identity, two bounded rolling cases and deterministic state replay work. The requested production migration remains incomplete. No whole-shell or default-production qualification is claimed.

### AB. Production readiness

**No.** Do not enable automatic rollback/remeshing by default. Missing work includes native SF2 correction/atlas/forecast adapters, adjusted-schedule general-SDF integration, legacy checkpoint/evidence migration, and live numerical-stage failure injection. Both present live cases have zero historical cell removals.

### AC. Recommended next task

Complete the native providers and legacy state/evidence migration, including general-SDF rollback, then repeat bounded production qualification. Do not start core integration yet.
'''
    (OUT/'report.md').write_text(report,encoding='utf8',newline='\n')
    changed=subprocess.check_output(['git','-c','core.longpaths=true','diff','--name-only','HEAD','--','research/sdf_cfd_mesher_v4'],text=True).splitlines()
    staged=subprocess.check_output(['git','-c','core.longpaths=true','diff','--cached','--name-only','--','research/sdf_cfd_mesher_v4'],text=True).splitlines()
    added=subprocess.check_output(['git','-c','core.longpaths=true','ls-files','--others','--exclude-standard','--','research/sdf_cfd_mesher_v4'],text=True).splitlines()
    future=[str((OUT/name).relative_to(REPO)).replace('\\','/') for name in ('COMMITTED_PATHS.json','AUTHORITY.json','HASHES.json')]
    paths=sorted(set(changed+staged+added+future));save(OUT/'COMMITTED_PATHS.json',paths)
    source={p:digest(REPO/p) for p in paths if p.endswith('.py')}
    evidence={str(p.relative_to(ROOT)).replace('\\','/'):digest(p) for p in OUT.rglob('*') if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json','seal.txt')}
    authority=dict(task='ImplementV4PurposeBuiltMixedFrontProductionRunner',classification=summary['classification'],base_head=BASE,
                   production_default_enabled=False,complete_production_migration=False,prior_authorities=verified,
                   source_sha256=source,evidence_sha256=evidence,report_sha256=digest(OUT/'report.md'),limitations=summary['limitations'])
    save(OUT/'AUTHORITY.json',authority)
    hashes={name:digest(OUT/name) for name in ('report.md','AUTHORITY.json')};save(OUT/'HASHES.json',hashes)
    print(json.dumps(hashes,indent=2))

if __name__=='__main__':run()
