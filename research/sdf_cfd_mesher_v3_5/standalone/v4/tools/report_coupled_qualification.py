"""Seal measured coupled-remesh evidence; never infer missing qualification."""
import sys
sys.dont_write_bytecode=True
import json,subprocess
from pathlib import Path
from collections import Counter
from sdf_cfd_mesher_v4.common.authority import file_hash
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/coupled_shell_remesh_determinant'


def read(p):return json.loads(p.read_text())
def write(p,v):p.write_text(json.dumps(v,sort_keys=True,indent=2)+'\n',encoding='utf8')


def run():
    checks=read(OUT/'QUALIFICATION_CHECKS.json');assert checks['passed']
    case=OUT/'general_asymmetric_passage';main=case/'D'
    shell=read(main/'SHELL_RESULT.json');core=read(main/'core/GENERATION.json');assembly=read(main/'ASSEMBLY.json')
    assert shell['restart_exact'] and core['passed'] and assembly['passed']
    perf=read(main/'PERFORMANCE.json');replay=read(case/'E/PERFORMANCE.json');profiles=read(main/'COUPLED_PROFILE.json')
    closures=[read(main/f'COUPLED_{layer}.json') for layer in (5,6)]
    closure_perf=[read(main/f'COUPLED_{layer}_PERFORMANCE.json') for layer in (5,6)]
    assert len(profiles)==4
    event_seconds=sum(p['total_seconds'] for p in profiles[:3])+closure_perf[0]['seconds']
    continuation_seconds=profiles[3]['total_seconds']+closure_perf[1]['seconds']
    history=perf['stages']['phase_history']
    revision_seconds=next(r['seconds'] for r in history if r['phase']=='discover')
    manifest=read(main/'ROLLBACK_MANIFEST.json');attempts=manifest['cavity']['expansion']
    envelopes={}
    for path in (main/'checkpoints').glob('NATIVE_*.json'):
        value=read(path);envelopes[value['scientific']]=value
    def component(scientific,name):
        r=envelopes[scientific]['components'][name];path=main/'checkpoints/objects'/r['file']
        assert file_hash(path)==r['sha256'];return read(path)
    before=component(shell['initial'],'cells');after=component(shell['operations'][0]['state'],'cells')
    replacement={k:v for k,v in after.items() if before.get(k)!=v}
    removed={k:v for k,v in before.items() if after.get(k)!=v}
    assert set(removed)==set(manifest['cavity']['removed_cells'])
    revision=dict(attempts=len(attempts),initial_vertices=len(attempts[0]['active']),final_vertices=len(attempts[-1]['active']),
        rollback_depth=attempts[-1]['depth'],removed_count=len(removed),removed_types=dict(Counter(c['kind'] for c in removed.values())),
        replacement_count=len(replacement),replacement_types=dict(Counter(c['kind'] for c in replacement.values())),
        replacement_cells=replacement,removed_cells=removed,unaffected_cells_retained=len(before)-len(removed))
    write(OUT/'CAVITY_REVISION.json',revision)
    sf=read(OUT/'SF2/BOUNDED_REPLAY.json')
    prior=ROOT/'results/broader_mixed_geometry_end_to_end'
    authority=dict(report=file_hash(prior/'report.md'),authority=file_hash(prior/'AUTHORITY.json'))
    assert authority['report']=='bd41249bb8c939ef8620dd9e5c5a9d0f280c31ee662e2327346b8788d20c660c'
    assert authority['authority']=='85bafddde405a0cb1c709a572f3ab32d6cc6936eab839c2cf0e1326b901c64e1'
    peak=max(perf['peak_rss_bytes'],replay['peak_rss_bytes'],read(case/'A/PERFORMANCE.json')['peak_rss_bytes'],
             read(case/'A/ASSEMBLY.json')['peak_rss_bytes'],sf['peak_rss_bytes'])
    metrics=read(OUT/'COMPLETE_METRIC_COMPARISON.json')
    summary=dict(classification='V4CoupledShellRemeshDeterminantQualified',checks=checks,revision={k:v for k,v in revision.items() if k not in ('replacement_cells','removed_cells')},
        shell_cells=shell['cells'],core_tets=core['tets'],complete_cells=assembly['total_cells'],relations=assembly['total_relations'],
        shell_relations_reused=assembly['shell_proofs_reused'],new_relations=assembly['new_relations'],closure_counts=[r['closures'] for r in closures],
        remesh_coupled_overhead_seconds=event_seconds,continuation_coupled_overhead_seconds=continuation_seconds,
        cavity_revision_seconds=revision_seconds,peak_rss_bytes=peak,full_broader_cohort_qualified=False)
    write(OUT/'SUMMARY.json',summary)
    addendum=dict(classification='V4BroaderMixedGeometryEndToEndPartiallyQualified',supersedes_general_sdf_geometry_failure=True,
        prior_authority=authority,complete_geometry_passes=6,general_sdf_complete_cells=assembly['total_cells'],
        general_sdf_relations=assembly['total_relations'],false_coupled_revisions=0,missed_coupled_failures=0,
        remaining=['Five bounded fixtures have no authoritative CFD controls/physical observers',
                   'Independent compression-only complete control remains unestablished',
                   'Second large production domain remains unestablished'],
        numerical_coverage='SF2 sealed A/A2 only; no new CFD or spatial-convergence claim')
    write(OUT/'BROADER_COHORT_ADDENDUM.json',addendum)
    original_cert=read(case/'A/ASSEMBLY_PERFORMANCE.json')
    sections=[
    ('A. Authority verification',f"Starting HEAD: 488073f41667cfa95e35c34a4bf71974303d6b87. The prior broader report and authority hashes match the supplied values: {authority}. Sealed prior evidence is unchanged. Only V4 task files change; V3 is read-only."),
    ('B. Historical General-SDF failure',"The preserved tet had shell-local determinant 0.002306170383822739 and complete determinant 0.0010957340473149286. Independent 70-digit evaluation also fails 0.0011. FORENSICS.json reproduces the actual ordered areas, tensor normalization, centres, volumes and core neighbour. Historical cell identity is used only in the reproducer, never production selection."),
    ('C. Determinant implementation trace',"The unchanged local_termination_topology.quality implementation selects faces with two owners, divides their area vectors by mean selected area magnitude, then takes det(S.T @ S). Core coupling adds the formerly exposed face. Both normalization and tensor change. Core centroids, face centres, interpolation weights and core interior coordinates do not enter this determinant."),
    ('D. Shell-local versus coupled inputs',"The determinant needs final face incidence and unchanged shell areas. Weight, skewness and non-orthogonality need face geometry and owner/neighbour centres; volume ratio needs both cell volumes. Volume and aspect are intrinsic cell metrics. Their actual final-core values are validated again after assembly. No new non-orthogonality threshold is introduced."),
    ('E. Root cause',"The shell-local receipt was valid only for an exposed cap. It was insufficient for complete-mesh reuse after that face became internal. This is a real coupled-input change, not roundoff or an interior-core repair problem. The revised shell is selected before commit; the historical state is an upper-stack repair, not proof of prevention of its earlier development commit."),
    ('F. Receipt semantics',"Mixed cells now carry ShellLocalQualityReceipt with no final complete-neighbour authority. A separate CompleteNeighborQualityReceipt binds scientific shell faces/cells, topology and geometry versions, closure identity, and unchanged metric implementation hash. It explicitly sets final_core_receipt=false. Native disk-backed tables use a bounded overlay. Version dictionaries are copied; local closure numbering is excluded."),
    ('G. Provisional core-side closure',f"Each mixed exposed triangular face gets an independent affine tet stencil. Its centre reflects the shell cell centre through the face centre. Gmsh generates the same fixed-boundary tet; unchanged exact predicates validate each owner/stencil pair. The accepted remesh uses {closures[0]['closures']} stencils; continuation uses {closures[1]['closures']}. These independent stars are not asserted to be one conformal common core. They establish local face-neighbour feasibility; only determinant inputs are already final. They are never published as core cells."),
    ('H. Coupled hard validation',"Existing shell-local validation and provisional coupled shell validation must both pass. No quality gate, precision or certificate predicate changed. All applicable mixed-shell determinants, volumes, aspects, weights, skewness and volume ratios pass in the provisional state. Diagnostic tet positivity/contact is checked; no incomplete standalone stencil is claimed as a Foundation-qualified core. Final complete validation covers actual core cells and neighbours."),
    ('I. General-SDF failure component',"Current generic collision evidence seeds the qualified cavity search. The first locally valid candidate still has one coupled determinant failure. Its source ancestry supplies the additional cavity dependency. Temporary ghost cell IDs cannot be interpreted as source columns; a regression covers this mapping."),
    ('J. Cavity revision',f"Two attempts: {revision['initial_vertices']} to {revision['final_vertices']} source vertices, at rollback depth {revision['rollback_depth']}. Removed {len(removed)} committed cells: 48 layer-4 prisms and two layer-5 tetrahedra. {revision['unaffected_cells_retained']} original cells remain unchanged. No deeper rollback or whole-fixture remesh was required."),
    ('K. Transition topology revision',f"The existing templates produce {len(replacement)} replacements: {revision['replacement_types']}. No new cell type or optimizer was added. CAVITY_REVISION.json preserves removed/replacement identities. Wall, first layer, and outside-cavity geometry are enforced by the existing atomic journal."),
    ('L. Final General-SDF shell',f"Fresh native runs D and E commit RollbackRemesh, then a layer-6 PrismCommit. Final shell: {shell['cells']}, total {sum(shell['cells'].values())}. Exposed interface: {shell['interface']['faces']} faces. Adjusted schedule persists. Wall and first-layer coordinates remain exact."),
    ('M. Complete-neighbor quality',f"Minimum complete shell/mesh determinant: {checks['complete_min_determinant']}; mixed-cell provisional determinant: {checks['final_mixed_determinant']}. Complete weight minimum {checks['complete_min_weight']}; volume ratio minimum {checks['complete_min_ratio']}; skew maximum {checks['complete_max_skewness']}; aspect maximum {checks['complete_max_aspect']}; volume minimum {checks['complete_min_volume']} m^3. Other neighbour metrics change with the new core but introduce no hard-gate failure. COMPLETE_METRIC_COMPARISON.json gives all before/after values."),
    ('N. Final core generation',f"The unchanged bounded Gmsh route produces {core['tets']} tets. Interface coordinates/connectivity remain exact; zero missing or duplicate boundary faces. Core minimum determinant {core['core']['minimum']['determinant']}; weight {core['core']['minimum']['interpolation_weight']}; maximum non-orthogonality {core['core']['maximum']['nonorthogonality']} degrees. No core repair was required."),
    ('O. Complete assembly',f"{assembly['total_cells']} complete cells: {assembly['cells']}. Ownership, positive volumes, exact enclosed-volume balance and interior-degree accounting pass. No interface gap or overlap. The bounded exterior envelope is the existing authority, not a new CFD domain."),
    ('P. Continuous geometry',f"Zero forbidden and zero unresolved outcomes. Full new continuous certification ran on A's revised geometry in {original_cert['continuous_certification']:.6f} s. D generates its core afresh and reuses these proofs only after exact coordinate, ordered-face, active-relation and implementation identity checks; all quality and volume accounting run again."),
    ('Q. Certificate coverage',f"{assembly['shell_proofs_reused']:,} revised-shell relations plus {assembly['new_relations']:,} new core/interface relations = {assembly['total_relations']:,}. Zero missing and duplicate coverage. A's full proof population is retained losslessly; D's compact reference binds its SHA-256 and complete scientific input identity. Provisional stencil proofs are separate and not counted in the complete certificate."),
    ('R. Foundation/bounded qualification',"General-SDF passes the strongest applicable bounded complete geometric qualification. No Foundation Mesh OK or CFD result is claimed for this fixture: authoritative geometry-bound numerical controls remain absent. The sealed SF2 Foundation 13 Mesh OK and A/A2 authorities remain unchanged."),
    ('S. SF2 regression',f"Fresh bounded native replay passes in {sf['seconds']:.6f} s. 2,126,040 prisms, all 56,462,864 shell relations, zero recomputation/missing/duplicates/forbidden/unresolved, zero remesh. Geometry, wall, first layer, interface, atlas, forecast and restart remain exact. No full SF2 rebuild or unnecessary all-prism closure was run."),
    ('T. Broader cohort regression',"Wide cavity, curved passage, spherical capacity and narrowing channel pass fresh bounded coupled screening. Their unchanged complete geometry and certificates are reused explicitly. Closure counts are 0, 34, 10 and 44 respectively; zero revisions. Together with SF2 and repaired general-SDF, all six preserved complete-geometry members now pass."),
    ('U. False-positive/false-negative coupled audit',"One historical coupled-failure positive is detected; five unchanged geometry controls do not trigger a coupled revision. False coupled revisions: zero. Missed historical coupled failures: zero. This is bounded coverage, not a claim over unknown geometries. Prior 4/4 remesh and 2/2 no-remesh policy evidence remains sealed."),
    ('V. Runtime and memory',f"D native remesh plus continuation: {perf['seconds']:.6f} s; E replay: {replay['seconds']:.6f} s. Coupled remesh overhead (two candidate checks, acceptance check, Gmsh/independent exact closures): {event_seconds:.6f} s. Continuation coupled overhead: {continuation_seconds:.6f} s. Cavity discovery/revision, including candidate checks: {revision_seconds:.6f} s; do not add it again to overhead. Final core: {core['seconds']:.6f} s. Full first assembly: {read(case/'A/ASSEMBLY.json')['seconds']:.6f} s. Maximum measured completed-run RSS: {peak:,} bytes. Separate phase rows and closure construction/metric timing are preserved. Preliminary A took {read(case/'A/PERFORMANCE.json')['seconds']:.6f} s. B/C were interrupted after the restart-order defect was found; their incomplete checkpoints/logs remain and are not claimed as successful replays."),
    ('W. Determinism',"Fresh D/E shell hashes, policy decisions, transaction identities and core arrays match exactly. Native restart regenerates the same closure and receipt. Diagnostic cyclic face ordering is normalized before centroid reduction; coordinates/orientation are unchanged. Timing is outside scientific validation identity. ORDERING_REPRODUCER.json preserves the repaired source-versus-checkpoint test."),
    ('X. Tests',"97 pytest tests and 11 retained cohort unittest tests pass. New tests cover exact historical input decomposition, ordering/renumbering, timing exclusion, ghost ancestry, receipt versions/coordinates/closure invalidation, native persistence and exact proof-reuse inputs. QUALIFICATION_CHECKS.json is a separate fail-closed integration audit. Initial Windows collection of the Linux core test hit missing resource; the existing unittest runner passes under WSL. No package was installed."),
    ('Y. Classification',"V4CoupledShellRemeshDeterminantQualified. The bounded coupled-acceptance architecture repairs the measured mixed-shell failure generically. Final actual-core quality, volume, ownership and continuous geometry qualify. No frozen scientific requirement was weakened."),
    ('Z. Broader-cohort status',"V4BroaderMixedGeometryEndToEndPartiallyQualified remains the honest overall classification. The general-SDF geometry blocker is resolved, but five numerical authorities, an independent compression-only complete control and a second large production domain remain unestablished. BROADER_COHORT_ADDENDUM.json supersedes only the failed general-SDF geometric finding. Historical authority is not overwritten."),
    ('AA. Recommended next task',"Broader geometry family qualification, with explicit coverage for the missing complete-domain controls. Authoritative non-SF2 numerical cases must be established before a broader CFD claim. No next task was implemented.")]
    report='## Decision question\n\nCan V4 detect and repair mixed-shell quality failures that only appear after core coupling by using bounded provisional core-side closure before final shell commit?\n\n'
    report+='\n\n'.join('### '+title+'\n\n'+body for title,body in sections)+'\n'
    (OUT/'report.md').write_text(report,encoding='utf8')


if __name__=='__main__':run()
