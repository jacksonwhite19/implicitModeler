"""Explain every historical broad-suite failure without changing mesh science."""
import sys
sys.dont_write_bytecode=True
import json
from pathlib import Path
from run_production_scale_resume import OUT,native,authority
from sdf_cfd_mesher_v4.common.authority import file_hash


def run():
    rows=[]
    def add(test,category,cause,action):rows.append(dict(test=test,category=category,cause=cause,action=action))
    for name in ('test_forensics::test_current_task_preservation_and_sealed_task2',
                 'test_setup::test_branch_and_unrelated_git_status_preserved',
                 'test_task3::test_current_task_unrelated_state_preserved',
                 'test_task3_repair::test_followup_preserves_unrelated_state'):
        add(name,'A','Historical unrelated dirty-state snapshot predates current qualified authority and legitimate external work.',
            'Keep historical sealed evidence; assert unrelated state is unchanged during the current test session.')
    for name in ('test_setup::test_fork_hashes_and_bounded_preservation[kernel.py]',
                 'test_task1::test_18_v3_preservation'):
        add(name,'A','V4 loader intentionally changed in qualified e09fc8bd bridge; vendor mathematical source is still hash-locked.',
            'Check loader against qualified 9924097c Git object; retain original V3/vendor and copied-source checks.')
    for name in ('test_setup::test_fork_hashes_and_bounded_preservation[naca_spline.rs]',
                 'test_task2::test_original_sources_unchanged'):
        add(name,'C','Live external Rust application file differs from the historical copy. It is not the isolated V4 production import.',
            'Preserve user file; continue checking frozen V3 and V4 copied sources. Session preservation covers unrelated live work.')
    add('test_setup::test_no_reparse_or_large_v4_files','A','Scaffold copy-size limit was incorrectly applied to later native disk-backed production evidence.',
        'Apply unchanged size/link checks to tracked V4 files; large local evidence stays indexed outside Git.')
    add('test_task3_repair::test_exact_task3_blocker','A','Current qualified loader now normalizes the historical affine-root input; expecting the old bounds failure is obsolete.',
        'Execute frozen pre-bridge vendor code for the historical rejection and test current canonical bounds equivalence.')
    add('test_task4j_certification::test_domain_is_part_of_key','C','Fixture used a different generator than exact_algebraic.T and SymPy could reuse a bound x symbol in CRootOf.',
        'Construct exact coefficient dictionary with kernel T; assert the expected algebraic root explicitly as well as cached/uncached equality.')
    old=native.ROOT/'results/first_layer_determinant_applicability/TESTS_WINDOWS.log'
    native.g.save(OUT/'BROAD_TEST_AUDIT.json',dict(rows=rows,counts={'A':8,'B':0,'C':3},
        qualification='No active scientific regression found among the eleven failures',
        note='A includes obsolete pre-first-layer historical assertions; they were not all caused by the first-layer change.',
        prior_log_sha256=file_hash(old),audit_tests_sha256=file_hash(OUT/'TESTS_AUDIT_FIXED.log'),
        mathematical_vendor_changed=False,certificate_mathematics_changed=False))
    bindings=[]
    import geometry_family_authority as geometry
    for spec in authority.cohort():
        x,f=geometry.build(spec)
        old=json.loads((native.ROOT/'results/broader_production_scale_geometry'/spec['name']/'WALL_AUTHORITY.json').read_text())
        assert native.g.q.array_hash(x)==old['coordinates'] and native.g.q.array_hash(f)==old['topology']
        bindings.append(dict(case=spec['name'],spec=spec,vertices=len(x),triangles=len(f),
            coordinate_hash=old['coordinates'],topology_hash=old['topology'],prior_wall_authority=file_hash(native.ROOT/'results/broader_production_scale_geometry'/spec['name']/'WALL_AUTHORITY.json'),
            h1=native.g.q.load_schedule().targets[0],schedule_hash=native.g.q.array_hash(native.g.q.load_schedule().targets),
            domain='Preserved bounded analytic geometry; no new CFD controls'))
    native.g.save(OUT/'GEOMETRY_BINDINGS.json',bindings)


if __name__=='__main__':run()
