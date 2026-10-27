"""Bounded provenance recovery only. No mesher or boundary constructor is run."""
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
from unittest.mock import patch

sys.dont_write_bytecode = True
from audit_sealed_l11_core_domain import (V4, REPO, SEALED, EXPECTED, read,
    require_equal, verify_record, v3_inventory, CommittedState, completion,
    measured, metrics, denied, file_hash, durable_json, identity)

OUT = V4/'results/capsule_core_domain_authority'
PRIOR = V4/'results/sealed_l11_core_handoff'
V3 = 'research/sdf_cfd_mesher_v3/'
FAMILY = V3+'fresh_supported_refinement_family_qualification_2026-09-09/'

CANDIDATES = [
    dict(name='capsule physical intent',category='production authority',
         scope='Adopted prism engineering input only',
         rule='75 mph; +Z; positive_sdf; measured projected length 0.15999741537843415 m',
         reason='No upstream, downstream, lateral or outer-discretization criterion',
         sources=['research/sdf_cfd_mesher_v4/results/initial_design_station_progression_request/ADVISOR_ADOPTION.md',
                  'research/sdf_cfd_mesher_v4/results/initial_design_station_progression_request/BOUND_PHYSICAL_INTENT.json',
                  'research/sdf_cfd_mesher_v4/results/qualification_prism_progression_convention/REQUEST.json']),
    dict(name='SF2 fixed wing domain',category='historical case-specific value',
         scope='Wing refinement and mesh/numerical contracts, not capsule domain sizing',
         rule='Cube [-2.5,2.5]^3 m; ceil(42/scale) divisions; SF2 scale=.64 -> 66; 52,272 triangles',
         reason='Physical bounds fixed; scale controls resolution, not body-relative distance. production_policy_promoted=false',
         sources=[FAMILY+'FRESH_BASELINE_CONTRACT.json',FAMILY+'bootstrap_fresh.py',
                  FAMILY+'SF2/A/farfield/CANDIDATE_BINDING.json',FAMILY+'FROZEN_STOPPING_CONTRACT.json']),
    dict(name='farfield subdivision',category='qualified reusable mechanism',
         scope='Shared-midpoint subdivision and topology preservation',
         rule='Refine supplied farfield while preserving geometry; unchanged [-2.5,2.5]^3 cube',
         reason='PhysicalFarfieldSensitivityUnresolved; explicitly no refined production farfield policy',
         sources=[V3+'farfield_only_refinement_qualification_2026-09-08/FINAL_AUTHORITATIVE_REPORT.md',
                  V3+'outer_converged_spatial_and_farfield_audit_2026-09-08/FINAL_AUTHORITATIVE_REPORT.md']),
    dict(name='orthogonal-box normal support',category='qualified reusable mechanism',
         scope='Conformal support geometry within an already supplied orthogonal box; experimental scope',
         rule='depth=min(h_ff/4, minimum triangle altitude/2, box width/8, protected clearance/4); N*u=1',
         reason='Requires box as input; does not choose its physical extent. Experimental; no production promotion',
         sources=[V3+'farfield_normal_support_qualification_2026-09-08/freeze.py',
                  V3+'farfield_normal_support_qualification_2026-09-08/FINAL_AUTHORITATIVE_REPORT.md']),
    dict(name='held-out geometry scaled cube',category='study-only fixture',
         scope='Held-out sphere/rounded-box/concave-pocket/narrow-gap meshing control',
         rule='half=2.5*S, S=fixture characteristic_scale_m',
         reason='Geometry-relative implementation does not establish physical truncation accuracy; referenced frozen policy absent',
         sources=[V3+'held_out_geometry_cohort/run_core_stage.py',
                  V3+'farfield_core_transition/freeze_inputs.py',
                  V3+'farfield_core_transition/FROZEN_FARFIELD_POLICY.sha256']),
    dict(name='V4 study envelope',category='study-only fixture',
         scope='Bounded mixed-core study only',
         rule='lo=interface.min-span; hi=interface.max+span',
         reason='Source explicitly excludes production CFD boundary authority',
         sources=['research/sdf_cfd_mesher_v4/tools/broader_geometry_core.py']),
    dict(name='V4 historical wing importer',category='historical case-specific value',
         scope='Native-interface experiment using SF2 support and sizing constants',
         rule='Load SF2/A/support/SUPPORT.npz and wing grading',
         reason='No exact capsule/request adoption; runner invokes core generation',
         sources=['research/sdf_cfd_mesher_v4/tools/generate_native_interface_core.py']),
    dict(name='aircraft extent-relative defaults',category='engineering convention',
         scope='Non-scoring mesher/solver plumbing on aircraft variants, +X flow',
         rule='Default padding: 1.5 X-spans upstream, 3 X-spans downstream, 2 Y/Z-spans lateral; minimum .15 m',
         reason='Configurable defaults; no capsule physical-domain error or wake criterion. Different +X convention',
         sources=['custom_cfd_mesher_experiment/scripts/gmsh_external_flow_mesher.py',
                  'custom_cfd_mesher_experiment/scripts/run_gmsh_optimizer_hardening_sweep.py',
                  'aircraft_optimizer_platform/software/optimizer/configs/gmsh_openfoam_external_aero_bl.v0_2.json']),
]


def require_domain_candidate(candidate):
    """Recovery gate only; passing this gate would still require geometry checks."""
    if candidate.get('category') != 'production authority':
        raise ValueError('NotProductionDomainAuthority')
    if candidate.get('scope') != 'complete external domain':
        raise ValueError('AuthorityScopeDoesNotCoverDomainSizing')
    for name in ('scientific','content','geometry','request'):
        require_equal(candidate.get('bindings',{}).get(name),EXPECTED[name],name)
    required = ('shape','flow_axis','upstream','downstream','lateral','tags',
                'orientation','components','auxiliary','exclusions','discretization')
    if not all(candidate.get('provenance',{}).get(k) for k in required):
        raise ValueError('IncompletePhysicalDomainProvenance')
    # No complete candidate was recovered. Do not manufacture an implementation
    # of successful domain qualification from a metadata-only gate.
    raise ValueError('BoundaryQualificationRequiredBeforePublication')


def source_evidence(path):
    p = REPO/path
    words = ('farfield','domain','upstream','downstream','production','physical',
             'scale','bounds','span','flow','Phi=','support','qualified','scoring')
    text = p.read_text(encoding='utf-8')
    lines = [dict(line=i,text=line[:1200]) for i,line in enumerate(text.splitlines(),1)
             if any(w.lower() in line.lower() for w in words)]
    return dict(path=path,sha256=file_hash(p),bytes=p.stat().st_size,
                relevant_lines=lines,excerpt_line_limit=1200)


def run():
    tick = measured()
    require_equal(subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                  read(OUT/'PREFLIGHT.json')['head'],'starting_head')
    a=read(SEALED/'AUTHORITY.json')
    require_equal(file_hash(SEALED/'AUTHORITY.json'),EXPECTED['authority_sha256'],'L11_authority')
    verify_record(a)
    sources = sorted({p for c in CANDIDATES for p in c['sources']})
    recovery_tick=measured()
    evidence=[source_evidence(p) for p in sources]
    candidates=[]
    for c in CANDIDATES:
        try:
            require_domain_candidate(c)
        except ValueError as exc:
            candidates.append(dict(c,adopted=False,rejection=str(exc)))
        else:
            raise RuntimeError('UnreviewedDomainPromotion')
    missing=[V3+'held_out_geometry_cohort/FROZEN_COHORT_POLICY.json',
             V3+'farfield_core_transition/FROZEN_FARFIELD_POLICY.json']
    missing_records=[dict(path=p,exists=(REPO/p).exists()) for p in missing]
    recovery_metrics=metrics(*recovery_tick)
    open_tick=measured()
    with patch.object(sqlite3,'connect',denied),patch.object(os,'link',denied),CommittedState(a['reference']) as state:
        require_equal(state.content_root,EXPECTED['content'],'content')
        require_equal(state.root,EXPECTED['access'],'access')
        opened=metrics(*open_tick)
        bound=completion(state)
        compact=dict(state['authority'])
        state.check()
    consistent={n:dict(sha256=file_hash(PRIOR/n),filename=n) for n in ('REPORT.md','REPORT.json')}
    for n,r in consistent.items():
        require_equal(r['sha256'],read(OUT/'PREFLIGHT.json')['prior_report_files'][n],'prior_'+n)
    result=dict(schema='V4CoreDomainProvenanceRecoveryAudit/1',
        classification='V4CoreDomainBoundaryBlockedMissingPhysicalDomainSizingAuthority',qualified=False,
        verified_L11=EXPECTED,current_front=bound,compact_prism_authority=compact,
        candidates=candidates,sources=evidence,missing_historical_records=missing_records,
        adopted_extents={},domain_authority=None,domain_boundary_constructed=False,
        known_input=dict(flow_axis=[0,0,1],fluid_side='positive_sdf',
            note='Adopted prism physical intent; not adoption of farfield CFD boundary conditions'),
        missing_inputs=['Physical-domain truncation policy and intended observables/error requirements',
            'Upstream boundary placement criterion','Downstream wake/outlet placement criterion',
            'Lateral/radial boundary placement criterion','Outer boundary-condition intent and tag mapping',
            'Qualified outer-boundary discretization','Auxiliary boundaries/exclusions and expected components'],
        not_run=['boundary construction','domain topology/manifold checks','outer/L11 intersections',
                 'domain authority publication','domain fresh reopen','core handoff'],
        report_filename_audit=dict(files=consistent,
            result='Both prior files are intact. Prior SHA label explicitly named REPORT.json; link targeted REPORT.md. Distinguish the two, not interchangeable hashes.'),
        counters=dict(historical_proof_rows=0,historical_quality_rows=0,sqlite_rows=0,
            full_prefix_scientific_hashes=0,deep_copies=0,hardlinks=0,extra_prism_layers=0,
            core_vertices=0,core_interior_points=0,tetrahedra=0,core_cells=0),
        metrics=dict(trusted_open=opened,provenance_capture=recovery_metrics,recorded_run=metrics(*tick)),
        measurement_scope='Recorded replay of source recovery and trusted access; interactive search time is not included',
        recommended_next_task='EstablishV4CapsulePhysicalDomainSizingContractWithoutCoreGeneration')
    result['identity']=identity(result)
    durable_json(OUT/'REPORT.json',result)
    print(json.dumps(dict(classification=result['classification'],identity=result['identity'],
        metrics=result['metrics'],prior_reports=consistent,candidate_count=len(candidates),sources=len(evidence)),indent=2))


if __name__ == '__main__':
    run()
