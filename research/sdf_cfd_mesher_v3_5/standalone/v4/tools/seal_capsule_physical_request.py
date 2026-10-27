"""Publish an unbound physical-evidence result, never a meshing request."""
import json
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from production_adaptive_surface import REPO,V4,save,sha,identity
from production_prism_sizing import contract_identity
from physical_prism_provenance import verify_sources
from audit_capsule_physical_request import OUT


def read(name):return json.loads((OUT/name).read_text())


def main():
    p=read('PREFLIGHT.json');r=read('V3_REFERENCE_CONDITION.json');h=read('H1_RECONSTRUCTION.json')
    s=read('SCHEDULE_PROVENANCE.json');m=read('EVIDENCE_MANIFEST.json');run=read('RUNTIME.json')
    wall=read('WALL_AND_CONTRACT_VERIFICATION.json');plate=read('FLAT_PLATE_RECOMPUTATION.json')
    suite=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite')
    assert int(suite.attrib['failures'])==int(suite.attrib['errors'])==0
    verify_sources(m['files'],REPO)
    assert not wall['generation_authorized'] and wall['decision']=='C'
    current=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
    owned=['research/sdf_cfd_mesher_v4/results/capsule_physical_prism_request/',
           'research/sdf_cfd_mesher_v4/tools/physical_prism_provenance.py',
           'research/sdf_cfd_mesher_v4/tools/audit_capsule_physical_request.py',
           'research/sdf_cfd_mesher_v4/tools/seal_capsule_physical_request.py',
           'research/sdf_cfd_mesher_v4/tests/test_physical_prism_provenance.py']
    unrelated=[v for v in current if not any(v[3:].startswith(q) for q in owned)]
    assert unrelated==p['starting_dirty']
    checks=dict(tests=int(suite.attrib['tests']),failures=int(suite.attrib['failures']),errors=int(suite.attrib['errors']),
        seconds=float(suite.attrib['time']),command='python -B -m pytest -q research/sdf_cfd_mesher_v4/tests/test_physical_prism_provenance.py',
        v3_task_modifications=0,unrelated_dirty_preserved=True,starting_dirty_count=155,unrelated_ending_dirty_count=len(unrelated),
        input_hashes_preserved=True,production_contract_unchanged=True)
    save(OUT/'CHECKS.json',checks)
    groups=[
        ('h1=2*yplus*nu/u_tau','A','Generic production method','Transfers as dimensional conversion with a justified shear estimate; not a Cf model.'),
        ('nu=mu/rho; u_tau=U*sqrt(Cf/2)','A','Generic production method','Cf must use the same reference velocity and physical convention.'),
        ('U=33.528 m/s; rho=1.225; mu=1.7894e-5; T=288.15 K','B','External-flow qualification-condition input','Reusable controlled reference candidate, not automatically the declared capsule condition.'),
        ('gamma=1.4; R_air=287.05287; Mach=0.09852657166963978','B','External-flow qualification-condition input','Source air-property model; no capsule compressibility policy adopted.'),
        ('L=1 m, airfoil chord; Re=rho*U*L/mu','D','Airfoil-specific assumption','Chord definition cannot be relabelled radius/diameter/capsule extent.'),
        ('Capsule radius=.04 m and axial segment half-length=.04 m','C','Geometry-specific input','Bound geometry only; neither defines the required physical model length.'),
        ('NACA 0012 XFOIL Cf envelope over alpha=0,2,4,6,8 degrees','D','Airfoil-specific assumption','Two-dimensional airfoil/wake model; no capsule applicability qualification.'),
        ('Ncrit=9; natural and forced transition x/c=.01','D','Airfoil-specific assumption','Cannot represent unspecified capsule roughness/transition/incidence.'),
        ('Surface-weighted p75 of positive attached Cf','D','Airfoil-specific assumption','Historical global design statistic, not a maximum-shear or uniform-y+ guarantee.'),
        ('y+=10; kOmegaSST and specified wall functions','E','Solver/wall-treatment-specific input','Historical flat-plate evidence; capsule/global y+ target not adopted or qualified.'),
        ('Full-thickness factor 2','A','Generic production method','Cell-centre half-height convention; not a safety factor.'),
        ('Additional multiplicative safety factor','G','Unknown / insufficient evidence','None in recovered formula; no independently calibrated capsule factor established.'),
        ('240 parsed panel rows; RMS<1e-4 and return code zero','F','Historical implementation detail','Original admission evidence missing; finite-only is not convergence proof.'),
        ('100 growth bisections; fewest N in 20..40; r=1.08..1.25, prefer <=1.20','F','Historical implementation detail','Preserved schedule search policy, not physics-derived universal bounds.'),
        ('Practical target total=.020 m','B','External-flow qualification-condition input','Declared NACA study intent. Original rationale beyond that constant is NotEstablished.'),
        ('SF2 stored h1=.000166527069; r=1.163359674; N=20','F','Historical implementation detail','Later stored decimal constants. No capsule transfer.'),
        ('Capsule model, characteristic length, Re and shear statistic','G','Unknown / insufficient evidence','Required before deriving capsule h1.'),
        ('Surface edges, area and curvature','C','Geometry-specific input','Tangential discretization/feasibility evidence only; not h1 inputs.')]
    transfers=[dict(quantity=q,category=c,classification=n,interpretation=t) for q,c,n,t in groups]
    save(OUT/'TRANSFER_CLASSIFICATION.json',dict(categories=dict(A='Generic production method',B='External-flow qualification-condition input',C='Geometry-specific input',D='Airfoil-specific assumption',E='Solver/wall-treatment-specific input',F='Historical implementation detail',G='Unknown / insufficient evidence'),rows=transfers))
    bound=[
        ('qualified geometry identity','Bound',wall['geometry_identity'],'Verified through the unchanged production wall loader'),
        ('qualified production surface identity','Bound',wall['surface_identity'],'All source authority files hash-verified'),
        ('explicit physical/production-test intent','Bound','User-directed external mesher qualification study','Intent is explicit; this does not approve numeric physical inputs'),
        ('request owner for complete numerical request','Unbound','NotEstablished','No complete physical request exists to own/freeze'),
        ('physical basis','Unbound','NotEstablished','Capsule shear/length/transition/wall-treatment and thickness authority missing'),
        ('hash-bound evidence','Bound',identity(m),'Recovered evidence manifest, not evidence of capsule model applicability'),
        ('fluid side','Bound','positive_sdf','External capsule fluid is outside; qualified primitive is distance-to-segment minus radius'),
        ('full first-prism normal-height definition','Bound','Full normal thickness = twice wall-to-cell-centre distance','Recovered convention; no numeric capsule height'),
        ('global/local meaning','Bound','global only','Current user scope and unchanged contract; local mapping NotApplicable'),
        ('explicit positive binary64 increments','Unbound','NotEstablished','No justified capsule h1 or progression'),
        ('layer count','Unbound','NotEstablished','SF2 search/count cannot be silently transferred'),
        ('nominal schedule total','Unbound','NotEstablished','Requires authoritative explicit increments'),
        ('separate physical total-thickness intent','Unbound','NotEstablished','No capsule boundary-layer thickness/coverage request')]
    binding=[dict(field=f,status=st,value=v,reason=why) for f,st,v,why in bound]
    blockers=[
        'Declare the capsule external reference condition and flow orientation/incidence; V3 freestream values are only a reusable candidate.',
        'Establish a capsule-applicable wall-shear/Cf model or independent evidence, including its reference velocity, valid regime and characteristic-length/streamwise-coordinate definition.',
        'Bind capsule transition, surface-condition/roughness and solver/wall-treatment/y+ policy; justify a representative global shear statistic.',
        'Establish capsule physical total-thickness intent and a supported growth/count selection policy.',
        'Freeze the complete global binary64 schedule, request owner and hash-bound physical basis.']
    classification='V4CapsulePhysicalPrismRequestUnboundPhysicalModelRequired'
    save(OUT/'REQUEST_BINDING_STATUS.json',dict(classification=classification,fields=binding,blockers=blockers,request_bound=False,
        contract_identity=contract_identity(),contract_modification_required=False,
        historical_replay_gap='Missing original XFOIL convergence admissions and policy/envelope JSON prevent unconditional full-precision historical replay; this is separate from capsule model applicability.'))
    free=shutil.disk_usage(REPO).free
    resources=dict(**run,wall_time_to_seal_seconds=time.time()-p['start_epoch'],starting_free_bytes=p['starting_free_bytes'],ending_free_bytes=free,
        repository_accessible_logical_bytes=p['repository_accessible_logical_bytes'],v4_accessible_logical_bytes=p['v4_accessible_logical_bytes'],
        scan_errors=len(p['scan_errors']),bytes_deleted=0,deleted_paths=[],cleanup_reason='No cleanup needed; accepted evidence and old dumps untouched',
        measurement_scope='Process RSS is the bounded audit only. Free-space snapshot precedes final metadata/git writes. Whole-session peak RSS NotMeasured.')
    save(OUT/'RESOURCES_AND_CLEANUP.json',resources)
    finite=h['conditional_reconstructions'][1];allrows=h['conditional_reconstructions'][0]
    lines=['# BindV4CapsulePhysicalPrismRequest','', '## A. Task classification','',
        f'**{classification}**. The common architecture is supportable, but a complete capsule physical prism request is not bound.',
        'Supported: recover the V3 method and reference inputs; recompute retained physical fields and conditional raw-dump reductions; bind unchanged capsule surface/geometry and global semantics; identify every missing physical field. Unsupported: capsule h1, a transferred NACA Cf/length model, full historical convergence replay, complete capsule schedule, prism quality, CFD accuracy or mesh readiness.','',
        '## B. V3 first-height provenance','',
        'The actual practical producer is physics_driven_prism_75mph/practical_near_wall_retry/analyze_policy.py. It reads xfoil_results_A/surface_envelope.npz produced by run_xfoil_envelope.py. The source labels XFOIL diagnostic, but the arithmetic directly consumes its Cf_design. The earlier derive_physics_schedule.py likewise uses the XFOIL minimum height despite its generic-provisional comments. Source behaviour takes precedence over those labels.',
        'Formula: nu=mu/rho; u_tau=U*sqrt(Cf_design/2); h1=2*yplus*nu/u_tau. h1 is full normal prism thickness. The factor two is the half-height cell-centre convention, not a safety factor. No additional multiplicative factor occurs in this formula. Capsule safety/design factor: NotEstablished.','',
        '| Quantity | Recovered value | Source/ownership |','|---|---|---|',
        f'| U | {r["U"]} m/s = 75 mph * 0.44704 | run_xfoil_envelope.py and retained xfoil.in; V3 study declaration |',
        f'| rho | {r["rho"]} kg/m^3 | V3 declared sea-level atmosphere |',
        f'| mu | {r["mu"]} Pa s | Same source |',
        f'| nu | {r["nu"]} m^2/s | Binary64 mu/rho; stored physicalProperties agrees |',
        '| target y+ | Historical 10 | Practical policy, co-design report; retained flat-plate comparison below |',
        '| wall treatment | Foundation 13 incompressibleFluid, kOmegaSST; noSlip, nutUSpaldingWallFunction, omegaWallFunction blended=true, kLowReWallFunction | analyze_policy.py and retained yplus_10 fields/configuration |',
        '| exact historical Cf_design | NotEstablished | Original envelope/policy and convergence-admission records absent |',
        f'| conditionally reconstructed Cf_design | {finite["cf_design"]} | Eight finite dumps; not proof of original convergence admission |',
        '| Cf method | Pointwise maximum positive Cf across admitted NACA 0012 cases, then arc-measure-weighted p75 | Actual reduction code; not a generic capsule correlation |',
        f'| Reynolds number | {r["reynolds"]} | rho*U*L/mu, L=1 m airfoil chord; retained VISC command agrees |',
        '| characteristic length | 1.0 m, NACA chord | run_xfoil_envelope.py; no capsule length inferred |',
        f'| temperature / gas model | {r["temperature_K"]} K; gamma={r["gamma"]}; R_air={r["gas_constant"]} J/(kg K) | Declared standard sea-level 15 C air model |',
        f'| speed of sound / Mach | {r["speed_of_sound"]} m/s; {r["mach"]} | sqrt(gamma*R*T), U/a; retained MACH command agrees |',
        '| stored SF2 h1 | 0.000166527069 m | Later baseline and exact nominal increments, not a capsule request |','',
        'Assumptions: isolated two-dimensional NACA 0012; incidence stations 0,2,4,6,8 degrees from 0.25-degree continuation; natural Ncrit=9 and forced transition at x/c=.01 on both sides. The airfoil solution uses subsonic compressibility correction; the Foundation comparison uses constant-viscosity incompressible RANS. No explicit roughness correction appears in the recovered sizing formula or named wall treatment. Capsule roughness/transition, incidence and regime are NotEstablished.',
        'The original author documentation describes an isolated-airfoil method with limited separated-flow capability and Karman-Tsien correction. It supplies no capsule applicability qualification: [XFOIL User Primer](https://web.mit.edu/drela/Public/web/xfoil/xfoil_doc.txt).','',
        'The practical y+ selection compares 270 retained plate samples at x>=0.1 m against the y+=5 anchor. The original guides are 2% integrated Cf and 5% local-p95 difference; these are not new capsule gates.','',
        '| Nominal y+ | Mean Cf | Integrated difference % | Local p95 difference % | Numeric original guides |','|---|---|---|---|---|']
    for row in plate['rows']:lines.append(f'| {row["target"]} | {row["mean_cf"]} | {row["integrated_error_percent"]} | {row["local_p95_error_percent"]} | {row["numeric_original_guides_pass"]} |')
    lines += ['', 'The historical report selects y+=10; retained fields corroborate its numerical comparison. Primary solver logs and the original manifest are missing, so complete historical solver-acceptance replay remains NotEstablished. The flat-plate correlation 0.0592*Re_x^(-1/5) is comparison evidence, not the practical Cf_design supplier.','',
        '## C. Binary64 recomputation','',
        f'nu={r["nu"]}; Re_chord={r["reynolds"]}; a={r["speed_of_sound"]}; Mach={r["mach"]}. Hex encodings are preserved in V3_REFERENCE_CONDITION.json.',
        'The ten retained dumps include two nonfinite tripped cases (6 and 8 degrees). All use identical source coordinate rows. Including both invalid dumps propagates NaNs and leaves only 120 samples after the historical reducer filters them. This is not an admissible replacement envelope.',
        f'All-ten diagnostic: Cf={allrows["cf_design"]}; u_tau={allrows["u_tau"]} m/s; h1={allrows["h1"]} m; absolute difference={allrows["absolute_difference_m"]} m; relative difference={allrows["relative_difference"]}.',
        f'Eight-finite conditional reconstruction: Cf={finite["cf_design"]}; u_tau={finite["u_tau"]} m/s; h1={finite["h1"]} m ({finite["hex"]["h1"]}). Preserved SF2 h1={finite["sf2_h1"]} m. Signed difference={finite["signed_difference_m"]} m; absolute difference={finite["absolute_difference_m"]} m; relative difference={finite["relative_difference"]}, or {finite["relative_difference_percent"]}%.',
        f'Conditional 20-layer growth from the original bisection is {finite["conditional_growth_20"]}; cumulative binary64 total is {finite["conditional_nominal_total"]} m. This is consistent with the later stored decimal precision, but does not prove the missing original admission membership or serialization history.',
        f'Cf inverted from stored SF2 h1 is {h["cf_implied_by_sf2"]}. This is circular diagnostic information, not an independent Cf authority.',
        'Exact unconditional historical Cf, h1 and recomputation difference: **NotEstablished**. No new XFOIL or CFD solve was run.',
        f'Earlier-policy diagnostics from its actual formulas: {json.dumps(r["earlier_policy_diagnostics"])}. These do not justify capsule values.','',
        '## D. Transfer classification','', '| Quantity or operation | Category | Interpretation |','|---|---|---|']
    for row in transfers:lines.append(f'| {row["quantity"]} | {row["category"]}: {row["classification"]} | {row["interpretation"]} |')
    lines += ['', '## E. Capsule applicability','',
        'The conversion method transfers conditionally. A documented common freestream/air-property condition is a reasonable qualification architecture, with V3 values available as a reference candidate. Its adoption for the capsule and a flow direction/incidence are not bound here. The selected wall treatment, shear model, representative design statistic and boundary-layer thickness evidence must be valid for each geometry.',
        'The NACA XFOIL Cf model does not establish a three-dimensional capsule shear authority. Required replacement: a validated capsule/axisymmetric-body or appropriate three-dimensional wall-shear model, measurements, or independent qualified CFD evidence for the declared incidence, transition/roughness and flow regime. A flat-plate preliminary estimate would need explicit applicability/qualification evidence before promotion; the presence of that formula in V3 is insufficient.',
        'Capsule radius .04 m and axial segment half-length .04 m remain exact geometry inputs. No radius, diameter, extent, area, volume or surface-edge-derived characteristic length was selected. Capsule characteristic length, Reynolds number, Cf_design and numeric h1: NotEstablished.',
        'A single global h1 is representable and was used by V3 with a representative p75 shear policy. It does not imply uniform y+ or a maximum-y+ guarantee. Scientific support for a capsule global value requires its own shear envelope and wall-treatment policy. That support is NotEstablished; no local h1 is introduced.',
        'Surface edge sizes stay tangential diagnostics. Neither edge size, surface area nor curvature enters the physical first-height calculation.','',
        '## F. Schedule provenance','',
        '| Quantity | Provenance | Capsule transfer |','|---|---|---|',
        '| h1=0.000166527069 m | Later SF2 literal and binary64 schedule; practical physical producer uses y+/shear, conditionally reproduced above | No |',
        '| growth=1.163359674 | Numerical solution for declared total; bounded search, not a universal physical constant | No |',
        '| count=20 | Fewest layers in 20..40 with preferred growth<=1.20; study cost/progression policy | No |',
        '| nominal total=0.020000000131704564 m | Sequential binary64 sum of preserved SF2 increments | No |',
        '| physical total intent=.020 m | Explicit practical-study TARGET_TOTAL; original justification beyond that declaration NotEstablished | No |','',
        'The earlier y+=0.8 workflow computed a turbulent flat-plate delta99 for total thickness. The practical workflow instead declares 20 mm. Similarity of those values is not proof that one was derived from the other. No capsule growth/count/total is fabricated.','',
        '## G. Contract status','',
        f'Existing contract identity remains `{contract_identity()}`. No defaults or optional fields were added. It can represent a fully supported global request without modification; physical evidence is the blocker.','',
        '| Required field | Status | Value / reason |','|---|---|---|']
    for row in binding:lines.append(f'| {row["field"]} | {row["status"]} | {row["value"]}; {row["reason"]} |')
    lines += ['', 'Bound evidence fields above do not form an accepted numeric request. REQUEST_BINDING_STATUS.json is deliberately not a V4ExternalPrismRequest.','',
        '## H. Geometry generation','',
        'Prism vertices generated = 0. Prisms generated = 0. Layer 1 generated = NO. Layer 2 generated = NO. Core generated = NO. No request file capable of authorizing generation was produced.',
        'All prism hard gates remain unchanged and untested here: positive volume/Jacobian, topology/ownership, zero forbidden/unresolved, determinant>=.0011, weight>=.055, skew<=3.63636, symmetric ratio>=.01, aspect<=1000. Sigma and G remain diagnostics; no hard floor/ceiling was added.','',
        '## I. Tests','',
        f'{checks["tests"]} passed; {checks["failures"]} failed; {checks["errors"]} errors. Command: `{checks["command"]}`. Test time {checks["seconds"]} s.',
        'Coverage: exact source hashes, source mutation rejection, recovered arithmetic, missing/nonpositive/nonfinite inputs, no physical defaults, finite output versus convergence admission, original RMS boundary, global/local distinction, nonpositive height rejection, binary64 schedule/total consistency, geometry/surface mismatch, unchanged contract, no generation. No mesh or CFD quality tests are claimed.','',
        '## J. Performance and resources','',
        f'Bounded audit time {run["seconds"]} s; peak RSS {run["peak_rss_bytes"]} bytes; retained RSS {run["retained_rss_bytes"]} bytes. Wall time from preflight start to seal {resources["wall_time_to_seal_seconds"]} s. Whole-session peak RSS: NotMeasured.',
        f'Repository accessible logical bytes {p["repository_accessible_logical_bytes"]}; V4 {p["v4_accessible_logical_bytes"]}; starting free {p["starting_free_bytes"]}; preseal ending free {free}. Disk audit excludes link/junction traversal and records {len(p["scan_errors"])} access errors.',
        'Largest V4 stores: capsule authenticated checkpoint 1,400,762,368 bytes; four-region pending receipts 1,149,747,200 bytes; repeated checkpoint objects 991,858,688 bytes; old capsule certificate JSON 902,427,369 bytes. They were neither scientific inputs nor cleanup targets.',
        'Bytes deleted=0; deleted paths=[]; no cleanup required. Accepted authorities and crash dumps preserved.','',
        '## K. Evidence identities','',
        f'Surface `{wall["surface_identity"]}`; geometry `{wall["geometry_identity"]}`; coordinates `{wall["coordinate_hash"]}`; topology `{wall["topology_hash"]}`. Sizing field remains `3a722b42e7ac182fd83a97f278138ebcedcffc3ba50b0034458418e6f5247e20`.',
        'EVIDENCE_MANIFEST.json binds each inspected source, raw dump, physical field, prior authority and wall artifact. HASHES.json binds this report and the new authority. No new production contract identity is created.','',
        '## L. Git state','',
        f'Branch `{p["branch"]}`; starting HEAD `{p["starting_head"]}`; starting dirty count 155; unrelated ending dirty count {len(unrelated)}. V3 task modifications=0. Source and surface hashes verified unchanged. Only the four new V4 tooling/test files and this fresh result directory are task-owned. Final commit is printed after sealing.','',
        '## M. Blockers','']
    lines += ['- '+b for b in blockers]
    lines += ['', 'Separate historical limitation: original XFOIL convergence records and reduced envelope/policy manifests are absent. Conditional reconstruction cannot be relabelled exact historical acceptance. Recovering those NACA records alone would still not qualify a capsule Cf model.','',
        '## N. Recommended next task','',
        '**DefineV4CapsuleExternalFlowQualificationSpecification**: bind the common freestream candidate or other authoritative condition, incidence, transition/surface condition and wall-treatment target; identify an applicable capsule shear and thickness model/evidence with its required length definition and global design statistic. This is a bounded physical-definition task, with no geometry generation. Its output must justify the schedule policy before numerical request binding.','']
    (OUT/'report.md').write_text('\n'.join(lines),encoding='utf-8')
    evidence={q.name:sha(q) for q in sorted(OUT.iterdir()) if q.is_file() and q.name not in ('AUTHORITY.json','HASHES.json')}
    authority=dict(classification=classification,request_bound=False,generation_authorized=False,contract_identity=contract_identity(),
        evidence=evidence,source_manifest_identity=identity(m),surface_identity=wall['surface_identity'],geometry_identity=wall['geometry_identity'],
        conditional_reconstruction=finite,exact_historical_reconstruction='NotEstablished',contract_fields=binding,blockers=blockers,
        new_physics_implemented=False,global_h1_supported='NotEstablished for capsule',wall_and_contract_unchanged=True,
        tests=checks,resources=resources,starting_head=p['starting_head'],
        implementation={str(V4.relative_to(REPO)/'tools'/name):sha(V4/'tools'/name) for name in ('physical_prism_provenance.py','audit_capsule_physical_request.py','seal_capsule_physical_request.py')},
        next_task='DefineV4CapsuleExternalFlowQualificationSpecification')
    test_path=V4/'tests/test_physical_prism_provenance.py'
    authority['implementation'][test_path.relative_to(REPO).as_posix()]=sha(test_path)
    save(OUT/'AUTHORITY.json',authority)
    hashes={q.name:sha(q) for q in sorted(OUT.iterdir()) if q.is_file() and q.name!='HASHES.json'}
    save(OUT/'HASHES.json',hashes)
    print(json.dumps(dict(classification=classification,report_sha256=hashes['report.md'],authority_sha256=hashes['AUTHORITY.json'],resources=resources),indent=2))


if __name__=='__main__':main()
