"""Seal the corrected bounded-envelope continuation, preserving prior reports."""
import json
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from audit_bounded_initial_flow_profile import OUT, BASE, V4, REPO, save
from sdf_cfd_mesher_v4.common.authority import file_hash, identity


def main():
    read=lambda n:json.loads((OUT/n).read_text())
    pre=read('PREFLIGHT.json');profile=read('PROFILE.json');study=read('ENVELOPE.json')
    tests=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite').attrib
    assert int(tests['failures'])==0 and int(tests['errors'])==0
    code=['tools/bounded_initial_flow_profile.py','tools/audit_bounded_initial_flow_profile.py',
          'tools/seal_bounded_initial_flow_profile.py','tests/test_bounded_initial_flow_profile.py']
    prefixes=['research/sdf_cfd_mesher_v4/'+x for x in code]+[
        'research/sdf_cfd_mesher_v4/results/bounded_external_flow_initial_profile/corrected_envelope/']
    status=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
    unrelated=[r for r in status if not any(r[3:].startswith(x) for x in prefixes)]
    assert unrelated==pre['starting_dirty']
    for row in read('INPUT_BINDINGS.json'):assert file_hash(REPO/row['path'])==row['sha256']
    text='''# Corrected bounded external-flow initial mesh profile

## A. Classification
V4BoundedSubsonicExternalFlowInitialMeshProfileQualifiedParameterized.
The user-specified test condition, named wall-treatment profile, conditional model-envelope arithmetic, sensitivity analysis and provenance checks qualify. No unique development station or complete prism schedule is qualified. This is not validation of arbitrary-body wall shear, achieved y+, or actual boundary-layer coverage. Capsule prism request remains UNBOUND.

This continues the sealed partial task. Prior reports, hashes and tests remain unchanged. The corrected inputs are preserved in CORRECTED_INPUT.md; this is user/project policy authority, not published physics evidence.

## B. Operating envelope
External aerodynamic air; 25–150 mph inclusive, exactly converted using 0.44704 m/s per mph to binary64 endpoints11.176 and67.056 m/s. The approximate body-scale range0.5–3.0 m is evaluated at endpoints0.5 and3.0 m for the engineering envelope. The profile treats L as a declared projected streamwise reference extent; that convention is not actual streamline length.

rho=1.225 kg/m^3; mu=1.7894e-5 Pa*s; T=288.15 K; nu=1.4607346938775508e-5 m^2/s. Recovered gamma1.4 and gas constant287.05287 J/(kg K) give sound speed340.293988026089 m/s and Mach0.032842190556546594–0.19705314333927956. This is a fixed qualification atmosphere, not a universal production default.

Re_L=382547.2225326926 at25mph/L0.5m through13771700.011176933 at150mph/L3m. Smooth-wall, constant-property, incompressible, attached ZPG, fully-turbulent-from-origin behavior is the SURROGATE idealization. No automatic transition prediction, roughness correction or arbitrary-body pressure-gradient correction is established.

## C. Model and sources
nu=mu/rho; Re_x=U*x/nu; local Cf(x)=0.0592/Re_x^0.2; u_design=U*sqrt(Cf(x_start)/2); full h1=2*y_target*nu/u_design. Coverage delta=0.37*x_end/Re_end^0.2. U is freestream speed; x is model development distance; nu is kinematic viscosity; Cf is local skin friction referenced to freestream dynamic pressure; u_design is the corresponding friction velocity. h1 is full first-prism normal thickness under the nominal half-height cell-centre convention.

Published evidence: MIT course notes, chapter9 printed pages133–134 (PDF608–609), https://web.mit.edu/snively/www/2_005%20Coursenotes-1.pdf. The source gives the local coefficient0.0592, the integrated plate-mean coefficient0.074, and turbulent thickness power law. Mean friction is not maximum friction. The notes discuss transition and a different very-high-Re formulation. No universal numerical validity gate was invented here.

Recovered V3 evidence: physics_driven_prism_75mph/derive_physics_schedule.py lines80–83 contains x/L0.1 and a different logarithmic Cf expression. This study uses only that station as a HISTORICAL candidate while holding the current power-law model fixed. It does not reproduce the earlier logarithmic model. The source comment's NASA attribution could not be independently recovered; https://geolab.larc.nasa.gov/APPS/YPlus/ was unavailable. No calculator supplied authority.

## D. Projected length
L=max_i(p_i dot e)-min_i(p_i dot e), with e the normalized flow vector. The existing tested diagnostic uses rational dot products of represented binary64 inputs. It is row-order invariant and reversal invariant. Exactly representable translation and coordinate permutation with matching flow permutation pass. Normalization rejects zero and nonfinite vectors. General floating-point translations can lose input precision; no bitwise invariance is claimed after such loss.

The measured mesh extent is not an exact continuous-SDF extremum or a boundary-layer streamline distance. No surface edge size, area, or geometry-type tag selects physics.

## E. Development-distance candidates and selection
Set x_start=alpha*L and investigate x_end=L as one projected body-length engineering coverage. alpha must be positive and <=1. No alpha is selected.

Candidates are: (1)0.1, directly recovered historical V3 station with incomplete external justification; (2)(0.0592/0.074)^5, the station whose local power-law Cf equals its integrated mean Cf; (3)1, local end-of-plate diagnostic. The mathematical mean-equivalent fraction is0.32768; the implemented binary64 calculation gives0.3276800000000001. These are source-derived diagnostic choices, not a sweep of arbitrary fractions. Neither mean nor endpoint is a maximum-shear design rule.

h1(alpha)/h1(1)=alpha^0.1. Changing0.1 to1 multiplies h1 by1.2589254117941673. Changing0.1 to the mean-equivalent candidate multiplies h1 by1.1260171197494693. Coverage is independent of alpha. No small sensitivity is used as a permission to select an unsupported fraction.

## F. Full envelope extrema
All values below are conditional POWER-LAW MODEL calculations, not measured flow or selected production sizes. Locations are (mph,L_m). Re_start scales with alpha. Re_L extrema are given above. Cf minimum is at(150,3); maximum at(25,0.5). Friction velocity minimum is at(25,3); maximum at(150,0.5). h1 and coverage minima are at(150,0.5), maxima at(25,3).

'''
    for r in study['results']:
        text+=f"### Candidate alpha={r['candidate']['alpha']}\n\n| Quantity | Minimum | Maximum |\n|---|---:|---:|\n"
        for key in ('re_shear_start','cf_design','u_tau','full_first_height_m','physical_coverage_estimate_m'):
            e=r['production']['extrema'][key]
            text+=f"| {key} | {e['minimum']['values'][key]} | {e['maximum']['values'][key]} |\n"
        text+='\n'
    text+='''Monotonic relations at fixed fluid properties and y+: Re_L proportional U*L; Cf proportional U^-0.2*(alpha*L)^-0.2; u_design proportional U^0.9*(alpha*L)^-0.1; h1 proportional U^-0.9*(alpha*L)^0.1; delta proportional U^-0.2*L^0.8. These establish corner extrema without a sampled search. Tests also compare interior points and exact replay.

Inconvenient limits are retained: the0.1L candidate has Re_start as low as38254.72225326926 in the primary envelope. The capsule goes lower. Fully turbulent physical flow is not demonstrated there. Re_L reaches13771700.011176933, beyond the range where the cited notes continue with the same simple power law without discussing high-Re alternatives. Thus calculation over the envelope does not establish model accuracy throughout it. If all0<alpha<=1 are permitted, the formal model has Cf and u_design unbounded above and h1 tending to zero as alpha tends to zero. That singular extrapolation is not real physical authority.

## G. Envelope versus per-case sizing
At each fixed candidate alpha, h1 varies by a mathematical factor6 across this speed/scale envelope. Binary64 ratios are6.0,6.0 and5.999999999999999 for the three candidates. Coverage varies by6.0. Applying the smallest model h1 to every case and the largest model coverage to every case would differ by up to those factors from the corresponding per-case model values. This is a numerical comparison, not a recommendation or a physical guarantee.

## H. Named wall-treatment profile
The user explicitly authorizes y+=10 with the recovered V3 configuration as a named V4 QUALIFICATION profile: Foundation13 incompressibleFluid; kOmegaSST; U noSlip; nutUSpaldingWallFunction; omegaWallFunction blended=true; kLowReWallFunction. This is now bound as test policy, not a production default.

Retained V3 flat-plate arithmetic showed target10 differed from the target5 anchor by0.3404454121781697 percent in integrated Cf and2.6447359556230707 percent in local p95 error, within historical study guides. Original complete solver-admission records remain missing. User adoption does not restore them or turn those study guides into universal gates. Foundation's Spalding model is continuous to the wall; it does not independently prove achieved y+ or arbitrary-body accuracy. Source: https://cpp.openfoam.org/v13/classFoam_1_1nutUSpaldingWallFunctionFvPatchScalarField.html.

## I. Capsule application
The recovered capsule axis is z. The primary qualification orientation is now declared +z from the user's axial-orientation instruction; -z gives the same projected extent for this diagnostic. Reused measured L=0.15999741537843415m. Analytic axial extent is0.16m, but the measured qualified-surface extent is used without substituting0.5m. This is a smaller algorithmic point outside the primary scale envelope, not an in-envelope production validation.

The current surface loader reverified unchanged coordinates/topology and scientific identity. Re_L spans122413.133730859–734478.802385154. Conditional actual-scale evaluations are authorized as this sensitivity study, not as a bound production request. Capsule minima/maxima below are over25–150mph: Re and u_design increase with speed; Cf, h1 and coverage decrease with speed.

'''
    for r in study['results']:
        text+=f"### Capsule candidate alpha={r['candidate']['alpha']}\n\n| Quantity | Minimum | Maximum |\n|---|---:|---:|\n"
        for key in ('re_shear_start','cf_design','u_tau','full_first_height_m','physical_coverage_estimate_m'):
            e=r['capsule']['extrema'][key]
            text+=f"| {key} | {e['minimum']['values'][key]} | {e['maximum']['values'][key]} |\n"
        text+='\n'
    text+='''## J. Total coverage
x_end=L_projected is accepted here as the explicit engineering surrogate requested for study, not an actual arbitrary-body boundary-layer claim. Its primary envelope is0.009885632642095665–0.059313795852573986m. Capsule coverage is0.0039730024958048–0.005685243730650682m. It may supply a declared physical_total_thickness_intent for a reviewed per-case engineering request after selecting case conditions and accepting this meaning. No complete capsule request is emitted. Nominal discrete schedule total remains a separate binary64 quantity.

## K. Progression
Existing schedule arithmetic is qualified. General selection of growth/count is not. Historical20..40 counts, preferred growth<=1.20, and20mm coverage remain historical policy. No new count, growth, last-layer size, or unsupported limit is selected. Progression remains explicitly unresolved in every envelope result.

## L. Production prism contract status
| Required field | Status | Reason |
|---|---|---|
| Qualified geometry/surface identities | Bound | Unchanged production surface loader verification |
| Physical or production-test intent | Bound at profile level | User-authorized qualification condition; not universal production default |
| Request owner | Bound at profile level | User-authorized V4 study; no complete per-case request issued |
| Physical basis | Partial | Named idealized model and limits; alpha selection unresolved |
| Hash-bound evidence | Bound for study | Corrected input, prior V3 sources and source PDF hashes |
| Fluid side | Available, not emitted in request | Exterior capsule uses positive SDF |
| Full first-prism height definition | Bound | Full normal thickness, nominal centre at half-height |
| Global/local meaning | Bound | Global only; local maps remain unqualified |
| Explicit positive binary64 increments | Unbound | No progression selected |
| Layer count | Unbound | No general selection qualified |
| Nominal schedule total | Unbound | No explicit schedule |
| Separate physical total-thickness intent | Parameterized | x_end=L engineering estimate; per-case speed still to select |

The current prism contract is unchanged. The new profile binder rejects production-case kind, missing y+ evidence, source hash mismatch, unsupported coverage claims and silent alpha adoption. It cannot authorize meshing.

## M. Geometry generation
Prism vertices generated=0; prisms generated=0; layer1=NO; layer2=NO; shell=NO; core=NO. Existing hard gates unchanged and not geometrically tested. Sigma/G remain diagnostics. No old repair task resumed.

## N. Tests
'''
    text+=f"{tests['tests']} tests; {tests['failures']} failures; {tests['errors']} errors; {tests['skipped']} skipped; {tests['time']} seconds. Five focused suites cover the new bounded profile plus prior projection, initial policy, physical provenance and production contract. Existing tests unchanged.\n\n"
    text+='''## O. Resources
'''
    resources=dict(repository_accessible_logical_bytes=pre['repository_accessible_logical_bytes'],
        v4_accessible_logical_bytes=pre['v4_accessible_logical_bytes'],access_errors=pre['access_error_count'],
        starting_free_bytes=pre['starting_free_bytes'],ending_free_bytes=shutil.disk_usage(REPO).free,
        bytes_deleted=0,deleted_paths=[],preflight_to_seal_seconds=time.time()-pre['start_epoch'],
        **read('RUNTIME.json'))
    save('RESOURCES.json',resources)
    text+='\n'.join(f'- {k}: {v}' for k,v in resources.items())+'\n\n'
    text+='''No cleanup needed; no old mesh, checkpoint, receipt, crash dump or scientific evidence deleted. Largest relevant V4 checkpoint remains1400762368bytes; receipt store1149747200bytes, as recorded by the preceding audit. Whole-session time before preflight was not instrumented.

## P. Identities
'''
    text+=f"Named parameterized profile: `{profile['binding']['profile_identity']}`.\n\n"
    text+='''Prior policy: e7e4570275ae79a0b9af25ba70ee19f49445ea242517071dc29fa9f059de528f.
Prism contract: 7de6b4527da1cd55659c1796927394cbd9af2f581cffbd85d2f891705abbf7dd.
Capsule surface: 14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd.
Capsule geometry: a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65.
Coordinates: 2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c.
Topology: d14f00450f2f67f7b98ab3efa1a7946393b7b793ccd64d4bdcdbf516342de2db.
Sizing field: 3a722b42e7ac182fd83a97f278138ebcedcffc3ba50b0034458418e6f5247e20.
MIT source PDF: 72ebf3bdcf60c1b8a008231f11777490a2b9ec9005cfee14c1a38df8dfcf9d0b.
All prior partial authorities are preserved; report/authority digests are in HASHES.json and final chat.

## Q. Git safety
'''
    text+=f"Starting HEAD {pre['starting_head']}; branch {pre['branch']}; unrelated dirty entries {len(unrelated)} before and at seal. V3 task changes0. Only this continuation's V4 files are staged. Final HEAD and path count are printed after commit.\n\n"
    text+='''## R. Blockers
No scientifically adopted global shear-design station alpha; no independently recovered source qualifying the historical0.1 fraction; incomplete physical applicability of the power-law estimate over all Reynolds/pressure-gradient conditions; no per-case capsule speed selected from the envelope; no qualified count/growth choice or explicit complete schedule. No achieved-y+ or mesh-quality claim exists. Mean/end-station diagnostics cannot resolve these by themselves.

## S. Recommended next task
QualifyV4InitialDesignStationAndProgressionRequest: establish a defensible explicitly named engineering shear station and progression selection, then bind one reviewed capsule operating point. Retain estimate limitations; do not generate prisms until the complete request is reviewed.
'''
    (OUT/'report.md').write_text(text,encoding='utf8')
    paths=[p for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')]
    paths += [V4/x for x in code]
    files=[dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p)) for p in paths]
    save('AUTHORITY.json',dict(classification='V4BoundedSubsonicExternalFlowInitialMeshProfileQualifiedParameterized',
        profile_identity=profile['binding']['profile_identity'],development_alpha=None,
        complete_capsule_request=False,geometry_generation_authorized=False,prisms_generated=0,
        tests=tests,starting_head=pre['starting_head'],unrelated_dirty_preserved=True,v3_modifications=0,
        files=files,evidence_identity=identity(files)))
    hashes={n:file_hash(OUT/n) for n in ('report.md','AUTHORITY.json')}
    save('HASHES.json',hashes)
    print(json.dumps(dict(hashes=hashes,profile_identity=profile['binding']['profile_identity'],resources=resources),indent=2))


if __name__=='__main__':main()
