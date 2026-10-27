"""Seal non-generative request evidence after focused tests."""
import json,time,shutil,subprocess
from pathlib import Path
from audit_design_station_request import OUT,V4,REPO,save
from sdf_cfd_mesher_v4.common.authority import file_hash

def main():
 p=json.loads((OUT/'PREFLIGHT.json').read_text()); b=json.loads((OUT/'BOUND_PHYSICAL_INTENT.json').read_text())
 rules=json.loads((OUT/'PROGRESSION_PROVENANCE.json').read_text()); runtime=json.loads((OUT/'RUNTIME.json').read_text())
 status=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
 owned='research/sdf_cfd_mesher_v4/'
 unrelated=[s for s in status if not (s[3:].startswith(owned+'results/initial_design_station_progression_request') or s[3:] in [owned+'tools/'+n for n in ['adopted_prism_physical_intent.py','audit_design_station_request.py','seal_design_station_request.py']] or s[3:]==owned+'tests/test_adopted_prism_physical_intent.py')]
 assert unrelated==p['starting_dirty'],'UnrelatedDirtyStateChanged'
 files=json.loads((OUT/'EVIDENCE_MANIFEST.json').read_text())['files']
 for f in files:assert file_hash(REPO/f['path'])==f['sha256']
 tests=dict(run=204,passed=204,failed=0,skipped=0,seconds=8.49,files=['test_adopted_prism_physical_intent.py','test_bounded_initial_flow_profile.py','test_external_flow_profile_diagnostics.py','test_initial_boundary_layer_policy.py','test_physical_prism_provenance.py','test_production_prism_sizing.py'])
 save('TESTS.json',tests)
 resources=dict(runtime=runtime,elapsed_since_preflight_seconds=time.time()-p['start_epoch'],ending_free_bytes=shutil.disk_usage(REPO).free,bytes_deleted=0,deletion_reason='No cleanup needed; no scientific evidence deleted',unrelated_dirty_preserved=True,V3_task_modifications=0)
 save('RESOURCES.json',resources)
 classification='V4CapsulePhysicalPrismIntentBoundProgressionRequired'
 s=['# QualifyV4InitialDesignStationAndProgressionRequest','', '## A. Classification',classification,
 'The adopted global full first-height and separate engineering coverage intent are bound. The production request is incomplete. No universal progression criterion was recovered. No actual wall-shear bound, actual y+ guarantee, production default, prism quality, first-layer qualification, or complete mesh is claimed.',
 '## B. Adopted qualification profile',
 'User/advisor-owned qualification choices: 75 mph = 33.528 m/s; rho=1.225 kg/m^3; mu=1.7894e-5 Pa*s; temperature=288.15 K. Flow +z; fluid positive-SDF exterior. Foundation13 incompressibleFluid; kOmegaSST; U noSlip; nutUSpaldingWallFunction; omegaWallFunction blended=true; kLowReWallFunction; target y+=10.',
 'L=max(dot(p,normalized_flow))-min(dot(p,normalized_flow)) over qualified surface vertices. Actual capsule scale is retained. x_start=0.1L; x_end=L. The 0.1 fraction is an explicit engineering convention, not a physical law. Surface edge size does not enter physical sizing.',
 'Re_x=U*x/nu; nu=mu/rho; Cf=0.0592/Re_start^0.2; u_design=U*sqrt(Cf/2); tau_design=rho*u_design^2; h1=2*yplus*nu/u_design; coverage=0.37*x_end/Re_end^0.2.',
 'Smooth, fully turbulent, incompressible, zero-pressure-gradient flat-plate surrogate. This is an initial design estimate, including low-Re use at the adopted station; it does not establish transition or attached turbulent capsule flow. The factor two models the first cell centre at half full thickness. Coverage is one projected body-length engineering intent, not actual arbitrary-body boundary-layer thickness. Prior model evidence and limitations remain hash-bound.',
 '## C. Capsule physical calculation','| Quantity | Binary64 value | Hex |','|---|---:|---|']
 for k,v in b['values'].items():s.append(f'| {k} | {v!r} | {b["binary64"][k]} |')
 s+=['## D. Progression provenance','Categories: A independently qualified only within stated scope; B case-specific physical policy; C historical engineering convention; D search/implementation convenience; E diagnostic; F unknown. No recovered rule selects production N/r.']
 for r in rules['rows']:
  s += [f'### {r["key"]} — {r["category"]}',r['rule'],r['measured_evidence'],f'Source: `{r["source"]["path"]}`. SHA-256: `{r["source"]["sha256"]}`. May select production count/growth: No.']
 s+=['## E. Progression analysis',
 'For a chosen integer N>=2, the positive-r geometric sum increases strictly with r. Fixing N and solving r is deterministic arithmetic, but requires a count policy. Fixing r requires a count/rounding/coverage policy. Geometry-based optimization requires later qualified feasibility evidence and is outside this non-generative task. No option supplies its own scientific selector.',
 f'T/h1={b["progression"]["ratio_binary64"]!r}; exact ratio of represented inputs={b["progression"]["ratio_exact"]}. Conditional on r>=1, N=2 through31 all admit real-arithmetic solutions:30 count choices. This condition is a diagnostic assumption, not a new growth gate. N=1 cannot match this T; uniform increments cannot exactly match this noninteger ratio. Floating-point schedules would require explicit arithmetic verification after policy selection.',
 'Fixture studies remain evidence for the arithmetic and tested geometry behavior they exercised. They do not establish production surface-to-core qualification. V3 interval studies show both lower and upper geometry-dependent ratio restrictions. V4 recovery trials did not qualify an accepted future schedule. No generic maximum/preferred growth or count is established.',
 '## F. Final schedule','N=NotEstablished; growth=NotEstablished; increments=NotEstablished; nominal schedule total=NotEstablished; nominal-minus-physical difference=NotEstablished. No schedule was manufactured. Physical intent is coverage above. The missing item is an explicitly supported or adopted progression selection criterion.',
 '## G. Contract status',
 'Bound as upstream physical-intent evidence: qualified geometry identity; qualified production surface identity; production-test intent; request owner; physical basis; hash-bound evidence; positive-SDF fluid side; full first-prism normal-height definition; global scope; separate physical total-thickness intent.',
 'Unbound: explicit positive binary64 increments; layer count; nominal schedule total. No required fields are NotApplicable. The existing contract is unchanged. Partial evidence uses V4BoundPhysicalIntentOnly/1; production admission rejects it with UnsupportedPrismRequestSchema. It is deliberately not an admitted V4ExternalPrismRequest/1.',
 '## H. Geometry generation','Prism vertices generated=0; prisms generated=0; layer1=NO; layer2=NO; shell=NO; core=NO. Wall geometry/topology and adaptive surface implementation are unchanged. Prism hard gates are unchanged and not geometrically tested. Sigma and G remain diagnostics.',
 '## I. Tests',json.dumps(tests,indent=2),
 '## J. Resources',json.dumps(resources,indent=2),
 'Accessible logical repository use=844063788606 bytes; V4=171767676613 bytes;1003 access errors limit totals. Starting free space=400149118976 bytes. Disk scan=42.20323419570923s. Largest relevant checkpoint=1400762368 bytes; receipt store=1149747200 bytes. Whole-task peak RSS=NotMeasured. Audit peak RSS is measured separately. No deletion.',
 '## K. Identities',f'New adopted physical profile: {b["profile_identity"]}',
 'Previous parameterized profile:45a4dfd2e1e64fb6e3529a4aee1caa9d3514e356f98b4f121ce9cde3a2a86b6e',
 'Prism contract:7de6b4527da1cd55659c1796927394cbd9af2f581cffbd85d2f891705abbf7dd',
 'Geometry:a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65',
 'Surface:14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd',
 'Coordinates:2270c352c34bba85dc898da4ea66dda1484123604d96511551899802d16d157c',
 'Topology:d14f00450f2f67f7b98ab3efa1a7946393b7b793ccd64d4bdcdbf516342de2db',
 'Sizing:3a722b42e7ac182fd83a97f278138ebcedcffc3ba50b0034458418e6f5247e20',
 'Report/authority hashes are in HASHES.json, avoiding self-reference. No complete request identity exists.',
 '## L. Git state',f'Branch:{p["branch"]}; starting HEAD:{p["starting_head"]}; unrelated starting entries155, preserved exactly at seal. Task-owned files: three tools, one focused test, and this fresh result directory. V3 task modifications0. Final commit is the commit containing this report, recorded in the terminal response.',
 '## M. Blockers','1. No qualified or explicitly adopted criterion selects count/growth.\n2. Therefore explicit increments, layer count, and nominal schedule total cannot yet bind. All adopted physical inputs are present; no additional physical input is claimed missing.',
 '## N. Recommended next task','AdoptV4QualificationPrismProgressionConvention: explicitly review and adopt one bounded progression selection convention, calculate and verify its binary64 schedule, and bind the complete request. Generate no geometry in that task.']
 (OUT/'report.md').write_text('\n\n'.join(s)+'\n',encoding='utf8')
 artifacts=[dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p)) for p in sorted(OUT.iterdir()) if p.is_file() and p.name not in ('AUTHORITY.json','HASHES.json')]
 for p in [V4/'tools'/n for n in ['adopted_prism_physical_intent.py','audit_design_station_request.py','seal_design_station_request.py']]+[V4/'tests/test_adopted_prism_physical_intent.py']:
  artifacts.append(dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p)))
 save('AUTHORITY.json',dict(classification=classification,profile_identity=b['profile_identity'],physical_intent_bound=True,complete_request=False,generation_authorized=False,progression='NotEstablished',tests=tests,artifacts=artifacts))
 save('HASHES.json',{n:file_hash(OUT/n) for n in ['report.md','AUTHORITY.json']})
 print(json.dumps(dict(hashes=json.loads((OUT/'HASHES.json').read_text()),resources=resources),indent=2))
if __name__=='__main__':main()
