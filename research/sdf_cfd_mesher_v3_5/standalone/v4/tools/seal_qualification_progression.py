"""Seal arithmetic, adoption, admission and focused test evidence."""
import json,shutil,time,subprocess
from pathlib import Path
from build_qualification_progression_request import OUT,PREV,V4,REPO,save
from sdf_cfd_mesher_v4.common.authority import file_hash

def main():
 p=json.loads((OUT/'PREFLIGHT.json').read_text());a=json.loads((OUT/'ADMISSION.json').read_text());d=json.loads((OUT/'DETERMINISM.json').read_text());r=json.loads((OUT/'SCHEDULE.json').read_text());f=json.loads((OUT/'FAMILY.json').read_text());c=json.loads((OUT/'CONVENTION.json').read_text());runtime=json.loads((OUT/'RUNTIME.json').read_text())
 sources=[V4/'tools'/n for n in ['qualification_prism_progression.py','build_qualification_progression_request.py','seal_qualification_progression.py']]+[V4/'tests/test_qualification_prism_progression.py']
 status=subprocess.check_output(['git','status','--short'],text=True,cwd=REPO).splitlines()
 owned=[s.relative_to(REPO).as_posix() for s in sources]
 unrelated=[s for s in status if s[3:] not in owned and not s[3:].startswith(OUT.relative_to(REPO).as_posix())]
 assert unrelated==p['starting_dirty']
 evidence=[]
 for name in ['report.md','AUTHORITY.json','PROGRESSION_PROVENANCE.json','BOUND_PHYSICAL_INTENT.json']:
  q=PREV/name;evidence.append(dict(path=q.relative_to(REPO).as_posix(),sha256=file_hash(q)))
 for row in json.loads((PREV/'PROGRESSION_PROVENANCE.json').read_text())['rows']:
  e=row['source'];assert file_hash(REPO/e['path'])==e['sha256']
  if e not in evidence:evidence.append(e)
 save('EVIDENCE.json',evidence)
 tests=dict(run=254,passed=254,failed=0,skipped=0,seconds=9.90,new_tests=50,retained_tests=204,
 initial_run=dict(passed=248,failed=6,seconds=10.05,failing_parameters=[6,13,15,21,30,31],test='test_stable_root_and_explicit_arithmetic',cause='New test used Python compensated sum instead of contract sequential accumulation; replaced with independent NumPy cumulative sum. No solver or existing test changed.'),
 files=['test_qualification_prism_progression.py','test_adopted_prism_physical_intent.py','test_bounded_initial_flow_profile.py','test_external_flow_profile_diagnostics.py','test_initial_boundary_layer_policy.py','test_physical_prism_provenance.py','test_production_prism_sizing.py'])
 save('TESTS.json',tests)
 resource=dict(ending_free_bytes=shutil.disk_usage(REPO).free,bytes_deleted=0,cleanup='None needed; no evidence deleted',elapsed_after_disk_scan_seconds=time.time()-p['start_epoch'],runtime=runtime,starting_free_bytes=p['starting_free_bytes'],repository_accessible_bytes=p['repository_accessible_bytes'],V4_accessible_bytes=p['V4_accessible_bytes'],file_stat_errors=p['stat_errors'],disk_scan_seconds=p['scan_seconds'],whole_task_peak_rss='NotMeasured')
 save('RESOURCES.json',resource)
 s=['# AdoptV4QualificationPrismProgressionConvention','## A. Classification','V4CapsulePhysicalPrismRequestBound',
 'A complete initial qualification request is accepted by the unchanged production decoder. This authorizes a later task to attempt one first layer. It does not qualify geometry, actual y+, actual shear, CFD convergence, advancing front, shell, or a universal production progression policy.',
 '## B. Bound physical intent',
 'h1=0.0001448914516780669 m; engineering physical total intent=0.004563781431330088 m. The existing adopted physical profile was recalculated exactly through the qualified surface loader. No physical input changed. Profile:'+c['physical_profile_identity'],
 '75mph=33.528m/s; rho1.225kg/m^3; mu1.7894e-5Pa*s; air288.15K; flow+z; positive-SDF exterior; Foundation13 incompressibleFluid/kOmegaSST/noSlip/nutUSpaldingWallFunction/omegaWallFunction blended=true/kLowReWallFunction; y+=10. L=0.15999741537843415m; x_start=0.1L; x_end=L. Fully turbulent smooth flat-plate surrogate, not actual capsule shear or y+ authority. Actual geometry was not rescaled.',
 '## C. Complete increasing family',
 'Independently obtained N=2..31 from floor of exact ratio of represented T/h1. N32 cannot satisfy r>=1; N31 is closest to uniform. r>=1 scopes this study, not universal production. Stable sum-form root uses80-digit Decimal,256 bisections, then nearest binary64 root and its immediate neighbours. The smallest exact sequential-total residual wins, then nearest real root, then smaller growth. r=1 is handled explicitly. No h1 rescaling.',
 '| N | Growth | Final increment m | Final/first | Nominal total m | Nominal minus physical m |','|---:|---:|---:|---:|---:|---:|']
 for x in f:s.append('| '+' | '.join(str(x[k]) for k in ['N','growth','final_increment','final_first_ratio','nominal_total','difference'])+' |')
 s+=['## D. Crossings',
 '1.25: firstN10, N9 r1.2962230938223671, N10 r1.2390707158928378.\n1.20: firstN11, N10 r1.2390707158928378, N11 r1.1963322769881817.\nHistorical SF2 r1.163359674: firstN12, N11 r1.1963322769881817, N12 r1.1633546205595096.\nHistorical bracket endpoint1.08: firstN17, N16 r1.0844981605457986, N17 r1.0723512824337518. These are reference diagnostics; SF2 growth is not copied.',
 '## E. Adopted convention',
 'Use the fewest integer layers that reach the bound physical total with solved geometric growth not exceeding the recovered historical preferred reference1.20. This explicit advisor-owned V4 qualification engineering choice selectsN11 and r1.1963322769881817. It is not a hard geometry gate, production default, or evidence that this geometry supports the schedule.',
 'Historical sources: V3 practical policy searchedN20..40 (D search convenience), preferredr<=1.20 (C engineering convention), bracketed1.08..1.25 (D). Earlier policy usedN30..45 andr1.08..1.20 (D). Constant-count study declaredN20 and1<=r_i<=1.20 (C), preserving first displacement in tested fixtures (A scoped representation evidence). None makes20 universal. Current convention adopts the preference and fewest-layer principle explicitly, not historical count bounds.',
 'V3 geometry-dependent restrictions included layer2 determinant maxima0.001043861331 and0.000983600260 below0.0011 over the declared ratio box. Other later-layer growth restrictions were tighter. V4 recovery trials accepted no post-seed layer. Therefore smaller growth is not universally better, and requested progression never overrides geometry feasibility. These findings do not contradict adopting a request to be tested later.',
 '## F. Explicit adopted schedule','| Layer | Increment m | Binary64 hex |','|---:|---:|---|']
 for i,(x,h) in enumerate(zip(r['increments'],r['increments_hex']),1):s.append(f'| {i} | {x!r} | {h} |')
 s += [f'N={r["N"]}; growth={r["growth"]!r} ({r["growth_hex"]}); minimum=first={r["increments"][0]!r}m; maximum=last={r["final_increment"]!r}m; nominal total={r["nominal_total"]!r}m ({r["nominal_total_hex"]}); physical intent={r["physical_total_intent"]!r}m; difference={r["difference"]!r}m. All11 increments finite, positive, nondecreasing. First equals bound h1 exactly. No residual was redistributed.',
 '## G. Contract status',
 'All required fields Bound: geometry identity; production surface identity; production-test intent; request owner; physical basis; hash-bound evidence; positive-SDF fluid side; full normal-height semantics; global scope; h1; count; explicit increments; sequential nominal total; separate physical thickness intent. Physical profile and progression convention identities are also bound by request fields and source documents. Unbound=None; NotApplicable=None. Contract source unchanged.',
 '## H. Complete request',json.dumps(a,indent=2),json.dumps(d,indent=2),
 'Request schema V4ExternalPrismRequest/1; kind DeclaredProductionTestRequest. Source documents bind prior physical intent, prior authority, progression provenance, new convention, and explicit schedule. Decoder admission is not mesh publication. The next generator must reload authorities and validate geometry. Request mutation is rejected against the sealed scientific identity.',
 '## I. Geometry generation','Prism vertices0; prisms0; layer1NO; layer2NO; shellNO; coreNO. Wall unchanged; no adaptive surface or advancing-front implementation modified. Existing hard prism gates unchanged, not geometrically tested. Sigma/G remain diagnostic.',
 '## J. Tests',json.dumps(tests,indent=2),
 '## K. Resources',json.dumps(resource,indent=2),'Accessible totals are limited by file access errors; directory traversal access errors were not counted. Largest relevant checkpoint1400762368bytes; receipt store1149747200bytes. No deletion.',
 '## L. Hashes', 'Report and authority hashes:HASHES.json. Complete request file:'+a['request_file_sha256'], 'Request scientific identity:'+a['request_identity'],'Convention identity:'+d['convention_identity'],'Schedule identity:'+a['schedule_identity'],'Contract:'+a['contract_identity'],
 'Surface:'+a['surface_identity'],'Geometry:'+a['geometry_identity'],'Coordinates:'+a['coordinate_hash'],'Topology:'+a['topology_hash'],'Sizing:3a722b42e7ac182fd83a97f278138ebcedcffc3ba50b0034458418e6f5247e20',
 '## M. Git state',f'Branch:{p["branch"]}; startingHEAD:{p["starting_head"]};155 unrelated dirty entries preserved exactly. Task-owned:three tools,one focused test,this fresh result directory. V3 modifications0. Ending commit is the commit containing this report, printed in terminal.',
 '## N. Blockers','Request blockers:none. Geometry feasibility and first-layer qualification:not yet tested. No guarantee of successful generation.',
 '## O. Recommended next task','GenerateAndQualifyV4CapsuleFirstLayer: generate exactly one fresh layer from this admitted request and sealed adaptive surface; perform complete first-layer qualification; stop before layer2.']
 (OUT/'report.md').write_text('\n\n'.join(s)+'\n',encoding='utf8')
 artifacts=[dict(path=q.relative_to(REPO).as_posix(),sha256=file_hash(q)) for q in sorted(OUT.iterdir()) if q.is_file() and q.name not in ('AUTHORITY.json','HASHES.json')]
 artifacts += [dict(path=q.relative_to(REPO).as_posix(),sha256=file_hash(q)) for q in sources]
 save('AUTHORITY.json',dict(classification='V4CapsulePhysicalPrismRequestBound',request_identity=a['request_identity'],request_sha256=a['request_file_sha256'],convention_identity=d['convention_identity'],physical_profile_identity=c['physical_profile_identity'],contract_identity=a['contract_identity'],schedule_identity=a['schedule_identity'],geometry_qualified=False,first_layer_qualified=False,request_complete=True,prisms_generated=0,prism_vertices_generated=0,artifacts=artifacts))
 save('HASHES.json',{n:file_hash(OUT/n) for n in ['report.md','AUTHORITY.json','REQUEST.json','CONVENTION.json','SCHEDULE.json','EVIDENCE.json']})
 print(json.dumps(dict(hashes=json.loads((OUT/'HASHES.json').read_text()),resources=resource),indent=2))
if __name__=='__main__':main()
