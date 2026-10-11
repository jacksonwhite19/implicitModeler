"""Build the declared capsule qualification request without generating geometry."""
import json,os,time
from pathlib import Path
import psutil
from qualification_prism_progression import family,adopt,first_crossing,verify_bound_request
from adopted_prism_physical_intent import bind_physical_intent
from production_surface_authority import load_production_wall
from production_prism_sizing import inspect_first_layer_inputs,contract_identity
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
V4=Path(__file__).resolve().parents[1];REPO=V4.parents[1]
OUT=V4/'results/qualification_prism_progression_convention'
PREV=V4/'results/initial_design_station_progression_request'
SURFACE=V4/'results/production_adaptive_capsule_surface_foundation/fresh'
def save(name,obj):(OUT/name).write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf8')
def main():
 start=time.perf_counter()
 for n,h in [('report.md','8c29feaf82c3b599a56e55041a89d9ad7801913314aa975fb7058f774f11d7e5'),('AUTHORITY.json','6a886f2c795a8d5f744ae7e77c78f8343e09a52e35333d474e0c763e8dd56d62')]:assert file_hash(PREV/n)==h
 prior=json.loads((PREV/'BOUND_PHYSICAL_INTENT.json').read_text())
 assert prior['profile_identity']=='82a721c13b59c37f6ce20ee7744a1a228a10f469aacac22d244669d61a68b5fb'
 assert bind_physical_intent(prior['scientific']['declaration'],surface_directory=SURFACE,evidence_root=REPO)==prior
 assert contract_identity()=='7de6b4527da1cd55659c1796927394cbd9af2f581cffbd85d2f891705abbf7dd'
 h=prior['values']['full_first_height_m'];t=prior['values']['physical_coverage_estimate_m']
 rows=family(h,t);replay=family(h,t);assert rows==replay
 assert [r['N'] for r in rows]==list(range(2,32))
 save('FAMILY.json',rows)
 historical=PREV/'PROGRESSION_PROVENANCE.json'
 convention=dict(schema='V4QualificationProgressionConvention/1',owner='User/advisor: AdoptV4QualificationPrismProgressionConvention',
  rule='Fewest integer layers in the increasing geometric family whose solved growth is <= the explicitly adopted historical engineering reference',
  growth_reference_hex=float(1.20).hex(),family_scope='N>=2,r>=1; qualification scope only',
  physical_profile_identity=prior['profile_identity'],production_default=False,hard_quality_gate=False,
  limitations='Requested schedule never overrides geometric feasibility. No actual y+ or valid prism geometry claim.',
  historical_evidence_sha256=file_hash(historical),implementation_sha256=file_hash(Path(__file__).with_name('qualification_prism_progression.py')),
  arithmetic='80-digit Decimal sum-form bisection; 256 iterations; nearest binary64 root and immediate neighbours; minimize exact sequential-total residual then root distance then growth')
 save('CONVENTION.json',convention);cid=identity(convention)
 selected=adopt(rows,float.fromhex(convention['growth_reference_hex']));save('SCHEDULE.json',selected)
 crossings={str(ref):first_crossing(rows,ref) for ref in [1.25,1.2,1.163359674,1.08]};save('CROSSINGS.json',crossings)
 def source(p):return dict(path=os.path.relpath(p,OUT).replace('\\','/'),sha256=file_hash(p))
 request=dict(schema='V4ExternalPrismRequest/1',kind='DeclaredProductionTestRequest',production_intent=True,
  request_owner=convention['owner'],physical_basis='Adopted 75mph air/+z/y+10 engineering flat-plate profile; global full h1; x_start=0.1L,x_end=L. Not actual capsule shear or y+ guarantee.',
  total_thickness_intent='Engineering surrogate coverage, separate from nominal binary64 schedule: '+t.hex()+' m',
  physical_total_thickness_intent_m_hex=t.hex(),geometry_identity=prior['scientific']['declaration']['geometry_identity'],
  surface_identity=prior['scientific']['declaration']['surface_identity'],fluid_side='positive_sdf',height_scope='global',scalar_quantity='normal_height_m',
  first_height_m_hex=h.hex(),requested_layer_count=selected['N'],growth_hex=selected['growth_hex'],increments_m_hex=selected['increments_hex'],
  schedule_total_m_hex=selected['nominal_total_hex'],physical_profile_identity=prior['profile_identity'],progression_convention_identity=cid,
  source_documents=[source(PREV/'BOUND_PHYSICAL_INTENT.json'),source(PREV/'AUTHORITY.json'),source(historical),source(OUT/'CONVENTION.json'),source(OUT/'SCHEDULE.json')])
 save('REQUEST.json',request)
 admission=inspect_first_layer_inputs(SURFACE,OUT/'REQUEST.json');assert admission['generation_authorized']
 _,_,wall=load_production_wall(SURFACE)
 decoded=verify_bound_request(OUT/'REQUEST.json',wall,identity(request))
 assert float(decoded['increments'][0]).hex()==h.hex()
 save('ADMISSION.json',admission)
 save('DETERMINISM.json',dict(exact=True,family_identity=identity(rows),schedule_identity=decoded['schedule_identity'],request_identity=identity(request),convention_identity=cid,scope='Independent full family calculation and source-authority reload; no mesh'))
 save('INPUT_VERIFICATION.json',dict(prior_report_sha256=file_hash(PREV/'report.md'),prior_authority_sha256=file_hash(PREV/'AUTHORITY.json'),physical_profile_identity=prior['profile_identity'],contract_identity=contract_identity(),wall=wall))
 mem=psutil.Process().memory_info();save('RUNTIME.json',dict(audit_seconds=time.perf_counter()-start,peak_rss_bytes=mem.peak_wset,retained_rss_bytes=mem.rss,whole_task_peak_rss='NotMeasured'))
 print(json.dumps(dict(selected=selected,crossings=crossings,admission=admission,convention_identity=cid),indent=2))
if __name__=='__main__':main()
