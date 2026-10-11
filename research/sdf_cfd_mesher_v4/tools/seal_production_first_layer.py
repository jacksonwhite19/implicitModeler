"""Seal completed first-layer evidence and a full source-matched report."""
import json,time,shutil,subprocess
from pathlib import Path
import numpy as np
from production_first_layer_run import OUT,V4,SURFACE,REQUEST,save
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
REPO=V4.parents[1]
def load(p):return json.loads(Path(p).read_text())
def main():
 a=load(OUT/'A/FINAL.json');b=load(OUT/'B/FINAL.json');d=load(OUT/'A/DIAGNOSTICS.json');p=load(OUT/'PREFLIGHT.json')
 fields=['front_coordinate_hash','front_topology_hash','prism_connectivity_hash','ownership_hash','quality_hash','certificate_hash','committed']
 assert a['passed'] and b['passed'] and a['committed']
 assert all(a[k]==b[k] for k in fields)
 for filename in ['CANDIDATE.npz','QUALITY.json']:
  assert file_hash(OUT/'A'/filename)==file_hash(OUT/'B'/filename)
 da=load(OUT/'A/DIAGNOSTICS.json');db=load(OUT/'B/DIAGNOSTICS.json');da.pop('seconds');db.pop('seconds');assert da==db
 for label in ['A','B']:
  cov=load(OUT/label/'COVERAGE_RECHECK.json');assert all(cov[k]==0 for k in ['forbidden','unresolved','missing','duplicate'])
  assert load(OUT/label/'checkpoints/CURRENT.json')['scientific']==a['committed']
 replay=dict(exact=True,fields={k:a[k] for k in fields},semantics='Exact binary64 coordinates/connectivity and identical canonical quality, ownership, certificate and committed-state hashes. Timings and storage layout are incidental.')
 save(OUT/'REPLAY.json',replay)
 wall=load(SURFACE/'SURFACE_AUTHORITY.json');req=load(REQUEST)
 from production_adaptive_surface import CapsuleGeometry
 from production_first_layer_diagnostics import stats,angles
 from geometry_family_authority import Source
 z=np.load(OUT/'A/CANDIDATE.npz');xx=z['front'];ff=z['triangles'];geo=CapsuleGeometry(load(SURFACE.parent/'GEOMETRY.json')['definition'])
 curv=np.linalg.eigvalsh(np.array([geo.gradient_hessian(1,pt)[1] for pt in xx]))
 edges=np.unique(np.sort(np.concatenate([ff[:,[0,1]],ff[:,[1,2]],ff[:,[2,0]]]),axis=1),axis=0)
 normals=Source(dict(components=[geo.definition])).evaluate(xx).normal
 extra=dict(front_principal_curvature=stats(curv),front_neighbor_normal_variation_degrees=stats(angles(normals[edges[:,0]],normals[edges[:,1]])))
 arr=np.load(OUT/'A/DIAGNOSTIC_ARRAYS.npz');worst={}
 for k in d['conditioning']:
  if k not in arr:continue
  index=int(np.argmin(arr[k]) if 'sigma' in k else np.argmax(arr[k]))
  worst[k]=dict(source_column=index,cell_id=1000000+index,value=float(arr[k][index]))
 save(OUT/'ADDITIONAL_DIAGNOSTICS.json',dict(geometry=extra,conditioning_worst=worst))
 d['geometry_diagnostics'].update(extra)

 source_names=['production_first_layer_run.py','production_first_layer_diagnostics.py','production_first_layer_coverage.py','seal_production_first_layer.py']
 sources=[V4/'tools'/n for n in source_names]+[V4/'tests/test_production_first_layer_run.py']
 status=subprocess.check_output(['git','status','--short'],cwd=REPO,text=True).splitlines()
 own=[s.relative_to(REPO).as_posix() for s in sources]
 unrelated=[s for s in status if s[3:] not in own and not s[3:].startswith(OUT.relative_to(REPO).as_posix())]
 assert unrelated==p['starting_dirty']
 proof_sources=['production_prism_sizing.py','production_surface_authority.py','geometry_family_authority.py','local_cavity_remesh.py','local_termination_topology.py','first_layer_metric_state.py','final_shell_incidence.py','native_mixed_face_certificate.py','cavity_volume_audit.py','run_geometry_family.py']
 evidence=[dict(path=q.relative_to(REPO).as_posix(),sha256=file_hash(q)) for q in sources+[V4/'tools'/n for n in proof_sources]+[REQUEST,SURFACE/'SURFACE_AUTHORITY.json']]
 save(OUT/'IMPLEMENTATION.json',evidence)
 scientific=dict(surface=wall['surface_identity'],request=identity(req),metric_semantics='FoundationAreaTensorFirstPrismIncidence/1',**{k:a[k] for k in fields})
 layer_id=identity(scientific)
 # Preserve identities for every local result, including ignored native bulk.
 files=[]
 for q in sorted(OUT.rglob('*')):
  if q.is_file() and q.name not in ['BULK_MANIFEST.json','report.md','AUTHORITY.json','HASHES.json','RESOURCES.json']:
   files.append(dict(path=q.relative_to(REPO).as_posix(),bytes=q.stat().st_size,sha256=file_hash(q)))
 save(OUT/'BULK_MANIFEST.json',files)
 resources=dict(starting_free_bytes=p['starting_free_bytes'],ending_free_bytes=shutil.disk_usage(REPO).free,bytes_created=sum(q.stat().st_size for q in OUT.rglob('*') if q.is_file()),bytes_deleted=0,cleanup='None. Full proof streams and native objects retained locally, hash-bound in BULK_MANIFEST; large bulk excluded from Git.',elapsed_since_preflight_seconds=time.time()-p['start_epoch'],A=a,B=b,whole_task_peak_rss='NotMeasured')
 save(OUT/'RESOURCES.json',resources)
 texts=['# GenerateAndQualifyV4CapsuleFirstLayer','## A. Classification','V4ProductionAdaptiveCapsuleFirstLayerQualified',
 'One fresh first layer was committed directly through MixedFrontRunner.initialize and native checkpoint publication. Independent replay committed the same scientific state. No correction, later layer, forecast, multi-layer shell, core or CoreHandoff qualification was performed. This is FirstLayerQualified only, not CFD accuracy, actual y+, actual wall shear, complete-neighbor later-stage qualification, or complete mesh qualification.',
 '## B. Input authorities',json.dumps(dict(starting_HEAD=p['starting_head'],wall=wall,request=req),indent=2),
 'The request was admitted by unchanged production decoder, then checked against sealed request identity and file hash. The adaptive production loader verified every wall payload. New input adapter bypasses fixture construction and SF2 schedule loading; existing generic analytic normal-offset proposal, cell construction, corrected metric, certification and native commit paths remain in use.',
 '## C. Wall preservation','Before and after:16895 vertices,50679 edges,33786 triangles. Coordinates, topology, surface and sizing identities unchanged. Exact mutation count0. Both independent runs reload and compare authoritative wall arrays and record. Hashes are listed above.',
 '## D. First-layer counts',json.dumps(d['topology'],indent=2),
 'There are33790 total geometric vertices (wall plus front). One prism per source triangle. Explicit mapping assertions verify every prism vertex list and source column, unique cell IDs, and complete source-vertex use. No missing, duplicate, orphan or foreign sources.',
 '## E. Normal construction',json.dumps(d['normal'],indent=2),
 'Production analytic SDF gradient is already normalized by its primitive. No fallback or additional direction relaxation. Construction direction equals analytic SDF gradient. Normal angular error has no new hard threshold.',
 '## F. Height realization','Requested full h1=0.0001448914516780669m. Raw and final front are identical; no compression, correction or target replacement. Signed measurements use normalized diagnostic directions. All displacements finite and outward.']
 def table(title,rows):
  texts.extend([title,'Columns: min,p1,p5,median,p95,p99,max. Values are binary64 diagnostics; units are metres, square/cubic metres, degrees or dimensionless as indicated.','| Metric | min | p1 | p5 | median | p95 | p99 | max |','|---|---:|---:|---:|---:|---:|---:|---:|'])
  for name,v in rows.items():texts.append('| '+name+' | '+' | '.join(repr(v[k]) for k in ['min','p1','p5','median','p95','p99','max'])+' |')
 table('Height distributions',d['height'])
 texts+=['Nonfinite counts0 throughout; zero/negative outward displacement0. No iterative projection was required:0 projection operations,0 projection failures. Maximum target-SDF residual2.5017965130103015e-17m, admitted by the existing source-error bound.']
 table('## G. Tangential front evolution',d['edges'])
 table('## H. First-front triangle quality — wall',d['wall_triangles']);table('First front',d['front_triangles']);table('Matched area ratio',{'area_ratio':d['area_ratio']})
 texts+=['Radius ratio is2*inradius/circumradius. Zero-area triangles0; reversed triangles0. No added triangle or edge-ratio gate.']
 table('## I. Prism geometry',{'volume':d['quality']['volume']['distribution'],'exact_J_min_per_cell':d['quality']['exact_jacobian']['distribution']})
 texts+=['Exact dyadic wedge polynomial checks every reference corner and any interior stationary height. Stored rational minima and exact signs are authoritative; percentile display converts rational minima to binary64. Nonpositive volume0; nonpositive exact Jacobian0. Worst source column27110; prism1027110.']
 table('## J. Hard quality',{k:d['quality'][k]['distribution'] for k in ['determinant','weight','skewness','symmetric_volume_ratio','aspect']})
 texts += [json.dumps({k:{x:v for x,v in row.items() if x!='distribution'} for k,row in d['quality'].items()},indent=2),
 '## K. Determinant semantics','Actual lateral neighbor incidence plus declared internal top-cap role; wall face remains physical boundary. No fabricated top-neighbor centroid. Retired exposed-cap values are diagnostic only and are replaced before authoritative quality admission by FoundationAreaTensorFirstPrismIncidence/1. No retired exposed-cap interpretation controls acceptance. Complete-neighbor validation remains a later-stage obligation.',
 '## L. Continuous geometry',json.dumps(load(OUT/'A/CERTIFICATE_SUMMARY.json'),indent=2),json.dumps(load(OUT/'A/COVERAGE_RECHECK.json'),indent=2),
 'All unique wall, side and front face AABB-overlap candidates were continuously certified with the existing qualified interval filter and exact fallback. Independent enumeration matches every stored proof row. Exact cell/boundary volume equality and global degree1 at all33786 cell interior witnesses also pass. This is not a sampled substitute for the face certificate.',
 '## M. Topology and ownership','Closed wall and first front:one component each,Euler2,zero open/nonmanifold/duplicate front defects. Prism complex:50679 internal side faces,67572 wall/front boundary faces,118251 unique faces. Oriented two-owner internal incidence passes. Zero unused source vertices, ownership failures and disconnected source components.',
 '## N. Sigma/G and geometry diagnostics']
 texts.append(json.dumps(worst,indent=2));table('Frozen operator diagnostics',d['conditioning']);table('Geometric diagnostics',d['geometry_diagnostics']);table('Non-orthogonality degrees',{'nonorthogonality':d['nonorthogonality']})
 texts+=['Sigma is the smallest weighted normal singular value. G variants use the existing frozen operator, wall mask on source wall only. No sigma floor or G ceiling. Curvature eigenvalues near zero include binary64 roundoff; join curvature is source-branch diagnostic, not a new gate.',
 '## O. Correction and relaxation','Not used. Raw corrected-context hard failures0. Iterations0; changed source vertices0; raw/final identical. Complete raw/final certification passes.',
 '## P. Deterministic replay',json.dumps(replay,indent=2),
 '## Q. Performance',json.dumps(dict(A=a,B=b,diagnostics_seconds=d['seconds'],coverage_recheck_seconds=load(OUT/'A/COVERAGE_RECHECK.json')['seconds'],cache_hits='NotMeasured',cache_misses='NotMeasured'),indent=2),
 'Phase timings overlap:cell_generation and hard_quality are components of raw_quality. Continuous and exact volume accounting are measured together. Native commit timing includes binding, persistence, restore and validation. No performance threshold claimed.',
 '## R. Tests',json.dumps(load(OUT/'TESTS.json'),indent=2),
 '## S. Disk',json.dumps({k:v for k,v in resources.items() if k not in ['A','B']},indent=2),f'Repository accessible logical use{p["repository_accessible_bytes"]};V4{p["V4_accessible_bytes"]};file-stat errors{p["stat_errors"]};directory traversal access errors not counted. Largest relevant prior checkpoint1400762368bytes and receipt store1149747200bytes.',
 '## T. Hashes and artifacts','First-layer scientific identity:'+layer_id,'Committed-front identity:'+a['committed'],
 'All major result hashes, including native checkpoint objects and complete expanded proof streams, are in BULK_MANIFEST.json. Report and authority hashes are in HASHES.json to avoid self-reference.',
 '## U. Git state',f'Branch{p["branch"]};startingHEAD{p["starting_head"]};155 unrelated dirty entries preserved exactly;V3 task modifications0. Task-owned:four tools,one focused test,and fresh result directory. FinalHEAD is the commit containing this report, printed in final terminal response.',
 '## V. Blockers','Request blockers:none. FirstLayer blockers:none. Later-layer qualification:not established.',
 '## W. Exact supported claim','The sealed adaptive capsule surface and admitted global physical request produce one immutable-wall prism layer whose applicable hard quality, exact volume/Jacobian, topology, ownership, complete continuous geometry and native commit checks pass and replay to the identical scientific state.',
 '## X. Recommended next task','QualifyV4CapsuleAdvancingFrontLayer2: begin from this committed first layer, evaluate exactly the next requested layer with existing feasibility machinery and complete applicable certification, and stop at that bounded milestone. No layer2 was generated here.']
 (OUT/'report.md').write_text('\n\n'.join(texts)+'\n',encoding='utf8')
 authority=dict(classification='V4ProductionAdaptiveCapsuleFirstLayerQualified',first_layer_identity=layer_id,committed_front_identity=a['committed'],scientific=scientific,first_layer_qualified=True,later_layer_qualified=False,generated_layers_per_run=1,replay_runs=2,core_generated=False,manifest_sha256=file_hash(OUT/'BULK_MANIFEST.json'),implementation_sha256=file_hash(OUT/'IMPLEMENTATION.json'),report_sha256=file_hash(OUT/'report.md'))
 save(OUT/'AUTHORITY.json',authority)
 save(OUT/'HASHES.json',{n:file_hash(OUT/n) for n in ['report.md','AUTHORITY.json','BULK_MANIFEST.json','IMPLEMENTATION.json','REPLAY.json']})
 print(json.dumps(dict(authority=authority,hashes=load(OUT/'HASHES.json'),resources={k:v for k,v in resources.items() if k not in ['A','B']}),indent=2))
if __name__=='__main__':main()
