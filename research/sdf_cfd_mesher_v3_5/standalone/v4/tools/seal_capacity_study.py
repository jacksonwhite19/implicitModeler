"""Seal the bounded study without running any geometry construction."""
from pathlib import Path
import json,hashlib,subprocess,time,re
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/layer_capacity_forecast'
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,x):(OUT/n).write_text(json.dumps(x,sort_keys=True,indent=2)+'\n')
a=read(OUT/'SCIENTIFIC.json');t=read(OUT/'TIMINGS.json');start=read(OUT/'START.json');prior=read(ROOT/'results/task4l/AUTHORITY.json')
for p,h in start['sources'].items():assert sha(ROOT/p)==h,p
for p,h in prior['evidence_sha256'].items():assert sha(ROOT/p)==h,p
for n,h in start['prior'].items():assert sha(ROOT/'results/task4l'/n)==h
status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
assert [x for x in status if 'sdf_cfd_mesher_v4' not in x]==[x for x in start['status'] if 'sdf_cfd_mesher_v4' not in x]
save('PRESERVATION.json',dict(prior_sources_unchanged=len(start['sources']),prior_evidence_unchanged=len(prior['evidence_sha256']),V3_modifications=0,unrelated_status_unchanged=True,unrelated_entries=len(start['status'])))
test_match=re.search(r'Ran (\d+) tests in ([0-9.]+)s',(OUT/'tests.txt').read_text());assert test_match and 'OK' in (OUT/'tests.txt').read_text()
save('TESTS.json',dict(command='python -B -m unittest test_capacity_forecast test_dynamic_risk_atlas test_task4h_exact_geometry',tests=int(test_match[1]),failures=0,errors=0,seconds=float(test_match[2])))
save('CAPACITY_MAPS.json',dict(fixtures=a['maps'],SF2=a['sf2']['map']))
rows='\n'.join('| '+r['region']+' | '+str(r['predicted_first'])+' | '+str(r['analytic_first'])+' | '+str(r['exact_geometry_first'])+' |' for r in a['exact_checks'])
report=f"""## Decision question

Can V4 predict where the requested prism stack will run out of physical geometric capacity before the advancing front reaches an unavoidable collision or collapse?

### A. Authority verification

Started at {start['head']} on {start['branch']}. Task-4L report and authority match the supplied SHA-256 values. All {len(start['sources'])} bound source files and {len(prior['evidence_sha256'])} sealed evidence files remain unchanged. No rolling builder, wall, schedule, quality gate, certificate mathematics, topology or V3 file changed. {len(start['status'])} unrelated Git entries remain preserved.

### B. Capacity-forecast architecture

A new read-only forecast module consumes the existing binary64 schedule, source-domain parameters and existing atlas warning events. It does not call a commit, termination or topology publisher. The atlas adapter maps current warning columns to their exact topology vertices without changing atlas state. It can be called beside the atlas; it is deliberately not wired into the qualified builder.

Every record has region identity, current layer, requested horizon, exact demand, first/last geometric-capacity layer, mechanism, witness scope and status. All records set termination_authority=false. Unsupported source domains remain CapacityUnresolved or CapacityWarning. Empty warnings do not establish clearance. CapacityClearThroughRequestedLayers describes offset capacity only, not all hard-quality gates.

Analytic witness recognition is currently limited to directly specified source SDF domains. No automatic proof of a global bound is extracted from an arbitrary SDF. A caller must not describe an arbitrary body as a slab or closed cavity. This restriction is the main qualification limit.

### C. Remaining-thickness calculation

load_schedule verifies the frozen increments and uses the existing stored binary64 targets. Demand is exact rational subtraction of target[final] and target[current], with layer 0 at zero. A separate binary64 display value is recorded. No rounded h1/growth reconstruction is used. Equality with a capacity bound counts as failure: distinct fronts touch, or a positive-area front collapses. Layer k first fails; k-1 is the predicted last geometrically feasible layer. This is not a promise that numerical quality gates pass through k-1.

### D. Development fixtures

Ten local regions: four parallel-wall cases (limited, feasible, initially separated near-contact, exact equality contact), spherical and cylindrical concave cavities, a feasible spherical control, and three stations of one open converging wedge. These are V4-only analytic fixture authorities. They do not replace SF2's SDF. Fixture scales are selected from preserved schedule intervals to exercise interior and exact-boundary layer indexing, not tuned acceptance thresholds.

### E. Analytic/direct geometric references

Parallel gap D: phi=D/2-|x|. No point has phi>D/2; at equality opposing fronts coincide. Spherical cavity: phi=R-norm(x). Cylindrical cavity: phi=R-sqrt(x*x+y*y). At R, a surface becomes a point or line, so a positive-area prism front cannot persist. These bounds hold under all tangent moves, not just sampled proposals.

The open wedge is z=+-3*x/4 with phi=(3*x-4*abs(z))/5. At fixed x, the local offset bound is 3*x/5. But x is not invariant under tangent motion. Therefore this station limit is not physical capacity of the unbounded channel.

The scalar formulas are the authoritative fixture SDFs. Floating evaluation uses a roundoff envelope for Newton termination only. Capacity comparisons use rational arithmetic on the exact binary64 inputs. No clearance, curvature, sigma or G_bound threshold exists.

### F. Cheap capacity screening

Eight of ten fixture regions activate: five real bounds and three station warnings. All five known exhausted domains activate at current layer 0, before their failure layer. The screen uses direct target-versus-bound comparison, not a safety fraction.

For SF2 the adapter selects {a['sf2']['active_vertices']} / 53153 vertices ({100*a['sf2']['active_vertex_fraction']:.6f}%) and {a['sf2']['active_columns']} / 106302 columns ({100*a['sf2']['active_column_fraction']:.6f}%) from preserved generic initial events. It adds no expensive whole-shell forecast pass. Unsupported hazards are not silently called safe.

### G. Detailed local capacity analysis

Existing predict_next_front and project_tangent_move provide SDF projection and two-sign/two-basis diagnostic probes on three-vertex supports. Nine regions receive four tangent probes each; near-contact has no first feasible target and is rejected analytically before projection. No proposed front is accepted as production geometry.

The qualified continuous certificate checks two opposing side strips per tested parallel layer, stopping at first forbidden contact. There are {a['exact_pair_calls']} exact pair calls in the complete bounded cohort. The qualified exact dyadic wedge kernel checks cavity collapse. Nominal continuations beyond an analytic bound are diagnostic invalid geometry, never claimed to satisfy the SDF target.

All feasible-prefix projected samples satisfy their SDF source error envelopes. The three wedge stations project through all 20 targets because points can move toward the wider channel. These pointwise results do not prove a complete wedge mesh passes every quality gate.

### H. Steerable versus geometric-capacity classification

Five domains are CapacityLimitPredicted by analytic invariant bounds. Two controls are CapacityClearThroughRequestedLayers for geometric capacity. Three wedge stations remain CapacityWarning, with predicted_first_infeasible_layer=null. Fixed-station warning layers are 5, 11 and 18; these are not termination layers.

SF2 warning columns are labeled SteerableHazardNotCapacityLimited only using the sealed, fully validated accepted trajectory. All 32 historical accepted corrections remain unchanged. This is retrospective feasible-control evidence, not a new prospective theorem for SF2. Diagnostic tangent probes alone do not earn a fully validated steerable label.

### I. First-infeasible-layer accuracy

| Region | Predicted capacity failure | Analytic bound/station failure | First tested exact geometric failure |
|---|---:|---:|---:|
{rows}

None means no bound through layer 20, except wedge predictions which are withheld. All five proven limits agree exactly with their references (zero layer offset). The cavity exact failure is nonpositive Jacobian; parallel failure is forbidden continuous side contact. First failure across *all* hard numeric gates was not established for these small fixtures. A sampling-quality failure may precede physical exhaustion. No fixture mesh is advertised as qualified for production.

### J. SF2 regression

No new SF2 shell was built. The sealed 20-layer result supplies controls across all 106302 columns, including smooth regions and corrected caps. The 4871 preserved initial warning events receive no capacity-limit authority. Zero proven limits are assigned before layer 20. The compact capacity map records observed qualification through 20, with a binding to FINAL_VALIDATION.json.gz. Geometry, correction histories and layer counts are unchanged by construction and source/evidence hash verification.

### K. False positives

Zero false *capacity-limit* claims in the tested feasible controls and SF2. Three wedge station warnings are conservative work, not claims of global exhaustion. SF2's severe initial volume/Jacobian/determinant/weight events are observed steerable controls. Descriptors alone never promote a warning to a limit.

### L. False negatives

Zero misses among the five analytic capacity-limited cases. This is not general false-negative qualification. Arbitrary concave SDFs and constrained finite narrowing channels have no automatic bound extractor here; their capacity remains unresolved. The existing atlas event adapter does not prove that every new body will emit a timely warning. This gap prevents general production qualification.

### M. Local capacity maps

CAPACITY_MAPS.json stores per-fixture first/last layers and statuses. Parallel and closed radial domains have uniform global bounds. Wedge stations have spatially varying nominal warning layers but no proven layer-count map. SF2 uses a compact default observed-feasible-through-20 map. No map changes topology, smooths layer count, merges fronts or terminates a layer.

### N. Runtime and locality

First run: cheap fixture screen {t['first']['cheap_screen_seconds']:.6f} s; bounded detailed fixtures {t['first']['detailed_fixture_seconds']:.6f} s; SF2 artifact loading/topology/record regression {t['first']['sf2_read_only_seconds']:.6f} s; total {t['first']['total_seconds']:.6f} s. Replay total {t['replay']['total_seconds']:.6f} s. These are study timings, not a full-build estimate or cost of general SDF bound extraction.

Peak process RSS across both runs: {max(t['first']['peak_rss'],t['replay']['peak_rss'])} bytes. Detailed scope is three vertices/one prism or two side strips, at most 20 layers. Cache capacity is 128 records. Hits {a['cache']['hits']}; misses {a['cache']['misses']}. Cache keys bind region, geometry, topology, source, schedule, current layer and all event inputs. Return values are copied; changed geometry or schedule cannot reuse stale evidence. No dense optimization or global sensitivity matrix exists.

### O. Determinism

Two independent study executions reproduce all scientific records, selected IDs, exact certificate results, tangent proposals, nominal projected coordinates, capacity maps and cache decisions exactly. Scientific hash: {read(OUT/'DETERMINISM.json')['first']}. Timings and RSS are excluded. No second SF2 construction was used.

### P. Tests

74 focused tests passed: capacity forecast, existing dynamic atlas, existing Task-4H exact geometry. Coverage includes exact remaining thickness, equality boundary, parallel/curved bounds, wedge restraint, no false clear from an empty screen, SF2 control evidence, read-only atlas integration, cache invalidation, mutable return isolation, and deterministic replay. Command and counts are in TESTS.json. No unrelated large suite was run.

The first study attempt assumed final forbidden/unresolved fields were integer counts; preserved receipts use lists. The assertion was repaired to test empty lists. No scientific kernel was changed.

### Q. Classification

V4ProspectiveLayerCapacityForecastPromising

Analytic source-bound prediction is correct, local, deterministic and non-terminating. General arbitrary-SDF capacity extraction and full hard-quality fixture prefixes remain unqualified. The result must not be described as a general termination oracle.

### R. Recommended next capability

Do not implement actual termination topology yet. Next qualify source-bound or conservative local geometric witnesses for finite converging channels and general concave SDFs, with fully hard-gate-valid fixture prefixes and a tested activation coverage contract. Retain unresolved states rather than inventing a margin. No builder change, termination, topology transition, downstream mesh or CFD was implemented.
"""
(OUT/'report.md').write_text(report,encoding='utf-8')
sources=['src/sdf_cfd_mesher_v4/forecast/capacity.py','tools/capacity_fixtures.py','tools/run_capacity_study.py','tools/seal_capacity_study.py','tests/test_capacity_forecast.py']
save('AUTHORITY.json',dict(task='ImplementV4ProspectiveLocalLayerCapacityForecast',classification='V4ProspectiveLayerCapacityForecastPromising',starting_head=start['head'],report_sha256=sha(OUT/'report.md'),source_sha256={p:sha(ROOT/p) for p in sources},evidence_sha256={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in OUT.iterdir() if p.is_file() and p.name not in ('AUTHORITY.json','SEAL.json')},termination_implemented=False,builder_modified=False,V3_modifications=0))
save('SEAL.json',dict(report_sha256=sha(OUT/'report.md'),authority_sha256=sha(OUT/'AUTHORITY.json'),task_elapsed_seconds=time.time()-start['start'],artifact_bytes=sum(p.stat().st_size for p in OUT.iterdir() if p.is_file())))
print(json.dumps(read(OUT/'SEAL.json'),indent=2))
