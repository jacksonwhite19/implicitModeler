"""Reduce completed transition evidence; no stage execution or scientific edits."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from fractions import Fraction

V4=Path(__file__).resolve().parents[1]
ROOT=V4.parents[1]
OUT=V4/'results/current_front_transition_round_trip'
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from path_preserving_deduplication import durable_json,file_hash
from check_current_front_encoding import allocation
from record_receipt_resume import disk


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))


def quality(path):
    # Reuse the existing standard-library reducer contract. The optional
    # streaming package is not installed; no tool installation is needed.
    content=read(path)
    def rows(prefix):return iter(content[prefix])
    def extrema(prefix,key,limit,lower=True):
        count=bad=0;small=large=None
        for r in rows(prefix):
            v=r[key];count+=1
            small=v if small is None else min(small,v)
            large=v if large is None else max(large,v)
            bad+=int(not math.isfinite(v) or (v<=0 if key=='volume' else (v<limit if lower else v>limit)))
        return dict(count=count,minimum=small,maximum=large,limit=limit,violations=bad)
    gates={k:extrema('cells',k,v,mode) for k,v,mode in
           [('volume',0,True),('determinant',.0011,True),('aspect',1000,False)]}
    gates.update({k:extrema('faces',k,v,mode) for k,v,mode in
           [('weight',.055,True),('skewness',3.63636,False),('symmetric_volume_ratio',.01,True)]})
    positive=count=0;minimum=None
    for r in rows('exact'):
        count+=1;positive+=int(r['jacobian_sign']==1 and r['volume_sign']==1)
        v=Fraction(r['jacobian']);minimum=v if minimum is None else min(minimum,v)
    gates['exact']=dict(count=count,positive_jacobian_and_volume=positive,minimum_jacobian=str(minimum))
    if not count or positive!=count or minimum<=0 or any(v['violations'] for k,v in gates.items() if k!='exact'):
        raise ValueError('ChangedQualifiedGateOutcome')
    return gates


def main():
    tick=time.perf_counter();base=read(OUT/'BASELINE.json');run=OUT/'A';folder=run/'L5'
    result=read(run/'RESULT.json');replay=read(folder/'REPLAY.json');reader=read(OUT/'READER.json')
    if not result['passed'] or not replay['passed'] or not reader['passed']:
        raise ValueError('IncompleteRoundTrip')
    control=read(OUT/'CONTROL.json')
    if replay['content']!=control['reference_result']['scientific']:
        raise ValueError('RetainedScientificIdentityMismatch')
    encodings=[]
    old=next(r for r in base['dependencies'] if r['layer']==5)
    for root in (folder,folder/'replay'):
        p=root/'NEW_FORECAST_DEPENDENCIES.json';body=read(p);shared=Path(body['shared']['path'])
        if body['expanded_sha256']!=old['semantic'] or body['expanded_bytes']!=old['metadata']['bytes']:
            raise ValueError('ExpandedLegacyDependencyMismatch')
        measured=allocation([p,shared])
        encodings.append(dict(path=str(p),sha256=file_hash(p),rows=len(body['rows']),
            shared_schedules=len(read(shared)['schedules']),expanded_sha256=body['expanded_sha256'],
            expanded_bytes=body['expanded_bytes'],physical_bytes=measured['logical_bytes'],
            allocated_bytes=measured['unique_allocation_bytes'],shared_sha256=file_hash(shared),
            avoided_logical_bytes=old['metadata']['bytes']-measured['logical_bytes'],
            avoided_allocation_bytes=old['metadata']['cluster_bytes']-measured['unique_allocation_bytes']))
    fresh=read(folder/'FRESH_CERTIFICATE.json');volume=read(folder/'VOLUME_DELTA.json')
    summary=dict(classification='V4CurrentFrontDependencyEncodingCompleteTransitionReplayQualified',
        passed=True,layer=5,no_layer6=True,scientific=replay['content'],replay=replay,
        reopened=read(folder/'REOPEN.json'),reader=reader,counts=read(folder/'GENERATION.json')['counts'],
        topology=read(folder/'GENERATION.json')['topology_audit'],hard_failures=read(folder/'GENERATION.json')['hard_failures'],
        gates=quality(folder/'NEW_QUALITY.json'),assembly=read(folder/'ASSEMBLY.json'),
        continuous={k:fresh[k] for k in ('passed','required_pairs','prior_population','combined_pairs','failures','methods','prior_payloads_read','prior_payloads_recomputed')},
        volume_degree=dict(exact_balance=volume['exact_balance'],degree_passed=volume['global_degree']['passed'],
            unresolved=volume['global_degree']['unresolved']),
        phases=result['phases'],worker_seconds=result['seconds'],encodings=encodings,
        retention=read(folder/'replay_retention/RESULT.json'),monitor={k:read(OUT/'MONITOR_FINAL.json')[k] for k in ('exit_code','peaks','seconds')})
    durable_json(OUT/'SUMMARY.json',summary)
    for r in base['protected']:
        if file_hash(ROOT/r['path'])!=r['sha256']:raise ValueError('ProtectedFileChanged',r['path'])
    for r in base['dependencies']:
        if file_hash(r['path'])!=r['record']['sha256']:raise ValueError('RetainedDependencyChanged',r['path'])
    for node in base['active_ancestry']:
        ref=node['reference']
        if ref['kind']=='recursive':
            d=Path(ref['directory']);pointer=read(d/'CURRENT.json')
            if pointer['root']!=ref['root'] or file_hash(d/pointer['file'])!=node['root_file_sha256']:
                raise ValueError('ChangedRetainedAncestry')
    sources=['tools/run_authorized_continuation.py','tools/finish_retained_l5.py',
             'tools/run_dependency_transition_round_trip.py','tools/collect_dependency_transition_round_trip.py',
             'tests/test_dependency_transition_round_trip.py','CURRENT_TASK.md','README.md',
             'CURRENT_FRONT_TRANSITION_ROUND_TRIP_REPORT.md']
    prefix='research/sdf_cfd_mesher_v4/'
    allowed={prefix+s for s in sources}
    def unrelated(lines):
        return [s for s in lines if s[3:] not in allowed and not s[3:].startswith(prefix+'results/current_front_transition_round_trip')]
    actual=subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()
    original=read(OUT/'STARTING_STATUS.json')
    expected=original['status'].splitlines()
    if unrelated(actual)!=unrelated(expected):raise ValueError('UnrelatedCaseSensitiveStatusChanged')
    preservation=dict(exact_case_sensitive_status=True,unrelated_entries=len(unrelated(expected)),
        protected_files=len(base['protected']),retained_dependency_files=len(base['dependencies']),
        active_ancestry_roots=len(base['active_ancestry'])-1,
        no_v3_kernel_or_sealed_state_edits=True,coverage='Selected hashes and exact status; not a whole-repository inventory')
    durable_json(OUT/'PRESERVATION.json',preservation)
    files=[p for p in OUT.rglob('*') if p.is_file() and p.name!='EVIDENCE.json']
    measured=allocation(files)
    combined=allocation([p for p in (V4/'results/wing_receipt_resume/A').rglob('*') if p.is_file()]+files)
    essential=['CONTROL.json','PRE_EXECUTION_REVIEW.json','REVIEW.json','SUMMARY.json','READER.json','PRESERVATION.json',
               'A/L5/REPLAY.json','A/L5/REOPEN.json','A/L5/CANDIDATE_PLAN.json']
    evidence=dict(starting_head=base['starting_head'],head_at_measurement=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        initial_disk=base['disk'],final_disk=disk(),retained={k:v for k,v in measured.items() if k!='records'},
        combined_campaign={k:v for k,v in combined.items() if k!='records'},
        preservation=preservation,essential=[dict(path=p,sha256=file_hash(OUT/p),bytes=(OUT/p).stat().st_size) for p in essential],
        sources=[dict(path=s,sha256=file_hash(V4/s)) for s in sources if (V4/s).exists()],
        elapsed_seconds=(time.time()*1000-original['task_start_ms'])/1000,reducer_seconds=time.perf_counter()-tick,
        aliases_not_added=True,cleanup='Only existing successful-only replay retention; immutable originals and active ancestry remain')
    durable_json(OUT/'EVIDENCE.json',evidence)
    print(json.dumps(dict(scientific=summary['scientific'],encodings=encodings,gates=summary['gates'],
        retained=evidence['retained'],preservation=preservation,final_disk=evidence['final_disk']),indent=2))


if __name__=='__main__':main()
