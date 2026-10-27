"""Replay sealed candidate classification without any geometry publication."""
import json
import sys
import time
from collections import Counter
import numpy as np
import psutil
from capsule_layer2_preflight import V4,L1,SURFACE,REQUEST,save,file_hash,identity,array_hash
from run_production_forecast_admission import restore_adapter
from production_surface_authority import load_production_wall
from production_l2_screening import face_inventory,close_risk
from production_collision_localization import classify,validate_pairs,topology_relations,risk_columns,METHODS

OUT=V4/'results/production_layer2_collision_localization'
PRIOR=V4/'results/production_layer2_risk_coverage'


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes((json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode())


def run(label):
    start=time.perf_counter()
    for name,digest in [('AUTHORITY.json','cf91b01f2a5a7ab019a9288bcb79dd2d7707ac91b84b5f4fc84e546d2c5bfbd7'),('report.md','92544bc5ae9a2891497a02b9eeab129777902ae21717b38d7fb67a100008c228')]:
        if file_hash(PRIOR/name)!=digest:raise ValueError('PreviousAuthorityMismatch')
    authority=json.loads((PRIOR/'AUTHORITY.json').read_text())
    for name in ('SCREEN_ARRAYS.npz','RUN_A.json'):
        if file_hash(PRIOR/name)!=authority['evidence'][name]:raise ValueError('SealedScreeningMismatch')
    prior=json.loads((PRIOR/'RUN_A.json').read_text())['scientific']
    adapter,times,before=restore_adapter();t=time.perf_counter()
    with np.load(PRIOR/'SCREEN_ARRAYS.npz') as z:
        high=z['provisional_high'];pairs=z['candidate_pairs'];labels=z['face_labels'];columns=z['face_columns'];kinds=z['face_kinds']
    validate_pairs(pairs,len(labels),'cbca550380d53a497dfd5fcd092f35f422e5338927a5e0cff12c51bdfd7fef82')
    if adapter.binding!=prior['binding']:raise ValueError('BindingMismatch')
    inventory=face_inventory(adapter.topology,*adapter.committed_prefix,high)
    for name,a in [('labels',labels),('columns',columns),('kinds',kinds)]:
        if not np.array_equal(a,inventory[name]):raise ValueError('InventoryMismatch:'+name)
    invhash={k:array_hash(v) for k,v in inventory.items()}
    if identity(dict(inventory=invhash,candidates=prior['collision_hashes']))!='3340cd971d792924c8c8a5a8f5ce3b71306bdb35151b56eae81ec52b3cb2d7e1':raise ValueError('CollisionIdentityMismatch')
    if prior['hard_events'] or prior['direct_seed_columns']:raise ValueError('DirectEventAuthorityChanged')
    points=np.concatenate((*adapter.committed_prefix,high))
    if not np.array_equal(points[labels],inventory['coordinates']):raise ValueError('FaceGeometryMismatch')
    times['sealed_candidate_load_and_verification']=time.perf_counter()-t
    print('sealed inventory verified',len(pairs),flush=True)
    t=time.perf_counter();relations=topology_relations(labels,pairs);times['topology_relations']=time.perf_counter()-t
    proofs=classify(points,labels,pairs,progress=lambda x:print(json.dumps(x),flush=True));times.update(proofs['timings'])
    codes=proofs['codes'];risk,seeds=risk_columns(codes,pairs,columns)
    closure=close_risk(adapter.topology,seeds);times['closure']=closure.pop('seconds')
    riskrows=[]
    for k in risk:
        i,j=map(int,pairs[k]);owners=np.unique(columns[[i,j]]);owners=owners[owners>=0]
        riskrows.append(dict(index=int(k),faces=[i,j],family='/'.join(sorted(kinds[[i,j]])),columns=owners.tolist(),
            source_wall_vertices=np.unique(labels[[i,j]]%len(high)).tolist(),relation=int(relations[k]),reason=METHODS[int(codes[k])]))
    family=Counter()
    for startbatch in range(0,len(pairs),50000):
        for (i,j),code in zip(pairs[startbatch:startbatch+50000],codes[startbatch:startbatch+50000]):
            family[('/'.join(sorted((str(kinds[i]),str(kinds[j])))),METHODS[int(code)])]+=1
    arrays=dict(codes=codes,witness=proofs['witness'],rotations=proofs['rotations'],relations=relations)
    hashes={k:array_hash(v) for k,v in arrays.items()}
    accounting=dict(candidates=len(pairs),certified=int(((codes>0)&(codes<10)).sum()),unresolved=int((codes==0).sum()),
        forbidden=int((codes==10).sum()),failure=int((codes==11).sum()),missing=0,duplicate=0)
    if sum(accounting[k] for k in ('certified','unresolved','forbidden','failure'))!=len(pairs):raise ValueError('IncompleteAccounting')
    decision=dict(status='CollisionLocalizationQualified' if not closure['blocked_groups'] and not accounting['failure'] else 'CollisionLocalizationBlocked',
        bounded_supports_complete=not bool(closure['blocked_groups']),final_continuous_certification=False,L2_admission=False,publication=False)
    wall,tri,surface=load_production_wall(SURFACE)
    if surface!=before['surface'] or file_hash(REQUEST)!=before['request'] or file_hash(L1/'A/checkpoints/CURRENT.json')!=before['pointer']:raise ValueError('InputMutation')
    scientific=dict(binding=adapter.binding,restored_prefix=array_hash(adapter.committed_prefix),inventory=invhash,candidate_identity=array_hash(pairs),
        collision_authority=prior['collision_identity'],direct_metric_identity=authority['direct_metric_identity'],direct_hard_events=0,
        topology_relations={str(int(v)):int(n) for v,n in zip(*np.unique(relations,return_counts=True))},
        proof_methods={METHODS[int(v)]:int(n) for v,n in zip(*np.unique(codes,return_counts=True))},
        proof_family_counts={f:{m:n for (ff,m),n in sorted(family.items()) if ff==f} for f in sorted({f for f,m in family})},
        array_hashes=hashes,exact_proofs_identity=identity(proofs['exact_rows']),filter_source=proofs['filter_source'],accounting=accounting,
        risk_rows=riskrows,risk_identity=identity(riskrows),seed_columns=seeds.tolist(),seed_identity=array_hash(seeds),closure=closure,
        decision=decision,decision_identity=identity(decision),preservation=dict(wall=True,L1=True,request=True),
        publication=dict(L2_generated=False,L2_committed=False,L3_generated=False,core_generated=False,committed_layer=1))
    result=dict(scientific=scientific,scientific_identity=identity(scientific),timings=times,total_seconds=time.perf_counter()-start,
        peak_rss=psutil.Process().memory_info().peak_wset,retained_rss=psutil.Process().memory_info().rss)
    save(OUT/(label+'.json'),result)
    if label=='RUN_A':
        np.savez_compressed(OUT/'PROOF_ROWS.npz',**arrays)
        save(OUT/'EXACT_PROOFS.json',proofs['exact_rows'])
    elif json.loads((OUT/'RUN_A.json').read_text())['scientific']!=scientific:raise ValueError('ReplayMismatch')
    print(json.dumps({k:v for k,v in result.items() if k!='scientific'}),flush=True)


if __name__=='__main__':run(sys.argv[1])
