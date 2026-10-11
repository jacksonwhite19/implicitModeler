"""Read-only complete evidence audits for a bound production layer."""
import argparse,collections,json,time
from pathlib import Path
import numpy as np
from production_bounded_layers import *
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items

def forecast_replay(layer):
    folder=configure(layer)
    if (folder/'A/FINAL.json').exists():raise ValueError('NoPostCommitForecastReplay')
    state,adapter,spec,restore_seconds=cycle.restore_base()
    forecast=cycle.BoundProductionForecastCache(adapter).forecast(adapter)
    expected=read(folder/'FORECAST.json')
    assert forecast.scientific_hash()==expected['identity'] and forecast.layers.tolist()==expected['layers']
    save(folder/'FORECAST_TIMED_REPLAY.json',dict(passed=True,identity=forecast.scientific_hash(),binding=adapter.binding,
        layers=forecast.layers.tolist(),timings=forecast.timings,restore_seconds=restore_seconds,
        note='Separate timed replay of the same current admission forecast, before publication. Original driver did not persist exclusive forecast timing.',**cycle.resources()))

def receipts(layer):
    folder=configure(layer);start=time.perf_counter();record=read(folder/'A/FINAL.json')
    store=NativeCheckpointStore(folder/'A/checkpoints');row=record['certificate_descriptor']
    # Fresh durable restore authenticates these descriptors separately. This
    # audit streams the same nested delta map; it never flattens that map.
    table=store._receipt(row,verify=False)
    if row['encoding']!='receipt_delta':raise ValueError('UnexpectedFlatPublicationRequiresAudit')
    left=iter(raw_items(table.base));right=iter(raw_items(table.updates))
    a=next(left,None);b=next(right,None);counts=collections.Counter()
    while a is not None or b is not None:
        if b is None or (a is not None and a[0]<b[0]):counts['left_only']+=1;a=next(left,None)
        elif a is None or b[0]<a[0]:counts['right_only']+=1;b=next(right,None)
        else:
            if a[1]==b[1]:counts['equal']+=1
            else:
                counts['changed']+=1;av=json.loads(a[1]);bv=json.loads(b[1])
                counts['changed_inputs_only' if {k:v for k,v in av.items() if k!='inputs'}=={k:v for k,v in bv.items() if k!='inputs'} else 'changed_other_fields']+=1
            a=next(left,None);b=next(right,None)
    cert=read(folder/'A/CERTIFICATE.json')
    assert not table.removed and not counts['changed_other_fields']
    assert counts['right_only']==cert['new'] and counts['left_only']+counts['equal']+counts['changed']==cert['reused']==len(table.base)
    assert len(table)==cert['required_pairs']==len(table.base)+counts['right_only']
    def physical(table):
        from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import ReceiptDelta
        if isinstance(table,ReceiptDelta):return {**physical(table.base),**physical(table.updates)}
        r=metadata(table.path);return {physical_key(r):r}
    save(folder/'RECEIPT_AUDIT.json',dict(counts=dict(counts),base_receipts=len(table.base),delta_receipts=len(table.updates),total_receipts=len(table),
        dependency_binding_updates=counts['changed_inputs_only'],new_interactions=counts['right_only'],unchanged_base=counts['left_only']+counts['equal'],
        base_unique_allocated=sum(r['cluster_bytes'] for r in physical(table.base).values()),delta_allocated=sum(r['cluster_bytes'] for r in physical(table.updates).values()),
        base_logical_bytes=sum(r['bytes'] for r in physical(table.base).values()),delta_bytes=sum(r['bytes'] for r in physical(table.updates).values()),
        proof_payloads_unchanged=True,removed=0,flattening=False,seconds=time.perf_counter()-start,**cycle.resources()))

def gates(layer):
    folder=configure(layer);volume=read(folder/'A/VOLUME.json');q=read(folder/'A/QUALITY.json');inc=read(folder/'A/INCIDENCE.json')
    result=dict(volume={k:volume[k] for k in ('boundary_volume','cell_volume','conditional_nonoverlap','exact_volume_balance','positive_cells','prerequisites','proof')},
        interior_degree=dict(passed=volume['interior_degree']['passed'],unresolved=len(volume['interior_degree']['unresolved']),counts=dict(collections.Counter(str(x['degree']) for x in volume['interior_degree']['values']))),
        exact_cells=len(volume['exact_cells']),quality_passed=q['passed'],quality_failures=q['failures'],incidence_passed=inc['passed'],incidence_failures=inc['failures'])
    assert result['quality_passed'] and result['incidence_passed'] and result['volume']['positive_cells'] and result['volume']['exact_volume_balance'] and result['interior_degree']['passed']
    save(folder/'FINAL_GATE_SUMMARY.json',result)

def fronts(layer):
    folder=configure(layer);wall,tri,_=cycle.load_production_wall(cycle.SURFACE)
    roots=[V4/'results/production_adaptive_capsule_first_layer',V4/'results/production_adaptive_capsule_layer2',L3,OUT/'L4',OUT/'L5']
    values=[wall]
    for k in range(layer):
        with np.load(roots[k]/'A/CANDIDATE.npz') as z:values.append(z['front'].copy())
    assert [array_hash(x) for x in values]==read(folder/'A/RESTORE.json')['coordinates']
    return values,tri

def regenerate(layer,record='REGENERATION.json',export=False):
    import hashlib
    import generalize_local_remesh as g
    from final_shell_incidence import check
    folder=configure(layer);start=time.perf_counter();values,tri=fronts(layer)
    p,c,o=g.s.construct(values[0],tri,values[1:],np.full(len(values[0]),layer))
    result={}
    for name,value in [('INCIDENCE.json',check(p,c,o)),('VOLUME.json',g.volume.audit(p,c,o,[tuple(map(int,f)) for f in tri]))]:
        payload=(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode();digest=hashlib.sha256(payload).hexdigest()
        path=folder/'A'/name
        expected=(file_hash(path) if path.exists() else next(r['sha256'] for r in read(folder/'RETENTION_INVENTORY.json')['rows'] if Path(r['path'])==path))
        if digest!=expected:raise ValueError('CompleteRegenerationMismatch:'+name)
        if export:
            (folder/'regenerated').mkdir(exist_ok=True);(folder/'regenerated'/name).write_bytes(payload)
        result[name]=dict(sha256=digest,bytes=len(payload),exact=True)
    save(folder/record,dict(passed=True,exports=result,front_hashes=[array_hash(x) for x in values],topology=array_hash(tri),
        implementation={str(Path(m.__file__).relative_to(V4)):file_hash(m.__file__) for m in [g,g.volume]},
        predictor_called=False,seconds=time.perf_counter()-start,**cycle.resources()))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['receipts','gates','regenerate','forecast_replay']);parser.add_argument('--layer',type=int,required=True);parser.add_argument('--record',default='REGENERATION.json');parser.add_argument('--export',action='store_true');args=parser.parse_args()
    if args.mode=='regenerate':regenerate(args.layer,args.record,args.export)
    else:globals()[args.mode](args.layer)
