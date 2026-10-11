"""Same qualified admission predicates, supplied a common committed base.
Not invoked by the architecture qualification.
"""
from production_single_layer_cycle import *
from accepted_production_contract import plan_transition, require_transition_authorization, production_owner
from path_preserving_deduplication import durable_json

def admit(base,adapter,spec,output,*,forecast_scope='remaining_schedule'):
    if forecast_scope not in ('remaining_schedule','next_transition'):raise ValueError('UnsupportedProductionForecastScope')
    require_transition_authorization(adapter)
    plan=plan_transition(base,adapter)
    OUT=output; owner=production_owner(adapter)
    INCREMENT_HEX=plan.get("increment_hex")
    save=lambda name,value: durable_json(OUT/name,value)
    started=time.perf_counter();restore_seconds=0.0
    next_layer=adapter.binding['next_layer']
    if owner is not None:
        premises=owner.trajectory_premises()
        save('TRAJECTORY_PREMISES.json',premises)
        if not premises['passed']:
            save('FIRST_FAILURE.json',dict(stage='trajectory_premises',code='ContinuationTrajectoryPremiseRejected',evidence=premises))
            raise ValueError('ContinuationTrajectoryPremiseRejected')
    last=next_layer if forecast_scope=='next_transition' else None
    forecast=BoundProductionForecastCache(adapter,last_layer=last).forecast(adapter)
    expected_layers=[next_layer] if last is not None else list(range(next_layer,len(adapter.schedule.increments)+1))
    assert forecast.layers.tolist()==expected_layers
    high=forecast.vertex['coordinates'][0]
    arrays={g+'__'+k:v for g in ('vertex','edge','triangle') for k,v in getattr(forecast,g).items()}
    np.savez_compressed(OUT/'FORECAST.npz',**arrays)
    forecast_record=dict(binding=adapter.binding,layers=forecast.layers.tolist(),identity=forecast.scientific_hash(),
        hard_events=forecast.hard_events(),arrays={k:array_hash(v) for k,v in arrays.items()},
        remaining_increments=([adapter.schedule.row_record(int(k)) for k in forecast.layers] if owner is not None else
            [dict(layer=int(k),value=float(adapter.remaining_increments[i]),hex=float(adapter.remaining_increments[i]).hex()) for i,k in enumerate(forecast.layers)]),
        curvature='Legacy source curvature placeholder suppressed; not measured by cheap forecast')
    if last is not None:forecast_record['scope']='NextTransitionOnly; full schedule bound; future lookahead not evaluated'
    save('FORECAST.json',forecast_record);print('forecast bound layers',forecast.layers.tolist(),flush=True)
    if owner is not None and any(forecast.hard_events().values()):
        save('FIRST_FAILURE.json',dict(stage='cheap_forecast',code='CheapHardEvent',events=forecast.hard_events()))
        raise ValueError('CheapHardEvent')
    screen=direct_screen(adapter.topology,adapter.committed_prefix,high,layer=next_layer)
    save('DIRECT.json',dict(counts=screen['counts'],events=screen['events'],topology=screen['topology'],
        exact_record_hash=screen['exact_record_hash'],distributions={g:{k:distributions(v) for k,v in screen[g].items() if v.ndim==1} for g in ('cells','sides','caps')},seconds=screen['seconds']))
    np.savez_compressed(OUT/'SCREEN_ARRAYS.npz',provisional_high=high,**{g+'__'+k:v for g in ('cells','sides','caps') for k,v in screen[g].items()})
    with (OUT/'EXACT_WEDGES.jsonl').open('w',encoding='utf8',newline='\n') as f:
        for row in screen['exact_records']:f.write(json.dumps(row,sort_keys=True,separators=(',',':'))+'\n')
    if owner is not None and screen['events']:
        save('FIRST_FAILURE.json',dict(stage='direct_screen',code='DirectHardEvent',events=screen['events'],
            counts=screen['counts'],exact_record_hash=screen['exact_record_hash']))
        raise ValueError('DirectHardEvent')
    inv=prefix_face_inventory(adapter.topology,adapter.committed_prefix,high)
    broad=collision_screen(inv);print('broad phase',broad['populations'],flush=True)
    points=np.concatenate((*adapter.committed_prefix,high))
    assert np.array_equal(points[inv['labels']],inv['coordinates'])
    proof=classify(points,inv['labels'],broad['pairs'],progress=lambda r:print('localization',r['completed'],flush=True))
    codes=proof['codes'];risks,seeds=risk_columns(codes,broad['pairs'],inv['columns'])
    direct_seeds=sorted({c for e in screen['events'] for c in e['columns']})
    seeds=np.union1d(seeds,direct_seeds).astype(np.int64);closure=close_risk(adapter.topology,seeds)
    closure_seconds=closure.pop('seconds')
    accounting=dict(candidates=len(codes),certified=int(((codes>0)&(codes<10)).sum()),unresolved=int((codes==0).sum()),
        forbidden=int((codes==10).sum()),failure=int((codes==11).sum()),missing=0,duplicate=0)
    np.savez_compressed(OUT/'LOCALIZATION.npz',pairs=broad['pairs'],codes=codes,witness=proof['witness'],rotations=proof['rotations'])
    save('LOCALIZATION.json',dict(accounting=accounting,methods={METHODS[int(k)]:int(v) for k,v in zip(*np.unique(codes,return_counts=True))},
        inventory={k:array_hash(v) for k,v in inv.items()},candidate_identity=array_hash(broad['pairs']),codes_identity=array_hash(codes),
        populations=broad['populations'],risk_seeds=seeds.tolist(),direct_seeds=direct_seeds,closure=closure,exact_rows=proof['exact_rows'],
        timings=dict(broad=broad['seconds'],closure=closure_seconds,**proof['timings'])))
    complete=dict.fromkeys(REQUIRED,True)
    complete['direct_metrics']=not screen['events'];complete['risk_supports']=not closure['blocked_groups']
    evidence=dict(binding=adapter.binding,increment_hex=INCREMENT_HEX,completeness=complete,
        cheap_hard_events=forecast.hard_events(),direct_hard_events=screen['events'],collision_accounting=accounting,
        risk_seeds=seeds.tolist(),patches=closure['groups'],mandatory_detail_requirements=[],
        authorities=dict(forecast=forecast.scientific_hash(),screen=file_hash(OUT/'DIRECT.json'),localization=file_hash(OUT/'LOCALIZATION.json')))
    if owner is not None:
        del evidence['increment_hex']
        evidence.update(increment_record=plan['increment_record'],trajectory_premises=premises)
    if last is not None:evidence['forecast_scope']=dict(kind='NextTransitionOnly/1',layers=forecast.layers.tolist(),schedule=adapter.schedule.sha256)
    decision=evaluate(evidence,identity(evidence),adapter.binding,adapter.front.coordinates,adapter.topology,high,
        schedule=adapter.schedule,**(dict(trajectory=owner) if owner is not None else {}))
    save('ADMISSION.json',dict(evidence=evidence,evidence_identity=identity(evidence),decision=decision,
        decision_identity=identity(decision),restore_seconds=restore_seconds,total_seconds=time.perf_counter()-started,**resources()))
    print('ADMISSION',decision,flush=True)
    if not decision['generation_attempt_authorized']:raise ValueError('ReadOnlyAdmissionBlocked')


class MeasuredStore(NativeCheckpointStore):
    def __init__(self,path,certificate):
        super().__init__(path);self.certificate=certificate;self.materialized=[];self.receipt_seconds=0.
    def _table(self,value,content_only=False):
        start=time.perf_counter();result=super()._table(value,content_only=content_only)
        if value is self.certificate:self.receipt_seconds=time.perf_counter()-start
        return result
    def _flat_table(self,value):
        result=super()._flat_table(value)
        if isinstance(value,ReceiptOverlay):self.materialized.append(dict(rows=len(value),file=result['file'],bytes=(self.objects/result['file']).stat().st_size))
        return result
