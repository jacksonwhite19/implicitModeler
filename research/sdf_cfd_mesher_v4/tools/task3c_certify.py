"""Current-layer exact checks and explicitly bounded future evidence."""
from task3_repair_study import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import active_patch
from sdf_cfd_mesher_v4.advancing_front.cap_certificate import certify_bounded,pair_population
from task3c_io import read as read_evidence,write as write_evidence
OUT=ROOT/'results/task3c'

class ProgressSession(CertificateSession):
    def __init__(self,label):super().__init__(8192);self.label=label;self.n=0
    def pair(self,a,b):
        r=super().pair(a,b);self.n+=1
        if self.n%2000==0:print(self.label,self.n,flush=True)
        return r

def run(replay=False):
    start=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,fullindex,oneindex,_=ctx
    prior=read_evidence(OUT/'component_00_closed_support.json');ev=next(e for e in prior['history'] if e.get('potential'))
    ids=np.array(prior['final']['patch']['active']);trial=np.array(ev['projection']['projected']);changed=np.any(trial!=coords[0,ids],axis=1)
    common=read_evidence(OUT/'COMMON_NEGATIVE_CAP.json');assert common['eligible']
    proposals=[dict(members=['component_00'],active=ids,changed=ids[changed],coordinates=trial[changed]),
        dict(members=['component_01','component_02','component_04'],active=np.array(common['verification_active_union']),
             changed=np.array(common['active_ids']),coordinates=np.array(common['projection']['projected']))]
    certificates=[];timings={};fc=LocalForecastCache(front,top,schedule,sdf,coords,normals)
    for gi,g in enumerate(proposals):
        label='+'.join(g['members']);beg=time.perf_counter();p=active_patch(top,g['active'])
        key=scientific_hash(dict(ids=g['changed'],coordinates=g['coordinates']))
        future=fc.forecast(g['changed'],g['coordinates']);dc=DetailedCache();r=dc.get(front,future,top,p.detailed);base=dc.get(front,fc.baseline,top,p.detailed)
        gate=numeric_gates(r,candidate_only=True);exact=exact_candidate_jacobians(front,top,future,p.detailed)
        assert gate['passed'] and exact['passed'],label
        collateral=no_new_earlier_failure(failure_times(base),failure_times(r));assert not collateral
        targets=[]
        for spec in components():
            if spec['name'] in g['members']:
                b=target_status(base,spec['columns'],spec['metric']);t=target_status(r,spec['columns'],spec['metric'])
                assert hazard_improves(b,t)
                targets.append(dict(component=spec['name'],baseline=b,candidate=t))
        edgeids=dependencies(top,g['changed'])[1];current=SimpleNamespace(vertex={'coordinates':future.vertex['coordinates'][:1]})
        idx=updated_index(oneindex,current,edgeids)
        if gi==0 and not replay:
            old=json.loads((OUT/'CERTIFICATE_0_CURRENT.json').read_text())
            assert old['proposal']==key and old['patch']==p.record()
            cs=old['current_certificate']|dict(pair_bound=250000,pair_identity_hash=pair_population(idx,p.detailed.side_edges)['pair_identity_hash'])
        else:cs=certify_bounded(idx,p.detailed,ProgressSession(label+':current'))
        print('CURRENT_DONE',label,cs['classification'],flush=True)
        idx=updated_index(fullindex,future,edgeids)
        cc=certify_bounded(idx,SimpleNamespace(side_edges=edgeids),ProgressSession(label+':future'))
        record=dict(proposal=key,members=g['members'],patch=p.record(),changed_vertices=g['changed'],layer1_coordinates=g['coordinates'],
            candidate_gate=gate,exact_jacobians=exact,current_certificate=cs,future_certificate=cc,
            current_layer_valid=cs['classification']=='LocalCoreSidePairsCertified',
            study_accepted=cs['classification']=='LocalCoreSidePairsCertified' and cc['classification']=='LocalCoreSidePairsCertified',
            targets=targets,new_earlier_numeric_failures=collateral,
            baseline_first_future_invalid=min(failure_times(base).values(),default=None),candidate_first_future_invalid=min(failure_times(r).values(),default=None),
            exact_known_pair=pair_evidence(front,top,future) if gi else None)
        data=dict(baseline=base,candidate=r,baseline_tails=sorted_tails(base),candidate_tails=sorted_tails(r),
            future_changed_coordinates=future.local['coordinates'],future_changed_normals=future.local['normal'])
        record['direct_scientific_hash']=scientific_hash(data)
        if replay:
            assert scientific_hash(read_evidence(OUT/f'PROPOSAL_{gi}_DIRECT.json'))==scientific_hash(data)
            assert scientific_hash(json.loads((OUT/f'CERTIFICATE_{gi}.json').read_text()))==scientific_hash(record)
        else:
            write_evidence(OUT/f'PROPOSAL_{gi}_DIRECT.json',data)
            (OUT/f'CERTIFICATE_{gi}.json').write_text(json.dumps(plain(record),sort_keys=True,separators=(',',':'))+'\n')
        certificates.append(record);timings[label]=dict(seconds=time.perf_counter()-beg,peak_rss=psutil.Process().memory_info().peak_wset)
        print('DONE',label,record['study_accepted'],cc['classification'],cc.get('reason'),flush=True)
    result=dict(proposals=certificates,scientific_hash=scientific_hash(certificates))
    if replay:assert json.loads((OUT/'CERTIFICATION.json').read_text())['scientific_hash']==result['scientific_hash']
    (OUT/('CERTIFICATION_REPLAY.json' if replay else 'CERTIFICATION.json')).write_text(json.dumps(plain(result),sort_keys=True,separators=(',',':'))+'\n')
    (OUT/('CERTIFICATION_REPLAY_TIMINGS.json' if replay else 'CERTIFICATION_TIMINGS.json')).write_text(json.dumps(timings|dict(total_seconds=time.perf_counter()-start),indent=2,sort_keys=True)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
