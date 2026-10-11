"""Independent bounded evidence collection; no changes to production state."""
import sys
sys.dont_write_bytecode=True
import json,time
import numpy as np
import mpmath as mp
import psutil
from first_layer_determinant_trace import OUT,g,production,geometry
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,ReceiptTable
from sdf_cfd_mesher_v4.common.authority import file_hash


def high_precision():
    records=[]
    with mp.workdps(80):
        for path in sorted(OUT.glob('*_TRACE.json')):
            for row in json.loads(path.read_text())['reproducers']:
                vertices={v:[mp.mpf(float(a)) for a in xyz] for v,xyz in zip(row['cell']['vertices'],row['coordinates'])}
                areas=[]
                for f in row['cell']['faces']:
                    pts=[vertices[v] for v in f];a=[mp.mpf(0)]*3
                    for p,q in zip(pts,pts[1:]+pts[:1]):
                        a=[a[0]+p[1]*q[2]-p[2]*q[1],a[1]+p[2]*q[0]-p[0]*q[2],a[2]+p[0]*q[1]-p[1]*q[0]]
                    areas.append([v/2 for v in a])
                states={}
                for name,state in row['states'].items():
                    chosen=[a for a,s in zip(areas,state['selected']) if s]
                    value=mp.mpf(0)
                    if chosen:
                        avg=sum(mp.sqrt(sum(v*v for v in a)) for a in chosen)/len(chosen)
                        value=abs(mp.det(mp.matrix([[sum(a[i]*a[j]/avg**2 for a in chosen) for j in range(3)] for i in range(3)])))
                    states[name]=dict(from_exact_binary64_coordinates=mp.nstr(value,75),
                        difference_from_binary64=float(value-mp.mpf(state['determinant'])),
                        same_gate_decision=bool((value>=mp.mpf('.0011'))==(state['determinant']>=.0011)))
                records.append(dict(source=path.name,cell=row['cell']['id'],states=states))
    assert all(s['same_gate_decision'] for r in records for s in r['states'].values())
    g.save(OUT/'HIGH_PRECISION.json',dict(digits=80,records=records,passed=True))


def collect():
    records=[]
    for spec in production.cohort():
        name=spec['name'];label='B' if name=='separated_regions_production' else 'A'
        folder=OUT/name/label
        if not (folder/'SHELL_RESULT.json').exists():raise ValueError('ProductionReplayNotFinished:'+name)
        first=json.loads((folder/'FIRST_LAYER.json').read_text());quality=first['quality']
        shell=json.loads((folder/'SHELL_RESULT.json').read_text());performance=json.loads((folder/'PERFORMANCE.json').read_text())
        # The production run already performed complete native restart twice.
        # Verify its sealed pointer and relevant components instead of reading
        # millions of unchanged certificate rows again merely for reporting.
        store=NativeCheckpointStore(folder/'checkpoints')
        pointer=json.loads((store.path/'CURRENT.json').read_text())
        manifest=store.path/pointer['checkpoint'];assert file_hash(manifest)==pointer['sha256']
        envelope=json.loads(manifest.read_text());components=envelope['components']
        assert shell['restart_exact'] and pointer['scientific']==envelope['scientific']==shell['final']
        qr=components['quality'];table=ReceiptTable(store._object(qr),qr['count'],qr['scientific'])
        for key in ('atlas','forecast'):store._object(components[key])
        accepted={r['id']:r for r in quality['cells']}
        actual={int(k[5:]):r['value'] for k,r in table.items() if k.startswith('cell:') and int(k[5:]) in accepted}
        equality=actual==accepted
        if not equality:raise ValueError('CommittedFirstLayerMetricDrift:'+name)
        trace=[json.loads(v) for v in (folder/'COLD_MEMORY.jsonl').read_text().splitlines()]
        durations=[r for r in trace if r['event']=='exit']
        initialization=next(r['duration_seconds'] for r in durations if r['phase']==['process','initialize'])
        admission=next(r['seconds'] for r in durations if r['phase']==['process','checkpoint_restart'])
        before=json.loads((OUT.parents[1]/'results/broader_production_scale_geometry'/name/'D/PERFORMANCE.json').read_text())
        prior=json.loads((OUT.parents[1]/'results/broader_production_scale_geometry'/name/'D/FIRST_LAYER.json').read_text())['quality']
        other_exact=(prior['faces']==quality['faces'] and prior['exact']==quality['exact'] and
            [{k:v for k,v in r.items() if k!='determinant'} for r in prior['cells']]==
            [{k:v for k,v in r.items() if k!='determinant'} for r in quality['cells']])
        if not other_exact:raise ValueError('OtherFirstLayerMetricDrift:'+name)
        records.append(dict(case=name,label=label,admitted=first['passed'],
            before_determinant_failures=sum(r['determinant']<.0011 for r in prior['cells']),
            before_determinant_min=min(r['determinant'] for r in prior['cells']),other_metric_values_exact=other_exact,
            determinant_min=min(r['determinant'] for r in quality['cells']),
            determinant_failures=sum(r['determinant']<.0011 for r in quality['cells']),
            weight_min=min(r['weight'] for r in quality['faces']),
            weight_failures=sum(r['weight']<.055 for r in quality['faces']),
            initial_correction_iterations=0 if not (folder/'FIRST_LAYER_CORRECTION.json').exists() else len(json.loads((folder/'FIRST_LAYER_CORRECTION.json').read_text())),
            before_rejected_seconds=before['seconds'],after_initialization_seconds=initialization,
            after_first_commit_seconds=admission,
            total_including_next_layer_seconds=performance['seconds'],peak_rss=performance['peak_rss_bytes'],
            layer=shell['current_layer'],cells=shell['cells'],remesh_events=shell['remesh_count'],
            post_commit_metrics_exact=equality,restart_exact=True,
            atlas_restart_exact=shell['restart_exact'],forecast_restart_exact=shell['restart_exact'],
            wall_exact=shell['wall_exact'],first_layer_exact=shell['first_layer_exact']))
    g.save(OUT/'CASE_RESULTS.json',records)


if __name__=='__main__':
    if '--high-precision' in sys.argv:high_precision()
    else:collect()
