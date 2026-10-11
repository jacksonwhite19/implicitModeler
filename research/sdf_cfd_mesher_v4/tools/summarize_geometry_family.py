"""Measured cohort inventory. Never promote a stopped prefix to full growth."""
import sys,json,time
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'src')]
from collections import Counter
import geometry_family_authority as authority
from run_geometry_family import OUT,remaining_columns
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs
from sdf_cfd_mesher_v4.common.authority import file_hash


def read(path,default=None):return json.loads(path.read_text()) if path.exists() else default
def write(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n',newline='\n')


def metrics(rows):
    minima={};maxima={}
    for row in rows:
        for key in ('volume','determinant','weight','volume_ratio','symmetric_volume_ratio','interpolation_weight'):
            if key in row:minima[key]=min(minima.get(key,row[key]),row[key])
        for key in ('aspect','skewness','nonorthogonality'):
            if key in row:maxima[key]=max(maxima.get(key,row[key]),row[key])
    return dict(minimum=minima,maximum=maxima)


def committed_history(folder,name,seen=None):
    """Follow immutable restart ancestry, not unrelated rejected run histories."""
    seen=set() if seen is None else set(seen)
    folder=folder.resolve()
    if folder in seen:raise ValueError('CyclicRestartHistory')
    seen.add(folder);prefix=[]
    start=read(folder/'RESTART_INPUT.json') or read(folder/'REPLAY_INPUT.json')
    if start:
        source=Path(start.get('source',start.get('source_checkpoint')))
        if not source.is_absolute():source=source.resolve()
        parent=source.parent.parent
        if parent!=folder:
            target=read(source)['scientific']
            for row in committed_history(parent,name,seen):
                prefix.append(row)
                if row.get('state')==target:break
            else:
                # A preserved initial checkpoint can precede recorded steps.
                prefix=[]
    rows=[];states=set()
    for row in prefix+read(folder/name,[]):
        if row.get('state') not in states:
            rows.append(row);states.add(row.get('state'))
    return rows


def summarize(folder):
    result=dict(label=folder.name,path=str(folder.relative_to(OUT)),failure=read(folder/'FAILURE.json'),
                performance=read(folder/'PERFORMANCE.json'),complete_performance=read(folder/'COMPLETE_PERFORMANCE.json'))
    trace=committed_history(folder,'TRACE.json');result['trace']=trace
    result['accepted_step_seconds']=sum(r['seconds'] for r in trace)
    result['retained_rss_by_step']=[dict(layer=r['layer'],operation=r['operation'],rss=r['rss']) for r in trace]
    result['shell_result']=read(folder/'SHELL_RESULT.json');result['partial_shell_result']=read(folder/'PARTIAL_SHELL_RESULT.json')
    result['assembly']=read(folder/'ASSEMBLY.json')
    result['core_trials']={p.parent.name:read(p) for p in folder.glob('*/GENERATION.json')}
    result['repair_trials']=read(folder/'repaired_core/TRIALS.json');result['repair']=read(folder/'repaired_core/REPAIR.json')
    result['first_layer']=read(folder/'FIRST_LAYER.json',{})
    if result['first_layer']:
        result['first_layer']=dict(passed=result['first_layer']['passed'],sdf=result['first_layer']['sdf'],
            metrics=metrics(result['first_layer']['quality']['cells']+result['first_layer']['quality']['faces']),
            failures=result['first_layer']['quality']['failures'])
    result['mechanisms']=Counter();result['accepted_corrections']=Counter()
    for op in committed_history(folder,'OPERATIONS.json'):
        for row in op['policy']['trace']:
            result['mechanisms'][row['state']]+=1
            if row['reason']=='Fully validated correction':result['accepted_corrections'][row['state']]+=1
    result['mechanisms']=dict(result['mechanisms']);result['accepted_corrections']=dict(result['accepted_corrections'])
    result['initial_correction']=bool((folder/'checkpoints/CURRENT.json').exists() and result['first_layer'] and not result['first_layer']['passed'])
    result['initial_correction_history']=[dict(stage=r['stage'],passed=r['passed'],trial_count=len(r.get('trials',[]))) for r in read(folder/'FIRST_LAYER_CORRECTION.json',[])]
    result['dependencies']=[dict(file=p.name,sha256=file_hash(p),groups=[len(v) for v in read(p)['groups']],selected=len(read(p)['selected']),layer=read(p)['layer']) for p in folder.glob('DEPENDENCY_GROUPS_*.json')]
    if not (folder/'checkpoints/CURRENT.json').exists():return result
    state=NativeCheckpointStore(folder/'checkpoints').restart();inputs=geometry_inputs(state)
    result['scientific']=scientific_hash(state);result['cells']=dict(Counter(c['kind'] for c in state['cells'].values()))
    result['total_shell_cells']=len(state['cells']);result['columns']=dict(Counter(c['status'] for c in state['columns'].values()))
    result['local_layer_distribution']=dict(Counter(str(c['last_prism_layer']) for c in state['columns'].values()))
    result['remaining_active_columns']=remaining_columns(state)
    values=[];records=0
    for _,receipt in state['quality'].items():
        if 'inputs' in receipt:assert all(inputs[k]==v for k,v in receipt['inputs'].items())
        if 'value' in receipt:values.append(receipt['value'])
        records+=1
    result['quality']=metrics(values);result['quality_records']=records
    relations=0;methods=Counter()
    for _,receipt in state['certificate'].items():
        assert all(inputs[k]==v for k,v in receipt['inputs'].items())
        assert receipt['value']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
        relations+=1;methods[receipt['value']['method']]+=1
    result['shell_relations']=relations;result['certificate_methods']=dict(methods)
    result['events']=[]
    for key,event in state['events'].items():
        c=event['cavity_manifest'];expanded=c['expansion'];last=next((r for r in reversed(expanded) if 'depth' in r),{})
        result['events'].append(dict(id=key,removed_cells=c['removed_cells'],removed_layers=dict(Counter(str(int(k)//1000000) for k in c['removed_cells'])),
            retained_layer=last.get('depth'),active_vertices=len(last.get('active',[])),seed=c['seed'],
            coupled_rejections=sum(r.get('complete_neighbor',{}).get('passed') is False for r in expanded)))
    if result['shell_result'] and result['shell_result']['final']==result['scientific'] and not result['remaining_active_columns']:
        result['failure_resolved']=result['failure'] is not None
        result['classification']='GeometryFamilyCaseBoundedQualified' if result['assembly'] and result['assembly']['passed'] else 'GeometryFamilyCasePartiallyQualified'
    else:result['classification']='GeometryFamilyCasePartiallyQualified'
    return result


def run():
    rows=[]
    for spec in authority.cohort()+authority.multi_region_definitions():
        case=OUT/spec['name'];runs={}
        for label in ('Q','S','T','R','U','V','X','Y','Z','W','M'):
            if (case/label).exists():runs[label]=summarize(case/label)
        rows.append(dict(spec=spec,runs=runs,determinism=read(case/'DETERMINISM.json'),core_determinism=read(case/'CORE_DETERMINISM.json')))
    write(OUT/'COHORT_MEASUREMENTS.json',dict(scope='Measured run inventory; no universal or CFD qualification claim',cases=rows))

if __name__=='__main__':run()
