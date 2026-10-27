"""Bounded Task-2 study; restored forecast, local geometry, production certificate.

No optimizer, compression solver, mesh export or CFD entry point is called.
"""
from pathlib import Path
import sys,os,time,json,csv,dataclasses,gc,threading,datetime
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
import numpy as np
import psutil
from scipy.stats import rankdata,spearmanr
from sdf_cfd_mesher_v4.common.authority import REPO,file_hash,array_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology,CommittedFrontState
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.forecast.detailed.archive import restore
from sdf_cfd_mesher_v4.forecast.detailed.patch import column_patch,patch_key
from sdf_cfd_mesher_v4.forecast.detailed.metrics import measure
from sdf_cfd_mesher_v4.forecast.detailed.cache import DetailedCache,fork_changed_forecast
from sdf_cfd_mesher_v4.forecast.detailed.continuous import SideIndex
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,canonical,scientific_hash
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import CertificateSession
from sdf_cfd_mesher_v4.runtime.scaling import v3_scaling_case
OUT=ROOT/'results/task2';OUT.mkdir(exist_ok=True)

def write(name,value):(OUT/name).write_bytes((json.dumps(plain(value),indent=2,sort_keys=True,allow_nan=False)+'\n').encode())
def stats(values):
    x=np.asarray(values).ravel();f=x[np.isfinite(x)]
    return dict(count=len(x),finite=len(f),**(dict(zip(('min','p10','median','p90','max'),map(float,np.quantile(f,[0,.1,.5,.9,1])))) if len(f) else {}))
def rss():return psutil.Process().memory_info().rss
class Stage:
    def __init__(self,name):self.name=name
    def __enter__(self):
        self.start=time.perf_counter();self.peak=rss();self.done=False
        def sample():
            while not self.done:
                self.peak=max(self.peak,rss())
                if self.peak>12884901888:
                    write('RESOURCE_STOP.json',dict(reason='WorkerRSSBudget',peak=self.peak));os._exit(91)
                time.sleep(.05)
        self.thread=threading.Thread(target=sample,daemon=True);self.thread.start();return self
    def __exit__(self,*args):
        self.done=True;self.thread.join();self.record=dict(seconds=time.perf_counter()-self.start,peak_rss=self.peak)
        stages[self.name]=self.record;write('TIMINGS.json',stages);print(self.name,self.record,flush=True)
stages={}

def cohort(front,top,forecast):
    records=[];patches=[];used=set()
    def add(anchor,reason,layer,value=None):
        patch=column_patch(top,int(anchor),rings=1)
        overlap=[j for j,p in enumerate(patches) if set(p.core_columns).intersection(patch.core_columns)]
        selection=dict(reason=reason,anchor=int(anchor),layer=int(layer),value=value)
        if overlap:
            records[overlap[0]]['additional_selections'].append(selection);return
        centre=front.coordinates[top.triangles[int(anchor)]].mean(0)
        records.append(dict(patch_id=f'patch_{len(records):03}',anchor=int(anchor),selection=selection,additional_selections=[],wall_centroid=centre.tolist(),layers=list(range(1,21)),**patch.record()))
        patches.append(patch);used.update(patch.core_columns)
    with (ROOT/'results/task1/DESCRIPTOR_EXTREMA.csv').open(newline='') as f:
        for r in csv.DictReader(f):
            i=int(r['source_index']);kind=r['entity']
            anchor=int(top.vertex_triangles.row(i)[0]) if kind=='vertex' else int(top.edge_triangles.row(i)[0]) if kind=='edge' else i
            add(anchor,'Task1 '+r['descriptor']+' '+r['extremum'],int(r['layer']),float(r['value']))
    curvature=forecast.vertex['section_curvature']
    for sense,fn in [('min',np.argmin),('max',np.argmax)]:
        k,i=np.unravel_index(fn(curvature),curvature.shape);add(top.vertex_triangles.row(i)[0],'section_curvature '+sense,k+1,float(curvature[k,i]))
    refs=json.loads((ROOT/'authorities/DATA_AUTHORITIES.json').read_text())['references']
    g=(REPO/next(r['path'] for r in refs if r['role']=='legacy synthetic field diagnostic; never a V4 gate')).parent
    path=g/'prism_placement_counterfactual_20260913/SF2/COHORTS.json';sha=file_hash(path);historical=json.loads(path.read_text())
    write('HISTORICAL_SELECTION_SOURCE.json',dict(path=path.relative_to(REPO).as_posix(),sha256=sha,role='Development selection only; not production risk logic'))
    weak=sorted(historical['groups']['weakest_0_01pct'])
    for anchor in (weak[0],weak[len(weak)//2],historical['groups']['absolute_minimum'][0]):add(anchor,'preserved V3 pathology location',20)
    for anchor in sorted(historical['groups']['healthy_physical_controls'])[:3]:add(anchor,'preserved V3 matched healthy control; not V4 truth',20)
    # Coverage samples are nearest source centroids to explicit coordinate targets.
    # They are sampling locations, never thresholds or production risk masks.
    centres=front.coordinates[top.triangles].mean(1)
    for point,layer in [((.1,-.2,.03),1),((.5,0,.05),10),((.95,.2,.01),20)]:
        anchor=int(np.argmin(np.sum((centres-np.asarray(point))**2,axis=1)));add(anchor,'spatial sampling control target '+str(point),layer)
    return records,patches

def rows_for(result,patch,front,forecast,top):
    c=result['cells'];records=[];face_rows=[]
    incident={}
    for j,fid in enumerate(result['face_ids']):
        kind,layer,entity,a,b=fid;row=dict(kind=kind,layer=layer,entity=entity,owner_column=a,neighbor_column=b,**{k:float(v[j]) for k,v in result['faces'].items()});face_rows.append(row)
        cells=[(layer,a),(layer,b)] if kind=='side' else [(layer,a),(layer+1,a)]
        for cell in cells:incident.setdefault(cell,[]).append(j)
    for k in range(20):
        for j,column in enumerate(patch.core_columns):
            verts=top.triangles[column];ed=top.triangle_edges[column]
            row={name:value[k,j].tolist() for name,value in c.items()}
            if not row['G_bound_available']:row['G_bound_unavailable_reason']='SingularProductionReconstructionMatrix; no substitute operator'
            row['wall_vertex_indices']=verts.tolist();row['wall_vertex_ids']=[front.vertex_ids[i] for i in verts]
            row['wall_centroid']=front.coordinates[verts].mean(0).tolist()
            row.update(edge_ratio_max=float(forecast.edge['ratio'][k,ed].max()),triangle_area_ratio=float(forecast.triangle['ratio'][k,column]),
                       neighbor_normal_angle_max_degrees=float(np.degrees(forecast.edge['normal_angle'][k,ed]).max()),
                       section_curvature_max=float(forecast.vertex['section_curvature'][k,verts].max()),
                       successive_direction_change_max_degrees=float(np.degrees(forecast.vertex['direction_change'][k,verts]).max()),
                       sdf_residual_abs_max=float(abs(forecast.vertex['residual'][k,verts]).max()),
                       normal_convergence_max=float(forecast.edge['normal_convergence'][k,ed].max()),
                       step_length_max=float(forecast.vertex['step_length'][k,verts].max()),
                       nominal_compression_factor=1.0)
            ids=incident[(k+1,int(column))]
            for name,fn in [('weight',np.min),('skewness',np.max),('symmetric_volume_ratio',np.min),('nonorthogonality',np.max)]:row[name]=float(fn(result['faces'][name][ids]))
            row['neighbor_face_scope']='all existing shell internal faces; exterior cap unavailable' if k==19 else 'complete existing shell internal faces'
            records.append(row)
    return records,face_rows

DESCRIPTORS=['edge_ratio_max','triangle_area_ratio','neighbor_normal_angle_max_degrees','section_curvature_max','successive_direction_change_max_degrees','sdf_residual_abs_max','normal_convergence_max','step_length_max']
DIRECT={'sigma':'low','G_bound':'high','determinant':'low','weight':'low','skewness':'high','symmetric_volume_ratio':'low','aspect':'high','nonorthogonality':'high','side_normal_separation_degrees':'low','jacobian':'low','volume':'low'}
def relationships(rows):
    table=[];rankings={};n=len(rows)
    for key in DESCRIPTORS+list(DIRECT):
        x=np.array([r[key] for r in rows]);rankings[key]=rankdata(x,method='average',nan_policy='omit').tolist()
    for desc in DESCRIPTORS:
        x=np.array([r[desc] for r in rows]);xhi=np.lexsort((np.arange(n),-x))[:5];xlo=np.lexsort((np.arange(n),x))[:5]
        for metric,sense in DIRECT.items():
            y=np.array([r[metric] for r in rows]);valid=np.isfinite(x)&np.isfinite(y);ids=np.flatnonzero(valid);tail=ids[np.lexsort((ids,y[ids] if sense=='low' else -y[ids]))[:5]]
            rho=float(spearmanr(x[valid],y[valid]).statistic) if len(np.unique(x[valid]))>1 and len(np.unique(y[valid]))>1 else None
            table.append(dict(descriptor=desc,metric=metric,sample_count=int(valid.sum()),unavailable_count=int((~valid).sum()),descriptor_range=[float(x[valid].min()),float(x[valid].max())],metric_range=[float(y[valid].min()),float(y[valid].max())],spearman=rho,
                              direct_tail_sense=sense,tail_count=5,top_five_coincidence=len(set(xhi)&set(tail)),bottom_five_coincidence=len(set(xlo)&set(tail)),
                              descriptor_top_five=[int(rows[i]['cell_id']) for i in xhi],descriptor_bottom_five=[int(rows[i]['cell_id']) for i in xlo],direct_tail_five=[int(rows[i]['cell_id']) for i in tail]))
    return table,rankings

def main():
    started=time.perf_counter();summary=dict(classification='V4LocalDetailedLookaheadAndRiskPatchForecastPartial',global_dense_allocation=False,optimizer_executed=False,compression_executed=False,committed_layers=0)
    with Stage('restore_task1'):
        front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule();sdf=NativeSdf()
        cache,record=restore(front,top,schedule,sdf);forecast=cache.result
        assert front.coordinate_hash=='3412cc0b84cc4ac092ad2cd89698cb9857ddb8428f3200db1e6d2e0ef12e7c94'
        assert top.sha256=='69477add150348f567e454cb2216a913124adecc6ec11c47f58471c0931868e7'
        write('RESTORE.json',record)
    with Stage('selection'):
        selection,patches=cohort(front,top,forecast);write('PATCH_SELECTION.json',selection)
    dc=DetailedCache();allrows=[];allfaces={};perpatch=[]
    with Stage('local_metrics'):
        for item,p in zip(selection,patches):
            result=dc.get(front,forecast,top,p);rows,faces=rows_for(result,p,front,forecast,top)
            for row in rows:row['patch_id']=item['patch_id']
            allrows.extend(rows)
            for row in faces:allfaces[(row['kind'],row['layer'],row['entity'])]=row
            perpatch.append(dict(patch_id=item['patch_id'],counts=result['counts'],unavailable=result['unavailable'],sigma=stats(result['cells']['sigma']),G_bound=stats(result['cells']['G_bound']),
                                 sigma_min=min(rows,key=lambda r:r['sigma']),G_bound_max=max((r for r in rows if r['G_bound_available']),key=lambda r:r['G_bound'],default=None),G_bound_unavailable=sum(not r['G_bound_available'] for r in rows)))
        allrows.sort(key=lambda r:r['cell_id']);faces=[allfaces[k] for k in sorted(allfaces)]
        write('DIRECT_PRISM_ROWS.json',allrows);write('DIRECT_FACE_ROWS.json',faces);write('PATCH_METRICS.json',perpatch)
        failures=dict(volume=sum(r['volume']<=0 or r['signed_volume']<=0 for r in allrows),jacobian=sum(r['jacobian']<=0 for r in allrows),determinant=sum(r['determinant']<.0011 for r in allrows),aspect=sum(r['aspect']>1000 for r in allrows),
                      interpolation_weight=sum(r['weight']<.055 for r in faces),skewness=sum(r['skewness']>3.63636 for r in faces),symmetric_volume_ratio=sum(r['symmetric_volume_ratio']<.01 for r in faces),invalid_topology=0,invalid_source_ownership=0)
        summary.update(patches=len(patches),measured_cells=len(allrows),measured_unique_faces=len(faces),G_bound_unavailable=sum(not r['G_bound_available'] for r in allrows),hard_gate_violations=failures,distributions={k:stats([r[k] for r in allrows]) for k in list(DIRECT)+['signed_volume']},extrema={k:min((r for r in allrows if np.isfinite(r[k])),key=lambda r:r[k]) if sense=='low' else max((r for r in allrows if np.isfinite(r[k])),key=lambda r:r[k]) for k,sense in DIRECT.items()})
    with Stage('relationships'):
        comparisons,ranks=relationships(allrows);write('DESCRIPTOR_COMPARISONS.json',comparisons);write('ENTITY_RANKS.json',dict(cell_ids=[r['cell_id'] for r in allrows],rankings=ranks))
    # Exact replay of all direct scientific cells/faces/descriptors from clean
    # detailed cache; the one restored whole-front forecast is reused.
    with Stage('direct_replay'):
        replay=[];replayfaces={}
        for item,p in zip(selection,patches):
            rr,ff=rows_for(measure(front,forecast,top,p),p,front,forecast,top)
            for row in rr:row['patch_id']=item['patch_id']
            replay.extend(rr)
            for row in ff:replayfaces[(row['kind'],row['layer'],row['entity'])]=row
        replay.sort(key=lambda r:r['cell_id']);ff=[replayfaces[k] for k in sorted(replayfaces)]
        sci=dict(selection=selection,cells=allrows,faces=faces,comparisons=comparisons,ranks=ranks)
        repcomp,repranks=relationships(replay);sci2=dict(selection=selection,cells=replay,faces=ff,comparisons=repcomp,ranks=repranks)
        h1=scientific_hash(sci);h2=scientific_hash(sci2);assert h1==h2,'DirectScientificNondeterminism'
        write('DETERMINISM.json',dict(direct_hash_run1=h1,direct_hash_run2=h2,direct_passed=True,continuous_replay='pending'))
    with Stage('cache_locality'):
        before=dict(hits=dc.hits,misses=dc.misses)
        for p in patches:dc.get(front,forecast,top,p)
        warm=dict(hits=dc.hits,misses=dc.misses)
        v=int(patches[0].core_vertices[0]);changed=front.coordinates.copy();old=float(changed[v,0]);changed[v,0]=np.nextafter(changed[v,0],np.inf)
        test=CommittedFrontState.create(0,changed,front.vertex_ids,front.triangles,front.source_ancestry,front.target_distances,front.geometry_authority,front.surface_authority)
        branch,local=fork_changed_forecast(cache,front,test,top,schedule,sdf,[v]);affected=dc.invalidate(patches,[v]);before_modified=dict(hits=dc.hits,misses=dc.misses)
        unchanged=[]
        for i,p in enumerate(patches):
            original=dc.get(front,forecast,top,p);r=dc.get(test,branch.result,top,p)
            if i not in affected:unchanged.append(r is original)
        assert all(unchanged)
        local.update(modification=dict(vertex=v,stable_id=front.vertex_ids[v],axis='x',before=old,after=float(changed[v,0]),delta=float(changed[v,0]-old),purpose='one binary64 ULP toward positive infinity; invalidation test only'),
                     detailed_before=before,detailed_warm=warm,detailed_before_modified=before_modified,detailed_after=dict(hits=dc.hits,misses=dc.misses),affected_patch_indices=affected,unrelated_patch_exact_hits=len(unchanged),unrelated_preserved=all(unchanged))
        write('CACHE_LOCALITY.json',local);del branch,test,changed;gc.collect()
    scaling=[]
    for rings in (0,1,2):
        with Stage('patch_size_'+str(rings)) as stage:
            p=column_patch(top,selection[0]['anchor'],rings);r=measure(front,forecast,top,p)
            blob=canonical(r);path=OUT/f'SCALING_{rings}.json';path.write_bytes(blob)
            rec=dict(rings=rings,**r['counts'],persistent_bytes=len(blob))
        rec.update(stage.record);scaling.append(rec)
    write('PATCH_SCALING.json',dict(local=scaling,rejected_global=dataclasses.asdict(v3_scaling_case()),dense_matrix_allocated=False))
    write('SUMMARY.json',summary)
    with Stage('continuous'):
        index=SideIndex(front,forecast,top);session=CertificateSession(cache_capacity=8192);continuous=[]
        for item,p in zip(selection,patches):
            print('CERTIFY',item['patch_id'],flush=True)
            def progress(r):write('CONTINUOUS_PROGRESS.json',dict(patch=item['patch_id'],**r))
            result=index.certify(p,session=session,progress=progress);result['patch_id']=item['patch_id'];continuous.append(result);write('CONTINUOUS.json',continuous)
            if result['classification']!='LocalCoreSidePairsCertified':
                summary.update(classification='V4LocalDetailedLookaheadAndRiskPatchForecastBlocked',blocker='Stop condition 7: '+result['reason'],continuous_completed_patches=len(continuous)-1)
                break
        else:
            # Replay exact pair outputs, scientific proof stream hashes included.
            second=[]
            for item,p in zip(selection,patches):
                result=index.certify(p,session=CertificateSession(),progress=None);result['patch_id']=item['patch_id'];second.append(result)
            scientific_cont=lambda rows:[{k:v for k,v in r.items() if k!='seconds'} for r in rows]
            final1=scientific_hash(dict(direct=sci,continuous=scientific_cont(continuous)))
            final2=scientific_hash(dict(direct=sci2,continuous=scientific_cont(second)))
            assert final1==final2,'ContinuousScientificNondeterminism'
            write('DETERMINISM.json',dict(direct_hash_run1=h1,direct_hash_run2=h2,direct_passed=True,full_hash_run1=final1,full_hash_run2=final2,continuous_replay=True))
            summary.update(classification='V4LocalDetailedLookaheadAndRiskPatchForecastComplete',continuous_completed_patches=len(continuous))
    summary.update(total_seconds=time.perf_counter()-started,stages=stages,peak_rss=max(s['peak_rss'] for s in stages.values()),initial_coordinate_hash=front.coordinate_hash,topology_hash=top.sha256)
    write('SUMMARY.json',summary);print('TASK2',summary['classification'],flush=True)

if __name__=='__main__':
    try:main()
    except Exception as exc:
        import traceback
        write('STOP.json',dict(classification='V4LocalDetailedLookaheadAndRiskPatchForecastBlocked',exception=type(exc).__name__,message=str(exc),traceback=traceback.format_exc(),stages=stages));raise
