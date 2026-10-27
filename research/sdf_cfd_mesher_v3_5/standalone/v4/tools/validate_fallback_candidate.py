"""Independent, staged full-gate assessment of a discovered mixed candidate.

This worker never publishes a partially validated candidate.
"""
import argparse,json,sys,time,gc,traceback,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import fast_milestone, generalize_local_remesh as g
from bounded_fallback_control import Events,require_space
from investigate_large_fallback import save
from immutable_append_certificate import certify


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--nominal',type=Path,required=True)
    p.add_argument('--front',type=Path,required=True);p.add_argument('--levels',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=1200);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    events=Events(a.output/'EVENTS.jsonl',a.seconds);result=dict(committed=False)
    try:
        events('stage_start',stage='load',guard=require_space(a.output,(a.source/'STATE.pickle.gz').stat().st_size,(a.nominal/'CHECKED.pickle.gz').stat().st_size))
        run=fast_milestone.load(a.source,a.output);provider=run.provider;provider.search_progress=events
        events('stage_end',stage='load')
        z=np.load(a.nominal/'REPRODUCER.npz');fronts=list(z['fronts']);fronts[-1]=np.load(a.front)['front'];levels=np.load(a.levels)
        events('stage_start',stage='hard_quality')
        points,cells,outer,checked,times=g.validation(provider.x,provider.tri,fronts,levels,False)
        events('stage_end',stage='hard_quality',times=times,failures=len(checked['quality']['failures']))
        if not checked['quality']['passed']:raise ValueError('MixedCandidateHardQualityFailure')
        coupled=provider.candidate_check(points,cells,outer)
        save(a.output/'COUPLED.json',coupled);result['coupled_passed']=coupled['passed']
        if not coupled['passed']:raise ValueError('MixedCandidateCompleteNeighborFailure')
        events('stage_start',stage='global_volume_degree')
        tick=time.perf_counter();volume=g.volume.audit(points,cells,outer,provider.tri)
        save(a.output/'VOLUME.json',volume)
        result['volume']=dict(positive_cells=volume['positive_cells'],exact_volume_balance=volume['exact_volume_balance'],degree=volume['interior_degree']['passed'],seconds=time.perf_counter()-tick)
        events('stage_end',stage='global_volume_degree',**result['volume'])
        if not all(result['volume'][k] for k in ('positive_cells','exact_volume_balance','degree')):raise ValueError('MixedCandidateFillingFailure')
        certificate=certify(run,points,cells,g.t.ownership(cells,outer,[tuple(reversed(f)) for f in provider.tri])[0],events)
        result['certificate']={k:v for k,v in certificate.items() if k not in ('proofs','faces','face_keys')}
        save(a.output/'CERTIFICATE.json',result['certificate'])
        if not certificate['passed']:raise ValueError('MixedCandidateContinuousGeometryFailure')
        from audit_capsule_scaling import final_coverage,AABBIndex
        original=AABBIndex.pairs
        def progress_pairs(self,*args,**kwargs):
            for i,pair in enumerate(original(self,*args,**kwargs),1):
                if i%500000==0:events('coverage_progress',pairs=i)
                yield pair
        AABBIndex.pairs=progress_pairs
        try:
            events('stage_start',stage='independent_full_coverage');tick=time.perf_counter()
            coverage=final_coverage(points,certificate['faces'],certificate['proofs'])
            result['coverage']=dict(coverage,seconds=time.perf_counter()-tick)
            events('stage_end',stage='independent_full_coverage',**result['coverage'])
        finally:AABBIndex.pairs=original
        ids=sorted({v for f in outer for v in f});lookup={v:i+1 for i,v in enumerate(ids)}
        from sdf_cfd_mesher_v4.common.authority import identity
        path=a.output/'CORE_INGEST_INPUT.json';save(path,dict(coordinates=points[ids].tolist(),triangles=[[lookup[v] for v in f] for f in outer],outer_hash=identity(outer)))
        def wsl(p):p=p.resolve();return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')
        events('stage_start',stage='core_ingestion')
        subprocess.run(['wsl','--','python3',wsl(ROOT/'tools/cavity_core_ingestion_smoke.py'),wsl(path)],check=True,capture_output=True)
        result['core_ingestion']=json.loads((a.output/'CORE_INGESTION.json').read_text())
        events('stage_end',stage='core_ingestion',passed=result['core_ingestion']['passed'])
        # Publication and neighbor continuation remain separate obligations.
        result['status']='FullCandidateGeometryChecked;NeighborContinuationAndTransactionPending'
    except BaseException as exc:
        result.update(stop=str(exc),traceback=traceback.format_exc())
        try:events('bounded_stop',reason=str(exc))
        except BaseException:pass
    finally:save(a.output/'RESULT.json',result)

if __name__=='__main__':main()
