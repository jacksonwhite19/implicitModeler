"""Bounded qualification worker over a preserved native production milestone."""
import argparse, collections, gc, gzip, json, pickle, sys, time, traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
import fast_milestone
import generalize_local_remesh as g
from bounded_fallback_control import Events, AttemptStopped, require_space, failure_components
from sdf_cfd_mesher_v4.common.authority import file_hash, array_hash, identity


def save(path, data):
    path.write_text(json.dumps(g.s.r.w.clean(data),sort_keys=True,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=600)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    events=Events(a.output/'EVENTS.jsonl',a.seconds)
    try:
        events('stage_start',stage='checkpoint_load',guard=require_space(a.output,(a.source/'STATE.pickle.gz').stat().st_size,151991682))
        run=fast_milestone.load(a.source,a.output)
        events('stage_end',stage='checkpoint_load',cells=len(run.topology.cells),relations=len(run.proofs))
        provider=run.provider;before=run.tx.current
        events('stage_start',stage='nominal_geometry')
        fronts,old_levels=provider.geometry(before,1);levels=np.full(len(provider.x),len(fronts))
        events('stage_end',stage='nominal_geometry',layer=len(fronts),front_hash=array_hash(fronts[-1]))
        events('stage_start',stage='nominal_validation')
        tick=time.perf_counter();points,cells,outer,checked,times=g.validation(provider.x,provider.tri,fronts,levels,False)
        events('stage_end',stage='nominal_validation',seconds=time.perf_counter()-tick,timings=times)
        events('stage_start',stage='determinant_applicability')
        checked=provider.candidate_quality(checked,fronts,levels)
        events('stage_end',stage='determinant_applicability',failures=len(checked['quality']['failures']))
        failures=checked['quality']['failures'];groups=failure_components(cells,failures)
        byid={c['id']:c for c in cells};quality={r['id']:r for r in checked['quality']['cells']};exact={r['id']:r for r in checked['quality']['exact']}
        components=[]
        for ids in groups:
            cc=[byid[i] for i in ids];vertices=sorted({v for c in cc for v in c['vertices']})
            columns=sorted({c['column'] for c in cc});source=sorted({int(v) for col in columns for v in provider.tri[col]})
            components.append(dict(cells=ids,source_columns=columns,source_vertices=source,
                layers=sorted({c['layer'] for c in cc}),bounds=[points[vertices].min(0),points[vertices].max(0)],
                quality=[quality[i] for i in ids],exact=[exact[i] for i in ids],
                faces=[r for r in checked['quality']['faces'] if any(i in ids for i in r['owners'])],
                heights=np.linalg.norm(fronts[-1][source]-fronts[-2][source],axis=1),
                normals=provider.geometry_source.Source(provider.spec).evaluate(fronts[-1][source]).gradient))
        save(a.output/'NOMINAL.json',dict(layer=len(fronts),cells=len(cells),relations_before=len(run.proofs),
            counts=collections.Counter(r['kind'] for r in failures),failures=failures,
            components=components,component_count=len(groups),ownership=checked['ownership'],vertex_links=checked['vertex_links'],
            sdf=provider.geometry_source.sdf_record(provider.spec,fronts[-1],provider.schedule.targets[len(fronts)-1]),timings=times))
        events('stage_start',stage='reproducer_storage')
        np.savez_compressed(a.output/'REPRODUCER.npz',wall=provider.x,triangles=provider.tri,fronts=np.asarray(fronts),levels=levels)
        with gzip.open(a.output/'CHECKED.pickle.gz','wb',compresslevel=1) as f:pickle.dump(checked,f,protocol=5)
        save(a.output/'REPRODUCER_AUTHORITY.json',dict(source=str(a.source),source_manifest=file_hash(a.source/'CURRENT.json'),
            arrays=file_hash(a.output/'REPRODUCER.npz'),checked=file_hash(a.output/'CHECKED.pickle.gz'),scope='Rejected nominal candidate; not acceptance authority'))
        events('stage_end',stage='reproducer_storage')
    except BaseException as exc:
        save(a.output/'STOP.json',dict(error=str(exc),traceback=traceback.format_exc(),accepted=False))
        raise

if __name__=='__main__':main()
