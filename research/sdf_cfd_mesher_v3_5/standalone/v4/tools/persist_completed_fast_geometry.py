"""Materialize an already accepted append from its saved outer coordinates.

No SDF projection or layer advancement is run. Every reconstructed native
geometry component must match the completed seal before data is published.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest
from sdf_cfd_mesher_v4.advancing_front.mixed_front import rebuild_dependencies
from sdf_cfd_mesher_v4.common.authority import file_hash
from production_fast_layer import native_append
from incremental_layer_validation import t
import cavity_volume_audit as volume


def main():
    p=argparse.ArgumentParser();p.add_argument('result',type=Path);a=p.parse_args();started=time.perf_counter()
    seal=json.loads((a.result/'SEAL.json').read_text());layer=json.loads((a.result/'LAYER.json').read_text())['layer']
    source=json.loads((a.result/'BOOTSTRAP.json').read_text())['source'];before=NativeCheckpointStore(source).restart()
    n=len(before['wall']['vertices'])
    def number(v):_,level,_,i=v.split(':');return int(level)*n+int(i)
    keys=sorted(before['vertices'],key=number)
    if [number(k) for k in keys]!=list(range(layer*n)):raise ValueError('NotCompleteAppendRepresentation')
    saved=json.loads((a.result/'CORE_INGEST_INPUT.json').read_text())
    points=np.concatenate([np.asarray([before['vertices'][k] for k in keys]),np.asarray(saved['coordinates'])])
    latest={}
    for c in before['cells'].values():
        if c['kind']!='prism':raise ValueError('NotOrdinaryPrismAppend')
        if c['layer']>latest.get(c['source'],{}).get('layer',0):latest[c['source']]=c
    tri=np.asarray([[number(v)%n for v in latest[str(i)]['vertices'][:3]] for i in range(len(before['wall']['triangles']))],dtype=int)
    cells=[]
    for k,c in sorted(before['cells'].items(),key=lambda kv:int(kv[0])):
        cells.append(t.cell([number(v) for v in c['vertices']],int(c['source']),c['layer']))
    added=[t.cell(np.r_[f+(layer-1)*n,f+layer*n],i,layer) for i,f in enumerate(tri)]
    cells+=added;outer=[tuple(map(int,f+layer*n)) for f in tri]
    state=native_append(before,cells,points,outer,added)
    import generalize_local_remesh as g
    schedule=g.q.load_schedule().targets
    rebuild_dependencies(before,state,schedule.tolist())
    for name,expected in seal['geometry_components'].items():
        if scientific_digest(state[name])!=expected:raise ValueError('SavedGeometryMismatch:'+name)
    _,ownership=t.ownership(cells,outer,[tuple(reversed(f)) for f in tri])
    if not ownership['passed']:raise ValueError('IndependentTopologyFailure')
    tick=time.perf_counter();audit=volume.audit(points,cells,outer,[tuple(map(int,f)) for f in tri])
    if not (audit['exact_volume_balance'] and audit['positive_cells'] and audit['interior_degree']['passed']):raise ValueError('IndependentVolumeFailure')
    target=a.result/'SEALED_GEOMETRY.npz'
    np.savez_compressed(target,points=points,cell_vertices=np.asarray([c['vertices'] for c in cells],dtype=np.int64),
        cell_ids=np.asarray([c['id'] for c in cells],dtype=np.int64),source_columns=np.asarray([c['column'] for c in cells],dtype=np.int64),
        layers=np.asarray([c['layer'] for c in cells],dtype=np.int64),outer=np.asarray(outer,dtype=np.int64),wall_triangles=tri,schedule=schedule)
    record=dict(passed=True,geometry_components_exact=True,ownership=ownership,volume_balance=audit['exact_volume_balance'],
        positive_cells=audit['positive_cells'],degree_passed=audit['interior_degree']['passed'],degree_unresolved=len(audit['interior_degree']['unresolved']),
        witnesses=len(audit['interior_degree']['values']),independent_volume_seconds=time.perf_counter()-tick,
        geometry_file=target.name,geometry_sha256=file_hash(target),seconds=time.perf_counter()-started,
        scope='Saved-coordinate reconstruction only; no layer generation or SDF projection')
    (a.result/'FINAL_REPRESENTATION_AUDIT.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record),flush=True)


if __name__=='__main__':main()
