"""Bounded coupled screening of unchanged, sealed cohort geometries."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import json,time
import numpy as np
import psutil
import coupled_shell_validation as c
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.complete_neighbor_receipt import bind_complete_neighbor,verify_complete_neighbor
from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/coupled_shell_remesh_determinant'


def run(name):
    tick=time.perf_counter();prior=ROOT/'results/broader_mixed_geometry_end_to_end'/name/'A'
    state=NativeCheckpointStore(prior/'checkpoints').restart()
    keys=sorted(state['vertices']);index={v:i for i,v in enumerate(keys)}
    p=np.array([state['vertices'][k] for k in keys])
    cells=[dict(id=int(k),column=int(v['source']),layer=v['layer'],kind=v['kind'],
        vertices=[index[x] for x in v['vertices']],faces=[tuple(index[x] for x in f) for f in v['faces']]) for k,v in state['cells'].items()]
    # Actual exposed topology, not one-face-per-column reconstruction.
    outer=[tuple(index[x] for x in state['faces'][k]['vertices']) for k in state['outer']]
    before=identity(geometry_inputs(state));checked=c.check(p,cells,outer,True)
    folder=OUT/'regression'/name;folder.mkdir(parents=True,exist_ok=True)
    assert checked['passed'],checked['failures']
    if checked['closures']:
        c.qualify_closures(folder,'final',p,cells,{'complete_neighbor':checked})
        receipt=bind_complete_neighbor(state,checked)
        assert verify_complete_neighbor(state,receipt,checked)
        c.g.save(folder/'RECEIPT.json',receipt)
    assert before==identity(geometry_inputs(state))
    assembly=json.loads((prior/'ASSEMBLY.json').read_text());assert assembly['passed']
    c.g.save(folder/'RESULT.json',dict(passed=True,geometry_exact=True,closures=checked['closures'],
        coupled_failures=0,revisions=0,geometry_identity=before,
        complete_geometry_reused=file_hash(prior/'ASSEMBLY.json'),
        complete_relations=assembly['total_relations'],scope='Fresh bounded coupled screening; unchanged final core and complete certificate reused',
        seconds=time.perf_counter()-tick,peak_rss_bytes=psutil.Process().memory_info().peak_wset))


if __name__=='__main__':
    if sys.argv[1]=='SF2':
        import broader_sf2_control as sf
        sf.OUT=OUT;sf.run()
    else:run(sys.argv[1])
