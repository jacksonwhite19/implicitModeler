"""Isolated complete metric profile plus bounded representation-overhead probes."""
from task4f_common import *
from task4f_instrument import Timers,install,same_arrays
import task4_prepare_next as prod
import sdf_cfd_mesher_v4.forecast.detailed.patch as patch_module
import psutil

def closure_once(patch,topology):
    """Identical closure conditions; hoist immutable membership construction."""
    E=patch_module.IncompleteStencil
    if patch.topology_identity!=topology.sha256:raise E('TopologyIdentityMismatch')
    expected=np.unique(topology.triangle_edges[patch.core_columns])
    if not np.array_equal(expected,patch.side_edges):raise E('MissingCoreSide')
    columns=set(patch.columns)
    for e in expected:
        owners=topology.edge_triangles.row(e)
        if len(owners)!=2 or not set(owners).issubset(columns):raise E('MissingSideNeighbor')
    if not set(np.unique(topology.triangles[patch.columns])).issubset(set(patch.vertices)):raise E('MissingVertex')

def bound_gain_only(m):
    # Exact preserved operator_model.gains G_bound expression, no new definition.
    return dict(G_bound=np.linalg.svd(m['B']@m['Pb'],compute_uv=False)[:,0])

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();top=prod.build_topology(front.triangles,len(front.coordinates))
    _,coords,_=prod.load_candidate(BASE/'LAYER3_CANDIDATE.npz',BASE/'INITIAL_FORECAST.npz')
    o=next(o for o in read(BASE/'LAYER3_CORRECTIONS.json') if o['patch']==4);p=prod.bounded_patch(top,np.array(o['scope']['active'])).detailed
    f=prod.SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords});t=Timers();original,met=install(t)
    t.enabled=False;a=time.perf_counter();plain=met.measure(front,f,top,p);old_seconds=time.perf_counter()-a
    t.rows.clear();t.enabled=True;profiled=met.measure(front,f,top,p);assert same_arrays(plain,profiled)
    save('METRIC_CALL_TREE.json',t.table());t.enabled=False
    # Direct uninstrumented timings for both small implementation-only ideas.
    a=time.perf_counter();met.require_closure(p,top);closure_old=time.perf_counter()-a
    a=time.perf_counter();closure_once(p,top);closure_new=time.perf_counter()-a
    op=met.operator_module();oldg=op.gains;captured=[];oldm=op.matrices
    def capture(sf,wall):
        r=oldm(sf,wall);captured.append(r);return r
    op.matrices=capture
    try:
        op.gains=bound_gain_only;a=time.perf_counter();narrow=met.measure(front,f,top,p);narrow_seconds=time.perf_counter()-a
    finally:op.gains=oldg;op.matrices=oldm
    assert same_arrays(plain,narrow),'UnconsumedGainRemovalChangedOutput'
    m=captured[0];a=time.perf_counter();allg=oldg(m);full_gain_seconds=time.perf_counter()-a
    a=time.perf_counter();one=bound_gain_only(m);one_gain_seconds=time.perf_counter()-a
    assert same_arrays(allg['G_bound'],one['G_bound'])
    # Both closure calls in measure are exercised, preserving every check.
    oldclosure=met.require_closure;oldpatchclosure=patch_module.require_closure
    try:
        met.require_closure=closure_once;patch_module.require_closure=closure_once
        a=time.perf_counter();fastclosure=met.measure(front,f,top,p);hoisted_seconds=time.perf_counter()-a
    finally:met.require_closure=oldclosure;patch_module.require_closure=oldpatchclosure
    assert same_arrays(plain,fastclosure)
    save('METRIC_PROFILE.json',dict(layer=3,patch=4,columns=len(p.columns),core_columns=len(p.core_columns),cells=20*len(p.columns),faces=len(plain['face_ids']),
        plain_seconds=old_seconds,instrumentation_equal=True,scientific_hash=scientific_hash(plain),
        closure=dict(old_seconds=closure_old,hoisted_seconds=closure_new,column_set_constructions_per_measure=2*len(p.side_edges),column_members_inserted_per_measure=2*len(p.side_edges)*len(p.columns),new_constructions_per_measure=2,full_measure_hoisted_seconds=hoisted_seconds,exact_equality=True),
        gains=dict(old_seconds=full_gain_seconds,only_required_G_bound_seconds=one_gain_seconds,svd_batches_before=10,svd_batches_required_by_outputs=2,full_measure_narrow_gain_seconds=narrow_seconds,exact_equality=True,unconsumed_outputs=sorted(set(allg)-{'G_bound'})),
        peak_rss=psutil.Process().memory_info().peak_wset,seconds=time.perf_counter()-start,production_integrated=False))

if __name__=='__main__':run()
