"""Study-only timers. AST wrapping preserves every original expression/order."""
from task4f_common import *
import ast,inspect,contextlib,functools

class Timers:
    def __init__(self):self.rows={};self.stack=[];self.enabled=True
    @contextlib.contextmanager
    def span(self,name,entities=0):
        if not self.enabled:yield;return
        a=time.perf_counter();cpu=time.process_time();entry=[0.];self.stack.append(entry)
        try:yield
        finally:
            dt=time.perf_counter()-a;self.stack.pop()
            if self.stack:self.stack[-1][0]+=dt
            r=self.rows.setdefault(name,dict(calls=0,total_seconds=0.,self_seconds=0.,cpu_seconds=0.,maximum_seconds=0.,entities=0))
            r['calls']+=1;r['total_seconds']+=dt;r['self_seconds']+=dt-entry[0];r['cpu_seconds']+=time.process_time()-cpu;r['maximum_seconds']=max(r['maximum_seconds'],dt);r['entities']+=int(entities)
    def wrap(self,obj,name,label=None,entities=lambda a,k:len(a[0]) if a else 0):
        old=getattr(obj,name)
        @functools.wraps(old)
        def f(*a,**k):
            with self.span(label or name,entities(a,k)):return old(*a,**k)
        setattr(obj,name,f);return old
    def table(self):return [dict(function=k,average_seconds=v['total_seconds']/v['calls'],**v) for k,v in sorted(self.rows.items())]

def instrument_statements(fn,timers,prefix,labels):
    """Wrap consecutive top-level statements by explicit source-text labels."""
    src=inspect.getsource(fn);tree=ast.parse(src);f=tree.body[0];body=[]
    for node in f.body:
        segment=ast.get_source_segment(src,node) or ''
        label=next((label for needle,label in labels if segment.startswith(needle)),None)
        if label:
            ctx=ast.Call(func=ast.Attribute(value=ast.Name(id='_audit_timer',ctx=ast.Load()),attr='span',ctx=ast.Load()),args=[ast.Constant(prefix+'.'+label)],keywords=[])
            body.append(ast.With(items=[ast.withitem(context_expr=ctx)],body=[node]))
        else:body.append(node)
    f.body=body;ast.fix_missing_locations(tree);env=dict(fn.__globals__);env['_audit_timer']=timers
    exec(compile(tree,inspect.getsourcefile(fn)+':audit','exec'),env)
    return env[fn.__name__]

def install(t):
    import sdf_cfd_mesher_v4.forecast.detailed.metrics as met
    import sdf_cfd_mesher_v4.forecast.detailed.patch as patch
    m=met.legacy_modules();cq=m['component_quality'];fg=cq.FG;pm=cq.PM;op=met.operator_module()
    for obj,names,pre in [(fg,['prism_cell_centres','face_area_centre_quad','face_area_centre_tri','internal_face_skewness'],'geometry'),(pm,['_exact_wedge_jacobian_and_volume','_foundation_determinant'],'cell'),(cq,['determinant'],'cell'),(op,['matrices','gains'],'operator')]:
        for name in names:
            t.wrap(obj,name,pre+'.'+name,lambda a,k:len(a[0]['N']) if isinstance(a[0],dict) and 'N' in a[0] else len(a[0]))
    cq.face_metrics=instrument_statements(cq.face_metrics,t,'face',[
        ('da=','weight_distances'),('w=','interpolation_weight'),('sk=','skewness'),('vr=','volume_ratio'),('d=','nonorthogonality_direction'),('cos=','nonorthogonality_cosine'),('return','nonorthogonality_angle')])
    t.wrap(cq,'face_metrics','face.all')
    original=met.cell_metrics
    met.cell_metrics=instrument_statements(original,t,'cell',[
        ('for ids','fan_geometry'),('sf=np.stack','fan_stack'),('area=','weighted_normal_area'),('sw=','weighted_sigma_svd'),
        ('try:','all_gain_operators'),('C,V=','centres_volume'),('jac,vol=','analytic_wedge'),('for layer','determinants'),
        ('d=','direction'),('d/=','direction_normalization'),('pn=','side_normal_projection'),('pn/=','side_normal_normalization'),
        ('sep=','side_normal_separation'),('B=','aspect_basis'),('aspect=','aspect'),('return','output')])
    t.wrap(met,'cell_metrics','cell.all')
    old=patch.require_closure
    def closure(p,top):
        with t.span('topology.require_closure',len(p.columns)):return old(p,top)
    patch.require_closure=closure;met.require_closure=closure
    t.wrap(met,'prisms','topology.materialize_prisms',lambda a,k:20*len(a[3].columns))
    t.wrap(met,'measure','detail.measure',lambda a,k:20*len(a[3].columns))
    return original,met

def same_arrays(a,b):
    if isinstance(a,dict):return a.keys()==b.keys() and all(same_arrays(a[k],b[k]) for k in a)
    if isinstance(a,np.ndarray):return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
    if isinstance(a,(list,tuple)):return len(a)==len(b) and all(same_arrays(x,y) for x,y in zip(a,b))
    return a==b

def dependency_counts(top,patch,old_x,new_x):
    # Coordinates are local 21-front arrays. Exact bit identity, no tolerance.
    changed=np.any(old_x.view(np.uint64)!=new_x.view(np.uint64),axis=2)
    tri=np.searchsorted(patch.vertices,top.triangles[patch.columns])
    cells=np.any(changed[:-1,tri]|changed[1:,tri],axis=2)
    own=np.array([top.edge_triangles.row(e) for e in patch.side_edges]);ix=np.searchsorted(patch.columns,own)
    side=cells[:,ix].any(2);core=np.searchsorted(patch.columns,patch.core_columns)
    caps=cells[:-1,core]|cells[1:,core]
    edges=np.unique(top.triangle_edges[patch.columns]);ev=np.searchsorted(patch.vertices,top.edges[edges]);valid=(ev<len(patch.vertices)).all(1)
    edge_changes=changed[1:,ev[valid]].any(2)
    return dict(changed_forecast_vertex_states=int(changed[1:].sum()),changed_source_vertices=int(changed.any(0).sum()),changed_edge_states=int(edge_changes.sum()),
        changed_triangle_front_states=int(changed[1:,tri].any(2).sum()),changed_prism_cells=int(cells.sum()),recomputed_prism_cells=int(cells.size),
        unchanged_prism_cells_recomputed=int(cells.size-cells.sum()),changed_face_metric_dependencies=int(side.sum()+caps.sum()),
        recomputed_face_metrics=int(side.size+caps.size),changed_cell_operator_rows=int(5*cells.sum()),recomputed_cell_operator_rows=int(5*cells.size),
        cell_redundancy_ratio=float(cells.size/cells.sum()) if cells.any() else None),cells
