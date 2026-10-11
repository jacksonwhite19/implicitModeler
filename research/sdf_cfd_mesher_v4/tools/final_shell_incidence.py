"""Prospective core incidence for fixed shell cell area tensors.

This does not supply core centroids, face weights, or a core certificate. It
uses the established Foundation tensor exactly, including every outer face
that the declared closed core handoff will make internal.
"""
import generalize_local_remesh as g

def check(points,cells,outer,*,active_cells=None):
    from collections import Counter
    counts=Counter(tuple(sorted(f)) for c in cells for f in c['faces'])
    interface={tuple(sorted(f)) for f in outer};rows=[];failures=[]
    # Use the retained Foundation batch kernels only for homogeneous native
    # prism rows. Preserve custom geometry owners and general/mixed inputs.
    fast=(g.t.geometry is g.t._STANDARD_GEOMETRY and type(points) is g.np.ndarray
          and points.dtype==g.np.dtype('float64') and points.ndim==2
          and points.shape[1]==3 and type(cells) is list and bool(cells)
          and (active_cells is None or type(active_cells) in (set,frozenset))
          and all(type(c) is dict and type(c.get('id')) is int
                  and c.get('kind')=='prism' and len(c.get('faces',()))==5
                  and all(len(f) in (3,4) for f in c['faces']) for c in cells)
          and len({c['id'] for c in cells})==len(cells)
          and all(tuple(map(len,c['faces']))==tuple(map(len,cells[0]['faces']))
                  for c in cells))
    if fast:
        from prism_hotpath_v2 import batch_geometry
        selected=[c for c in cells if active_cells is None or c['id'] in active_cells]
        def geometries():
            for start in range(0,len(selected),4096):
                batch=selected[start:start+4096];values=batch_geometry(points,batch)
                for c in batch:yield c,values[c['id']]
    else:
        def geometries():
            for c in cells:
                if active_cells is None or c['id'] in active_cells:
                    yield c,g.t.geometry(points,c)
    for c,geometry in geometries():
        mask=g.np.array([counts[tuple(sorted(f))]==2 or tuple(sorted(f)) in interface for f in c['faces']])
        S=geometry['areas'][mask]
        avg=g.np.linalg.norm(S,axis=1).mean();tensor=(S/avg).T@(S/avg)
        determinant=float(g.np.linalg.det(tensor))
        row=dict(id=c['id'],column=c['column'],cell_type=c['kind'],determinant=determinant)
        rows.append(row)
        if not determinant>=.0011:failures.append(dict(kind='final_incidence_determinant',**row))
    return dict(passed=not failures,cells=rows,failures=failures,
        scope='Cell determinant only, with every declared shell/core interface face internal; actual core neighbour metrics still required')


def merge(checked,points,cells,outer):
    result=check(points,cells,outer);checked['complete_incidence']=result
    if not result['passed']:
        checked['quality']['failures']+=result['failures']
        checked['quality']['passed']=False;checked['passed']=False
    return checked
