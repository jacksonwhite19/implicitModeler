"""Bounded conforming subdivision checks; final surface authority is unchanged."""
from collections import defaultdict
import numpy as np

def refinement_edges(vertices,face,policy):
    ids=face.vertices
    rows=[(tuple(sorted((ids[i],ids[(i+1)%3]))),float(np.linalg.norm(
        vertices[ids[i]].point-vertices[ids[(i+1)%3]].point))) for i in range(3)]
    if min(length for _,length in rows)>policy.minimum_edge_m:return [key for key,_ in rows]
    eligible=[(length,key) for key,length in rows if length>policy.minimum_edge_m]
    # A protected short edge does not forbid subdivision along a long edge.
    # Select only one to preserve the short edge, with deterministic ties.
    return [max(eligible)[1]] if eligible else []

def validate_split(authority,old_vertices,old_faces,vertices,faces,changed):
    from audit_adaptive_surface import topology
    x=np.asarray([v.point for v in vertices]);f=np.asarray([v.vertices for v in faces],dtype=np.int64)
    before=topology(np.asarray([v.point for v in old_vertices]),np.asarray([v.vertices for v in old_faces]),
                    expected_components=None,expected_euler=None)
    after=topology(x,f,expected_components=before['connected_components'],expected_euler=before['euler'])
    if not before['passed'] or not after['passed']:raise ValueError('CarrierSplitTopologyFailure',before,after)
    maximum_residual=0.
    for index in range(len(old_vertices),len(vertices)):
        v=vertices[index];residual=abs(float(authority.implicit_value(v.sheet_ref,v.point)))
        if not np.isfinite(residual) or residual>1e-10:raise ValueError('CarrierSplitProjectionFailure',index,residual)
        maximum_residual=max(maximum_residual,residual)
    affected=set()
    for index in changed:
        face=faces[index];p=x[list(face.vertices)];cross=np.cross(p[1]-p[0],p[2]-p[0])
        if not np.isfinite(p).all() or not np.isfinite(cross).all() or not np.linalg.norm(cross)>0:
            raise ValueError('CarrierSplitDegenerateChild',index)
        if not cross@authority.exact_normal(face.sheet_ref,p.mean(0))>0:
            raise ValueError('CarrierSplitOrientationFailure',index)
        affected.update(face.vertices)
    links={v:[] for v in affected}
    for a,b,c in f:
        for v,left,right in ((a,b,c),(b,c,a),(c,a,b)):
            if v in links:links[v].append((left,right))
    for vertex,edges in links.items():
        adjacency=defaultdict(set)
        for a,b in edges:adjacency[a].add(b);adjacency[b].add(a)
        seen=set();todo=[next(iter(adjacency))]
        while todo:
            v=todo.pop()
            if v not in seen:seen.add(v);todo.extend(adjacency[v]-seen)
        if (len(set(tuple(sorted(e)) for e in edges))!=len(edges) or len(seen)!=len(adjacency)
                or any(len(v)!=2 for v in adjacency.values())):raise ValueError('CarrierSplitLinkFailure',int(vertex))
    return dict(before=before,after=after,affected_links_checked=len(links),
                changed_faces_checked=len(changed),maximum_new_vertex_residual_m=maximum_residual)
