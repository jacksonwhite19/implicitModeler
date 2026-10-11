"""Private numerical audit consumer of its own call-local sampled normals.

The final audit alone supplies these arrays while its input/source custody is
active. This creates no reusable proof, external receipt or acceptance owner.
Other geometry/feature owners retain the stock provider's operation.
"""
import numpy as np
from general_sdf_authority import (GeneralSdfAuthority, _STANDARD_EXACT_NORMAL,
    _STANDARD_NUMERICAL_SAMPLES)
from historical_capsule_surface import provider

_NORMAL_OWNER=_STANDARD_EXACT_NORMAL
_SAMPLE_OWNER=_STANDARD_NUMERICAL_SAMPLES


def topology_report(points,faces,face_ref,curve_edges,authority,samples,centres):
    if (type(authority) is not GeneralSdfAuthority or not authority.numerical_acceptance_active()
            or getattr(authority.exact_normal,'__func__',None) is not _NORMAL_OWNER
            or getattr(authority.numerical_surface_samples,'__func__',None) is not _SAMPLE_OWNER
            or len(curve_edges) or samples is None):
        return provider.topology_report(points,faces,face_ref,curve_edges,None,authority)
    authority.verify()
    q=points[faces]
    expected=(q[:,0]+q[:,1]+q[:,2])/3.0
    if expected.tobytes()!=centres.tobytes():
        return provider.topology_report(points,faces,face_ref,curve_edges,None,authority)
    normals=samples['normals'][len(points)+samples['edge_count']:]
    if (normals.shape!=(len(faces),3) or samples['vertex_count']!=len(points)
            or set(face_ref)!={authority._root_ref}):raise ValueError('DetachedNumericalOwnershipSamples')
    directed=np.concatenate((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]))
    _,counts=np.unique(np.sort(directed,axis=1),axis=0,return_counts=True)
    duplicate_faces=len(faces)-len(np.unique(np.sort(faces,axis=1),axis=0))
    cross=np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0])
    # Retain the scalar reductions and short-circuit predicate used by V3.
    orientation_failures=sum(np.linalg.norm(n)==0 or np.dot(n,normal)<=0
                             for n,normal in zip(cross,normals))
    authority.verify()
    row=dict(open_edges=int(np.count_nonzero(counts==1)),overfull_edges=int(np.count_nonzero(counts>2)),
        duplicate_faces=duplicate_faces,orientation_failures=int(orientation_failures),
        curve_graph_failures=0,persistent_curve_count=0)
    row['topology_pass']=not any(row[k] for k in ('open_edges','overfull_edges','duplicate_faces',
                                                'orientation_failures','curve_graph_failures'))
    return row
