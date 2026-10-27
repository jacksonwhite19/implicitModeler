"""Explicit parent numeric-index construction and full reconstruction audit."""
from pathlib import Path
import numpy as np
import capsule_user_domain_handoff as h
import profile_prism_assembly as old
import recursive_epoch_recovery as recovery
import qualify_recursive_layers9to11 as production
from prism_persistent_views import persistent_component_views
from persistent_numeric_incidence import write_segment,PackedSegment

OUT=old.OUT.parent/'persistent_publication_optimization'


def build_for_state(base, triangles, vertex_count, out):
    """Existing deep index qualification with explicit case inputs; no advance."""
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    tick=h.measured();n=vertex_count
    def index(v):
        parts=v.split(':');return int(parts[1])*n+int(parts[3])
    from prism_hotpath_v2 import prism_cell,reverse_cycle
    import local_termination_topology as topology
    cells=[]
    for key in sorted(base['cells'],key=int):
        row=base['cells'][key]
        if row['kind']!='prism':raise ValueError('NonPrismPersistentBootstrap')
        cells.append(prism_cell([index(v) for v in row['vertices']],int(row['source']),int(row['layer'])))
    owners,audit=topology.ownership(cells,[tuple(map(int,f+base.layer*n)) for f in triangles],
        [tuple(reversed(tuple(map(int,f)))) for f in triangles])
    if not audit['passed']:raise ValueError('ParentTopologyRejected')
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import cycle,face_key
    for key,values in owners.items():
        name=face_key(tuple(f'layer:{v//n}:source:{v%n}' for v in values[0][1]))
        row=base['faces'][name]
        if set(map(int,row['owners']))!={owner for owner,_ in values}:raise ValueError('WrongScientificOwners')
        for owner,face in values:
            if cycle(tuple(index(v) for v in row['owners'][str(owner)]))!=cycle(face):raise ValueError('WrongScientificWinding')
    root=write_segment(out/'parent_incidence',owners,base.content_root)
    p=out/'parent_incidence/cells.npy'
    with p.open('wb') as stream:
        np.save(stream,np.array([[c['id'],c['column'],c['layer'],*c['vertices']] for c in cells],dtype='<i8'),allow_pickle=False)
        stream.flush()
        import os
        os.fsync(stream.fileno())
    import immutable_state_access as access
    body=access.read(out/'parent_incidence/MANIFEST.json')
    body['files']['cells']=dict(path=str(p.resolve()),**access.attest(p,h.file_hash(p)))
    body['topology']=audit
    h.durable_json(out/'parent_incidence/MANIFEST.json',body);root=h.identity(body)
    with PackedSegment(out/'parent_incidence',root,base.content_root) as packed:
        # Deep qualification against independently read scientific face rows.
        assert len(packed)==len(owners)
        assert all(packed[k]==v for k,v in owners.items())
        audit=packed.deep_verify()
    h.durable_json(out/'PARENT_INCIDENCE_AUTHORITY.json',dict(root=root,parent_content=base.content_root,
        source=base.reference,deep=audit,metrics=h.metrics(*tick),scientific_faces_checked=len(owners)))
    return h.read(out/'PARENT_INCIDENCE_AUTHORITY.json')


def build():
    ref,_,_=production.parent(11);record=recovery.discover(old.OUT/'epoch_access')
    renewed=next(n['reference'] for n in record['nodes'] if n['source']==ref)
    with production.CommittedState(renewed) as base,persistent_component_views(base):
        adapter,_=production.inputs(base)
        return build_for_state(base,adapter.topology.triangles,len(adapter.front.coordinates),OUT)

if __name__=='__main__':build()
