"""Deep source-row equivalence qualification for the immutable face lookup."""
from pathlib import Path
import capsule_user_domain_handoff as h
import qualify_recursive_layers9to11 as production
from prism_persistent_views import persistent_component_views
from persistent_numeric_incidence import open_geometry
from prism_source_lookup import write_lookup,source_face

if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'results/persistent_publication_optimization'
    auth=h.read(out/'PARENT_INCIDENCE_AUTHORITY.json');tick=h.measured()
    ref=dict(directory=str(out/'parent_incidence'),root=auth['root'],content=auth['parent_content'])
    with production.CommittedState(auth['source']) as base,persistent_component_views(base),open_geometry(ref,base.content_root) as packed:
        adapter,_=production.inputs(base);n=len(adapter.front.coordinates)
        from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
        count=0
        for _,rows in packed.ordered_items():
            row=source_face(rows,n);key=face_key(row['vertices'])
            if row!=base['faces'][key]:raise ValueError('WrongScientificFaceRow')
            count+=1
        lookup=write_lookup(out/'parent_face_lookup',packed.ordered_items(),n,packed.root,base.content_root)
        h.durable_json(out/'PARENT_LOOKUP_AUTHORITY.json',dict(reference=lookup,numeric_root=packed.root,
            parent_content=base.content_root,rows_deep_compared=count,metrics=h.metrics(*tick)))
