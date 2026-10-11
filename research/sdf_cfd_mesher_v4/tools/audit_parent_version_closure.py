"""Qualification-only proof that version keys belong to existing geometry."""
from pathlib import Path
import capsule_user_domain_handoff as h
import profile_prism_assembly as old
import qualify_recursive_layers9to11 as p
import recursive_epoch_recovery as recovery
from prism_persistent_views import persistent_component_views

def bind_for_state(base,numeric_root,path):
    """Existing exact version-closure check, independent of the study paths."""
    allowed={'cell:'+key for key in base['cells']}|{'face:'+key for key in base['faces']}
    keys={key for key in base['versions'] if key.startswith(('cell:','face:'))}
    if not keys<=allowed:raise ValueError('VersionWithoutGeometry')
    body=dict(schema='V4GeometryVersionClosure/1',parent_content=base.content_root,numeric_root=numeric_root,
        version_keys=len(keys),geometry_keys=len(allowed),orphan_versions=0,closed=True)
    h.durable_json(path,body)
    return h.identity(body)


if __name__=='__main__':
    out=old.OUT.parent/'persistent_publication_optimization'
    prior=h.read(out/'PARENT_INCIDENCE_AUTHORITY.json');tick=h.measured()
    with p.CommittedState(prior['source']) as base,persistent_component_views(base):
        root=bind_for_state(base,prior['root'],out/'PARENT_VERSION_CLOSURE.json')
        h.durable_json(out/'PARENT_VERSION_AUDIT.json',dict(root=root,metrics=h.metrics(*tick)))
