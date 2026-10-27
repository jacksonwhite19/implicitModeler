"""One-time rectangular checkpoint import into native mixed entity blocks."""
from pathlib import Path
import json,gc
import numpy as np
from ..common.authority import array_hash,identity,file_hash
from .native_checkpoint import NativeCheckpointStore
from .native_prism_entities import prism_views,write_array
from .mixed_front import VERSION,wall_authority
from .rolling_cache import load_candidate
from .legacy_mixed_migration import read_checkpoint_prefix


def import_prism_checkpoint(store,layer,destination,verified_inventory,native_manifest):
    """Legacy store exists only within this import call; no provider retains it."""
    checkpoint=NativeCheckpointStore(destination);objects=checkpoint.objects
    if (checkpoint.path/'CURRENT.json').exists():raise ValueError('ImportDestinationAlreadyPublished')
    rows,prefix=read_checkpoint_prefix(store,layer);front=store.front;top=store.top;run=store.path.parent
    for k,row in enumerate(verified_inventory['layers'][:layer]):
        if array_hash(prefix[k])!=row['coordinates']:raise ValueError('PrismImportInventoryMismatch')
    wall=wall_authority({str(i):x.tolist() for i,x in enumerate(front.coordinates)},
                        {str(i):t.tolist() for i,t in enumerate(top.triangles)})
    views=prism_views(objects,[front.coordinates,*prefix],top.triangles,top.edges)
    state=dict(version=VERSION,wall=wall,first_layer={str(i):x.tolist() for i,x in enumerate(prefix[0])},
        **views,outer=[f'cap:{layer}:{i}' for i in range(len(top.triangles))],
        authority=dict(geometry=front.geometry_authority,surface=front.surface_authority,
            source_ids=list(front.vertex_ids),source_ancestry=[list(x) for x in front.source_ancestry],
            schedule=array_hash(store.schedule.targets)),versions=dict(topology=0,geometry=layer),events={})
    # Nominal schedules share immutable backing; adjusted schedules remain
    # explicit per-source exceptions instead of 53,153 duplicate lists.
    state['local_schedule']=dict(nominal=store.schedule.targets.tolist(),actual={},default='nominal')
    state['quality']={f'layer:{k}':row for k,row in enumerate(verified_inventory['quality'][:layer],1)}
    for k in range(1,layer+1):
        row=dict(state['quality'][f'layer:{k}'])
        with np.load(run/f'LAYER_{k:02}_CONDITIONING.npz') as z:
            row['conditioning']={name:write_array(objects,z[name]) for name in z.files}
        state['quality'][f'layer:{k}']=row
    manifest=native_manifest.metadata()
    state['certificate']={f'layer:{k}':row for k,row in enumerate(manifest['blocks'][:layer],1)}
    state['authority']['native_manifest']=dict(source_edges=manifest['source_edges'],scientific=manifest['scientific'])
    candidate,coordinates,normals=load_candidate(run/f'LAYER{layer}_CANDIDATE.npz',run/'INITIAL_FORECAST.npz')
    if not np.array_equal(candidate.view(np.uint64),prefix[-1].view(np.uint64)) or array_hash(coordinates)!=rows[-1]['forecast_key']:raise ValueError('PrismImportForecastMismatch')
    state['forecast']=dict(coordinates=write_array(objects,coordinates),normals=write_array(objects,normals),
        current_layer=layer,source_vertices=list(front.vertex_ids),topology=array_hash(top.triangles),
        warning_evidence=rows[-1]['warnings'],corrections=rows[-1]['corrections'])
    with np.load(run/f'LAYER{layer}_ATLAS.npz') as z:
        meta=json.loads(str(z['metadata']))
        state['atlas']=dict(metadata=meta,scientific=str(z['scientific_hash']),
            arrays={name:write_array(objects,z[name]) for name in ('states','generations','last_evaluated')})
    checkpoint.publish(state)
    del state,prefix,coordinates,normals,candidate;gc.collect()
    return checkpoint.restart()
