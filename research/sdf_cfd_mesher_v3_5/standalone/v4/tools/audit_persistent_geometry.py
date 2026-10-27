"""Qualification-only full reconstruction of the private numeric closure."""
from pathlib import Path
import sys
import capsule_user_domain_handoff as h
from persistent_numeric_incidence import open_geometry,CellSequence
import local_termination_topology as topology
from prism_hotpath_v2 import reverse_cycle
import numpy as np

OUT=Path(__file__).resolve().parents[1]/'results/persistent_publication_optimization'

def audit(mode):
    folder=OUT/('normal_'+mode+'_prism');plan=h.read(folder/'CANDIDATE_PLAN.json');binding=plan['persistent_geometry']
    tick=h.measured()
    topology.reverse_cycle=reverse_cycle
    with open_geometry(binding,plan['scientific']) as index:
        cells=list(CellSequence(index))
        top=max(c['layer'] for c in cells)
        last=[c for c in cells if c['layer']==top]
        first=[c for c in cells if c['layer']==1]
        outer=[tuple(c['vertices'][3:]) for c in last]
        wall=[tuple(reversed(c['vertices'][:3])) for c in first]
        owners,summary=topology.ownership(cells,outer,wall)
        if summary!=binding['topology']:raise ValueError('DeepTopologySummaryMismatch')
        count=0
        for key,rows in index.ordered_items():
            if owners.pop(key,None)!=rows:raise ValueError('DeepOwnershipMismatch')
            count+=1
        if owners:raise ValueError('DeepMissingIncidence')
        ids=[c['id'] for c in cells]
        if len(set(ids))!=len(cells):raise ValueError('DeepDuplicateCell')
        expected=h.read(OUT.parent/'recursive_layers9to11/L11/GENERATION.json')
        if summary!=expected['topology_audit'] or len(cells)!=expected['counts']['combined_cells']:raise ValueError('WrongQualifiedTopology')
        from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
        delta=h.read(folder/'GEOMETRY_DELTA.json')
        for key,row in delta['faces'].items():
            if index.scientific_face(key)!=row:raise ValueError('DeepSourceLookupMismatch')
        expected_versions={'cell:'+k for k in delta['cells']}|{'face:'+k for k in delta['faces']}
        if set(delta['versions'])!=expected_versions:raise ValueError('DeepVersionClosureMismatch')
        volume=h.read(folder/'VOLUME_DELTA.json');sealed=h.read(OUT.parent/'recursive_layers9to11/L11/VOLUME_DELTA.json')
        if volume!=sealed or not volume['exact_balance'] or not volume['global_degree']['passed']:raise ValueError('DeepGlobalMismatch')
        quality=h.read(folder/'QUALITY_DELTA.json')
        if len(quality)!=219609 or len({r['key'] for r in quality})!=len(quality):raise ValueError('DeepQualityPopulationMismatch')
        result=dict(passed=True,faces=count,cells=len(cells),summary=summary,new_quality_delta_rows=len(quality),
            inherited_quality_count_source='Unchanged sealed component authority; complete changed quality independently compared',
            degree_volume_equal=True,version_closure=True,source_face_rows_checked=len(delta['faces']),metrics=h.metrics(*tick),
            scope='Full numeric cell/face reconstruction; exact complete ownership and summary comparison; separate from normal timing')
    h.durable_json(OUT/('DEEP_'+mode+'.json'),result)

if __name__=='__main__':audit(sys.argv[1])
