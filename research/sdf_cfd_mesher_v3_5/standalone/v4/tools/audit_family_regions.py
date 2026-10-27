"""Measure cavity source support from wall connectivity, without fixture IDs."""
import sys,json
from run_geometry_family import OUT,g
from summarize_geometry_family import committed_history
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore

def run(name,label):
    folder=OUT/name/label;state=NativeCheckpointStore(folder/'checkpoints').restart()
    parent={k:k for k in state['wall']['vertices']}
    def root(k):
        while parent[k]!=k:parent[k]=parent[parent[k]];k=parent[k]
        return k
    for face in state['wall']['triangles'].values():
        for v in face[1:]:
            a,b=root(face[0]),root(v);parent[max(a,b)]=min(a,b)
    roots=sorted({root(k) for k in parent});labels={k:i for i,k in enumerate(roots)}
    component={k:labels[root(k)] for k in parent};events=[]
    for key,event in state['events'].items():
        cavity=event['cavity_manifest'];last=next(r for r in reversed(cavity['expansion']) if 'active' in r)
        touched=sorted({component[str(v)] for v in last['active']})
        events.append(dict(event=key,wall_components=touched,cavity_vertices=len(last['active']),removed_cells=len(cavity['removed_cells']),retained_layer=last['depth']))
    envelopes={}
    for path in (OUT/name).glob('*/checkpoints/NATIVE_*.json'):
        value=json.loads(path.read_text());envelopes[value['scientific']]=(path,value)
    actual=[]
    for op in committed_history(folder,'OPERATIONS.json'):
        if op['operation']!='RollbackRemesh':continue
        def cells(key):
            path,envelope=envelopes[key]
            return json.loads((path.parent/'objects'/envelope['components']['cells']['file']).read_text())
        before=cells(op['policy']['context']['base']);after=cells(op['state'])
        changed=[c for k,c in before.items() if after.get(k)!=c]+[c for k,c in after.items() if before.get(k)!=c]
        components=sorted({component[state['wall']['triangles'][c['source']][0]] for c in changed})
        actual.append(dict(event=op['event']['event'],components=components,changed_cell_records=len(changed)))
    result=dict(actual_topology_changes=actual,max_actual_components=max((len(r['components']) for r in actual),default=0),wall_connected_components=len(roots),events=events,max_components_per_event=max((len(e['wall_components']) for e in events),default=0),
        scope='Connectivity-derived source support. Component labels are diagnostic only and never select production behavior.')
    g.save(folder/'REGION_INTERACTIONS.json',result)

if __name__=='__main__':run(*sys.argv[1:])
