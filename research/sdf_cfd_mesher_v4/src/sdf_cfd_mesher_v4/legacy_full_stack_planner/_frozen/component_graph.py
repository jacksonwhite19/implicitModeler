"""Deterministic ownership of controlled geometry and immutable context."""
from common import *
import numpy as np
def connected_owners(controlled,expression_supports):
 v=np.unique(controlled);where={int(a):i for i,a in enumerate(v)};parent=np.arange(len(v))
 def root(i):
  while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
  return i
 edges=[]
 for support in expression_supports:
  ids=[where[int(a)] for a in sorted(set(support)) if int(a) in where]
  for b in ids[1:]:
   a0,b0=root(ids[0]),root(b)
   if a0!=b0:parent[max(a0,b0)]=min(a0,b0)
   edges.append((int(v[ids[0]]),int(v[b])))
 groups={}
 for i,a in enumerate(v):groups.setdefault(root(i),[]).append(int(a))
 out=sorted(groups.values(),key=lambda z:tuple(z))
 return out,sorted(set(edges))
def topology_supports(faces,edge_faces):
 return [list(map(int,row)) for row in faces]+[np.unique(faces[p]).tolist() for p in edge_faces]
def assert_context_immutable(owned,context,all_owned):
 foreign=set(context)-set(owned)
 assert not foreign.intersection(all_owned-set(owned)),'CrossComponentMutableContext'
def discover():
 from component_quality import load_state,global_failure_rows,context,FG,target_columns
 started=time.perf_counter();s,nom,old,_=load_state('baseline');state_rows={};changed=set()
 for name in ('baseline','patch000','patch019'):
  ss,nn,_,dv=load_state(name);rows=global_failure_rows(ss,nn);state_rows[name]=rows;changed.update(map(int,dv))
  write(D/'discovery'/f'{name}_physical_failures.json',rows);print(name,'physical_failure_rows',len(rows),flush=True)
 targets=target_columns();directions=set(map(int,s.faces[targets].ravel()))|changed
 physical=set(v for rows in state_rows.values() for r in rows for v in r['vertices']);controlled=sorted(directions|physical)
 e,ef=FG.unique_manifold_edges(s.faces);supports=topology_supports(s.faces,ef);groups,edges=connected_owners(controlled,supports)
 records=[];all_owned=set(controlled)
 for i,group in enumerate(groups):
  c=context(s.faces,group);assert_context_immutable(group,c['vertices'],all_owned)
  rows=[r for rows in state_rows.values() for r in rows if set(r['vertices']).intersection(group)]
  # A physical predicate never spans two owners.
  assert all(set(r['vertices']).intersection(all_owned).issubset(group) for r in rows)
  rec=dict(index=i,owned_vertices=group,direction_vertices=sorted(set(group)&directions),compression_vertices=sorted(set(group)&physical),columns=c['columns'].tolist(),affected_columns=c['affected'].tolist(),context_vertices=c['vertices'].tolist(),target_columns=np.intersect1d(targets,c['affected']).tolist(),failure_rows=rows,physical_extent=(s.wall[group].max(0)-s.wall[group].min(0)).tolist())
  records.append(rec)
 write(D/'COMPONENT_GRAPH.json',dict(components=records,controlled_vertices=controlled,edges=edges,component_count=len(groups),size_distribution=stats([len(v)for v in groups]),largest=max(map(len,groups)),domain='Frozen union of research direction support and physical-failure support; immutable external lineages',contact_clearance_admission='Pending finite geometry validation; topology graph alone does not certify geometric disconnectedness',discovery_seconds=time.perf_counter()-started))
 print('ownership components',len(groups),'sizes',[len(v)for v in groups],flush=True)
if __name__=='__main__':discover()
