"""Legal contact comes exclusively from ordered mesh topology."""
EDGE_CORNERS=((0,1),(1,3),(3,2),(2,0))
PARAM_CORNERS=((0,0),(1,0),(0,1),(1,1))
def contact(a,b):
 common=set(a.ids)&set(b.ids)
 for label in common:
  assert a.points[a.ids.index(label)]==b.points[b.ids.index(label)],'SharedTopologyCoordinateMismatch'
 ae={frozenset((a.ids[i],a.ids[j])) for i,j in EDGE_CORNERS}
 be={frozenset((b.ids[i],b.ids[j])) for i,j in EDGE_CORNERS}
 edges=ae&be
 return {'vertices':common,'edges':edges}
def contains(c,patch,U,V):
 for i,(u,v) in enumerate(PARAM_CORNERS):
  if U==u and V==v and patch.ids[i] in c['vertices']:return True
 for i,j in EDGE_CORNERS:
  if frozenset((patch.ids[i],patch.ids[j])) not in c['edges']:continue
  u0,v0=PARAM_CORNERS[i];u1,v1=PARAM_CORNERS[j]
  if u0==u1 and U==u0 and 0<=V<=1:return True
  if v0==v1 and V==v0 and 0<=U<=1:return True
 return False
def legal_edge(c,patch,axis,value):
 indices=[i for i,p in enumerate(PARAM_CORNERS) if p[axis]==value]
 return frozenset(patch.ids[i] for i in indices) in c['edges']
