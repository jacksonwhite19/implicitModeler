"""Geometry-only complete canonical unordered AABB pair discovery."""
import numpy as np
def boxes(patches):
 x=np.asarray(patches,dtype=float);return x.min(axis=1),x.max(axis=1)
def brute_force(lo,hi,ids=None):
 ids=np.arange(len(lo)) if ids is None else np.asarray(ids);out=set()
 for a in range(len(lo)):
  for b in np.flatnonzero(np.all(hi[a+1:]>=lo[a],axis=1)&np.all(lo[a+1:]<=hi[a],axis=1))+a+1:
   out.add(tuple(sorted((int(ids[a]),int(ids[b])))))
 return out
def bvh_pairs(lo,hi,ids=None,leaf_capacity=8,reverse_work=False,swap_children=False):
 ids=np.arange(len(lo)) if ids is None else np.asarray(ids);nodes=[]
 def build(ind):
  n=len(nodes);L=lo[ind].min(0);U=hi[ind].max(0);node={'lo':L,'hi':U,'indices':ind,'children':None};nodes.append(node)
  if len(ind)>leaf_capacity:
   axis=int(np.argmax(U-L));order=sorted(ind,key=lambda j:((lo[j,axis]+hi[j,axis])*.5,int(ids[j])));m=len(order)//2;node['children']=(build(order[:m]),build(order[m:]))
  return n
 if not len(lo):return set(),{'discoveries':0,'duplicates_removed':0}
 root=build(list(range(len(lo))));work=[(root,root)];out=set();discoveries=0
 while work:
  ia,ib=work.pop(0 if reverse_work else -1);a,b=nodes[ia],nodes[ib]
  if np.any(a['hi']<b['lo']) or np.any(b['hi']<a['lo']):continue
  ca,cb=a['children'],b['children']
  if ca is None and cb is None:
   for i in a['indices']:
    for j in b['indices']:
     if i==j or (ia==ib and i>j):continue
     if np.any(hi[i]<lo[j]) or np.any(hi[j]<lo[i]):continue
     pair=tuple(sorted((int(ids[i]),int(ids[j]))));discoveries+=1;out.add(pair)
  elif ia==ib:
   l,r=ca;pending=[(l,l),(l,r),(r,r)]
   work.extend(reversed(pending) if swap_children else pending)
  elif cb is None or (ca is not None and len(a['indices'])>=len(b['indices'])):
   pending=[(ca[0],ib),(ca[1],ib)];work.extend(reversed(pending) if swap_children else pending)
  else:
   pending=[(ia,cb[0]),(ia,cb[1])];work.extend(reversed(pending) if swap_children else pending)
 return out,{'discoveries':discoveries,'duplicates_removed':discoveries-len(out),'node_count':len(nodes)}
