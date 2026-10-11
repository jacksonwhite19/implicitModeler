"""Conservative source-component dependency groups from current geometry.

Connected wall topology is indivisible here. Disconnected sources share a
dependency group when their proposed front boxes intersect. No case IDs or
distance thresholds enter the grouping.
"""
import numpy as np


def groups(tri,points,columns):
    parent=np.arange(len(points))
    def root(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=int(parent[i])
        return int(i)
    def join(a,b):
        a,b=root(int(a)),root(int(b));parent[max(a,b)]=min(a,b)
    for a,b,c in tri:join(a,b);join(a,c)
    components={}
    for column in sorted(map(int,columns)):components.setdefault(root(int(tri[column,0])),[]).append(column)
    keys=sorted(components);boxes={}
    for k in keys:
        p=points[np.unique(tri[components[k]])];boxes[k]=(p.min(0),p.max(0))
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            lo,hi=boxes[a];bl,bh=boxes[b]
            if np.all(lo<=bh) and np.all(bl<=hi):join(a,b)
    result={}
    for k,cols in components.items():result.setdefault(root(k),[]).extend(cols)
    return [sorted(result[k]) for k in sorted(result)]
