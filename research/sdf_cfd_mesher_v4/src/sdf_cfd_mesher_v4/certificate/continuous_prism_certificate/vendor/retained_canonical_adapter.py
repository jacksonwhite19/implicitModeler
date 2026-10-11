"""Canonical input/output wrapper; retained paths are byte-identical modules."""
from rank_geometry import Patch
import retained_rank_dispatch as retained
from coincident_rank3_certificate import coincident

def canonical_pair(a,b):
 # Equality of labels is unchanged. Integer relabeling removes hash-seed
 # dependent frozenset endpoint iteration before unchanged contact functions.
 labels=sorted(set(a.ids)|set(b.ids),key=lambda x:(type(x).__name__,repr(x)))
 ids={label:i for i,label in enumerate(labels)}
 a=Patch(a.points,tuple(ids[x] for x in a.ids),a.declared_dimension)
 b=Patch(b.points,tuple(ids[x] for x in b.ids),b.declared_dimension)
 if (b.rank,b.points,b.ids,b.declared_dimension)<(a.rank,a.points,a.ids,a.declared_dimension):a,b=b,a
 return a,b

def canonical_object(x):
 if isinstance(x,dict):return {k:canonical_object(x[k]) for k in sorted(x)}
 if isinstance(x,(list,tuple)):return [canonical_object(v) for v in x]
 return x

def certify(a,b):
 a,b=canonical_pair(a,b)
 r=coincident(a,b) if a.declared_dimension==2 and b.declared_dimension==2 else None
 if r is None:r=retained.certify(a,b)
 return canonical_object(r)
