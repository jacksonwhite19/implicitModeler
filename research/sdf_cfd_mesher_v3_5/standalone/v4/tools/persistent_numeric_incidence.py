"""Read-only packed face incidence with immutable newest-first delta segments."""
from collections.abc import Mapping
from contextlib import ExitStack
from pathlib import Path
import heapq
import numpy as np
import immutable_state_access as access
from path_preserving_deduplication import durable_json,file_hash
from sdf_cfd_mesher_v4.common.authority import identity

SCHEMA='V4PersistentNumericIncidence/1'


def pack_key(face):
    key=tuple(sorted(map(int,face)))
    if len(key) not in (3,4) or len(set(key))!=len(key):raise ValueError('InvalidFace')
    return key+(-1,)*(4-len(key))


def write_segment(path, rows, parent_content, parent_segment=None, *, cells=None, topology=None):
    """Qualification/construction writer. Never changes inherited segments."""
    path=Path(path);path.mkdir(exist_ok=False)
    ordered=sorted(rows);keys=np.array([pack_key(k) for k in ordered],dtype='<i8')
    owners=np.full((len(keys),2),-1,dtype='<i8')
    winding=np.full((len(keys),2,4),-1,dtype='<i8')
    for i,key in enumerate(ordered):
        values=rows[key]
        if not 1<=len(values)<=2:raise ValueError('InvalidFaceIncidence')
        for j,(owner,face) in enumerate(values):
            if tuple(sorted(face))!=key:raise ValueError('WrongFaceVertices')
            owners[i,j]=owner;winding[i,j,:len(face)]=face
        if len(values)==2:
            from prism_hotpath_v2 import reverse_cycle
            if values[0][0]==values[1][0] or not reverse_cycle(values[0][1],values[1][1]):
                raise ValueError('ConflictingOwners')
    files={}
    for name,value in (('keys',keys),('owners',owners),('winding',winding)):
        p=path/(name+'.npy')
        with p.open('wb') as f:
            np.save(f,value,allow_pickle=False);f.flush()
            import os
            os.fsync(f.fileno())
        files[name]=dict(path=str(p.resolve()),**access.attest(p,file_hash(p)))
    if cells is not None:
        p=path/'cells.npy'
        value=np.asarray([[c['id'],c['column'],c['layer'],*c['vertices']] for c in cells],dtype='<i8')
        with p.open('wb') as f:
            np.save(f,value,allow_pickle=False);f.flush()
            import os
            os.fsync(f.fileno())
        files['cells']=dict(path=str(p.resolve()),**access.attest(p,file_hash(p)))
    body=dict(schema=SCHEMA,parent_content=parent_content,parent_segment=parent_segment,files=files,
              rows=len(keys),internal=int(np.count_nonzero(owners[:,1]>=0)),boundary=int(np.count_nonzero(owners[:,1]<0)),epoch=access.epoch())
    if topology is not None:body['topology']=topology
    durable_json(path/'MANIFEST.json',body)
    return identity(body)


class PackedSegment(Mapping):
    def __init__(self,path,expected,parent_content):
        self.stack=ExitStack();self.closed=False;self.directory=Path(path);self.root=expected
        try:
            p=Path(path)/'MANIFEST.json';self.stack.enter_context(access.guard(p));self.body=access.read(p)
            if identity(self.body)!=expected:raise ValueError('WrongIncidenceRoot')
            if self.body['schema']!=SCHEMA or self.body['parent_content']!=parent_content:raise ValueError('WrongIncidenceParent')
            if self.body['epoch']!=access.epoch():raise ValueError('IncrementalEpochChanged')
            for name,r in self.body['files'].items():
                self.stack.enter_context(access.guard(r['path']))
                if access.token(r['path'])!=r['token']:raise ValueError('CorruptIncidenceSegment')
                setattr(self,name,np.load(r['path'],mmap_mode='r',allow_pickle=False))
            if self.keys.shape!=(self.body['rows'],4) or self.owners.shape!=(self.body['rows'],2) or self.winding.shape!=(self.body['rows'],2,4):raise ValueError('WrongIncidenceShape')
            self.search=self.keys.view(np.dtype([(str(i),'<i8') for i in range(4)])).reshape(-1)
        except BaseException:
            self.close();raise

    def close(self):
        self.closed=True;self.stack.close()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def live(self):
        if self.closed:raise ValueError('ClosedIncidence')
    def __len__(self):self.live();return len(self.keys)
    def __iter__(self):
        self.live()
        for row in self.keys:yield tuple(int(v) for v in row if v>=0)
    def __getitem__(self,key):
        self.live();packed=pack_key(key)
        needle=np.array(packed,dtype=self.search.dtype)
        i=int(np.searchsorted(self.search,needle))
        if i==len(self.keys) or tuple(self.keys[i])!=packed:raise KeyError(key)
        return [(int(owner),tuple(int(v) for v in self.winding[i,j] if v>=0)) for j,owner in enumerate(self.owners[i]) if owner>=0]
    def scientific_face(self,key):
        if not hasattr(self,'lookup'):raise ValueError('MissingSourceLookup')
        return self.lookup.row(key)
    @property
    def cell_segments(self):return (self.cells,)
    def ordered_items(self):
        self.live()
        for key,owners,winding in zip(np.asarray(self.keys),np.asarray(self.owners),np.asarray(self.winding)):
            yield tuple(int(v) for v in key if v>=0),[(int(owner),tuple(int(v) for v in winding[j] if v>=0)) for j,owner in enumerate(owners) if owner>=0]
    def deep_verify(self):
        self.live()
        for r in self.body['files'].values():
            if file_hash(r['path'])!=r['sha256']:raise ValueError('CorruptIncidenceContent')
        keys=list(self)
        if keys!=sorted(set(keys)):raise ValueError('DuplicateOrUnorderedFace')
        internal=boundary=0
        from prism_hotpath_v2 import reverse_cycle
        for key in keys:
            rows=self[key]
            if any(tuple(sorted(f))!=key for _,f in rows):raise ValueError('WrongFaceVertices')
            if len(rows)==2:
                if rows[0][0]==rows[1][0] or not reverse_cycle(rows[0][1],rows[1][1]):raise ValueError('ConflictingOwners')
                internal+=1
            elif len(rows)==1:boundary+=1
            else:raise ValueError('MissingFaceOwner')
        if (internal,boundary)!=(self.body['internal'],self.body['boundary']):raise ValueError('WrongIncidenceCounts')
        return dict(rows=len(keys),internal=internal,boundary=boundary)


class DeltaIncidence(Mapping):
    """A child owns only changed/new faces. Parent rows remain referenced."""
    def __init__(self,parent,delta):self.parent=parent;self.delta=delta
    def __getitem__(self,key):
        return self.delta[key] if key in self.delta else self.parent[key]
    def __iter__(self):
        previous=None
        for key in heapq.merge(iter(self.parent),iter(sorted(self.delta))):
            if key!=previous:
                yield key
                previous=key
    def __len__(self):return len(self.parent)+sum(k not in self.parent for k in self.delta)
    def ordered_items(self):
        newer=iter(sorted(self.delta.items()));item=next(newer,None)
        for key,rows in self.parent.ordered_items():
            while item is not None and item[0]<key:
                yield item;item=next(newer,None)
            if item is not None and item[0]==key:
                yield item;item=next(newer,None)
            else:yield key,rows
        while item is not None:
            yield item;item=next(newer,None)


class CellSequence:
    """Lazy immutable parent cells plus a small new-cell tail."""
    def __init__(self,packed,added=()):self.packed=packed;self.added=list(added)
    def __len__(self):return sum(len(a) for a in self.packed.cell_segments)+len(self.added)
    def __iter__(self):
        from prism_hotpath_v2 import prism_cell
        for segment in self.packed.cell_segments:
            for row in np.asarray(segment):
                yield prism_cell(list(map(int,row[3:])),int(row[1]),int(row[2]))
        yield from self.added
    def __add__(self,added):return CellSequence(self.packed,self.added+list(added))
    def selected(self,ids):
        from prism_hotpath_v2 import prism_cell
        new={c['id']:c for c in self.added};out=[]
        for key in sorted(ids):
            if key in new:out.append(new[key]);continue
            found=None
            for segment in reversed(self.packed.cell_segments):
                i=int(np.searchsorted(segment[:,0],key))
                if i<len(segment) and segment[i,0]==key:
                    found=segment[i];break
            if found is None:raise ValueError('MissingInheritedCell')
            out.append(prism_cell(list(map(int,found[3:])),int(found[1]),int(found[2])))
        return out


def advance(parent,additions,outer,wall):
    """Check every changed incidence; inherit authenticated unchanged witnesses."""
    from prism_hotpath_v2 import reverse_cycle
    from collections import defaultdict
    delta={};created=0;completed=0;created_keys=set()
    for cell in additions:
        for face in cell['faces']:
            key=tuple(sorted(face))
            if key not in delta:
                try:prior=parent[key]
                except KeyError:prior=[];created+=1;created_keys.add(key)
                else:
                    if len(prior)!=1:raise ValueError('AppendTouchesInternalFace')
                    completed+=1
                delta[key]=list(prior)
            if any(owner==cell['id'] for owner,_ in delta[key]):raise ValueError('DuplicateCellFace')
            delta[key].append((cell['id'],tuple(face)))
    intended={tuple(sorted(f)) for f in outer}|{tuple(sorted(f)) for f in wall}
    failures=[]
    for key,rows in sorted(delta.items()):
        if key in intended:
            if len(rows)!=1:failures.append(('BoundaryOwnership',key,len(rows)))
        elif len(rows)!=2:failures.append(('InternalOwnership',key,len(rows)))
        elif not reverse_cycle(rows[0][1],rows[1][1]):failures.append(('InternalOrientation',key))
    if any(tuple(sorted(f)) not in delta for f in outer):raise ValueError('MissingTerminalFace')
    edges=defaultdict(list)
    for i,face in enumerate(outer):
        for a,b in zip(face,face[1:]+face[:1]):edges[tuple(sorted((a,b)))].append((a,b,i))
    bad=[(key,val) for key,val in edges.items() if len(val)!=2 or val[0][:2]!=val[1][:2][::-1]]
    inherited=parent.body['topology']
    if not inherited['passed']:raise ValueError('UnqualifiedParentTopology')
    old_boundary=parent.body['boundary'];old_internal=parent.body['internal']
    new_internal=sum(len(rows)==2 for rows in delta.values())
    new_boundary=sum(len(rows)==1 for rows in delta.values())
    result=dict(passed=not failures and not bad,failures=failures,outer_bad_edges=bad,
        internal_faces=old_internal+new_internal,boundary_faces=old_boundary-completed+new_boundary,
        outer_faces=len(outer),outer_edges=len(edges),outer_duplicate_faces=len(outer)-len(set(tuple(sorted(f)) for f in outer)))
    if result['internal_faces']+result['boundary_faces']!=len(parent)+created:raise ValueError('WrongIncrementalFaceCount')
    child=DeltaIncidence(parent,delta);child.created_keys=created_keys
    return child,result

class PackedClosure(DeltaIncidence):
    """Recursive committed numeric parent plus one immutable changed segment."""
    def __init__(self,reference,content):
        self.stack=ExitStack();self.reference=reference
        try:
            if reference['child_content']!=content:raise ValueError('WrongGeometryClosureContent')
            version=reference.get('version_closure')
            if version is not None and 'path' in version:
                self.stack.enter_context(access.guard(version['path']));summary=access.read(version['path'])
                if identity(summary)!=version['root'] or not summary['closed'] or summary['parent_content']!=reference['parent']['content'] or summary['numeric_root']!=reference['parent']['root']:
                    raise ValueError('WrongVersionClosure')
            self.parent=self.stack.enter_context(open_geometry(reference['parent'],reference['parent']['content']))
            d=reference['delta']
            self.delta=self.stack.enter_context(open_geometry(d,d['content']))
            if self.delta.body['parent_segment']!=reference['parent']:raise ValueError('WrongGeometryAncestry')
            if self.delta.body['topology']!=reference['topology']:raise ValueError('WrongTopologySummary')
            self.body=dict(topology=reference['topology'],internal=reference['topology']['internal_faces'],boundary=reference['topology']['boundary_faces'])
        except BaseException:
            self.stack.close();raise
    def __len__(self):
        self.delta.live();return self.body['internal']+self.body['boundary']
    def scientific_face(self,key):
        try:return self.delta.scientific_face(key)
        except KeyError:return self.parent.scientific_face(key)
    @property
    def cell_segments(self):return (*self.parent.cell_segments,*self.delta.cell_segments)
    def __enter__(self):return self
    def __exit__(self,*args):self.stack.close()
    def ordered_items(self):
        newer=iter(self.delta.ordered_items());item=next(newer,None)
        for key,rows in self.parent.ordered_items():
            while item is not None and item[0]<key:
                yield item;item=next(newer,None)
            if item is not None and item[0]==key:
                yield item;item=next(newer,None)
            else:yield key,rows
        while item is not None:
            yield item;item=next(newer,None)


def open_geometry(reference,content):
    if 'child_content' in reference:return PackedClosure(reference,content)
    if reference['content']!=content:raise ValueError('WrongGeometryParent')
    result=PackedSegment(reference['directory'],reference['root'],content)
    result.reference=reference
    if 'lookup' in reference:
        try:
            from prism_source_lookup import SourceLookup
            result.lookup=SourceLookup(result,reference['lookup'],result.stack)
        except BaseException:
            result.close();raise
    return result
