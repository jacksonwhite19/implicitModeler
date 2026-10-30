"""Standalone direct-path exterior-body surface contract.

Original connected surface/degree owners remain unchanged. Local topology and
nesting are conditional until every original global obligation passes. Parent
hashes must come from the existing live lease or authenticated reader manifest.
This module cannot create either authority or a successful mesh receipt.
"""
from collections import defaultdict
from fractions import Fraction as F
import numpy as np

SCHEMA='V35DirectExteriorBodySurfaces/1'
CONTRACT=dict(schema=SCHEMA,scope='ordinary_prisms;nonnested_exterior_bodies;one_fluid_region',
    local='original_connected_surface_per_component;all_links;exact_role_orientation',
    lineage='authenticated_parent_points_prisms_ids_wall_cap;bijective_columns;reversed_cycles',
    farfield='one_connected_original_surface;original_box_and_enclosure',
    nesting='exact_representative_degree_or_strict_AABB_separator;conditional_on_complete_global_collision',
    mandatory_global='unchanged_boundary_collision;all_vertex_enclosure;all_face_collision;positive_cells;degree_one;exact_volume_balance',
    unsupported='nested_cavities;fluid_pockets;multiple_fluid_regions',scientific_acceptance=False)

def profile():
    from sdf_cfd_mesher_v4.common.authority import identity
    return dict(schema=SCHEMA,identity=identity(CONTRACT))

def require(value,label):
    if not value:raise ValueError(label)

def cycle(a,b):
    a=tuple(map(int,a));b=tuple(map(int,b))
    return len(a)==len(b) and any(a==b[i:]+b[:i] for i in range(len(b)))

def equal(a,b):
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()

def partition(points,faces,connected_surface,check):
    require(points.dtype==np.dtype('<f8') and points.ndim==2 and points.shape[1]==3
            and np.isfinite(points).all(),'InvalidPoints')
    require(faces.dtype==np.dtype('<i8') and faces.ndim==2 and faces.shape[1]==3
            and len(faces)>0 and np.all(faces>=0) and np.all(faces<len(points)),'InvalidFaces')
    adj=defaultdict(set)
    for k,face in enumerate(faces):
        if k%1024==0:check()
        for a,b in zip(face,np.roll(face,-1)):
            adj[int(a)].add(int(b));adj[int(b)].add(int(a))
    unseen=set(adj);groups=[]
    while unseen:
        todo=[min(unseen)];seen=set()
        while todo:
            a=todo.pop()
            if a in seen:continue
            seen.add(a);todo.extend(adj[a]-seen)
        unseen-=seen;groups.append(sorted(seen));check()
    labels={v:i for i,vertices in enumerate(groups) for v in vertices}
    face_ids=[[] for _ in groups]
    for k,face in enumerate(faces):face_ids[labels[int(face[0])]].append(k)
    result=[]
    for vertices,ids in zip(groups,face_ids):
        vertices=np.asarray(vertices,dtype='<i8');rows=faces[ids]
        inverse=np.full(len(points),-1,dtype='<i8');inverse[vertices]=np.arange(len(vertices))
        local=inverse[rows];p=points[vertices]
        # Retain the ORIGINAL duplicate/edge-cycle/link checks per component.
        report=connected_surface(p,local,check)
        exact=[tuple(F(float(x)) for x in row) for row in p]
        volume6=F(0)
        for i,(a,b,c) in enumerate(local):
            if i%1024==0:check()
            x,y,z=exact[a],exact[b],exact[c]
            volume6+=x[0]*(y[1]*z[2]-y[2]*z[1])+x[1]*(y[2]*z[0]-y[0]*z[2])+x[2]*(y[0]*z[1]-y[1]*z[0])
        require(volume6!=0,'ZeroComponentOrientedVolume')
        result.append(dict(vertices=vertices,faces=rows,local=local,points=p,report=report,
                           volume6=volume6,bounds=np.array([p.min(0),p.max(0)])))
    return result

def inherited(parent,points,wall,cap,prisms,check):
    """Caller must authenticate parent; this function only compares its inputs."""
    for name,value in [('wall',wall),('outer',cap),('cell_vertices',prisms)]:
        require(equal(parent[name],value),'ChangedInherited_'+name)
    n=len(parent['points'])
    require(n<=len(points) and equal(points[:n],parent['points']),'ChangedInheritedPoints')
    require(prisms.dtype==np.dtype('<i8') and prisms.ndim==2 and prisms.shape[1]==6
            and len(prisms)>0 and np.all(prisms>=0) and np.all(prisms<n),'OrdinaryPrismPopulationRequired')
    forward={};backward={}
    for i,row in enumerate(prisms):
        if i%1024==0:check()
        require(len(set(map(int,row)))==6,'RepeatedPrismVertex')
        for a,b in zip(row[:3],row[3:]):
            a,b=int(a),int(b)
            require(a not in forward or forward[a]==b,'AmbiguousColumnSuccessor')
            require(b not in backward or backward[b]==a,'AmbiguousColumnPredecessor')
            forward[a]=b;backward[b]=a
    roots=set(map(int,wall.ravel()));terminals=set(map(int,cap.ravel()))
    require(roots==set(forward)-set(backward),'ChangedWallColumnRoots')
    require(terminals==set(backward)-set(forward),'ChangedCapColumnTerminals')
    mapping={};visited=set()
    for i,root in enumerate(sorted(roots)):
        if i%1024==0:check()
        current=root;path=set()
        while current in forward:
            require(current not in path,'CyclicInheritedColumn')
            path.add(current);current=forward[current]
        require(current in terminals,'MissingColumnTerminal')
        mapping[root]=current;visited.update(path);visited.add(current)
    require(visited==set(map(int,prisms.ravel())),'ExtraInheritedColumnPopulation')
    require(len(set(mapping.values()))==len(roots)==len(terminals),'NonbijectiveWallCapColumns')
    expected={tuple(sorted(map(int,face))):tuple(map(int,face)) for face in cap}
    require(len(expected)==len(cap)==len(wall),'ChangedInheritedFacePopulation')
    for face in wall:
        mapped=tuple(mapping[int(v)] for v in reversed(face));key=tuple(sorted(mapped))
        require(key in expected and cycle(mapped,expected[key]),'WallCapCycleCorrespondence')
    return mapping

def disjoint_boxes(a,b):
    return bool(np.any(a['bounds'][1]<b['bounds'][0]) or np.any(b['bounds'][1]<a['bounds'][0]))

def degree_at(container,point,degree,check):
    check();sample=tuple(F(float(x)) for x in point)
    result=degree(container['points'],list(map(tuple,container['local'])),[sample]);check()
    require(not result['unresolved'] and len(result['values'])==1,'UnresolvedComponentNesting')
    return result['values'][0]['degree']

def contract(parent,points,wall,cap,farfield,prisms,connected_surface,degree,check):
    mapping=inherited(parent,points,wall,cap,prisms,check)
    walls=partition(points,wall,connected_surface,check)
    caps=partition(points,cap,connected_surface,check)
    fars=partition(points,farfield,connected_surface,check)
    require(len(fars)==1,'ConnectedFarfieldRequired')
    require(all(c['volume6']<0 for c in walls),'InwardFluidWallRequired')
    require(all(c['volume6']>0 for c in caps+fars),'OutwardCapFarfieldRequired')
    cap_labels={int(v):i for i,c in enumerate(caps) for v in c['vertices']}
    pairs=[]
    for c in walls:
        mapped={mapping[int(v)] for v in c['vertices']}
        labels={cap_labels[v] for v in mapped}
        require(len(labels)==1,'SplitWallCapComponent')
        j=next(iter(labels))
        require(mapped==set(map(int,caps[j]['vertices'])),'ChangedComponentCorrespondence')
        pairs.append(j)
    require(len(set(pairs))==len(walls)==len(caps),'ExtraOrMergedCapComponent')
    nesting=[]
    for i,w in enumerate(walls):
        c=caps[pairs[i]]
        # A representative suffices ONLY conditional on the later complete
        # nonintersection/embedding proofs. Never promote these samples alone.
        require(degree_at(c,w['points'][0],degree,check)==1,'WallOutsideCorrespondingCap')
        require(degree_at(w,c['points'][0],degree,check)==0,'CapInsideCorrespondingWall')
        require(degree_at(fars[0],w['points'][0],degree,check)==1
                and degree_at(fars[0],c['points'][0],degree,check)==1,'ComponentOutsideFarfield')
        for j in range(i):
            other=caps[pairs[j]]
            if disjoint_boxes(c,other):relation='strict_disjoint_AABBs'
            else:
                require(degree_at(c,other['points'][0],degree,check)==0
                        and degree_at(other,c['points'][0],degree,check)==0,'NestedCapComponentsUnsupported')
                relation='degree_zero_both_directions_conditional_on_global_collision'
            nesting.append(dict(components=[j,i],relation=relation))
    def summary(c):
        return dict(vertices=len(c['vertices']),faces=len(c['faces']),edges=c['report']['edges'],
                    euler=len(c['vertices'])-c['report']['edges']+len(c['faces']),
                    signed_volume=str(c['volume6']/6),vertex_links_passed=c['report']['links']['passed'])
    return dict(schema=SCHEMA,contract=profile(),local_contract_passed=True,
        scientific_acceptance=False,parent_authentication='EXISTING_CALLER_AUTHORITY_REQUIRED',
        components={k:[summary(c) for c in v] for k,v in [('wall',walls),('cap',caps),('farfield',fars)]},
        wall_to_cap_components=pairs,nesting=nesting,
        nesting_scope='Exact representative degrees are conditional on original global nonintersection and positive-cell proofs; no continuous SDF/fluid assertion.',
        mandatory_remaining=['boundary_collision','enclosure','collision','filling'],
        unchanged_prior_requirements=['preservation','domain','incidence','winding','interface_qa','joined_qa','physical_qa'])


FIELDS=dict(points='<f8',background_tets='<i8',farfield='<i8',prisms='<i8',
            prism_ids='<i8',tetrahedron_ids='<i8',wall='<i8',cap='<i8')

class BoundSurfaces:
    """Role-bound construction interface used by the unchanged joined auditor.

Roles match exact compact arrays, never call order or geometry family. Binding
does not evaluate/publish local predicates before the surfaces checkpoint.
"""
    def __init__(self,legacy,parent,arrays,authority_check,binding):
        from sdf_cfd_mesher_v4.common.authority import array_hash,identity
        import copy
        authority_check()
        require(parent['binding']==binding and binding.get('surface_contract')==profile(),'ChangedDirectSurfaceParentBinding')
        n=parent['prefix_points']
        require(type(n) is int and 0<n<=len(arrays['points']),'InvalidSurfacePrefix')
        self._arrays={k:arrays[k] for k in ('points','prisms','prism_ids','wall','cap','farfield')}
        for key in ('prisms','prism_ids','wall','cap'):
            require(array_hash(arrays[key])==parent['array_hashes'][key],'ChangedDirectSurfaceParent_'+key)
        require(array_hash(arrays['points'][:n])==parent['array_hashes']['points'],'ChangedDirectSurfaceParent_points')
        self._parent=dict(points=arrays['points'][:n].copy(),cell_vertices=arrays['prisms'].copy(),
                          wall=arrays['wall'].copy(),outer=arrays['cap'].copy())
        self._parent_record=parent;self._parent_pin=identity(parent);self._binding=copy.deepcopy(binding)
        self._pins={k:array_hash(v) for k,v in self._arrays.items()}
        self._roles={}
        for role,key in [('wall','wall'),('prefix_front','cap'),('farfield','farfield')]:
            p,f=self._compact(arrays['points'],arrays[key]);self._roles[role]=(p.copy(),f.copy())
        self._legacy=legacy;self._authority=authority_check;self._result=None
        self.d=legacy.d;self.degree=legacy.degree
        self.inherited_digest=legacy.inherited_digest;self.verify_inherited_join=legacy.verify_inherited_join

    @staticmethod
    def _compact(points,faces):
        from finite_research.direct_evidence_modules.joined_audit import compact_surface
        return compact_surface(points,faces)

    def _check(self,check):
        from sdf_cfd_mesher_v4.common.authority import array_hash,identity
        self._authority();check()
        require(identity(self._parent_record)==self._parent_pin
                and self._parent_record['binding']==self._binding,'ChangedDirectSurfaceParentBinding')
        require(all(array_hash(v)==self._pins[k] for k,v in self._arrays.items()),'ChangedDirectSurfaceRoleArrays')

    def surface(self,points,faces,check):
        self._check(check)
        roles=[role for role,(p,f) in self._roles.items() if equal(points,p) and equal(faces,f)]
        require(len(roles)==1,'UnknownOrAmbiguousDirectSurfaceRole')
        if self._result is None:
            a=self._arrays
            self._result=contract(self._parent,a['points'],a['wall'],a['cap'],a['farfield'],a['prisms'],
                                  self._legacy.surface,self._legacy.degree,lambda:self._check(check))
        self._check(check)
        # Each role receipt carries the same conditional composite contract;
        # the unchanged joined auditor binds geometry and publishes it itself.
        import copy
        return dict(role=roles[0],**copy.deepcopy(self._result))

def bind(legacy,parent,arrays,check,binding):
    return BoundSurfaces(legacy,parent,arrays,check,binding)
