from pathlib import Path
import hashlib,json,re
import numpy as np
NAME="DelaunayDirectSizing095FixedBoundary155/1"

def cycle_keys(faces):
    return sorted(min((first,second,third),(second,third,first),(third,first,second))
        for first,second,third in np.asarray(faces))

def expected_options():
    return {"Mesh.Algorithm3D":1,"Mesh.Algorithm":6,"General.NumThreads":4,
        "Mesh.MaxNumThreads1D":1,"Mesh.MaxNumThreads2D":1,"Mesh.MaxNumThreads3D":4,
        "Mesh.ElementOrder":1,"Mesh.Optimize":0,"Mesh.OptimizeNetgen":0,"Mesh.OptimizeThreshold":0.3,
        "Mesh.Reproducible":1,"Mesh.RandomSeed":1,"Mesh.RandomFactor":1e-12,"Mesh.RandomFactor3D":1e-12,
        "Mesh.Smoothing":0,"Mesh.MeshSizeFromPoints":0,"Mesh.MeshSizeFromCurvature":0,"Mesh.MeshSizeExtendFromBoundary":0,
        "Mesh.MeshSizeFactor":1.,"Mesh.Binary":1,"Mesh.SaveAll":1}

def read_options(module,factor=1.):
    expected=expected_options();expected["Mesh.MeshSizeFactor"]=factor;actual={name:module.option.getNumber(name) for name in expected}
    assert actual==expected
    return actual

def sizing_options(module,maximum,factor=1.):
    actual={name:module.option.getNumber(name) for name in ("Mesh.MeshSizeFactor","Mesh.MeshSizeMin","Mesh.MeshSizeMax")}
    assert actual=={"Mesh.MeshSizeFactor":factor,"Mesh.MeshSizeMin":0.,"Mesh.MeshSizeMax":maximum}
    return actual

def surface_check(module,cap,outer,volume_empty=True):
    tags,coordinates,_=module.model.mesh.getNodes()
    tags=np.asarray(tags,dtype=np.int64);order=np.argsort(tags);tags=tags[order]
    points=np.asarray(coordinates,dtype="<f8").reshape(-1,3)[order]
    expected=np.concatenate((cap["points"],outer["points"]))
    assert np.array_equal(tags,np.arange(1,len(expected)+1)) and points.tobytes()==expected.tobytes()
    kinds,_,groups=module.model.mesh.getElements(2)
    assert list(kinds)==[2]
    actual=np.asarray(groups[0],dtype="<i8").reshape(-1,3)-1
    wanted=np.concatenate((cap["triangles"],outer["triangles"]+len(cap["points"])))
    assert cycle_keys(actual)==cycle_keys(wanted)
    for entity,faces in ((1,cap["triangles"]),(2,outer["triangles"]+len(cap["points"]))):
        if not len(faces):continue
        kinds,_,groups=module.model.mesh.getElements(2,entity)
        assert list(kinds)==[2]
        assert cycle_keys(np.asarray(groups[0],dtype="<i8").reshape(-1,3)-1)==cycle_keys(faces)
    if len(cap["points"]):
        assert module.model.getEntities(3)==[(3,1)]
        assert sorted(module.model.getBoundary([(3,1)],combined=False,oriented=True,recursive=False))==[(2,-1),(2,2)]
    if volume_empty:
        _,element_groups,_=module.model.mesh.getElements(3)
        assert sum(len(group) for group in element_groups)==0
    return dict(passed=True,cap_vertices=len(cap["points"]),farfield_vertices=len(outer["points"]),
        cap_faces=len(cap["triangles"]),farfield_faces=len(outer["triangles"]),
        exact_coordinate_bytes=True,oriented_surface_connectivity=True,volume_elements=0)

def import_boundary(module,cap,outer):
    module.model.add("fixed_boundary_delaunay095")
    module.model.addDiscreteEntity(2,1)
    module.model.mesh.addNodes(2,1,np.arange(1,len(cap["points"])+1),cap["points"].ravel())
    module.model.mesh.addElementsByType(1,2,np.arange(1,len(cap["triangles"])+1),(cap["triangles"]+1).ravel())
    module.model.addDiscreteEntity(2,2)
    offset=len(cap["points"])+1
    module.model.mesh.addNodes(2,2,np.arange(offset,offset+len(outer["points"])),outer["points"].ravel())
    module.model.mesh.addElementsByType(2,2,np.arange(len(cap["triangles"])+1,len(cap["triangles"])+len(outer["triangles"])+1),
        (outer["triangles"]+offset).ravel())
    module.model.mesh.reclassifyNodes()
    module.model.mesh.createTopology(False,True)
    module.model.addDiscreteEntity(3,1,[2,-1])
    module.model.mesh.createGeometry([(3,1)])
    assert module.model.getEntities(3)==[(3,1)]
    return surface_check(module,cap,outer)

def stage_shape(core,points,tets,cap,outer,policy):
    from finite_support.transition_front_targets import tetrahedron_ratio,triangle_ratio,population_report
    audit,owners,boundary,call_data=core.boundary_audit(points,tets,cap["points"],cap["triangles"],_return_call_data=True)
    lookup={point.tobytes():index for index,point in enumerate(points)}
    assert len(lookup)==len(points)
    cap_ids=np.asarray([lookup[point.tobytes()] for point in cap["points"]])
    far_ids=np.asarray([lookup[point.tobytes()] for point in outer["points"]])
    assert points[cap_ids].tobytes()==cap["points"].tobytes() and points[far_ids].tobytes()==outer["points"].tobytes()
    wanted=np.concatenate((cap_ids[cap["triangles"]][:,::-1],far_ids[outer["triangles"]]))
    assert cycle_keys(call_data["boundary_uses"])==cycle_keys(wanted)
    assert audit["nonmanifold_faces"]==0
    unique=np.unique(owners);targets=policy["tet_transition"]["quality_targets"]
    populations={}
    for label,scores,kind in (("interface_tetrahedra",tetrahedron_ratio(points,tets[unique]),"outer_tetrahedra"),
        ("all_bulk_tetrahedra",tetrahedron_ratio(points,tets),"outer_tetrahedra"),
        ("preserved_cap_triangles",triangle_ratio(cap["points"],cap["triangles"]),"outer_triangles")):
        assert np.isfinite(scores).all() and np.all(scores>0)
        populations[label]=population_report(scores,targets[kind])
    return dict(passed_boundary=True,points=len(points),tetrahedra=len(tets),cap_faces=len(owners),
        unique_interface_tetrahedra=len(unique),exact_cap_farfield=True,cap_audit=audit,populations=populations)

class Profile:
    def __init__(self,module,cap,growth,maximum,on_generate):
        from background_native_query import NativeQueryField
        from background_direct_callback import DirectCallback,SessionBridge
        self.module=module;self.settings=[];self.sessions=[];self.generation_calls=0;self.closed=False
        self.field=NativeQueryField(cap["points"][cap["triangles"]],cap["seeds"],g=growth,h_max=maximum)
        self.callback=None;self.on_generate=on_generate
        try:
            assert self.field.native_report()["native_enabled"]
            self.callback=DirectCallback(self.field)
            self.input_identity=self.callback.report()["input_identity"]
            self.bridge=SessionBridge(module,self._generate,module.finalize);self.bridge.owner=self.callback
        except BaseException:
            if self.callback is not None:self.callback.close()
            self.field.close()
            raise
    def setup(self,module):
        from gmsh_core_setup import setup_gmsh
        assert module is self.module and not self.closed
        setup_gmsh(module,1)
        module.option.setNumber("General.NumThreads",4)
        module.option.setNumber("Mesh.MaxNumThreads3D",4)
        self.settings.append(read_options(module))
    def install(self):
        from generic_core_sizing import GmshSizingAdapter
        GmshSizingAdapter(self.field).install(self.module)
        self.bridge.install(self.callback)
        sizing_options(self.module,self.field.h_max)
    def finalize(self):
        return self.bridge.finalize()
    def _generate(self,dimension):
        assert dimension==3 and self.generation_calls==0 and self.callback.registered
        sizing_options(self.module,self.field.h_max);read_options(self.module)
        self.module.option.setNumber("Mesh.MeshSizeFactor",.95)
        options=sizing_options(self.module,self.field.h_max,.95);read_options(self.module,.95)
        self.generation_calls+=1
        self.on_generate(dict(method=NAME,dimension=3,factor=.95,algorithm=1,requested_threads=4,options=options,
            callback_identity=self.input_identity,generation_calls=self.generation_calls))
        result=self.module.model.mesh.generate(3)
        assert sizing_options(self.module,self.field.h_max,.95)==options
        self.sessions.append(dict(dimension=3,before=1.,factor=.95,passed=True))
        return result
    def generate(self):
        return self.bridge.generate(3)
    def finish(self,candidate):
        self.callback.require_passed();report=self.callback.report()
        assert report["input_identity"]==self.input_identity
        assert self.generation_calls==(1 if candidate else 0)
        assert self.sessions==([dict(dimension=3,before=1.,factor=.95,passed=True)] if candidate else [])
        assert len(self.settings)==(4 if candidate else 2)
        assert report["registrations"]==2 and report["removals"]==2 and not self.callback.registered
        assert not report["quarantined"] and not report["python_failure"] and report["native_failure"]==0
        assert report["cache_capacity"]==0 and report["exact_coordinate_cache_hits"]==0
        assert report["evaluations"]==report["native_calls"]+report["fallback_calls"]
        if candidate:assert report["evaluations"]>0
        result=dict(passed=True,method=NAME,generation_factor_hex=float(.95).hex(),generation_sessions=self.sessions,
            settings=self.settings,callback=report,farfield_session="Exact retained boundary reuse; zero explicit2Dgeneration",
            region_factor="Inherited GEntity constant1; sole fresh discrete volume; source-established, no Python getter",
            generation_calls=self.generation_calls,closed=True)
        self.close()
        return result
    def close(self):
        if self.closed:return
        try:
            if self.module.isInitialized():self.finalize()
            if self.callback is not None:
                self.callback.close()
                assert not self.callback.pointer and not self.callback.quarantined
        finally:
            self.field.close();self.closed=True

def cycle_tests():
    from itertools import permutations
    cases=[np.empty((0,3),dtype="<i8"),np.asarray(list(permutations((1,2,3))),dtype="<i8"),
        np.asarray([[1,1,2],[1,2,1],[2,1,1],[1,2,3],[1,2,3]],dtype="<i8")]
    for faces in cases:
        expected=sorted(min(tuple(np.roll(face,offset)) for offset in range(3)) for face in faces)
        assert cycle_keys(faces)==expected
    base=np.asarray([[1,2,3],[4,5,6]],dtype="<i8")
    assert cycle_keys(base)==cycle_keys(base[:,[1,2,0]])
    for changed in (base[:,::-1],base[:-1],np.repeat(base[:1],2,axis=0)):
        assert cycle_keys(base)!=cycle_keys(changed)
    return dict(passed=True,exact_reference_cases=len(cases),orientation_and_multiplicity_preserved=True)

def focused_tests():
    class Option:
        def __init__(self):self.values=dict(expected_options(),**{"Mesh.MeshSizeMin":0.,"Mesh.MeshSizeMax":.0038})
        def getNumber(self,name):return self.values[name]
        def setNumber(self,name,value):self.values[name]=value
    class Model:
        def __init__(self):self.option=Option()
    module=Model();read_options(module);sizing_options(module,.0038)
    refusals=0
    for name,value in (("Mesh.MeshSizeFactor",.95),("Mesh.Algorithm3D",10),("Mesh.OptimizeThreshold",.5),("Mesh.MeshSizeMax",.004)):
        previous=module.option.values[name];module.option.values[name]=value
        try:
            read_options(module);sizing_options(module,.0038)
        except AssertionError:refusals+=1
        else:raise AssertionError("ChangedSizingProfileAccepted")
        finally:module.option.values[name]=previous
    assert refusals==4
    from types import SimpleNamespace
    module.option.setNumber("Mesh.MeshSizeFactor",.95)
    read_options(module,.95);sizing_options(module,.0038,.95)
    module.option.setNumber("Mesh.MeshSizeFactor",1.)
    try:read_options(module,.95)
    except AssertionError:pass
    else:raise AssertionError("WrongGenerationFactorAccepted")
    observed=[]
    module.model=SimpleNamespace(mesh=SimpleNamespace(generate=lambda dimension:observed.append((dimension,module.option.getNumber("Mesh.MeshSizeFactor")))))
    profile=Profile.__new__(Profile)
    profile.module=module;profile.generation_calls=0;profile.callback=SimpleNamespace(registered=True)
    profile.field=SimpleNamespace(h_max=.0038);profile.input_identity="control";profile.sessions=[];profile.on_generate=lambda record:None
    profile._generate(3)
    assert observed==[(3,.95)] and profile.sessions==[dict(dimension=3,before=1.,factor=.95,passed=True)]
    try:profile._generate(3)
    except AssertionError:pass
    else:raise AssertionError("RepeatedGenerationAccepted")
    return dict(passed=True,cycle_controls=cycle_tests(),execution_controls=execution_tests(),refused_changed_factor_algorithm_threshold_maximum=refusals,
        effective_factor=.95,original_field_unscaled=True)

def execution_evidence(trace):
    messages=trace["messages"];joined="\n".join(messages)
    before=trace["callback_before"];after=trace["callback_after"]
    deltas={key:after[key]-before[key] for key in ("evaluations","native_calls","fallback_calls","native_failure")}
    markers={key:bool(re.search(pattern,joined)) for key,pattern in dict(
        refinement=r"3D refinement terminated \(\d+ nodes total\)",
        cavities=r"\d+ Delaunay cavities modified for star shapeness",
        missed=r"\d+ nodes could not be inserted",
        created=r"\d+ tetrahedra created in").items()}
    warnings=[message for message in messages if message.lower().startswith("warning")]
    errors=[message for message in messages if message.lower().startswith("error")]
    forbidden=[message for message in messages if re.search(r"meshing (?:surface|curve)\s+\d+|hxt|fallback|falling back|wrong mesh element size|no tetrahedra|no elements in volume",message,re.I)]
    passed=(all(markers.values()) and not warnings and not errors and not forbidden
        and trace["algorithm_before"]==trace["algorithm_after"]==1
        and deltas["native_calls"]>0 and deltas["evaluations"]==deltas["native_calls"]
        and deltas["fallback_calls"]==deltas["native_failure"]==0 and not after["python_failure"])
    return dict(passed=bool(passed),algorithm=1,backend="standard Delaunay insertion",markers=markers,
        orchestration_messages=[message for message in messages if re.search(r"meshing [12]d",message,re.I)],
        callback_deltas=deltas,warnings=warnings,errors=errors,forbidden_messages=forbidden,
        iteration_limit_reached="Max. number of iterations reached" in joined,
        insertion_diagnostics=[message for message in messages if re.search(r"nodes could not be inserted|tetrahedra created|refinement terminated|cavities modified",message)],
        requested_threads=4,actual_parallelism="NOT_MEASURED",
        scope="Source-dispatch and generation-scoped log evidence plus option readback and query deltas; not binary instrumentation or universal algorithm proof.")

def execution_tests():
    before=dict(evaluations=0,native_calls=0,fallback_calls=0,native_failure=0,python_failure=False)
    after=dict(before,evaluations=3,native_calls=3)
    messages=["Info: 3D refinement terminated (4 nodes total):","Info: 0 Delaunay cavities modified for star shapeness",
        "Info: 0 nodes could not be inserted","Info: 1 tetrahedra created in 1 sec."]
    trace=dict(messages=messages,callback_before=before,callback_after=after,algorithm_before=1,algorithm_after=1)
    assert execution_evidence(trace)["passed"]
    assert execution_evidence(dict(trace,messages=messages+["Info: Meshing 1D...","Info: Meshing 2D..."]))["passed"]
    for bad in ("Info: Meshing surface 2 (Discrete surface)","Info: Meshing curve 4 (Copy)","Info: HXT","Warning: unsupported","Error: Wrong mesh element size","Info: falling back"):
        assert not execution_evidence(dict(trace,messages=messages+[bad]))["passed"]
    assert not execution_evidence(dict(trace,messages=messages[:-1]))["passed"]
    assert not execution_evidence(dict(trace,callback_after=before))["passed"]
    assert not execution_evidence(dict(trace,algorithm_after=10))["passed"]
    for key,value in (("fallback_calls",1),("native_failure",1),("python_failure",True)):
        assert not execution_evidence(dict(trace,callback_after=dict(after,**{key:value})))["passed"]
    assert execution_evidence(dict(trace,messages=messages+["Info: Max. number of iterations reached"]))["iteration_limit_reached"]
    return dict(passed=True,backend_markers=True,headers_diagnostic=True,entitywork_fallback_warning_error_refused=True,
        positive_native_queries_required=True,boundary=boundary_tests(),negative_closure=terminal_tests())

def boundary_snapshot(module):
    record=dict(entities={},storage_owners={},direct_owners={},duplicate_storage_tags=[],all_node_tags=[],
        boundary_incidence=[],volume_boundary={},boundary_nodes={},capture_errors=[])
    try:_capture_boundary(module,record)
    except BaseException as error:record["capture_errors"].append(repr(error))
    return record

def _capture_boundary(module,record):
    tags,raw,_=module.model.mesh.getNodes();tags=np.asarray(tags,dtype=np.int64)
    record["all_node_tags"]=sorted(map(int,tags))
    points=np.asarray(raw,dtype="<f8").reshape(-1,3)
    assert len(points)==len(tags) and len(np.unique(tags))==len(tags) and np.isfinite(points).all()
    coordinates={int(tag):point for tag,point in zip(tags,points)}
    entities=record["entities"];storage=record["storage_owners"];direct=record["direct_owners"]
    boundary_incidence=set();duplicates=record["duplicate_storage_tags"]
    for dim,entity in module.model.getEntities():
        node_tags,_,_=module.model.mesh.getNodes(dim,entity,includeBoundary=False)
        for tag in node_tags:
            key=str(int(tag))
            if key in storage:duplicates.append(key)
            storage[key]=[dim,entity]
        if dim>2:continue
        kinds,element_tags,connectivity=module.model.mesh.getElements(dim,entity)
        elements={};element_ids={}
        for kind,ids,connected in zip(kinds,element_tags,connectivity):
            count=module.model.mesh.getElementProperties(int(kind))[3]
            rows=np.asarray(connected,dtype=np.int64).reshape(-1,count)
            if len(ids)!=len(rows) or len(set(map(int,ids)))!=len(ids):record["capture_errors"].append("InvalidElementTags")
            boundary_incidence.update(map(int,rows.ravel()))
            record["boundary_incidence"]=sorted(boundary_incidence)
            canonical=cycle_keys(rows) if int(kind)==2 else sorted(tuple(map(int,row)) for row in rows)
            elements[str(int(kind))]=[list(map(int,row)) for row in canonical]
            element_ids[str(int(kind))]=sorted(map(int,ids))
        upward,downward=module.model.getAdjacencies(dim,entity)
        entities[str(dim)+":"+str(entity)]=dict(type=module.model.getType(dim,entity),
            nodes=sorted(map(int,node_tags)),elements=elements,element_tags=element_ids,
            boundary=sorted([list(map(int,row)) for row in module.model.getBoundary([(dim,entity)],combined=False,oriented=True,recursive=False)]),
            upward=sorted(map(int,upward)),downward=sorted(map(int,downward)))
    for tag in tags:
        coordinate,_,dim,entity=module.model.mesh.getNode(int(tag))
        direct[str(int(tag))]=[int(dim),int(entity)]
        if np.asarray(coordinate,dtype="<f8").tobytes()!=coordinates[int(tag)].tobytes():record["capture_errors"].append("NodeCoordinateReadbackMismatch:"+str(int(tag)))
    boundary_ids=boundary_incidence|{int(tag) for tag,owner in direct.items() if owner[0]<3}
    volume_boundary={str(entity):sorted([list(map(int,row)) for row in module.model.getBoundary([(dim,entity)],combined=False,oriented=True,recursive=False)])
        for dim,entity in module.model.getEntities(3)}
    record["volume_boundary"]=volume_boundary
    record["boundary_nodes"]={str(tag):[float(value).hex() for value in coordinates[tag]] for tag in sorted(boundary_ids)}

def boundary_difference(before,after):
    def canonical_entities(record):return {key:{field:value for field,value in entity.items() if field!="element_tags"} for key,entity in record["entities"].items()}
    def valid(record):
        tags=set(map(str,record["all_node_tags"]))
        faces=[tuple(sorted(face)) for entity in record["entities"].values() for face in entity["elements"].get("2",[])]
        element_ids=[tag for entity in record["entities"].values() for ids in entity["element_tags"].values() for tag in ids]
        return (not record.get("capture_errors") and not record["duplicate_storage_tags"] and len(tags)==len(record["all_node_tags"])
            and tags==set(record["storage_owners"])==set(record["direct_owners"])
            and record["storage_owners"]==record["direct_owners"] and len(faces)==len(set(faces)) and len(element_ids)==len(set(element_ids)))
    added=sorted(set(after["all_node_tags"])-set(before["all_node_tags"]))
    added_boundary=[tag for tag in added if str(tag) in after["boundary_nodes"]]
    added_interior=[tag for tag in added if str(tag) not in after["boundary_nodes"] and after["direct_owners"].get(str(tag),[-1])[0]==3]
    removed=sorted(set(before["all_node_tags"])-set(after["all_node_tags"]))
    changed_nodes=[tag for tag in sorted(set(before["boundary_nodes"])|set(after["boundary_nodes"]))
        if before["boundary_nodes"].get(tag)!=after["boundary_nodes"].get(tag)
        or before["direct_owners"].get(tag)!=after["direct_owners"].get(tag)]
    checks=dict(pre_ownership=valid(before),post_ownership=valid(after),
        exact_boundary_nodes=not changed_nodes,exact_entities=canonical_entities(before)==canonical_entities(after),
        exact_incidence=before["boundary_incidence"]==after["boundary_incidence"],
        exact_volume_boundary=before["volume_boundary"]==after["volume_boundary"],
        no_removed_nodes=not removed,no_added_boundary_nodes=not added_boundary,
        new_nodes_classified=len(added)==len(added_boundary)+len(added_interior))
    return dict(passed=all(checks.values()),checks=checks,changed_boundary_node_tags=changed_nodes,
        added_boundary_node_tags=added_boundary,added_interior_node_tags=added_interior,removed_node_tags=removed,
        element_tags_equal={key:before["entities"].get(key,{}).get("element_tags")==after["entities"].get(key,{}).get("element_tags")
            for key in sorted(set(before["entities"])|set(after["entities"]))},
        scope="Exact binary64 boundary coordinates, node tags/classification, entity topology and oriented element connectivity; element numbering reported separately. All retained new nodes classified, not a unique recovery-Steiner identification.")

def boundary_tests():
    import copy
    owners={str(tag):[2,1] for tag in (1,2,3)}
    base=dict(entities={"2:1":dict(type="Discrete surface",nodes=[1,2,3],elements={"2":[[1,2,3]]},element_tags={"2":[1]},boundary=[],upward=[1],downward=[])},
        storage_owners=owners,direct_owners=copy.deepcopy(owners),duplicate_storage_tags=[],all_node_tags=[1,2,3],
        boundary_incidence=[1,2,3],volume_boundary={"1":[[2,1]]},boundary_nodes={str(tag):["0x0.0p+0"]*3 for tag in (1,2,3)})
    interior=copy.deepcopy(base);interior["all_node_tags"].append(4)
    for field in ("storage_owners","direct_owners"):interior[field]["4"]=[3,1]
    assert boundary_difference(base,interior)["passed"]
    for mutation in ("coordinate","face","entity","owner","boundary_addition"):
        changed=copy.deepcopy(interior)
        if mutation=="coordinate":changed["boundary_nodes"]["1"][0]="0x1.0p+0"
        elif mutation=="face":changed["entities"]["2:1"]["elements"]["2"]=[[1,3,2]]
        elif mutation=="entity":changed["entities"]["2:2"]=changed["entities"].pop("2:1")
        elif mutation=="owner":changed["direct_owners"]["1"]=[3,1]
        else:
            changed["boundary_nodes"]["4"]=["0x0.0p+0"]*3
            changed["direct_owners"]["4"]=changed["storage_owners"]["4"]=[2,1]
        assert not boundary_difference(base,changed)["passed"]
    return dict(passed=True,interior_addition_allowed=True,exact_boundary_changes_refused=5,acquisition=acquisition_tests())

def acquisition_tests():
    from types import SimpleNamespace
    class Mesh:
        def __init__(self):
            self.points={1:[0.,0.,0.],2:[1.,0.,0.],3:[0.,1.,0.]}
            self.owners={1:(2,1),2:(2,1),3:(2,1)};self.storage=dict(self.owners)
            self.face=[1,2,3];self.element_tag=1;self.truncate=False;self.duplicate=False
        def getNodes(self,dim=-1,entity=-1,includeBoundary=False):
            tags=sorted(self.points) if dim==-1 else [tag for tag,owner in self.storage.items() if owner==(dim,entity)]
            if self.duplicate and dim==3:tags.append(1)
            raw=[value for tag in tags for value in self.points[tag]]
            return tags,raw[:-3] if self.truncate and dim==-1 else raw,[]
        def getNode(self,tag):return self.points[tag],[],*self.owners[tag]
        def getElements(self,dim,entity):return ([2],[[self.element_tag]],[self.face]) if dim==2 else ([],[],[])
        def getElementProperties(self,kind):return ("Triangle",2,1,3,[],3)
    mesh=Mesh()
    model=SimpleNamespace(mesh=mesh,getEntities=lambda dim=-1:[pair for pair in [(0,1),(1,1),(2,1),(3,1)] if dim==-1 or pair[0]==dim],
        getType=lambda dim,entity:"Discrete",getAdjacencies=lambda dim,entity:([],[]),
        getBoundary=lambda entities,**kwargs:[(2,1)] if entities==[(3,1)] else [])
    module=SimpleNamespace(model=model);before=boundary_snapshot(module)
    assert boundary_difference(before,before)["passed"]
    mesh.points[4]=[0.,0.,1.];mesh.owners[4]=mesh.storage[4]=(3,1)
    assert boundary_difference(before,boundary_snapshot(module))["passed"]
    mesh.element_tag=20;assert boundary_difference(before,boundary_snapshot(module))["passed"]
    mesh.face=[1,2,4];assert not boundary_difference(before,boundary_snapshot(module))["passed"];mesh.face=[1,2,3]
    mesh.storage[4]=(2,1);assert not boundary_difference(before,boundary_snapshot(module))["passed"];mesh.storage[4]=(3,1)
    del mesh.storage[4];assert not boundary_difference(before,boundary_snapshot(module))["passed"];mesh.storage[4]=(3,1)
    mesh.duplicate=True;assert not boundary_difference(before,boundary_snapshot(module))["passed"];mesh.duplicate=False
    mesh.truncate=True;assert boundary_snapshot(module)["capture_errors"]
    return dict(passed=True,dim0_to3=True,interior_and_element_renumbering_allowed=True,incidence_ownership_coverage_and_length_refused=True)

def terminal_checks(operations):
    records={}
    for name,operation in operations:
        try:operation();records[name]=dict(passed=True)
        except BaseException as error:records[name]=dict(passed=False,error=repr(error))
    return dict(passed=all(row["passed"] for row in records.values()),checks=records)

def terminal_tests():
    visited=[]
    def refused():visited.append("refused");raise ValueError("control")
    record=terminal_checks([("raw",refused),("boundary",lambda:visited.append("boundary")),("closure",lambda:visited.append("closure"))])
    assert visited==["refused","boundary","closure"] and not record["passed"] and record["checks"]["closure"]["passed"]
    return dict(passed=True,failed_raw_does_not_suppress_boundary=True,negative_path_closure_attempted=True)
