"""Copied V4 export contracts: exact transfer, Foundation readback, checkMesh."""
from pathlib import Path
import pathlib,ast,hashlib,json,re,subprocess,shlex,time
import numpy as np
from build_native import ROOT,linux
from native import execute,operation as owned_operation

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def functions():
    namespace=dict(pathlib=pathlib,sha=sha)
    for name,names in [('run_farfield_complete.py',{'write_msh'}),('run_complete_stage.py',{'foam_header','create_case'})]:
        source=ROOT/'export_sources'/name
        nodes=[n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name in names]
        if {n.name for n in nodes}!=names:raise ValueError('V4ExportFunctionClosureChanged')
        exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec'),namespace)
    return namespace

def operation(program,args,folder,seconds=600,*,trace_files=False):
    result=owned_operation(program,args,folder,seconds,trace_files=trace_files)
    return dict(result,returncode=result['exit_code'])

def export(archive,folder,*,trace_files=False,_deadline=None):
    from finite_research.direct_evidence_modules.export_adapter import mapped_cell_order,checkmesh_result,cyclic
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    with np.load(archive,allow_pickle=False) as arrays:
        points=arrays['points'];prisms=arrays['prisms'];tets=arrays['tetrahedra'];wall=arrays['wall'];far=arrays['farfield']
    f=functions();transfer=folder/'complete_transfer.msh';case=folder/'openfoam_case';audit=folder/'foundation_audit';audit.mkdir()
    writer=f['write_msh'](transfer,points,wall,far,prisms,tets);f['create_case'](case)
    def allowance():
        remaining=600 if _deadline is None else min(600,_deadline-time.monotonic())
        if remaining<1:raise TimeoutError('V35SharedNativeExportDeadline')
        return int(remaining)
    commands={};commands['gmshToFoam']=operation('gmshToFoam',[linux(transfer),'-case',linux(case)],folder,seconds=allowance(),trace_files=trace_files)
    if commands['gmshToFoam']['returncode']:raise ValueError('V35GmshToFoamRefusal')
    commands['foundationAudit']=execute('contractAudit',case,allowance(),('-outputDir',linux(audit)),trace_files=trace_files)
    commands['checkMesh']=operation('checkMesh',['-constant','-allGeometry','-allTopology','-case',linux(case)],folder,seconds=allowance(),trace_files=trace_files)
    check=checkmesh_result((folder/'checkMesh.stdout').read_text(),(folder/'checkMesh.stderr').read_text(),commands['checkMesh']['returncode'])
    def raw(name,dtype):return np.fromfile(audit/name,dtype=dtype)
    foam_points=raw('points.f64','<f8').reshape(-1,3)
    if not np.isfinite(foam_points).all() or len(foam_points)!=len(points):raise ValueError('ExportPointInventoryMismatch')
    lookup={p.tobytes():i for i,p in enumerate(points)}
    if len(lookup)!=len(points):raise ValueError('AmbiguousExportPointIdentity')
    mapping=np.array([lookup[p.tobytes()] for p in foam_points],dtype='<i8')
    if len(np.unique(mapping))!=len(points):raise ValueError('NonbijectiveExportPointIdentity')
    off=raw('cell_offsets.u64','<u8');cp=raw('cell_points.i32','<i4');cells=list(prisms)+list(tets)
    cell_map=mapped_cell_order(cells,off,cp,mapping,lambda:None)
    patterns_prism=((0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5))
    patterns_tet=((0,2,1),(0,1,3),(1,2,3),(2,0,3))
    expected={}
    for i,cell in enumerate(cells):
        for pattern in (patterns_prism if len(cell)==6 else patterns_tet):
            cycle=tuple(map(int,np.asarray(cell)[list(pattern)]));expected.setdefault(tuple(sorted(cycle)),[]).append((i,cycle))
    fo=raw('face_offsets.u64','<u8');fp=raw('face_points.i32','<i4');own=raw('owner.i32','<i4');nei=raw('neighbour.i32','<i4');seen=set();native_boundary={}
    for i in range(len(own)):
        cycle=tuple(map(int,mapping[fp[int(fo[i]):int(fo[i+1])]]));key=tuple(sorted(cycle));rows=expected.get(key,())
        if key in seen or len(rows)!=(2 if i<len(nei) else 1):raise ValueError('ExportFaceIncidenceMismatch')
        owners={cid:c for cid,c in rows};owner=int(cell_map[own[i]])
        if owner not in owners or not cyclic(cycle,owners[owner]):raise ValueError('ExportOwnerWindingMismatch')
        if i<len(nei):
            neighbour=int(cell_map[nei[i]])
            if owner==neighbour or neighbour not in owners or not cyclic(cycle[::-1],owners[neighbour]):raise ValueError('ExportNeighbourWindingMismatch')
        else:native_boundary[i]=key
        seen.add(key)
    if seen!=set(expected):raise ValueError('MissingExportFace')
    boundary=(case/'constant/polyMesh/boundary').read_text();patches={}
    for name,body in re.findall(r'(\w+)\s*\{([^}]+)\}',boundary):
        start=re.search(r'\bstartFace\s+(\d+)\s*;',body);count=re.search(r'\bnFaces\s+(\d+)\s*;',body)
        if start and count:patches[name]=(int(start[1]),int(count[1]))
    intended={'wall':{tuple(sorted(map(int,x))) for x in wall},'farfield':{tuple(sorted(map(int,x))) for x in far}}
    if set(patches)!=set(intended):raise ValueError('ExportBoundaryLabelsMismatch')
    for name,(start,count) in patches.items():
        if {native_boundary[i] for i in range(start,start+count)}!=intended[name]:raise ValueError('ExportBoundaryConnectivityMismatch')
    np.savez(folder/'OPENFOAM_MAPPING.npz',point_to_source=mapping,cell_to_source=cell_map)
    report=dict(passed=check['passed'],conversion_fidelity=True,point_bijection_exact=True,cell_bijection_exact=True,all_face_incidence_and_winding_exact=True,boundary_labels=list(patches),writer=writer,checkmesh=check,commands=commands,source_archive_sha256=sha(archive),mapping_sha256=sha(folder/'OPENFOAM_MAPPING.npz'),counts=dict(points=len(points),prisms=len(prisms),tetrahedra=len(tets)),CFD_qualified=False)
    (folder/'EXPORT_RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
    if not check['passed']:raise ValueError('LiteralFoundationMeshOKRequired')
    return report
