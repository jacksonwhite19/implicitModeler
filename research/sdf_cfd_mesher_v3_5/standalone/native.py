"""V3 explicit-displacement physical construction and native identity readback."""
from pathlib import Path
import csv, hashlib, json, shutil, subprocess, time,re,shlex,os
import numpy as np
from build_native import ROOT, linux

HEADER='FoamFile\n{{\nformat ascii;\nclass {klass};\nobject {obj};\n}}\n'
def counted(lines):return str(len(lines))+'\n(\n'+'\n'.join(lines)+'\n)'
def vec(v):return '('+' '.join(format(float(x),'.17g') for x in v)+')'
def vf(a):return counted([vec(v) for v in a])
def lf(a):return counted([str(int(v)) for v in a])
def put(p,text):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text)

def execute(name,case,seconds,extra=(),*,trace_files=False):
    return operation(linux(ROOT/'native_bin'/name),['-case',linux(case),*extra],case,seconds,trace_files=trace_files,name=name)

def operation(program,args,case,seconds=600,*,trace_files=False,name=None):
    name=name or Path(program).name
    scope=case/(name+'.linux_scope.json');payload=case/(name+'.launch.sh')
    argv=[program,*map(str,args)]
    limit=int(os.environ.get('V35_LINUX_AS_BYTES',16*2**30))
    trace=('strace -f -e trace=file -o '+shlex.quote(linux(case/(name+'.file_trace.log')))+' ') if trace_files else ''
    # Resolve utility names in the sourced Foundation environment, then record
    # exact argv under a new owned process group before exec.
    encoded=json.dumps(dict(argv=argv),separators=(',',':'))[:-1]
    script='#!/bin/bash\nset -e\n'
    script+='printf '+shlex.quote(encoded+',"pid":%d,"start_ticks":%s}\n')+' "$$" "$(awk \'{print $22}\' /proc/$$/stat)" > '+shlex.quote(linux(scope))+'\n'
    script+='exec prlimit --as='+str(limit)+' -- '+trace+'timeout --kill-after=5s '+str(seconds)+'s '+' '.join(shlex.quote(x) for x in argv)+'\n'
    payload.write_bytes(script.encode())
    command=['wsl','-d','Ubuntu-22.04','--','bash','-lc','source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; exec setsid --wait bash '+shlex.quote(linux(payload))]
    start=time.perf_counter()
    with (case/(name+'.stdout')).open('w') as out,(case/(name+'.stderr')).open('w') as err:
        proc=subprocess.run(command,stdout=out,stderr=err,timeout=seconds+20)
    result=dict(command=command,seconds=time.perf_counter()-start,exit_code=proc.returncode,linux_address_space_limit_bytes=limit)
    put(case/(name+'.json'),json.dumps(result,indent=2))
    if proc.returncode:raise RuntimeError('NativeConsumerFailed:'+name)
    return result

def construct(positions,faces,case,*,seconds=600,trace_files=False):
    positions=np.asarray(positions,dtype='<f8');faces=np.asarray(faces,dtype='<i8');case=Path(case).resolve()
    if case.exists():raise ValueError('FreshNativeOutputRequired')
    if positions.ndim!=3 or positions.shape[2]!=3 or len(positions)<3 or not np.isfinite(positions).all():raise ValueError('InvalidNativePositions')
    if faces.ndim!=2 or faces.shape[1]!=3 or faces.min()<0 or faces.max()>=positions.shape[1]:raise ValueError('InvalidNativeFaces')
    case.mkdir(parents=True)
    p=positions[0];delta=np.subtract(positions[1:],positions[:-1],dtype='<f8');F=len(faces);V=len(p)
    tolerance=(2*len(delta)+8)*np.finfo(float).eps/(1-(2*len(delta)+8)*np.finfo(float).eps)*np.max(abs(p)+np.sum(abs(delta),axis=0))
    if tolerance>1e-12:raise ValueError('NativeArithmeticReserveExceeded')
    # This file is complete before the first physical insertion.
    np.savez(case/'SEALED_PLAN.npz',positions=positions,faces=faces,delta=delta)
    put(case/'SEALED_PLAN.json',json.dumps(dict(sha256=hashlib.sha256((case/'SEALED_PLAN.npz').read_bytes()).hexdigest(),layers=len(delta),arithmetic_tolerance_m=tolerance,construction='V3StockFoundationStagedInsertion/1')))
    mesh=case/'constant/polyMesh'
    for name,klass,body in [('points','vectorField',vf(p)),('faces','faceList',counted(['3('+' '.join(map(str,t))+')' for t in faces])),('owner','labelList',lf(np.zeros(F,dtype=int))),('neighbour','labelList',counted([])),('boundary','polyBoundaryMesh','2\n(\nouterFront\n{type patch;nFaces '+str(F)+';startFace 0;}\nphysicalWall\n{type wall;nFaces 0;startFace '+str(F)+';}\n)')]:put(mesh/name,HEADER.format(klass=klass,obj=name)+body+'\n')
    put(case/'system/controlDict',HEADER.format(klass='dictionary',obj='controlDict')+'application contractStaged;startFrom startTime;startTime 0;stopAt endTime;endTime 1;deltaT 1;writeControl timeStep;writeInterval 1;writeFormat binary;writePrecision 17;writeCompression off;runTimeModifiable false;\n')
    body=['sourcePatches (outerFront);','exposedPatchName physicalWall;','outputRegion physicalLayer;','meshFaceLayers '+lf(np.ones(F,dtype=int))+';','meshPointLayers '+lf(np.ones(V,dtype=int))+';','meshPointExpansionRatio '+counted(['1']*V)+';','meshPointFirstLayerDisp '+vf(delta[0])+';']
    put(case/'system/directAddPatchCellLayerDict',HEADER.format(klass='dictionary',obj='directAddPatchCellLayerDict')+'\n'.join(body))
    first=execute('contractDirect',case,seconds,trace_files=trace_files)
    source=case/'0/physicalLayer/polyMesh'
    if not source.is_dir():raise RuntimeError('PhysicalRegionMissing')
    for f in list(mesh.iterdir()):
        if f.is_file():
            (case/'source_owner_package').mkdir(exist_ok=True);shutil.copyfile(f,case/'source_owner_package'/f.name)
    for f in source.iterdir():
        if f.is_file():shutil.copyfile(f,mesh/f.name)
    with (case/'postProcessing/directLayer/pointAncestry.csv').open() as stream:
        rows=list(csv.DictReader(stream))
    pa=np.full(2*V,-1,dtype='<i4');pl=pa.copy()
    for r in rows:
        src=int(r['sourceMeshPoint']);base=int(r['baseOutputPoint']);top=int(r['addedPointIds']);pa[[base,top]]=src;pl[base]=0;pl[top]=1
    if (pa<0).any() or len(set(zip(pa,pl)))!=2*V:raise ValueError('FirstPointAncestryMismatch')
    ca=np.full(F,-1,dtype='<i4');outer={}
    with (case/'postProcessing/directLayer/faceAncestry.csv').open() as stream:
        face_rows=list(csv.DictReader(stream))
    for r in face_rows:
        src=int(r['sourceMeshFace']);cell=int(r['addedCellIds']);outerface=int(r['layerFaceIds'].split('|')[-1]);ca[cell]=src;outer[outerface]=src
    if (ca<0).any():raise ValueError('FirstCellAncestryMismatch')
    body=['sourcePatch outerFront;','stageDisplacements '+counted([vf(z) for z in delta])+';','expectedFronts '+counted([vf(z) for z in positions])+';','sourceFaces '+counted([lf(t) for t in faces])+';','coordinateTolerance '+format(tolerance,'.17g')+';','initialPointAncestors '+lf(pa)+';','initialPointLevels '+lf(pl)+';','initialCellAncestors '+lf(ca)+';','initialPatchFaceAncestors '+lf([outer[k] for k in sorted(outer)])+';']
    put(case/'system/stagedAddPatchCellLayerDict',HEADER.format(klass='dictionary',obj='stagedAddPatchCellLayerDict')+'\n'.join(body)+'\n')
    later=execute('contractStaged',case,seconds,trace_files=trace_files)
    last=case/'postProcessing/stagedLayer'/('prefix_'+str(len(delta)))
    native=readback(last,positions,faces,tolerance)
    report=dict(passed=True,first=first,later=later,physical_prisms=F*len(delta),carrier_fluid_cells=0,points=len(native['points']),max_coordinate_error_m=native['max_coordinate_error_m'],minimum_native_volume_m3=float(native['volumes'].min()),scope='Native identity/construction only; not V4 shell admission')
    put(case/'NATIVE_RESULT.json',json.dumps(report,indent=2))
    return native,report

def readback(folder,positions,faces,tolerance):
    def raw(name,dtype):return np.fromfile(folder/name,dtype=dtype)
    points=raw('points.f64','<f8').reshape(-1,3);anc=raw('point_ancestor.i32','<i4');level=raw('point_level.i32','<i4')
    ca=raw('cell_ancestor.i32','<i4');cl=raw('cell_level.i32','<i4');vol=raw('cell_volumes.f64','<f8');off=raw('cell_offsets.u64','<u8');cp=raw('cell_points.i32','<i4')
    n=positions.shape[1];N=len(positions)-1;F=len(faces)
    if not np.isfinite(tolerance) or not 0<=tolerance<=1e-12 or not np.isfinite(positions).all() or not np.isfinite(points).all():raise ValueError('NonfiniteNativeCoordinates')
    if len(points)!=(N+1)*n or len(vol)!=N*F or len(anc)!=len(points) or len(ca)!=N*F:raise ValueError('NativePhysicalCountsMismatch')
    if len(level)!=len(points) or len(cl)!=len(vol) or len(off)!=len(vol)+1 or off[0]!=0 or off[-1]!=len(cp) or np.any(off[1:]<off[:-1]) or np.any((cp<0)|(cp>=len(points))):raise ValueError('NativeArrayStructureMismatch')
    if np.any((anc<0)|(anc>=n)|(level<0)|(level>N)) or len(set(zip(level,anc)))!=len(points):raise ValueError('NativePointIdentityMismatch')
    expected=positions[level,anc]
    difference=float(abs(points-expected).max())
    if difference>tolerance:raise ValueError('NativeCoordinateMismatch')
    if np.any((ca<0)|(ca>=F)|(cl<1)|(cl>N)) or len(set(zip(cl,ca)))!=len(vol):raise ValueError('NativeCellIdentityMismatch')
    for i in range(len(vol)):
        ids=cp[int(off[i]):int(off[i+1])]
        if len(ids)!=6 or set(zip(level[ids],anc[ids]))!=set((k,int(v)) for k in (cl[i]-1,cl[i]) for v in faces[ca[i]]):raise ValueError('NativeConnectivityMismatch')
    if not np.isfinite(vol).all() or np.any(vol<=0):raise ValueError('NativeNonpositiveVolume')
    ordered=np.empty_like(positions);ordered[level,anc]=points
    # Bind actual native face cycles and both cell owners, not only six-point
    # sets. Every expected wedge face must occur once in the native polyMesh.
    fo=raw('face_offsets.u64','<u8');fp=raw('face_points.i32','<i4');own=raw('owner.i32','<i4');nei=raw('neighbour.i32','<i4')
    if len(fo)!=len(own)+1 or fo[0]!=0 or fo[-1]!=len(fp) or np.any(fo[1:]<fo[:-1]) or np.any((fp<0)|(fp>=len(points))) or len(nei)>len(own) or np.any((own<0)|(own>=len(vol))) or np.any((nei<0)|(nei>=len(vol))):raise ValueError('NativeFaceStructureMismatch')
    point_map=level.astype('<i8')*n+anc
    patterns=((0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5))
    expected_faces={}
    for i in range(len(vol)):
        row=np.r_[(cl[i]-1)*n+faces[ca[i]],cl[i]*n+faces[ca[i]]]
        for pattern in patterns:
            cycle=tuple(map(int,row[list(pattern)]));key=tuple(sorted(cycle))
            expected_faces.setdefault(key,[]).append((i,cycle))
    def same_cycle(a,b):return len(a)==len(b) and any(tuple(a)==tuple(b[j:]+b[:j]) for j in range(len(b)))
    seen=set();native_faces=[];boundary_kind={}
    for i in range(len(own)):
        cycle=tuple(map(int,point_map[fp[int(fo[i]):int(fo[i+1])]]));key=tuple(sorted(cycle));rows=expected_faces.get(key,())
        if key in seen or len(rows)!=(2 if i<len(nei) else 1):raise ValueError('NativeFaceIncidenceMismatch')
        owners={cid:c for cid,c in rows}
        if int(own[i]) not in owners or not same_cycle(cycle,owners[int(own[i])]):raise ValueError('NativeOwnerCycleMismatch')
        if i<len(nei):
            if own[i]==nei[i] or int(nei[i]) not in owners or not same_cycle(cycle[::-1],owners[int(nei[i])]):raise ValueError('NativeNeighbourCycleMismatch')
        else:
            levels={v//n for v in cycle}
            if levels=={0}:boundary_kind[i]='physicalWall'
            elif levels=={N}:boundary_kind[i]='outerFront'
            else:raise ValueError('UnexpectedNativePhysicalBoundary')
        seen.add(key);native_faces.append(cycle)
    if seen!=set(expected_faces):raise ValueError('MissingNativeFace')
    boundary=(folder.parents[2]/'constant/polyMesh/boundary').read_text(errors='strict')
    patches={}
    for name,body in re.findall(r'(\w+)\s*\{([^}]+)\}',boundary):
        start=re.search(r'\bstartFace\s+(\d+)\s*;',body);count=re.search(r'\bnFaces\s+(\d+)\s*;',body);kind=re.search(r'\btype\s+(\w+)\s*;',body)
        if start and count and kind:patches[name]=(int(start[1]),int(count[1]),kind[1])
    expected_boundary=set(range(len(nei),len(own)));actual_boundary=set()
    for name,(start,count,kind) in patches.items():
        if name not in ('physicalWall','outerFront') or kind!=('wall' if name=='physicalWall' else 'patch'):raise ValueError('NativeBoundaryLabelMismatch')
        for i in range(start,start+count):
            if i in actual_boundary or boundary_kind.get(i)!=name:raise ValueError('NativeBoundaryIncidenceMismatch')
            actual_boundary.add(i)
    if actual_boundary!=expected_boundary:raise ValueError('MissingNativeBoundaryLabel')
    return dict(points=points,positions=ordered,point_ancestor=anc,point_level=level,cell_ancestor=ca,cell_level=cl,volumes=vol,max_coordinate_error_m=difference,folder=str(folder),faces=native_faces,owner=own,neighbour=nei,boundary_patches=patches)
