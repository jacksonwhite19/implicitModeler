from study import *
from displacement_record import load_record
import csv

HEADER='FoamFile\n{{\nformat ascii;\nclass {klass};\nobject {obj};\n}}\n'
def counted(lines):return str(len(lines))+'\n(\n'+'\n'.join(lines)+'\n)'
def vec(v):return '('+' '.join(format(float(z),'.17g') for z in v)+')'
def vf(a):return counted([vec(x) for x in a])
def lf(a):return counted([str(int(x)) for x in a])
def put(p,s):safe(p).write_bytes(s.encode())
def control(case):put(case/'system/controlDict',HEADER.format(klass='dictionary',obj='controlDict')+'application contractStaged;\nstartFrom startTime;startTime 0;stopAt endTime;endTime 1;deltaT 1;writeControl timeStep;writeInterval 1;writeFormat binary;writePrecision 17;writeCompression off;runTimeModifiable false;\n')

def prepare_first(record,case):
 m,a=load_record(record);case=safe(case);case.mkdir(exist_ok=False);x=a['positions'];f=a['faces'];p=x[0];F=len(f);V=len(p)
 mesh=case/'constant/polyMesh'
 for name,klass,body in [('points','vectorField',vf(p)),('faces','faceList',counted(['3('+' '.join(map(str,t))+')' for t in f])),('owner','labelList',lf(np.zeros(F,dtype=int))),('neighbour','labelList',counted([])),('boundary','polyBoundaryMesh','2\n(\nouterFront\n{type patch;nFaces '+str(F)+';startFace 0;}\nphysicalWall\n{type wall;nFaces 0;startFace '+str(F)+';}\n)')]:put(mesh/name,HEADER.format(klass=klass,obj=name)+body+'\n')
 control(case)
 body=['sourcePatches (outerFront);','exposedPatchName physicalWall;','outputRegion physicalLayer;','meshFaceLayers '+lf(np.ones(F,dtype=int))+';','meshPointLayers '+lf(np.ones(V,dtype=int))+';','meshPointExpansionRatio '+counted(['1']*V)+';','meshPointFirstLayerDisp '+vf(a['delta'][0])+';']
 put(case/'system/directAddPatchCellLayerDict',HEADER.format(klass='dictionary',obj='directAddPatchCellLayerDict')+'\n'.join(body))
 write(case/'OWNERSHIP.json',dict(source_owner_package='One complete closed production-style solid; numerical source owner only.',output='Standalone physical prism region; zero carrier cells.',wall='physicalWall',current_exterior='outerFront'))
 return m,a

def exec_case(exe,case,tag,extra=''):
 script=HERE/'runtime'/f'{tag}.sh'
 put(script,'#!/bin/bash\nsource /opt/openfoam13/etc/bashrc >/dev/null 2>&1\nset -e\n"'+wsl(HERE/'bin'/exe)+'" -case "'+wsl(case)+'" '+extra+'\n')
 bash(script,tag)

def copy_first_to_physical(record,case):
 m,a=load_record(record);V=len(a['positions'][0]);F=len(a['faces'])
 source=case/'0/physicalLayer/polyMesh';target=case/'constant/polyMesh'
 assert source.is_dir()
 # Retain original numerical source files; no recursive deletion.
 if not (case/'source_owner_package').exists():
  for p in sorted(target.iterdir()):
   if p.is_file():shutil.copyfile(p,safe(case/'source_owner_package'/p.name))
 for p in sorted(source.iterdir()):
  if p.is_file():shutil.copyfile(p,target/p.name)
 dump=case/'postProcessing/stagedLayer/prefix_1';dump.mkdir(parents=True,exist_ok=True)
 rows=list(csv.DictReader((case/'postProcessing/directLayer/pointAncestry.csv').open()));pa=np.full(2*V,-1,dtype='<i4');pl=np.full(2*V,-1,dtype='<i4')
 for r in rows:
  src=int(r['sourceMeshPoint']);base=int(r['baseOutputPoint']);top=int(r['addedPointIds']);pa[[base,top]]=src;pl[base]=0;pl[top]=1
 assert (pa>=0).all() and len(set(zip(pa,pl)))==2*V
 ca=np.full(F,-1,dtype='<i4');outer={}
 for r in csv.DictReader((case/'postProcessing/directLayer/faceAncestry.csv').open()):
  src=int(r['sourceMeshFace']);cell=int(r['addedCellIds']);outerface=int(r['layerFaceIds'].split('|')[-1]);ca[cell]=src;outer[outerface]=src
 assert (ca>=0).all()
 # The native outer patch is ordered by its actual global face index.
 patchfaces=[outer[k] for k in sorted(outer)]
 for name,val in [('point_ancestor',pa),('point_level',pl),('cell_ancestor',ca),('cell_level',np.ones(F,dtype='<i4'))]:val.tofile(dump/(name+'.i32'))
 disps=counted([vf(z) for z in a['delta']]);expected=counted([vf(z) for z in a['positions']])
 body=['sourcePatch outerFront;','stageDisplacements '+disps+';','expectedFronts '+expected+';','sourceFaces '+counted([lf(t) for t in a['faces']])+';','coordinateTolerance '+format(m['native_coordinate_tolerance_m'],'.17g')+';','initialPointAncestors '+lf(pa)+';','initialPointLevels '+lf(pl)+';','initialCellAncestors '+lf(ca)+';','initialPatchFaceAncestors '+lf(patchfaces)+';']
 put(case/'system/stagedAddPatchCellLayerDict',HEADER.format(klass='dictionary',obj='stagedAddPatchCellLayerDict')+'\n'.join(body)+'\n')
 return dump
