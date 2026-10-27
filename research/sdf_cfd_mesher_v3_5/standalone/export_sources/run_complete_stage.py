#!/usr/bin/env python3
"""Assemble one frozen held-out layer/core mesh and run Foundation 13."""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, pathlib, re, resource, shlex, subprocess, sys, time
import numpy as np

HERE=pathlib.Path(__file__).resolve().parent; V3=HERE.parent
PRIOR=V3/"refined_gmsh_layer_realization/run_gmsh_layer.py"
FOUNDATION=V3/"hxt_complete_mesh_preserved_core/bin/foundationCompleteAudit"
def load(name,path): s=importlib.util.spec_from_file_location(name,path); m=importlib.util.module_from_spec(s); sys.modules[name]=m; s.loader.exec_module(m); return m
prior=load("cohort_complete_math",PRIOR)
def sha(p): return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def write_json(p,o): pathlib.Path(p).write_text(json.dumps(o,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n",encoding="ascii"); return sha(p)
def signed_surface_volume(p,f): t=p[f]; return float(np.sum(np.einsum("ij,ij->i",t[:,0],np.cross(t[:,1],t[:,2])))/6)
def dist(a):
 a=np.asarray(a,dtype=np.float64).reshape(-1)
 if not len(a): return {"count":0}
 q=np.quantile(a,[.01,.05,.5,.95,.99]); return {"count":int(len(a)),"minimum":float(a.min()),"p01":float(q[0]),"p05":float(q[1]),"median":float(q[2]),"p95":float(q[3]),"p99":float(q[4]),"maximum":float(a.max())}
def tet_volume(p): return np.einsum("ij,ij->i",p[:,1]-p[:,0],np.cross(p[:,2]-p[:,0],p[:,3]-p[:,0]))/6
def foam_header(obj): return f"FoamFile\n{{\n    format ascii;\n    class dictionary;\n    object {obj};\n}}\n"
def create_case(case):
 (case/"system").mkdir(parents=True); (case/"constant").mkdir(); (case/"0").mkdir()
 (case/"system/controlDict").write_text(foam_header("controlDict")+"""application checkMesh;
startFrom startTime; startTime 0; stopAt endTime; endTime 1; deltaT 1;
writeControl timeStep; writeInterval 1; writeFormat ascii; writePrecision 17;
writeCompression off; runTimeModifiable false; functions {}
""")
def timed(command,cwd,name,timeout=900):
 out=cwd/f"{name}.stdout"; err=cwd/f"{name}.stderr"; res=cwd/f"{name}.resource.txt"; start=time.perf_counter()
 with out.open("wb") as oo,err.open("wb") as ee: p=subprocess.run(["/usr/bin/time","-v","-o",str(res.resolve()),"bash","-lc",command],cwd=cwd,stdout=oo,stderr=ee,timeout=timeout)
 txt=res.read_text(errors="replace") if res.exists() else ""; m=re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)",txt)
 return {"exit_code":p.returncode,"runtime_seconds":time.perf_counter()-start,"peak_rss_bytes":int(m.group(1))*1024 if m else None,"stdout_sha256":sha(out),"stderr_sha256":sha(err)}
def write_msh(path,coords,wall,far,prisms,tets):
 with path.open("w",encoding="ascii",buffering=4<<20,newline="\n") as s:
  s.write('$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$PhysicalNames\n3\n2 2 "wall"\n2 3 "farfield"\n3 1 "fluid"\n$EndPhysicalNames\n')
  s.write(f'$Nodes\n{len(coords)}\n')
  for i,(x,y,z) in enumerate(coords,1): s.write(f'{i} {x:.17g} {y:.17g} {z:.17g}\n')
  s.write('$EndNodes\n'); total=len(wall)+len(far)+len(prisms)+len(tets); s.write(f'$Elements\n{total}\n'); tag=1
  for row in wall: s.write(f'{tag} 2 2 2 1 '+" ".join(str(int(v)+1) for v in row)+'\n'); tag+=1
  for row in far: s.write(f'{tag} 2 2 3 2 '+" ".join(str(int(v)+1) for v in row)+'\n'); tag+=1
  for row in prisms: s.write(f'{tag} 6 2 1 1 '+" ".join(str(int(v)+1) for v in row)+'\n'); tag+=1
  for row in tets: s.write(f'{tag} 4 2 1 1 '+" ".join(str(int(v)+1) for v in row)+'\n'); tag+=1
  s.write('$EndElements\n')
 return {"sha256":sha(path),"bytes":path.stat().st_size,"nodes":len(coords),"elements":total}
def main():
 ap=argparse.ArgumentParser(); ap.add_argument("--fixture",required=True); ap.add_argument("--replay",default="A"); ap.add_argument("--layer-dir",required=True); ap.add_argument("--core-dir",required=True); ap.add_argument("--output",required=True); a=ap.parse_args()
 ld=pathlib.Path(a.layer_dir); cd=pathlib.Path(a.core_dir); out=pathlib.Path(a.output); out.mkdir(parents=True,exist_ok=False); start=time.perf_counter()
 shadow=np.load(ld/"actual_shadow.npy"); wall=np.load(ld/"wall_faces.npy"); prisms=np.load(ld/"canonical_prisms.npy"); n=len(wall); nv=shadow.shape[1]
 nt=np.load(cd/"netgen_node_tags.npy"); core_coords=np.load(cd/"netgen_coordinates.npy"); core_tets=np.load(cd/"netgen_connectivity_tags.npy"); iface_tags=np.load(cd/"netgen_interface_tags.npy"); far_tags=np.load(cd/"netgen_farfield_tags.npy")
 tagpos={int(t):i for i,t in enumerate(nt)}; c2m=np.full(len(nt),-1,dtype=np.int64)
 for source,tag in enumerate(iface_tags): c2m[tagpos[int(tag)]]=15*nv+source
 mixed_coords=list(shadow.reshape(-1,3)); next_index=len(mixed_coords)
 for i,p in enumerate(core_coords):
  if c2m[i]<0: c2m[i]=next_index; mixed_coords.append(p); next_index+=1
 mixed_coords=np.asarray(mixed_coords,dtype=np.float64); tet_positions=np.asarray([[tagpos[int(v)] for v in row] for row in core_tets],dtype=np.int64); mixed_tets=c2m[tet_positions]
 far=np.asarray([[c2m[tagpos[int(far_tags[v])]] for v in row] for row in ((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,4,7),(0,7,3),(1,2,6),(1,6,5),(0,1,5),(0,5,4),(3,7,6),(3,6,2))],dtype=np.int64)
 interface=15*nv+wall
 # Exact one-owner interface audit on the final mapped tets.
 patterns=((0,2,1),(0,1,3),(1,2,3),(2,0,3)); lookup={tuple(sorted(map(int,r))):i for i,r in enumerate(interface)}; owners=np.full(n,-1,dtype=np.int64); owner_count=np.zeros(n,dtype=np.int8)
 for cell,tet in enumerate(mixed_tets):
  for pat in patterns:
   k=tuple(sorted(int(tet[i]) for i in pat)); fid=lookup.get(k)
   if fid is not None: owners[fid]=cell; owner_count[fid]+=1
 interface_exact=bool(np.all(owner_count==1))
 angles=prior.surface_minimum_angles(shadow[0],wall); peval=prior.evaluate_prisms(shadow,wall,angles); tetvol=tet_volume(mixed_coords[mixed_tets])
 prismvol=float(np.sum(peval["volume"])); corevol=float(np.sum(tetvol)); total=prismvol+corevol
 policy=json.loads((HERE/"FROZEN_COHORT_POLICY.json").read_text()); S=float(policy["geometry"][a.fixture]["characteristic_scale_m"]); half=2.5*S; expected=(2*half)**3-signed_surface_volume(shadow[0],wall); tolerance=max(4096*np.finfo(float).eps*max(expected,float(np.sum(np.abs(peval["volume"])))+float(np.sum(np.abs(tetvol)))),1e-9)
 if not interface_exact or np.any(tetvol<=0) or peval["summary"]["positive_volume_failures"] or abs(total-expected)>tolerance: raise RuntimeError("MixedAssemblyHardGateFailed")
 np.save(out/"mixed_coordinates.npy",mixed_coords,allow_pickle=False); np.save(out/"mixed_prisms.npy",prisms,allow_pickle=False); np.save(out/"mixed_tets.npy",mixed_tets,allow_pickle=False)
 assembly={"schema":"V3HeldOutMixedAssembly/v1","pass":True,"counts":{"points":len(mixed_coords),"prisms":len(prisms),"tetrahedra":len(mixed_tets),"cells":len(prisms)+len(mixed_tets),"wall_faces":len(wall),"interface_faces":n,"farfield_faces":12},"topology":{"interface_one_prism_one_tet":interface_exact,"remaining_interface_boundary":0,"external_patches":["wall","farfield"]},"geometry":{"prism_volume_m3":prismvol,"core_volume_m3":corevol,"complete_volume_m3":total,"expected_volume_m3":expected,"volume_error_m3":abs(total-expected),"volume_tolerance_m3":tolerance,"nonpositive_prisms":peval["summary"]["positive_volume_failures"],"nonpositive_tets":int(np.sum(tetvol<=0))},"prism_determinant":peval["summary"]["determinant"]}
 write_json(out/"assembly.json",assembly)
 transfer=write_msh(out/"complete_transfer.msh",mixed_coords,wall,far,prisms,mixed_tets)
 case=out/"openfoam_case"; create_case(case); gm=timed(f"source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; gmshToFoam {shlex.quote(str((out/'complete_transfer.msh').resolve()))} -case {shlex.quote(str(case.resolve()))}",out,"gmshToFoam")
 if gm["exit_code"]!=0: raise RuntimeError("gmshToFoamFailed")
 audit=out/"foundation_audit"; audit.mkdir(); fa=timed(f"source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; {shlex.quote(str(FOUNDATION.resolve()))} -case {shlex.quote(str(case.resolve()))} -outputDir {shlex.quote(str(audit.resolve()))}",out,"foundationAudit")
 cm=timed(f"source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; checkMesh -constant -allTopology -allGeometry -case {shlex.quote(str(case.resolve()))}",out,"checkMesh")
 text=(out/"checkMesh.stdout").read_text(errors="replace"); summary=json.loads((audit/"foundation_audit_summary.json").read_text())
 mesh_ok="Mesh OK." in text; failed=[line.strip() for line in text.splitlines() if "Failed" in line or line.strip().startswith("***")]
 nonorth=re.search(r"Mesh non-orthogonality Max:\s*([0-9.eE+-]+).*average:\s*([0-9.eE+-]+)",text); skew=re.search(r"Max skewness =\s*([0-9.eE+-]+)",text)
 poly=case/"constant/polyMesh"; files={p.name:sha(p) for p in poly.iterdir() if p.is_file()}; polyhash=hashlib.sha256("".join(f"{k}:{files[k]}\n" for k in sorted(files)).encode()).hexdigest()
 foundation={"schema":"V3HeldOutFoundation/v1","fixture":a.fixture,"replay":a.replay,"literal_mesh_ok":mesh_ok,"checkMesh_exit_code":cm["exit_code"],"failed_lines":failed,"counts":summary.get("counts",summary),"maximum_nonorthogonality":float(nonorth.group(1)) if nonorth else None,"average_nonorthogonality":float(nonorth.group(2)) if nonorth else None,"maximum_skewness_checkMesh":float(skew.group(1)) if skew else None,"polyMesh_sha256":polyhash,"polyMesh_files":files,"transfer":transfer,"gmshToFoam":gm,"foundation_audit":fa,"checkMesh":cm,"runtime_seconds":time.perf_counter()-start,"peak_rss_bytes":int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024)}
 write_json(out/"foundation_result.json",foundation); print(json.dumps({"assembly":assembly,"foundation":foundation},sort_keys=True)); return 0 if mesh_ok else 4
if __name__=="__main__": raise SystemExit(main())
