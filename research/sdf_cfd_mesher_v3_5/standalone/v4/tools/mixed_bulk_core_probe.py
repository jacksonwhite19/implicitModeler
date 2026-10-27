"""Existing protected discrete core route on the qualified mixed interface."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
from types import SimpleNamespace
import importlib.util,json,time,resource
import numpy as np
import gmsh

def main():
    root=Path(__file__).resolve().parents[3];out=root/'research/sdf_cfd_mesher_v4/results/fixed_interface_bulk_core_repair';dest=out/('mixed_core_B' if len(sys.argv)>1 and sys.argv[1]=='B' else 'mixed_core');dest.mkdir(exist_ok=True)
    path=root/'research/sdf_cfd_mesher_v3/airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py'
    spec=importlib.util.spec_from_file_location('mixed_core_reference',path);old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
    p=out/'source/mixed_interface';x=np.load(p/'points.npy');faces=np.load(p/'connectivity.npy').reshape(-1,3)
    # Existing box-domain topology, bounded to this small fixture.
    span=np.ptp(x,axis=0).max();lo=x.min(0)-span;hi=x.max(0)+span
    far=np.array([[lo[0],lo[1],lo[2]],[hi[0],lo[1],lo[2]],[hi[0],hi[1],lo[2]],[lo[0],hi[1],lo[2]],[lo[0],lo[1],hi[2]],[hi[0],lo[1],hi[2]],[hi[0],hi[1],hi[2]],[lo[0],hi[1],hi[2]]])
    ff=np.array(((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,4,7),(0,7,3),(1,2,6),(1,6,5),(0,1,5),(0,5,4),(3,7,6),(3,6,2)))
    authority=SimpleNamespace(outer_points=x,outer_faces=faces,face_sheet_ids=('mixed',)*len(faces),curves=(),corners=())
    h=float(np.median(np.linalg.norm(x[faces[:,1]]-x[faces[:,0]],axis=1)))
    started=time.perf_counter();gmsh.initialize([],False)
    try:
        old.setup_gmsh(gmsh,1);gmsh.model.add('fixed_mixed_core');s,c,k,_=old.transfer_boundary(gmsh,authority,far,ff)
        callback=old.CoreSizeCallback(x,h,span,1.3);gmsh.model.mesh.setSizeCallback(callback);gmsh.model.mesh.generate(3)
        for _ in range(3):gmsh.model.mesh.optimize('Gmsh',force=True)
        expected=old.signed_surface_volume(far,ff)-old.signed_surface_volume(x,faces)
        result,arrays=old.validate_core(gmsh,authority,far,ff,s,c,k,expected)
        for name,a in arrays.items():np.save(dest/(name+'.npy'),a)
        np.save(dest/'far_points.npy',far);np.save(dest/'far_faces.npy',ff)
        gmsh.write(str(dest/'core.msh'))
        (dest/'GENERATION.json').write_text(json.dumps(dict(validation=result,seconds=time.perf_counter()-started,peak_rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024),sort_keys=True,indent=2)+'\n')
    finally:gmsh.finalize()

if __name__=='__main__':main()
