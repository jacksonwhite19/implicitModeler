"""Read-only core audit using the preserved Foundation-equivalent definitions."""
from pathlib import Path
import ast
import importlib.util
import json
import sys
sys.dont_write_bytecode = True
import time
import numpy as np


def audit(folder, interface=None):
    start = time.perf_counter()
    v3 = Path(__file__).resolve().parents[2] / 'sdf_cfd_mesher_v3'
    source = v3 / 'wing_body_first_row_authority/solve_coordinate_feasibility.py'
    scope = dict(np=np, PATS=((0,2,1),(0,1,3),(1,2,3),(2,0,3)))
    for node in ast.parse(source.read_text()).body:
        if isinstance(node, ast.FunctionDef) and node.name in {'volume_np', 'determinant_np'}:
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
    spec = importlib.util.spec_from_file_location('foundation_reference', v3 / 'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    fg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fg)
    xyz = np.load(folder / 'coordinates.npy', mmap_mode='r')
    tets = np.load(folder / 'tets.npy', mmap_mode='r')
    count = len(tets)
    volume = np.empty(count)
    determinant = np.empty(count)
    aspect = np.empty(count)
    centre = np.empty((count, 3))
    pats = np.array(scope['PATS'])
    for begin in range(0, count, 50000):
        sl = slice(begin, min(begin+50000, count))
        q = xyz[tets[sl]]
        volume[sl] = scope['volume_np'](q)
        centre[sl] = q.mean(axis=1)
        av = np.stack([.5*np.cross(q[:,b]-q[:,a],q[:,c]-q[:,a]) for a,b,c in pats], axis=1)
        avg = np.linalg.norm(av, axis=2).mean(axis=1)
        vv = av / avg[:,None,None]
        determinant[sl] = np.linalg.det(np.einsum('nfi,nfj->nij', vv, vv))
        ss = np.abs(av).sum(axis=1)
        aspect[sl] = np.maximum(ss.max(axis=1)/(ss.min(axis=1)+1e-300), ss.sum(axis=1)/(6*np.maximum(volume[sl],1e-300)**(2/3)))
    rows = tets[:,pats].reshape(-1,3)
    keys = np.sort(rows, axis=1)
    idx = np.lexsort((keys[:,2],keys[:,1],keys[:,0]))
    sk = keys[idx]
    starts = np.flatnonzero(np.r_[True,np.any(sk[1:]!=sk[:-1],axis=1)])
    counts = np.diff(np.r_[starts,len(sk)])
    if np.any(counts > 2):
        raise ValueError('NonManifoldCore')
    internal = starts[counts == 2]
    minima = dict(volume=float(volume.min()), determinant=float(determinant.min()), interpolation_weight=1., volume_ratio=1.)
    maxima = dict(aspect=float(aspect.max()), skewness=0., nonorthogonality=0.)
    failures = dict(volume=int(np.count_nonzero(volume<=0)), determinant=int(np.count_nonzero(determinant<.0011)), aspect=int(np.count_nonzero(aspect>1000)), interpolation_weight=0, volume_ratio=0, skewness=0)
    failed_pairs=[]
    for begin in range(0,len(internal),50000):
        entries=internal[begin:begin+50000]; ir=idx[entries]; jr=idx[entries+1]
        own=ir//4; nei=jr//4; fp=xyz[rows[ir]]
        area,fc=fg.face_area_centre_tri(fp)
        da=np.abs(np.einsum('ij,ij->i',area,fc-centre[own]));db=np.abs(np.einsum('ij,ij->i',area,centre[nei]-fc))
        weight=np.minimum(da,db)/(da+db);ratio=np.minimum(volume[own],volume[nei])/np.maximum(volume[own],volume[nei])
        for j in np.flatnonzero(weight<.055):
            failed_pairs.append(dict(owner=int(own[j]),neighbour=int(nei[j]),vertices=rows[ir[j]].tolist(),weight=float(weight[j])))
        skew=fg.internal_face_skewness(fp,fc,area,centre[own],centre[nei])
        delta=centre[nei]-centre[own]
        nonorth=np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',area,delta)/(np.linalg.norm(area,axis=1)*np.linalg.norm(delta,axis=1)),-1,1)))
        for name,values,limit in [('interpolation_weight',weight,.055),('volume_ratio',ratio,.01)]:
            minima[name]=min(minima[name],float(values.min()));failures[name]+=int(np.count_nonzero(values<limit))
        maxima['skewness']=max(maxima['skewness'],float(skew.max()));maxima['nonorthogonality']=max(maxima['nonorthogonality'],float(nonorth.max()))
        failures['skewness']+=int(np.count_nonzero(skew>3.63636))
    if interface is not None and (interface / 'owner_geometry_6.npy').exists():
        shell_centres,shell_volumes=fg.prism_cell_centres(np.load(interface/'owner_geometry_6.npy'))
        face_order=np.load(interface/'owner_faces_6.npy')
        if not np.array_equal(face_order,np.arange(len(face_order))):
            raise ValueError('PrismOwnerOrderMismatch')
        face_points=np.load(interface/'points.npy')[np.load(interface/'connectivity.npy').reshape(-1,3)]
        owners=np.load(folder/'interface_owner.npy')
        area,fc=fg.face_area_centre_tri(face_points)
        da=np.abs(np.einsum('ij,ij->i',area,fc-shell_centres));db=np.abs(np.einsum('ij,ij->i',area,centre[owners]-fc))
        iw=np.minimum(da,db)/(da+db);iv=np.minimum(shell_volumes,volume[owners])/np.maximum(shell_volumes,volume[owners])
        isk=fg.internal_face_skewness(face_points,fc,area,shell_centres,centre[owners])
        for name,values,limit in [('shell_interface_weight',iw,.055),('shell_interface_ratio',iv,.01)]:
            minima[name]=float(values.min());failures[name]=int(np.count_nonzero(values<limit))
        maxima['shell_interface_skewness']=float(isk.max());failures['shell_interface_skewness']=int(np.count_nonzero(isk>3.63636))
    bad_det=np.flatnonzero(determinant<.0011)
    io=np.load(folder/'interface_owner.npy')
    result=dict(scope='Core cells, core/core faces and shell interface where owner geometry is supplied; support face metrics pending',cells=count,minimum=minima,maximum=maxima,failures=failures,passed=not any(failures.values()),seconds=time.perf_counter()-start,
                determinant_reproducer=[dict(cell=int(i),value=float(determinant[i]),vertices=tets[i].tolist(),shell_interface_owner=bool(i in io)) for i in bad_det],
                weight_reproducer=failed_pairs)
    (folder/'HARD_QUALITY.json').write_text(json.dumps(result,sort_keys=True,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':
    audit(Path(sys.argv[1]), Path(sys.argv[2]) if len(sys.argv)>2 else None)
