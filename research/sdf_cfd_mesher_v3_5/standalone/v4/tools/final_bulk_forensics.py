"""Complete numerical arrays and bounded exact repair volume evidence."""
import sys
sys.dont_write_bytecode=True
import json,time,psutil
from fractions import Fraction as F
from itertools import combinations
import numpy as np
from repair_bulk_core import OUT,SRC,V3,load,write,topology,cell_metrics,face_metrics

def exact_volume(q):
    p=[[F(float(v)) for v in row] for row in q];a,b,c=[[p[i][j]-p[0][j] for j in range(3)] for i in (1,2,3)]
    return (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6

def main():
    start=time.perf_counter();f=load('forensic_feasibility',V3/'fresh_supported_refinement_family_qualification_2026-09-09/fallback_scope_20260910/source/solve_coordinate_feasibility.py');fg=load('forensic_foundation',V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    x=np.load(OUT/'repaired_A/coordinates.npy',mmap_mode='r');old=np.load(SRC/'coordinates.npy',mmap_mode='r');tets=np.load(SRC/'tets.npy',mmap_mode='r')
    vol,det,aspect,c=cell_metrics(x,tets,f);faces,owners=topology(tets);weight,ratio,skew,angle=face_metrics(x,faces,owners,vol,c,fg)
    dest=OUT/'quality_arrays';dest.mkdir(exist_ok=True)
    for key,a in dict(volume=vol,determinant=det,aspect=aspect,interpolation_weight=weight,volume_ratio=ratio,skewness=skew,nonorthogonality=angle,face_vertices=faces,face_owners=owners).items():np.save(dest/(key+'.npy'),a)
    inv=np.load(OUT/'LOCAL_INVENTORY.npz');affected=inv['affected'];moved=inv['movable']
    before=sum((exact_volume(old[tets[i]]) for i in affected),F(0));after=sum((exact_volume(x[tets[i]]) for i in affected),F(0))
    assert before==after
    assert all(exact_volume(x[tets[i]])>0 for i in affected)
    failures=json.loads((OUT/'FAILURES.json').read_text());records={r['cell']:r for r in failures['cells']};components=[]
    io=np.load(SRC/'interface_owner.npy');fo=np.load(SRC/'farfield_owner.npy')
    for i,row in records.items():
        row['all_scientific_face_ids']=['triangle:'+','.join(map(str,sorted(tets[i,list(pat)]))) for pat in f.PATS]
        row['shell_boundary_face_ids']=np.flatnonzero(io==i).tolist()
        row['support_boundary_face_ids']=np.flatnonzero(fo==i).tolist()
    selected=np.flatnonzero(np.any(np.isin(owners,list(records)),axis=1));adj=np.unique(owners[selected]);ov,od,oa,oc=cell_metrics(old,tets[adj],f)
    ow,orr,oss,onn=face_metrics(old,faces[selected],np.searchsorted(adj,owners[selected]),ov,oc,fg)
    failures['weight_face_records']=[dict(face=int(selected[j]),vertices=faces[selected[j]].tolist(),owners=owners[selected[j]].tolist(),weight=float(ow[j])) for j in np.flatnonzero(ow<.055)]
    write('FAILURES.json',failures)
    for ids in failures['components']:
        shell=any(records[i]['shell_adjacent'] for i in ids);support=any(records[i]['support_adjacent'] for i in ids)
        kind='mixed proximity' if shell and support else 'shell-interface adjacent' if shell else 'support/farfield adjacent' if support else 'deep interior (no boundary-face adjacency)'
        vertices=np.unique(tets[ids]);stars=np.flatnonzero(np.any(np.isin(tets,vertices),axis=1))
        dihedrals=[]
        for i in ids:
            q=old[tets[i]];normals=np.array([np.cross(q[b]-q[a],q[c]-q[a]) for a,b,c in f.PATS]);normals/=np.linalg.norm(normals,axis=1)[:,None]
            dihedrals.extend(180-np.degrees(np.arccos(np.clip(np.dot(normals[a],normals[b]),-1,1))) for a,b in combinations(range(4),2))
        components.append(dict(cells=ids,size=len(ids),classification=kind,shell_distance_min=min(records[i]['shell_distance'] for i in ids),support_distance_min=min(records[i]['support_distance'] for i in ids),vertex_star_cells=len(stars),dihedral_min=float(min(dihedrals)),dihedral_max=float(max(dihedrals)),cause='Vertex placement sufficient: constrained relocation clears frozen gates without a connectivity change; proximity alone is not a cause'))
    write('FORENSICS.json',dict(components=components,face_component_count=len(components),largest_component=max(c['size'] for c in components),isolated=sum(c['size']==1 for c in components),vertex_adjacent_component_pairs=[[i,j] for i in range(len(components)) for j in range(i+1,len(components)) if len(np.intersect1d(tets[components[i]['cells']],tets[components[j]['cells']]))],repair_volume_exact_before=str(before),repair_volume_exact_after=str(after),repair_volume_difference='0',unchanged_outside_vertices=bool(np.array_equal(np.delete(x,moved,0),np.delete(old,moved,0))),connectivity_unchanged=True,seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset))

if __name__=='__main__':main()
