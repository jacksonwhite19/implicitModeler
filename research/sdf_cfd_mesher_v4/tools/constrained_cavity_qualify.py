"""Qualify and apply disjoint local replacements; never publish a bad core."""
import sys,time,json,itertools
from pathlib import Path
from fractions import Fraction as F
import numpy as np
import psutil
from constrained_cavity import *
from constrained_cavity_audit import OUT,V4
from native_mixed_face_certificate import certify
from cavity_volume_audit import degree

def unique_faces(t):
    rows=t[:,PATS].reshape(-1,3);_,ix=np.unique(np.sort(rows,axis=1),axis=0,return_index=True)
    return [tuple(map(int,f)) for f in rows[np.sort(ix)]]

def validate_cavity_source(x,t,data,retired):
    ids=set(map(int,data['global_cells']))
    if len(ids)!=len(data['global_cells']):raise ValueError('DuplicateCavitySourceCell')
    if retired&ids:raise ValueError('OverlappingReplacementCavities')
    if not np.array_equal(x[data['global_vertices']].view('u8'),data['x'].view('u8')):raise ValueError('StaleCavityCoordinates')
    if not np.array_equal(t[data['global_cells']],data['global_vertices'][data['t']]):raise ValueError('StaleCavityConnectivity')
    return ids

def local_seal(data,x,t):
    iv=invariants(x,data['t'],t);q,m=quality(data,x,t)
    if not iv['passed'] or not q['passed']:raise ValueError('CavityHardGateFailed')
    ticks=time.perf_counter();points=[[F(float(z)) for z in p] for p in x]
    samples=[tuple(sum(points[v][j] for v in tet)/4 for j in range(3)) for tet in t]
    dg=degree(x,data['f'],samples);degrees=time.perf_counter()-ticks
    changed=np.setdiff1d(np.unique(t),np.unique(data['f']));inside={str(int(i)):interior_point_valid(x[i],data['x'],data['t']) for i in changed}
    if not dg['passed'] or not all(inside.values()):raise ValueError('CavityFillingFailed')
    return dict(invariants=iv,quality=q,degree=dg,degree_seconds=degrees,interior_points_strict=inside)

def apply_inputs(src,interface,replacements,out):
    start=time.perf_counter();out.mkdir(exist_ok=False);x,t,ci,iface=load_core(src,interface);q0,m0=evaluate(x,t,ci,iface);old_ci=ci.copy();points=list(x);newcells=[];retired=set();records=[];newpoint_count=0
    for inp,ca in replacements:
        d=dict(np.load(inp));ids=validate_cavity_source(x,t,d,retired);c=np.load(ca);xx,tt=c['x'],c['t'];rec=local_seal(d,xx,tt)
        retired|=ids;mp={int(j):int(v) for j,v in enumerate(d['global_vertices']) if j in set(d['boundary_vertices'])}
        # Boundary ids are immutable. Interior coordinates are allowed to be
        # deleted/replaced; compact the full core after applying both deltas.
        for j in np.unique(tt):
            if int(j) not in mp:mp[int(j)]=len(points);points.append(xx[j]);newpoint_count+=1
        new=np.array([[mp[int(j)] for j in cell] for cell in tt]);newcells.extend(new);rec.update(input=str(inp),candidate=str(ca),removed=len(ids),replacement=len(new));records.append(rec)
    newcells=np.asarray(newcells);xx=np.array(points);kept=np.delete(t,sorted(retired),axis=0);allcells=np.vstack((kept,newcells));nkept=len(kept)
    used=np.unique(allcells);mapping=np.full(len(xx),-1,dtype=int);mapping[used]=np.arange(len(used));xx=xx[used];allcells=mapping[allcells];ci=mapping[ci];newcells=allcells[nkept:]
    q,m=evaluate(xx,allcells,ci,iface)
    beforebad={scientific_key(x[t[i]]) for i in m0['bad']};afterbad={scientific_key(xx[allcells[i]]) for i in m['bad']}
    if not afterbad<=beforebad:raise ValueError('NewOutsideHardFailure')
    print(json.dumps(dict(stage='local_quality_passed',bad_before=q0['bad_cells'],bad_after=q['bad_cells'],new_tets=len(newcells))),flush=True)
    # Establish membership in the actual terminal core, independently of the
    # old cavity. The complete frozen interface is used, with core orientation.
    # This plus complete new-face/interface intersections excludes new-shell
    # overlap even with shell cells beyond the immediate interface owners.
    tick=time.perf_counter();rpoints=[[F(float(z)) for z in p] for p in xx]
    samples=[tuple(sum(rpoints[v][j] for v in tet)/4 for j in range(3)) for tet in newcells]
    outer_degree=degree(xx,ci[iface.f][:,::-1],samples);outer_degree['seconds']=time.perf_counter()-tick
    save(out/'TERMINAL_INTERFACE_DEGREE.json',outer_degree)
    if not outer_degree['passed']:raise ValueError('ReplacementOutsideTerminalCore')
    # Every changed face against all preserved core faces and the complete
    # shell interface/its owner-cell faces. Interior containment is excluded by
    # the unchanged cavity boundary, exact volume and local global degree.
    faces=unique_faces(allcells);index={tuple(sorted(f)):i for i,f in enumerate(faces)};active={index[tuple(sorted(f))] for f in unique_faces(newcells)}
    coords=list(xx);lookup={p.tobytes():i for i,p in enumerate(xx)}
    patterns={4:PATS.tolist(),5:[[0,3,2,1],[0,1,4],[1,2,4],[2,3,4],[3,0,4]],6:[[0,2,1],[3,4,5],[0,1,4,3],[1,2,5,4],[2,0,3,5]]}
    for p in sorted(interface.glob('owner_geometry_*.npy')):
        arity=int(p.stem.rsplit('_',1)[1])
        for cell in np.load(p):
            ids=[]
            for v in cell:
                if v.tobytes() not in lookup:lookup[v.tobytes()]=len(coords);coords.append(v)
                ids.append(lookup[v.tobytes()])
            for pat in patterns[arity]:
                f=tuple(ids[j] for j in pat);key=tuple(sorted(f))
                if key not in index:index[key]=len(faces);faces.append(f)
    tick=time.perf_counter();cert=certify(np.array(coords),faces,active);cert['seconds']=time.perf_counter()-tick
    cert['proofs']=list(cert['proofs']);save(out/'LOCAL_CERTIFICATE.json',cert)
    if not cert['passed']:raise ValueError('LocalContinuousGeometryFailed')
    tags=np.arange(1,len(xx)+1)
    for name,value in dict(coordinates=xx,tets=allcells,node_tags=tags,interface_tags=tags[ci],interface_owner=m['interface_owner']).items():np.save(out/(name+'.npy'),value)
    np.savez_compressed(out/'METRICS.npz',**m);save(out/'QUALITY.json',q);save(out/'CAVITY_SEALS.json',records)
    result=dict(before=q0,after=q,removed_cells=len(retired),replacement_tets=len(newcells),interior_vertices_net_change=len(xx)-len(x),replacement_interior_positions=newpoint_count,new_hard_failures=sorted(afterbad-beforebad),frozen_bad_before=int(np.all(np.isin(t[m0['bad']],old_ci),axis=1).sum()),frozen_bad_after=int(np.all(np.isin(allcells[m['bad']],ci),axis=1).sum()),certificate={k:v for k,v in cert.items() if k!='proofs'},active_faces=len(active),certificate_faces=len(faces),seconds=time.perf_counter()-start,peak_rss_bytes=psutil.Process().memory_info().peak_wset,retained_rss_bytes=psutil.Process().memory_info().rss,qualified_complete_core=q['passed'],qualified_local=True)
    save(out/'RESULT.json',result);print(json.dumps(result),flush=True)
    return result

def main():
    base=V4/'results/bent_terminal_shell_fixed_interface_core';interface=V4/'results/terminal_capacity_aware_remesh_acceptance/interface'
    replacements=[(OUT/'merged/group_0_ring_3.npz',OUT/'g0_r3_safe_one/optimized_CANDIDATE.npz'),(OUT/'merged/group_1_ring_3.npz',OUT/'g1_margin/CANDIDATE.npz')]
    apply_inputs(base/'final_stars',interface,replacements,OUT/'accepted_local')

if __name__=='__main__':main()
