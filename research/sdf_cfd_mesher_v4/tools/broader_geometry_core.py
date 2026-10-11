"""Bounded core handoff for one source-keyed interface model.

Uses the preserved Gmsh controls and extractors. Domain orientation determines
whether the supplied closed interface encloses core or excludes shell material.
The exterior envelope is the existing bounded mixed-core study domain, never
a production CFD boundary-condition authority.
"""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
import argparse, json, time, resource
import numpy as np
import gmsh
from repair_bulk_core import load, V3
from audit_native_core_quality import audit
from audit_mixed_bulk_probe import centres


def save(path, value):
    Path(path).write_text(json.dumps(value, sort_keys=True, indent=2,
        default=lambda v: v.tolist() if isinstance(v, np.ndarray) else v.item())+'\n')


def components(faces):
    parent = list(range(len(faces)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]; i = parent[i]
        return i
    seen = {}
    for i, face in enumerate(faces):
        for v in face:
            if int(v) in seen:
                a,b = root(i),root(seen[int(v)]); parent[max(a,b)] = min(a,b)
            else: seen[int(v)] = i
    groups = {}
    for i in range(len(faces)): groups.setdefault(root(i), []).append(i)
    return [np.asarray(v) for _,v in sorted(groups.items())]


def extract_all_volumes(gmsh, old):
    """The reference extractor selected volume 1; include every core region."""
    tags,raw,_=gmsh.model.mesh.getNodes()
    tags=np.asarray(tags,dtype=np.int64); xyz=np.asarray(raw,dtype=np.float64).reshape(-1,3)
    order=np.argsort(tags,kind='stable'); tags,xyz=tags[order],xyz[order]
    blocks=[row for _,tag in sorted(gmsh.model.getEntities(3)) for row in old.get_elements(gmsh,3,tag)]
    if any(kind!=4 and len(ids) for kind,ids,_ in blocks): raise ValueError('UnsupportedCoreVolumeElements')
    blocks=[row for row in blocks if row[0]==4 and len(row[1])]
    if not blocks: raise ValueError('NoTetrahedra')
    ids=np.concatenate([r[1] for r in blocks]); connectivity=np.concatenate([r[2] for r in blocks])
    order=np.argsort(ids,kind='stable');ids,connectivity=ids[order],connectivity[order]
    assert len(np.unique(ids))==len(ids)
    positions=np.searchsorted(tags,connectivity)
    assert np.all(positions<len(tags)) and np.array_equal(tags[positions],connectivity)
    return dict(node_tags=tags,coordinates=xyz,tet_tags=ids,connectivity_tags=connectivity,tets=positions)


def run(interface, dest, algorithm=1, source_spec=None):
    if dest.exists():
        if any(p.name!='CORE_INPUT.json' for p in dest.iterdir()):
            raise ValueError('CoreGenerationRequiresUnusedOutput')
    else:dest.mkdir(parents=True, exist_ok=False)
    # A sealed shell alone does not establish core-handoff feasibility. Persist
    # the identity-bound descriptive audit before invoking any native mesher.
    from core_handoff_audit import pre_core_audit
    save(dest/'PRE_CORE_HANDOFF.json',pre_core_audit(interface,source_spec))
    old = load('cohort_core_reference', V3/'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py')
    fg = load('cohort_foundation', V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    x = np.load(interface/'points.npy'); offsets = np.load(interface/'offsets.npy')
    if not np.all(np.diff(offsets)==3): raise ValueError('TriangulationAuthorityRequired')
    faces = np.load(interface/'connectivity.npy').reshape(-1,3)
    groups = components(faces)
    volumes = [old.signed_surface_volume(x, faces[g]) for g in groups]
    if all(v < 0 for v in volumes): exterior = False
    elif all(v > 0 for v in volumes): exterior = True
    else: raise ValueError('NestedMixedDomainNeedsPreservedDomainAuthority')
    far = np.empty((0,3)); ff = np.empty((0,3),dtype=int)
    span = np.ptp(x,axis=0).max()
    if exterior:
        lo=x.min(0)-span; hi=x.max(0)+span
        far=np.array([[lo[0],lo[1],lo[2]],[hi[0],lo[1],lo[2]],
            [hi[0],hi[1],lo[2]],[lo[0],hi[1],lo[2]],
            [lo[0],lo[1],hi[2]],[hi[0],lo[1],hi[2]],
            [hi[0],hi[1],hi[2]],[lo[0],hi[1],hi[2]]])
        ff=np.array(((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,4,7),(0,7,3),
            (1,2,6),(1,6,5),(0,1,5),(0,5,4),(3,7,6),(3,6,2)))
    tick=time.perf_counter(); gmsh.initialize([],False)
    try:
        old.setup_gmsh(gmsh,1); gmsh.option.setNumber('Mesh.Algorithm3D',algorithm); gmsh.model.add('native_bounded_core')
        for i, group in enumerate(groups,1):
            gmsh.model.addDiscreteEntity(2,i)
            ids=np.unique(faces[group])
            gmsh.model.mesh.addNodes(2,i,(ids+1).tolist(),x[ids].ravel().tolist())
            gmsh.model.mesh.addElementsByType(i,2,(group+1).tolist(),(faces[group]+1).ravel().tolist())
        if exterior:
            gmsh.model.addDiscreteEntity(2,100)
            tags=np.arange(len(x)+1,len(x)+1+len(far))
            gmsh.model.mesh.addNodes(2,100,tags.tolist(),far.ravel().tolist())
            gmsh.model.mesh.addElementsByType(100,2,(np.arange(len(ff))+len(faces)+1).tolist(),tags[ff].ravel().tolist())
        gmsh.model.mesh.reclassifyNodes(); gmsh.model.mesh.createTopology(False,True)
        if exterior: gmsh.model.addDiscreteEntity(3,1,[100]+[-i for i in range(1,len(groups)+1)])
        else:
            for i in range(1,len(groups)+1): gmsh.model.addDiscreteEntity(3,i,[-i])
        gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
        h=float(np.median(np.linalg.norm(x[faces[:,1]]-x[faces[:,0]],axis=1)))
        callback=old.CoreSizeCallback(x,h,float(span),1.3)
        gmsh.model.mesh.setSizeCallback(callback); gmsh.model.mesh.generate(3)
        for _ in range(3): gmsh.model.mesh.optimize('Gmsh',force=True)
        arrays=extract_all_volumes(gmsh,old)
        xyz=arrays['coordinates']; tets=arrays['tets']
        lookup=old.exact_coordinate_map(arrays['node_tags'],xyz)
        expected=np.concatenate([x,far])
        matches=[lookup.get(np.asarray(p,dtype='<f8').tobytes(),[]) for p in expected]
        assert all(len(v)==1 for v in matches)
        tags=np.array([v[0] for v in matches]); order=np.argsort(arrays['node_tags'])
        positions=order[np.searchsorted(arrays['node_tags'][order],tags)]
        assert np.array_equal(expected.view('u8'),xyz[positions].view('u8'))
        occ=tets[:,np.asarray(old.TET_FACES)].reshape(-1,3)
        keys=np.sort(occ,axis=1); unique,first,counts=np.unique(keys,axis=0,return_index=True,return_counts=True)
        assert np.all(counts<=2)
        boundary={tuple(k):int(first[i]//4) for i,k in enumerate(unique) if counts[i]==1}
        mapped=positions[:len(x)][faces]
        wanted={tuple(sorted(f)) for f in mapped}
        wanted.update(tuple(sorted(f)) for f in positions[len(x):][ff])
        assert set(boundary)==wanted
        owners=np.array([boundary[tuple(sorted(f))] for f in mapped])
        arrays.update(interface_owner=owners,interface_tags=tags[:len(x)],farfield_tags=tags[len(x):])
        for key,value in arrays.items(): np.save(dest/(key+'.npy'),value)
        np.save(dest/'far_points.npy',far); np.save(dest/'far_faces.npy',ff)
        audit(dest)
        quality=json.loads((dest/'HARD_QUALITY.json').read_text())
        sc=np.empty((len(faces),3)); sv=np.empty(len(faces))
        for path in sorted(interface.glob('owner_geometry_*.npy')):
            arity=path.stem.rsplit('_',1)[1]; ids=np.load(interface/f'owner_faces_{arity}.npy')
            sc[ids],sv[ids]=centres(np.load(path),fg)
        tc,tv=centres(xyz[tets],fg); q=x[faces]; area,fc=fg.face_area_centre_tri(q)
        da=abs(np.einsum('ij,ij->i',area,fc-sc)); db=abs(np.einsum('ij,ij->i',area,tc[owners]-fc))
        weight=np.minimum(da,db)/(da+db); ratio=np.minimum(sv,tv[owners])/np.maximum(sv,tv[owners])
        skew=fg.internal_face_skewness(q,fc,area,sc,tc[owners])
        metrics=dict(weight_min=float(weight.min()),ratio_min=float(ratio.min()),skew_max=float(skew.max()),
            weight_failures=int((weight<.055).sum()),ratio_failures=int((ratio<.01).sum()),skew_failures=int((skew>3.63636).sum()))
        save(dest/'GENERATION.json',dict(core=quality,interface=metrics,tets=len(tets),
            interface_coordinates_exact=True,missing_faces=0,duplicate_faces=0,
            domain='preserved bounded exterior envelope' if exterior else 'enclosed cavity components',
            components=len(groups),algorithm=algorithm,seconds=time.perf_counter()-tick,
            peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
            passed=quality['passed'] and not any(metrics[k] for k in ('weight_failures','ratio_failures','skew_failures'))))
        gmsh.write(str(dest/'core.msh'))
    finally: gmsh.finalize()


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('interface',type=Path); p.add_argument('output',type=Path)
    p.add_argument('--algorithm',type=int,choices=[1,10],default=1)
    p.add_argument('--source-spec',type=Path,help='Preserved source JSON, either spec or {spec: ...}; enables branch-bound depth where supported')
    args=p.parse_args()
    spec=json.loads(args.source_spec.read_text()) if args.source_spec else None
    if spec is not None:spec=spec.get('spec',spec)
    run(args.interface,args.output,args.algorithm,spec)
