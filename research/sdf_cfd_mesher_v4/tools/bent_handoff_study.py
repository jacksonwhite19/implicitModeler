"""Read-only stack lineage and diagnostic-core correlation. No mesh repair."""
import json, time, shutil, gc
from collections import Counter, defaultdict
import numpy as np
import psutil
from scipy.spatial import cKDTree
from bent_handoff_inputs import OUT, SHELL, INTERFACE, CORE, FALLBACK, V4
from fixed_core_study import save, load_core, evaluate, components
from core_handoff_audit import surface, distribution, medial_depth, nearest_surface, lineage_class, pre_core_audit
from geometry_family_authority import Source

FIELDS = ('longest_edge','shortest_edge','edge_ratio','area','minimum_angle','maximum_angle','circumradius','inradius','radius_ratio')

def event(stage, **kw):
    p=psutil.Process();r=dict(stage=stage,seconds=time.perf_counter()-START,rss=p.memory_info().rss,
        peak_rss=getattr(p.memory_info(),'peak_wset',p.memory_info().rss),disk_free=shutil.disk_usage(OUT).free,**kw)
    with (OUT/'EVENTS.jsonl').open('a') as f:f.write(json.dumps(r)+'\n')
    print(json.dumps(r),flush=True)

def vtk(path,x,cells,types,fields):
    with path.open('w') as f:
        f.write('# vtk DataFile Version 3.0\nV4 handoff diagnostics; no new gates\nASCII\nDATASET UNSTRUCTURED_GRID\n')
        f.write(f'POINTS {len(x)} double\n');np.savetxt(f,x,fmt='%.17g')
        f.write(f'CELLS {len(cells)} {sum(len(c)+1 for c in cells)}\n')
        for c in cells:f.write(str(len(c))+' '+' '.join(map(str,c))+'\n')
        f.write(f'CELL_TYPES {len(cells)}\n');np.savetxt(f,np.asarray(types),fmt='%d')
        f.write(f'CELL_DATA {len(cells)}\n')
        for k,v in fields.items():
            f.write(f'SCALARS {k} double 1\nLOOKUP_TABLE default\n');np.savetxt(f,np.asarray(v),fmt='%.17g')

def extract():
    from terminal_shell_store import restart
    d=restart(SHELL/'checkpoint')
    np.savez_compressed(OUT/'SEALED_GEOMETRY.npz',points=d['points'],wall=d['wall'])
    save(OUT/'SOURCE.json',dict(spec=d['spec'],scientific=d['scientific'],geometry_digest=d['geometry_digest'],quality_digest=d['quality_digest']))
    save(OUT/'CELL_LINEAGE.json',d['cells']);save(OUT/'SCHEDULE.json',d['state']['local_schedule'])
    event('sealed_extraction',shell_reconstructed=False)

def stack():
    spec=json.loads((OUT/'SOURCE.json').read_text())['spec'];source=Source(spec)
    z=np.load(OUT/'SEALED_GEOMETRY.npz');wall=z['wall'];n=int(wall.max())+1;layers=z['points'].reshape(-1,n,3)
    m=json.loads((INTERFACE/'INTERFACE.json').read_text());x=np.load(INTERFACE/'points.npy');f=np.load(INTERFACE/'connectivity.npy').reshape(-1,3)
    src=np.array([int(v.split(':')[-1]) for v in m['vertex_ids']]);lev=np.array([int(v.split(':')[1]) for v in m['vertex_ids']])
    assert np.array_equal(x.view('u8'),layers[lev,src].view('u8'))
    a=surface(x,f);assert a['edge_length'].max()==0.029053300573073853
    own=np.array([r['cell_type'] for r in m['ancestry']]);column=np.array([int(r['source']) for r in m['ancestry']]);last=np.array([r['column']['last_prism_layer'] for r in m['ancestry']])
    wall_a=surface(layers[0],wall);wall_edges={tuple(e):i for i,e in enumerate(wall_a['edges'])}
    rows=[]
    for k,p in enumerate(layers):
        # Complete committed fronts 0..18; layer19 uses actual mixed outer faces.
        v=surface(p,wall) if k<len(layers)-1 else a
        rows.append(dict(layer=k,scope='wall' if k==0 else 'complete committed prism top' if k<len(layers)-1 else 'actual mixed terminal interface (includes exposed layer18 faces)',
            edge=distribution(v['edge_length']),triangles={key:distribution(v[key]) for key in FIELDS}))
    save(OUT/'LAYER_STATISTICS.json',rows)
    incident=defaultdict(list)
    for i,face in enumerate(f):
        for aa,bb in ((0,1),(1,2),(2,0)):incident[tuple(sorted((int(face[aa]),int(face[bb]))))].append(i)
    lineage=[];length_history=[]
    for j,e in enumerate(a['edges']):
        pair=src[e];faces=incident[tuple(e)];we=wall_edges.get(tuple(sorted(pair)))
        lens=np.linalg.norm(layers[:,pair[1]]-layers[:,pair[0]],axis=1) if we is not None else np.full(len(layers),np.nan)
        length_history.append(lens)
        kind=lineage_class(pair,wall_edges,a['edge_length'][j],lens[0],lev[e],own[faces])
        lineage.append(dict(edge=j,vertices=e.tolist(),source_vertices=pair.tolist(),source_wall_edge=we,endpoint_layers=lev[e].tolist(),faces=faces,
            source_columns=column[faces].tolist(),owners=own[faces].tolist(),last_prism_layer=last[faces].tolist(),
            classification=kind,wall_length=None if we is None else float(lens[0]),terminal_length=float(a['edge_length'][j]),
            length_over_wall=None if we is None else float(a['edge_length'][j]/lens[0]),
            remesh_association=bool(np.any(own[faces]!='prism') or np.any(last[faces]<19)),
            topology_change_layer=19 if len(set(lev[e]))>1 or we is None else None,
            first_appearance_layer=0 if we is not None and len(set(lev[e]))==1 else 19,
            source_pair_first_layer=0 if we is not None else None))
    hist=np.array(length_history);np.savez_compressed(OUT/'EDGE_LINEAGE_ARRAYS.npz',edges=a['edges'],source_vertices=src[a['edges']],lengths=hist,terminal_length=a['edge_length'])
    save(OUT/'EDGE_LINEAGE.json',lineage)
    depth,binding=medial_depth(x,spec);fd,_=medial_depth(a['centre'],spec);ed,_=medial_depth(x[a['edges']].mean(1),spec)
    er=a['edge_length']/ed;fr=a['circumradius']/fd
    edge_normal=np.array([np.degrees(np.arccos(np.clip(a['normal'][ids[0]]@a['normal'][ids[1]],-1,1))) for ids in (incident[tuple(e)] for e in a['edges'])])
    face_normal=np.zeros(len(f))
    for j,e in enumerate(a['edges']):face_normal[incident[tuple(e)]]=np.maximum(face_normal[incident[tuple(e)]],edge_normal[j])
    maxj=int(np.argmax(a['edge_length']));maxedge=a['edges'][maxj];maxsrc=src[maxedge]
    # Source surface path: projected straight chord samples provide an available
    # path, not a geodesic claim. The analytic nearest medial curve is unchanged.
    samples=np.linspace(0,1,1001);chord=(1-samples[:,None])*x[maxedge[0]]+samples[:,None]*x[maxedge[1]]
    wallchord=(1-samples[:,None])*layers[0,maxsrc[0]]+samples[:,None]*layers[0,maxsrc[1]]
    s=source.evaluate(chord);ws=source.evaluate(wallchord)
    target=json.loads((OUT/'SCHEDULE.json').read_text())['nominal_targets']
    forensic=dict(**lineage[maxj],vertex_keys=[m['vertex_ids'][v] for v in maxedge],endpoint_coordinates=x[maxedge],wall_endpoint_coordinates=layers[0,maxsrc],
        length_by_layer=hist[maxj],layer19_source_row_is_unused=bool(np.all(lev[maxedge]<19)),
        wall_columns=np.flatnonzero(np.sum(np.isin(wall,maxsrc),axis=1)==2),
        wall_chord_phi=distribution(ws.phi),terminal_chord_phi=distribution(s.phi),
        endpoint_normal_angle_degrees=float(np.degrees(np.arccos(np.clip(s.normal[0]@s.normal[-1],-1,1)))),
        source_adjacency_verified=bool(tuple(sorted(maxsrc)) in wall_edges),nominal_schedule=target,
        length_reduction_from_wall=float(1-a['edge_length'][maxj]/hist[maxj,0]))
    # Exact analytic source projection for same-level edge, using existing normal.
    target_value=source.evaluate(x[maxedge]).phi
    projected=chord+(float(target_value.mean())-s.phi)[:,None]*s.normal
    forensic.update(projected_chord_path_polyline_length=float(np.linalg.norm(np.diff(projected,axis=0),axis=1).sum()),
        projected_path_residual=float(abs(source.evaluate(projected).phi-target_value.mean()).max()),
        path_method='1000-segment analytic-normal projection of chord; available sampled path, not shortest geodesic',
        terminal_endpoint_depth=depth[maxedge],edge_midpoint_depth=float(ed[maxj]))
    save(OUT/'MAX_EDGE.json',forensic)
    tails={}
    for pct in (.9,.95,.99,.995,.999):
        threshold=float(np.quantile(a['edge_length'],pct));ids=np.flatnonzero(a['edge_length']>=threshold)
        tails[str(pct)]=dict(descriptive_cutoff=threshold,count=len(ids),classes=dict(Counter(lineage[j]['classification'] for j in ids)),
            remesh_associated=sum(lineage[j]['remesh_association'] for j in ids),wall_lengths=distribution(hist[ids,0]),
            terminal_over_wall=distribution(a['edge_length'][ids]/hist[ids,0]))
    byowner={k:dict(faces=int((own==k).sum()),**{key:distribution(a[key][own==k]) for key in FIELDS},
        depth=distribution(fd[own==k]),radius_over_depth=distribution(fr[own==k])) for k in np.unique(own)}
    maxratio=int(er.argmax())
    report=dict(lineage_counts=dict(Counter(r['classification'] for r in lineage)),owner=byowner,tails=tails,
        depth=dict(vertex=distribution(depth),face_centroid=distribution(fd),edge_midpoint=distribution(ed),binding=binding),
        edge_over_depth=distribution(er),radius_over_depth=distribution(fr),
        max_edge_over_depth=dict(edge=maxratio,ratio=float(er[maxratio]),midpoint=x[a['edges'][maxratio]].mean(0),length=float(a['edge_length'][maxratio]),depth=float(ed[maxratio]),lineage=lineage[maxratio]),
        layer_maxima=[r['edge']['max'] for r in rows],wall_to_terminal_ratio=distribution(a['edge_length']/hist[:,0]),
        termination=dict(last_layer_counts=dict(Counter(map(str,last))),owners=dict(Counter(own)),source_columns=len(set(column)),
            terminal_faces_using_lower_vertices=int(np.any(lev[f]<19,axis=1).sum())),
        interface_exact=True,wall_first_layer_unchanged=True)
    save(OUT/'STACK_SUMMARY.json',report)
    np.savez_compressed(OUT/'TERMINAL_FIELDS.npz',points=x,faces=f,source=src,levels=lev,owner=own,column=column,last=last,
        depth=depth,face_depth=fd,edge_depth=ed,edge_over_depth=er,radius_over_depth=fr,normal_variation=face_normal,**a)
    vtk(OUT/'terminal_edges.vtk',x,a['edges'],np.full(len(a['edges']),3),dict(length=a['edge_length'],depth=ed,length_over_depth=er,wall_length=hist[:,0]))
    vtk(OUT/'terminal_faces.vtk',x,f,np.full(len(f),5),dict(area=a['area'],longest_edge=a['longest_edge'],minimum_angle=a['minimum_angle'],depth=fd,radius_over_depth=fr,owner=np.array([{'prism':0,'pyramid':1,'tetrahedron':2}[v] for v in own]),last_prism_layer=last))
    save(OUT/'PRE_CORE_HANDOFF.json',pre_core_audit(INTERFACE,spec));event('stack_complete',lineage=report['lineage_counts'])

def correlations():
    x,t,ci,iface=load_core(CORE,INTERFACE);q,m=evaluate(x,t,ci,iface)
    assert q['identity']=='538e0eed8f09c4ee598b540d622e21281832d303b8fcf7027a54a19498d0f411'
    assert q['failures']['determinant']==168 and q['bad_cells']==168
    save(OUT/'DIAGNOSTIC_CORE_QUALITY.json',q)
    a=np.load(OUT/'TERMINAL_FIELDS.npz');fx=a['points'];f=a['faces'];bad=m['bad'];centres=m['centres']
    nearest,distance=nearest_surface(centres,fx[f],lambda i,n:event('failure_mapping',done=i,total=n) if i%1024==0 else None)
    facebad=np.bincount(nearest[bad],minlength=len(f));direct=np.isin(np.arange(len(t)),m['interface_owner'])
    descriptor=dict(longest_edge=a['longest_edge'][nearest],edge_ratio=a['edge_ratio'][nearest],circumradius=a['circumradius'][nearest],
        minimum_angle=a['minimum_angle'][nearest],depth=a['face_depth'][nearest],radius_over_depth=a['radius_over_depth'][nearest],
        edge_over_depth=a['longest_edge'][nearest]/a['face_depth'][nearest],normal_variation=a['normal_variation'][nearest],distance=distance)
    # Match each failure to the closest passing cell centre in the same nearest
    # owner class and direct-incidence stratum; without replacement, stable ties.
    passing=np.setdiff1d(np.arange(len(t)),bad);matched=[];used=set();kinds=a['owner'][nearest]
    for b in sorted(bad):
        choices=np.array([p for p in passing[(kinds[passing]==kinds[b])&(direct[passing]==direct[b])] if p not in used])
        if not len(choices):raise ValueError('PassingControlStratumEmpty')
        dd=((centres[choices]-centres[b])**2).sum(1);pick=int(choices[np.argmin(dd)]);used.add(pick);matched.append(pick)
    matched=np.array(matched);comparison={key:dict(failing=distribution(v[bad]),matched_passing=distribution(v[matched]),all_passing=distribution(v[passing]),
        paired_failure_larger=int((v[np.sort(bad)]>v[matched]).sum()),paired_equal=int((v[np.sort(bad)]==v[matched]).sum())) for key,v in descriptor.items()}
    quantile_tests={}
    for key in ('longest_edge','edge_over_depth','edge_ratio','normal_variation','radius_over_depth'):
        v=descriptor[key];quantile_tests[key]=[]
        for pct in (.5,.9,.95,.99):
            cut=float(np.quantile(a['longest_edge'] if key=='longest_edge' else v,pct));mask=v>=cut
            quantile_tests[key].append(dict(percentile=pct,descriptive_cutoff=cut,population=int(mask.sum()),failures=int(mask[bad].sum()),
                failure_rate=float(mask[bad].sum()/mask.sum()),other_failure_rate=float((~mask[bad]).sum()/(~mask).sum()) if (~mask).any() else None))
    owner_tests={k:dict(population=int((kinds==k).sum()),failures=int((kinds[bad]==k).sum()),matched=int((kinds[matched]==k).sum())) for k in np.unique(kinds)}
    last=a['last'][nearest];transition=np.any(a['levels'][f[nearest]]<19,axis=1)
    # Nearest edges queried over all segments for the failing and control rows.
    edges=a['edges'];e0=fx[edges[:,0]];dv=fx[edges[:,1]]-e0;den=(dv*dv).sum(1)
    records=[]
    for typ,ids in [('failing',np.sort(bad)),('matched_passing',matched)]:
        for b in ids:
            v=centres[b]-e0;alpha=np.clip((v*dv).sum(1)/den,0,1);dist=np.linalg.norm(v-alpha[:,None]*dv,axis=1);e=int(dist.argmin());face=int(nearest[b])
            inc=np.flatnonzero(m['interface_owner']==b)
            records.append(dict(population=typ,cell=int(b),tet_vertices=t[b],coordinates=x[t[b]],centre=centres[b],determinant=float(m['determinant'][b]),
                direct_interface_faces=inc,nearest_face=face,nearest_edge=e,nearest_edge_distance=float(dist[e]),owner=str(kinds[b]),last_prism_layer=int(last[b]),
                remesh_transition_association=bool(transition[b]),**{key:float(v[b]) for key,v in descriptor.items()}))
    save(OUT/'FAILURE_CONTROL_MAP.json',records)
    np.savez_compressed(OUT/'CORE_INTERFACE_MAP.npz',nearest_face=nearest,distance=distance,determinant=m['determinant'],bad=bad,matched=matched,direct=direct,**{k:v for k,v in descriptor.items() if k!='distance'})
    result=dict(core=q,components=len(components(bad,m['owners'])),direct_interface_failures=int(direct[bad].sum()),non_direct_failures=int((~direct[bad]).sum()),
        comparison=comparison,descriptive_tail_tests=quantile_tests,owner=owner_tests,
        transition=dict(population=int(transition.sum()),failures=int(transition[bad].sum()),matched=int(transition[matched].sum())),
        matching='Nearest passing centroid without replacement; same nearest shell owner type and direct interface incidence; stable failing cell order.',
        inference_limit='Associations on a retained optimized core are not causal refinement evidence. Non-direct cells can still contain frozen vertices.')
    save(OUT/'CORRELATION.json',result)
    vtk(OUT/'core_failures.vtk',x,t[bad],np.full(len(bad),10),dict(determinant=m['determinant'][bad],nearest_face=nearest[bad],distance=distance[bad],edge_over_depth=descriptor['edge_over_depth'][bad]))
    vtk(OUT/'overlap.vtk',fx,f,np.full(len(f),5),dict(nearest_failure_count=facebad,longest_edge=a['longest_edge'],radius_over_depth=a['radius_over_depth']))
    event('correlation_complete',direct_bad=int(direct[bad].sum()),owner=owner_tests)

START=time.perf_counter()
if __name__=='__main__':
    import sys
    mode=sys.argv[1] if len(sys.argv)>1 else 'all'
    if mode=='extract':extract()
    if mode in ('all','stack'):stack()
    if mode in ('all','correlation'):correlations()
    event('finished',mode=mode)
