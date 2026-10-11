"""Source-matched diagnostics on a realized single layer; no future predictor."""
import json,time
from fractions import Fraction
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import generalize_local_remesh as g
from production_first_layer_diagnostics import stats,angles,triangle
from production_l2_screening import direct_screen
from production_adaptive_surface import CapsuleGeometry
from geometry_family_authority import Source
from sdf_cfd_mesher_v4.conditioning import matrices,gains
from sdf_cfd_mesher_v4.common.authority import array_hash,identity


def analyze(folder,prefix,high,tri,definition,increment,prior_quality,forecast_path):
    started=time.perf_counter();layer=len(prefix);base=layer-1;wall=prefix[0];low=prefix[-1]
    fronts=[*prefix,high];labels=['wall',*[f'L{k}' for k in range(1,layer+1)]]
    n=len(wall);top=g.q.build_topology(tri,n);edges=top.edges;source=Source(dict(components=[definition]))
    samples=[source.evaluate(x) for x in fronts];direction=samples[-2].normal;gradient=samples[-2].gradient
    gradient_unit=gradient/np.linalg.norm(gradient,axis=1)[:,None]
    displacement=high-low;normal_height=(displacement*direction).sum(1);prior=low-prefix[-2]
    actual_direction=displacement/np.linalg.norm(displacement,axis=1)[:,None]
    height=dict(euclidean=np.linalg.norm(displacement,axis=1),normal=normal_height,
        sdf_normal=(displacement*gradient_unit).sum(1),tangential=np.linalg.norm(displacement-normal_height[:,None]*direction,axis=1),
        requested_minus_achieved=increment-normal_height,achieved_requested=normal_height/increment)
    directions=dict(method='Existing SDF Newton predictor from the committed front to the bound cumulative target',
        angle_sdf=stats(angles(direction,gradient)),angle_prior=stats(angles(direction,prior)),
        realized_angle_sdf=stats(angles(actual_direction,gradient)),realized_angle_prior=stats(angles(actual_direction,prior)),
        gradient_norm=stats(np.linalg.norm(gradient,axis=1)),normal_norm=stats(np.linalg.norm(direction,axis=1)),
        zero=int((np.linalg.norm(direction,axis=1)<=0).sum()),nonfinite=int((~np.isfinite(direction)).sum()),
        inward=int(((direction*gradient).sum(1)<=0).sum()),fallbacks=0)
    geometry=CapsuleGeometry(definition);front_quality={};edge_arrays=[];tri_arrays=[]
    for label,x,sample in zip(labels,fronts,samples):
        lengths=np.linalg.norm(x[edges[:,1]]-x[edges[:,0]],axis=1);t=triangle(x,tri)
        edge_arrays.append(lengths);tri_arrays.append(t)
        cross=np.cross(x[tri[:,1]]-x[tri[:,0]],x[tri[:,2]]-x[tri[:,0]])
        adjacent=np.asarray([top.edge_triangles.row(i) for i in range(len(edges))])
        curvature=np.linalg.eigvalsh(np.array([geometry.gradient_hessian(1,pt)[1] for pt in x]))
        front_quality[label]=dict(edges=stats(lengths),triangles={k:stats(v) for k,v in t.items()},
            neighbouring_normal_degrees=stats(angles(sample.normal[edges[:,0]],sample.normal[edges[:,1]])),
            neighbouring_triangle_normal_degrees=stats(angles(cross[adjacent[:,0]],cross[adjacent[:,1]])),
            analytic_SDF_hessian_eigenvalues=stats(curvature),zero_area=int((t['area']<=0).sum()),
            reversal=int(((cross*samples[0].normal[tri].mean(1)).sum(1)<=0).sum()),nonfinite=int((~np.isfinite(x)).sum()))
    matched={}
    for old in (base,0):
        matched[f'L{layer}/{labels[old]}']=dict(edge_ratio=stats(edge_arrays[-1]/edge_arrays[old]),
            edge_change=stats(edge_arrays[-1]-edge_arrays[old]),area_ratio=stats(tri_arrays[-1]['area']/tri_arrays[old]['area']),
            area_change=stats(tri_arrays[-1]['area']-tri_arrays[old]['area']))
    print('front diagnostics complete',flush=True)
    points,cells,outer=g.s.construct(wall,tri,fronts[1:],np.full(n,layer))
    owners,own=g.t.ownership(cells,outer,[tuple(reversed(t)) for t in tri]);byid={c['id']:c for c in cells}
    q=json.loads((folder/'QUALITY.json').read_text());quality={}
    for scope in (f'L{layer}','combined'):
        cr=[r for r in q['cells'] if scope=='combined' or byid[r['id']]['layer']==layer]
        fr=[r for r in q['faces'] if scope=='combined' or any(byid[k]['layer']==layer for k in r['owners'])]
        record={}
        for key,rows,gate,upper in [('volume',cr,0,False),('determinant',cr,.0011,False),('aspect',cr,1000,True),
                ('weight',fr,.055,False),('skewness',fr,3.63636,True),('symmetric_volume_ratio',fr,.01,False)]:
            v=np.array([r[key] for r in rows]);i=int(v.argmax() if upper else v.argmin())
            bad=(v>gate if upper else v<gate) if key!='volume' else v<=0
            record[key]=dict(distribution=stats(v),gate=gate,failures=int((bad|~np.isfinite(v)).sum()),
                margin=float(gate-v[i] if upper else v[i]-gate),worst=rows[i])
        exact=[r for r in q['exact'] if scope=='combined' or byid[r['id']]['layer']==layer]
        v=np.array([float(Fraction(r['jacobian'])) for r in exact]);i=int(v.argmin())
        record['exact_jacobian']=dict(distribution=stats(v),gate=0,failures=sum(r['jacobian_sign']<=0 for r in exact),margin=float(v[i]),worst=exact[i])
        quality[scope]=record
    previous=json.loads(prior_quality.read_text());old={r['id']:r['determinant'] for r in previous['cells']}
    rows=[r for r in q['cells'] if byid[r['id']]['layer']==base]
    terminal=np.array([old[r['id']] for r in rows]);completed=np.array([r['determinant'] for r in rows])
    prefix_rows=[r for r in q['cells'] if byid[r['id']]['layer']<base]
    prefix_delta=np.array([r['determinant']-old[r['id']] for r in prefix_rows])
    determinant=dict(prior_terminal=stats(terminal),actual_next_neighbour=stats(completed),difference=stats(completed-terminal),
        unchanged_lower_prefix_difference=stats(prefix_delta),terminal_new=quality[f'L{layer}']['determinant']['distribution'],
        prior_hash=array_hash(terminal),completed_hash=array_hash(completed),difference_hash=array_hash(completed-terminal),
        semantics='Actual available neighbours; declared internal terminal cap only on new top front. No future centroid constructed.')
    geom=[g.t.geometry(points,c) for c in cells];sf=np.array([r['areas'] for r in geom]);mask=np.zeros((len(cells),5),bool)
    mask[:,0]=[c['layer']==1 for c in cells];ops=gains(matrices(sf,mask));conditioning={}
    for label,selection in [(f'L{k}',np.array([c['layer']==k for c in cells])) for k in range(1,layer+1)]+[('combined',np.ones(len(cells),bool))]:
        conditioning[label]={k:stats(v[selection]) for k,v in ops.items()}
    # Non-orthogonality is diagnostic; evaluate on every actual internal pair.
    cq=g.q.legacy_modules()['component_quality'];index={c['id']:i for i,c in enumerate(cells)}
    nonorth={f'L{layer}':[],'combined':[]}
    for entries in owners.values():
        if len(entries)!=2:continue
        (a,face),(b,_)=entries;vertices=points[list(face)][None]
        S,C=(cq.FG.face_area_centre_tri(vertices) if len(face)==3 else cq.FG.face_area_centre_quad(vertices))
        ga,gb=geom[index[a]],geom[index[b]]
        val=float(cq.face_metrics(vertices,S,C,ga['centre'][None],gb['centre'][None],np.array([ga['volume']]),np.array([gb['volume']]))[3][0])
        nonorth['combined'].append(val)
        if byid[a]['layer']==layer or byid[b]['layer']==layer:nonorth[f'L{layer}'].append(val)
    print('quality and determinant diagnostics complete',flush=True)
    screen=direct_screen(top,np.asarray(prefix),high,layer=layer);forecast=np.load(forecast_path);comparison={}
    def compare(name,a,b):
        a=np.asarray(a,float);b=np.asarray(b,float);delta=a-b
        comparison[name]=dict(equal=bool(np.array_equal(a,b,equal_nan=True)),absolute_difference=stats(np.abs(delta).ravel()),
            relative_difference=stats(np.abs(np.divide(delta,b,out=np.zeros_like(delta),where=b!=0)).ravel()))
    for group in ('cells','sides','caps'):
        for key,value in screen[group].items():compare(group+'__'+key,value,forecast[group+'__'+key])
    compare('coordinates',high,forecast['provisional_high'])
    realized=sorted((r for r in q['cells'] if byid[r['id']]['layer']==layer),key=lambda r:r['column'])
    for key in ('volume','determinant','aspect'):compare('final_validator__cells__'+key,[r[key] for r in realized],forecast['cells__'+key])
    edge_index={tuple(v):i for i,v in enumerate(edges)}
    for key in ('weight','skewness','symmetric_volume_ratio'):
        side=np.full(len(edges),np.nan);cap=np.full(len(tri),np.nan)
        for row in q['faces']:
            layers=[byid[i]['layer'] for i in row['owners']]
            if layers==[layer,layer]:side[edge_index[tuple(sorted(set(v%n for v in row['vertices'])))]]=row[key]
            elif set(layers)=={base,layer}:cap[byid[row['owners'][0]]['column']]=row[key]
        assert np.isfinite(side).all() and np.isfinite(cap).all()
        compare('final_validator__sides__'+key,side,forecast['sides__'+key]);compare('final_validator__caps__'+key,cap,forecast['caps__'+key])
    assert not screen['events']
    physical_edges=set()
    for entries in owners.values():
        face=entries[0][1];physical_edges.update(tuple(sorted((face[i],face[(i+1)%len(face)]))) for i in range(len(face)))
    pe=np.array(sorted(physical_edges))
    graph=coo_matrix((np.ones(2*len(pe)),(np.r_[pe[:,0],pe[:,1]],np.r_[pe[:,1],pe[:,0]])),shape=(len(points),len(points))).tocsr()
    fg=coo_matrix((np.ones(2*len(edges)),(np.r_[edges[:,0],edges[:,1]],np.r_[edges[:,1],edges[:,0]])),shape=(n,n)).tocsr()
    continuity=all(c['vertices']==list(map(int,np.r_[tri[c['column']]+(c['layer']-1)*n,tri[c['column']]+c['layer']*n])) for c in cells)
    topology=dict(front=screen['topology'],front_components=connected_components(fg)[0],new_prisms=len(tri),combined_cells=len(cells),
        vertices=len(points),edges=len(pe),unique_faces=len(owners),internal_faces=sum(len(v)==2 for v in owners.values()),
        boundary_faces=sum(len(v)==1 for v in owners.values()),ownership=own,vertex_links=g.t.vertex_links(outer),
        combined_components=connected_components(graph)[0],euler=len(points)-len(pe)+len(owners)-len(cells),
        source_column_continuity=continuity,duplicate_cells=len(cells)-len(set(c['id'] for c in cells)),
        unused_vertices=int(len(points)-len(np.unique(np.array([c['vertices'] for c in cells])))))
    assert continuity and not topology['duplicate_cells'] and not topology['unused_vertices'] and own['passed']
    result=dict(layer=layer,increment=increment,directions=directions,height={k:stats(v) for k,v in height.items()},
        height_nonpositive=int((normal_height<=0).sum()),fronts=front_quality,matched=matched,quality=quality,
        determinant=determinant,conditioning=conditioning,nonorthogonality={k:stats(v) for k,v in nonorth.items()},
        forecast=comparison,topology=topology,seconds=time.perf_counter()-started)
    from path_preserving_deduplication import durable_json
    durable_json(folder/'DIAGNOSTICS.json',result)
    np.savez_compressed(folder/'DIAGNOSTIC_ARRAYS.npz',**height,prior_terminal=terminal,completed_neighbour=completed,**ops)
    return result


if __name__=='__main__':
    from production_single_layer_cycle import OUT,V4,SURFACE,INCREMENT_HEX,CONFIG
    with np.load(V4/'results/production_adaptive_capsule_first_layer/A/CANDIDATE.npz') as z:wall=z['wall'];first=z['front'];tri=z['triangles']
    with np.load(V4/'results/production_adaptive_capsule_layer2/A/CANDIDATE.npz') as z:second=z['front']
    with np.load(OUT/'A/CANDIDATE.npz') as z:high=z['front']
    assert array_hash(second)==CONFIG['front_coordinates']
    definition=json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition']
    analyze(OUT/'A',[wall,first,second],high,tri,definition,float.fromhex(INCREMENT_HEX),
        V4/'results/production_adaptive_capsule_layer2/A/QUALITY.json',OUT/'SCREEN_ARRAYS.npz')
