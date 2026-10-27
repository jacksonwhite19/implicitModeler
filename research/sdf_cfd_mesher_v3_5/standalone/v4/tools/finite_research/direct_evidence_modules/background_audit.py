import numpy as np
from finite_research.direct_evidence_modules import driver

def validate_mesh(points,rows,arity):
    driver.require(points.ndim==2 and points.shape[1]==3 and len(points)>0,'point shape')
    driver.require(rows.ndim==2 and rows.shape[1]==arity and len(rows)>0,'connectivity shape')
    driver.require(np.all(rows>=0) and np.all(rows<len(points)),'connectivity domain')
    driver.require(np.all(np.diff(np.sort(rows,axis=1),axis=1)>0),'distinct cell vertices')

def map_join(prefix_points,front_ids,core_points,tets,inner,far,faces):
    validate_mesh(core_points,tets,4)
    driver.require(len(inner)==len(front_ids) and len(np.unique(inner))==len(inner),'interface mapping')
    driver.require(np.all(inner>=0) and np.all(inner<len(core_points)),'interface index domain')
    driver.same(core_points[inner],prefix_points[front_ids],'exact interface bits')
    other=np.setdiff1d(np.arange(len(core_points)),inner)
    mapping=np.empty(len(core_points),dtype='<i8');mapping[inner]=front_ids;mapping[other]=len(prefix_points)+np.arange(len(other))
    points=np.concatenate([prefix_points,core_points[other]])
    driver.same(points[:len(prefix_points)],prefix_points,'preserved prefix point bytes')
    return points,mapping[tets],list(map(tuple,mapping[far][faces])),other

def interface_metrics(points,cells,owners,faces,prefix_ids,check):
    import local_termination_topology as topology
    from background_mixed_geometry import batch_geometry
    cq=topology.q.legacy_modules()['component_quality'];byid={cell['id']:cell for cell in cells}
    values=[];failures=[]
    for start in range(0,len(faces),4096):
        check();batch=faces[start:start+4096];pairs=[]
        for face in batch:
            uses=owners[tuple(sorted(face))]
            old=[owner for owner,cycle in uses if owner in prefix_ids];new=[owner for owner,cycle in uses if owner not in prefix_ids]
            driver.require(len(uses)==2 and len(old)==len(new)==1,'exact joined interface owners')
            pairs.append((old[0],new[0]))
        geometry=batch_geometry(points,[byid[owner] for owner in sorted({owner for pair in pairs for owner in pair})])
        vertices=points[np.asarray(batch,dtype='<i8')];areas,centres=cq.FG.face_area_centre_tri(vertices)
        old=[geometry[pair[0]] for pair in pairs];new=[geometry[pair[1]] for pair in pairs]
        metrics=cq.face_metrics(vertices,areas,centres,np.array([row['centre'] for row in old]),np.array([row['centre'] for row in new]),
            np.array([row['volume'] for row in old]),np.array([row['volume'] for row in new]))
        block=np.column_stack(metrics);driver.require(np.isfinite(block).all(),'finite interface metrics');values.append(block)
        for offset,pair in enumerate(pairs):
            weight,skew,ratio,angle=map(float,block[offset])
            if not weight>=.055 or not skew<=3.63636 or not ratio>=.01:
                failures.append(dict(vertices=list(map(int,batch[offset])),owners=list(pair),weight=weight,skewness=skew,symmetric_volume_ratio=ratio,nonorthogonality=angle))
    values=np.vstack(values)
    return dict(passed=not failures,faces=len(faces),failures=failures,failure_count=len(failures),
        minimum_weight=float(values[:,0].min()),maximum_skewness=float(values[:,1].max()),
        minimum_symmetric_volume_ratio=float(values[:,2].min()),maximum_nonorthogonality=float(values[:,3].max()),
        nonorthogonality_role='descriptive_only',scientific_acceptance=False)
