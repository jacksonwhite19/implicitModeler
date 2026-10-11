"""Common dense terminal wake samples. No solver, interpolation tuning, or mesh edit."""
import hashlib
import numpy as np
from scipy.spatial import cKDTree
import capsule_broad_global_family as f
import capsule_pressure_diagnostic as p


def locations():
    # D/32 resolves the diagnostic sampling grid below the smallest new target.
    # It is not a CFD resolution or acceptance threshold.
    grid=np.arange(-32,33)*f.D/32
    disk=np.array([(x,y) for x in grid for y in grid if x*x+y*y<=f.D*f.D*(1+1e-14)])
    return np.array([[x,y,.08+s*f.D] for s in (.5,1,2,4,6,8,10) for x,y in disk]),len(disk)


def disk_metrics(points,velocity):
    deficit=np.maximum(33.528-velocity[:,2],0)
    weight=float(deficit.sum())
    if weight==0:return dict(deficit_integral=0,centroid=None,width_rms=None)
    center=np.sum(points[:,:2]*deficit[:,None],axis=0)/weight
    width=np.sqrt(np.sum(deficit*np.sum((points[:,:2]-center)**2,axis=1))/weight)
    return dict(deficit_integral=weight*(f.D/32)**2,centroid=center.tolist(),centroid_displacement=float(np.linalg.norm(center)),width_rms=float(width),mean_U=velocity.mean(0).tolist(),minimum_Uz=float(velocity[:,2].min()),maximum_deficit=float(deficit.max()))


def run():
    points,per_station=locations();root=f.ROOT/'wake';root.mkdir(exist_ok=True);result={}
    for name in ('Baseline','W1','R1','C2','C2-R','G1'):
        if name=='Baseline':case=p.c.OUT/'pilot';assembly=f.prior.BASE/'assembly'
        else:
            folder={'W1':f.OLD,'R1':f.OLD/'R1'}.get(name,f.ROOT/name)
            case=folder/'pilot';assembly=folder/'assembly'
        x=np.load(assembly/'coordinates.npy');pr=np.load(assembly/'prisms.npy');tet=np.load(assembly/'tets.npy')
        centers=np.concatenate([x[pr].mean(1),x[tet].mean(1)])
        distances,indices=cKDTree(centers).query(points)
        rankids=[p.l.labels(case/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
        assert np.array_equal(np.sort(np.concatenate(rankids)),np.arange(len(centers)))
        values={}
        for field,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
            a=np.empty((len(centers),dim))
            for i,ids in enumerate(rankids):a[ids]=p.l.u.e.read_internal(case/f'processor{i}'/'500'/field,dim)
            assert np.isfinite(a).all()
            values[field]=a[indices]
        np.savez_compressed(root/(name+'.npz'),points=points,cell_ids=indices,distances=distances,**values)
        stations={}
        for j,s in enumerate((.5,1,2,4,6,8,10)):
            sl=slice(j*per_station,(j+1)*per_station)
            stats=disk_metrics(points[sl],values['U'][sl]);stats['sampling_distance']=f.statistics(distances[sl])
            stats['sampled_fields']={field:f.statistics(a[sl]) for field,a in values.items() if field!='U'}
            stations[str(s)]=stats
        result[name]=dict(stations=stations,archive_sha256=f.h.file_hash(root/(name+'.npz')),scientific_arrays={field:f.h.array_hash(a) for field,a in dict(points=points,cell_ids=indices,distances=distances,**values).items()})
    record=f.h.seal(dict(schema='CapsuleCommonDenseWake/1',method='Same fixed Cartesian disk radiusD at s/D=0.5,1,2,4,6,8,10. Grid spacingD/32=0.0025m. Nearest canonical cell centroid; piecewise-constant samples. Positive axial deficit weights. No rigorous interpolation-error bound is claimed; distances are retained. Disk truncation and terminal iterative phase remain limitations.',point_count=len(points),points_per_station=per_station,variants=result))
    f.h.durable_json(f.ROOT/'DENSE_WAKE.json',record)
    print(record['identity'])


if __name__=='__main__':run()
