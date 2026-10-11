"""Read Foundation yPlus patch output, preserving global wall-face identity."""
import hashlib,json,re,sys
from pathlib import Path
import numpy as np
import capsule_steady_models as s
from capsule_rans_long_pilot import labels as binary_labels

def labels(path):
    data=path.read_bytes();header=data[:data.index(b'}')]
    if re.search(rb'format\s+binary\s*;',header):return binary_labels(path)
    if not re.search(rb'format\s+ascii\s*;',header):raise ValueError('UnknownLabelEncoding')
    m=re.search(rb'\n(\d+)\s*\(',data)
    if not m:raise ValueError('MissingLabelList')
    end=data.index(b')',m.end());values=np.fromstring(data[m.end():end].decode('ascii'),sep=' ',dtype=np.int64)
    if len(values)!=int(m[1]):raise ValueError('WrongAsciiLabelCount')
    return values

def patch_layout(path,name='wall'):
    text=path.read_text();m=re.search(r'\b'+name+r'\s*\{([^}]+)\}',text)
    if not m:raise ValueError('MissingWallPatch')
    return tuple(int(re.search(k+r'\s+(\d+)',m[1])[1]) for k in ('startFace','nFaces'))

def scalar_patch(path,count):
    data=path.read_bytes();m=re.search(rb'\bwall\s*\{[^{}]*?\bvalue\s+',data)
    if not m:raise ValueError('MissingWallValue')
    tail=data[m.end():]
    uniform=re.match(rb'uniform\s+([-+0-9.eE]+)\s*;',tail)
    if uniform:return np.full(count,float(uniform[1]))
    a=re.match(rb'nonuniform\s+List<scalar>\s+(\d+)\s*\(',tail)
    if not a or int(a[1])!=count:raise ValueError('WrongPatchCount')
    if re.search(rb'format\s+ascii\s*;',data[:m.start()]):
        end=tail.index(b')',a.end());values=np.fromstring(tail[a.end():end].decode('ascii'),sep=' ')
        if len(values)!=count:raise ValueError('WrongAsciiPatchCount')
        return values
    end=a.end()+8*count
    if tail[end:end+1]!=b')':raise ValueError('WrongPatchBinaryLayout')
    return np.frombuffer(tail[a.end():end],dtype='<f8').copy()

def pressure_patch(proc,time='500'):
    """A zeroGradient patch may omit value; its value is the owner-cell value."""
    import capsule_pressure_diagnostic as p
    proc=Path(proc);first,count=patch_layout(proc/'constant/polyMesh/boundary')
    path=proc/time/'p';data=path.read_bytes()
    m=re.search(rb'\bwall\s*\{([^}]+)\}',data);assert m
    assert re.search(rb'\btype\s+zeroGradient\s*;',m[1])
    if re.search(rb'\bvalue\s+',m[1]):return scalar_patch(path,count)
    owner=labels(proc/'constant/polyMesh/owner')[first:first+count]
    values=p.l.u.e.read_internal(path,1).reshape(-1)
    assert len(values)>int(owner.max())
    return values[owner]


def analyze(name,root=None):
    root=Path(root) if root else s.ROOT;d=root/name;case=d/'pilot';pm=case/'constant/polyMesh'
    start,count=patch_layout(pm/'boundary');assert count==33786
    values=np.full(count,np.nan);seen=np.zeros(count,dtype=int);files={}
    for rank in range(6):
        p=case/f'processor{rank}';first,n=patch_layout(p/'constant/polyMesh/boundary')
        ids=np.abs(labels(p/'constant/polyMesh/faceProcAddressing')[first:first+n])-1-start
        assert np.all((ids>=0)&(ids<count))
        field=p/'500/yPlus';values[ids]=scalar_patch(field,n);seen[ids]+=1
        files[str(field.relative_to(d))]=s.h.file_hash(field)
    assert np.all(seen==1) and np.all(np.isfinite(values)) and np.all(values>=0)
    x=np.load(d/'assembly/coordinates.npy');pr=np.load(d/'assembly/prisms.npy');wall=np.load(d/'assembly/wall.npy')
    triangles={tuple(sorted(t)):t for t in wall};owner=labels(pm/'owner')[start:start+count]
    centers=[];areas=[]
    for cell in owner:
        assert cell<len(pr)
        options=[triangles[tuple(sorted(t))] for t in (pr[cell,:3],pr[cell,3:]) if tuple(sorted(t)) in triangles]
        assert len(options)==1
        q=x[options[0]];centers.append(q.mean(0));areas.append(np.linalg.norm(np.cross(q[1]-q[0],q[2]-q[0]))/2)
    centers=np.array(centers);areas=np.array(areas);z=centers[:,2];angle=np.arctan2(centers[:,1],centers[:,0])
    masks=dict(front_cap=z<-.04,mid_body=np.abs(z)<=.04,rear_cap=z>.04)
    sector=np.floor((angle+np.pi)/(2*np.pi)*8).astype(int)%8
    masks.update({f'azimuth_{i}':sector==i for i in range(8)})
    out=s.h.seal(dict(schema='FoundationSolvedWallYPlus/1',mesh=name,iteration=500,faces=count,
        source=s.h.file_hash(s.ROOT/'model_sources/src/functionObjects/field/yPlus/yPlus.C'),
        method='Installed Foundation yPlus function object; nutUSpaldingWallFunction::yPlus; global faceProcAddressing; exact canonical prism-wall incidence',
        statistics=s.f.statistics(values),area_weighted_mean=float(np.dot(values,areas)/areas.sum()),
        regions={k:s.f.statistics(values[m]) for k,m in masks.items()},region_definition='Front z<-0.04; middle abs(z)<=0.04; rear z>0.04; eight equal azimuth sectors starting at -pi',
        canonical_values_sha256=hashlib.sha256(values.astype('<f8').tobytes()).hexdigest(),files=files))
    np.savez_compressed(d/'WALL_YPLUS.npz',global_faces=np.arange(start,start+count),values=values,centers=centers,areas=areas)
    s.h.durable_json(d/'WALL_YPLUS.json',out);print(json.dumps(out))

if __name__=='__main__':analyze(sys.argv[1],sys.argv[2] if len(sys.argv)>2 else None)
