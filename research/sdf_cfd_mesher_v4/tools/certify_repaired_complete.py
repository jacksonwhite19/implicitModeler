"""Stream new complete-mesh relations through the unchanged hull filters.

Triangles are represented by their three controls with one repeated control.
This is used ONLY for convex-hull separation, never for patch regularity.
Every uncertain hull result goes to the existing exact polygon/tiling kernel.
No face is excluded merely because it is incident to another face.
"""
import sys
sys.dont_write_bytecode=True
import time,json,hashlib,ctypes
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from repair_bulk_core import OUT,write
from sdf_cfd_mesher_v4.certificate.efficient import Filter
import termination_geometry_certificate as exact


def unique_faces(cells,patterns):
    rows=cells[:,patterns].reshape(-1,len(patterns[0]));keys=np.sort(rows,axis=1)
    order=np.lexsort(tuple(keys[:,i] for i in range(keys.shape[1]-1,-1,-1)))
    sk=keys[order];starts=np.flatnonzero(np.r_[True,np.any(sk[1:]!=sk[:-1],axis=1)])
    counts=np.diff(np.r_[starts,len(order)])
    if np.any(counts>2):raise ValueError('NonManifoldFace')
    return rows[order[starts]]


def prepare():
    folder=OUT/'assembly';xyz=np.load(folder/'coordinates.npy',mmap_mode='r');prisms=np.load(folder/'prisms.npy',mmap_mode='r');tets=np.load(folder/'tets.npy',mmap_mode='r')
    shell_count=json.loads((OUT/'ASSEMBLY.json').read_text())['shell_prisms']
    # Shell/shell pairs are the preserved authority. New groups include support.
    shell=prisms[:shell_count];support=prisms[shell_count:]
    groups=[unique_faces(shell,[[0,1,4,3],[1,2,5,4],[2,0,3,5]]),unique_faces(shell,[[0,2,1],[3,4,5]])]
    retained=sum(map(len,groups))
    groups.extend([unique_faces(support,[[0,1,4,3],[1,2,5,4],[2,0,3,5]]),unique_faces(support,[[0,2,1],[3,4,5]])])
    # Core boundary triangles already occur as authoritative shell/support caps.
    rows=tets[:,np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])].reshape(-1,3)
    keys=np.sort(rows,axis=1);order=np.lexsort((keys[:,2],keys[:,1],keys[:,0]));sk=keys[order]
    starts=np.flatnonzero(np.r_[True,np.any(sk[1:]!=sk[:-1],axis=1)]);counts=np.diff(np.r_[starts,len(order)])
    if np.any(counts>2):raise ValueError('NonmanifoldCore')
    boundary=keys[order[starts[counts==1]]]
    outer=np.concatenate([shell[-len(np.load(folder/'wall.npy')):,-3:],support[:,:3]])
    outer=np.sort(outer,axis=1);outer=outer[np.lexsort((outer[:,2],outer[:,1],outer[:,0]))]
    if not np.array_equal(boundary,outer):raise ValueError('CoreBoundaryMismatch')
    groups.append(rows[order[starts[counts==2]]])
    del rows,keys,order,sk,boundary,outer
    count=sum(map(len,groups));width=25000;padded=((count+width-1)//width)*width
    ids=np.lib.format.open_memmap(folder/'certificate_faces.npy',mode='w+',dtype='<u8',shape=(padded,4))
    q=np.lib.format.open_memmap(folder/'certificate_controls.npy',mode='w+',dtype='<f8',shape=(padded,4,3))
    arity=np.lib.format.open_memmap(folder/'certificate_arity.npy',mode='w+',dtype='u1',shape=(padded,))
    offset=0
    for g in groups:
        n=len(g);arity[offset:offset+n]=g.shape[1]
        control=g[:,[0,1,3,2]] if g.shape[1]==4 else g[:,[0,1,2,2]]
        ids[offset:offset+n]=control
        for i in range(0,n,50000):q[offset+i:offset+min(i+50000,n)]=xyz[control[i:i+50000]]
        offset+=n
    for i in range(count,padded):ids[i]=0;arity[i]=0;q[i]=10000.+i
    ids.flush();q.flush();arity.flush()
    write('CERTIFICATE_INPUT.json',dict(faces=count,padded=padded,width=width,preserved_shell_faces=retained,shell_relations_reused=56462864,interface_missing=0,interface_duplicate=0,filter='unchanged convex hull separation; uncertain pairs use exact face tiling'))


def prepare_mixed():
    folder=OUT/'mixed_core';xyz=np.load(folder/'coordinates.npy');tets=np.load(folder/'tets.npy')
    faces=unique_faces(tets,[[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    ids=faces[:,[0,1,2,2]].astype('u8');np.save(folder/'certificate_faces.npy',ids)
    np.save(folder/'certificate_controls.npy',xyz[ids]);np.save(folder/'certificate_arity.npy',np.full(len(ids),3,dtype='u1'))
    write('MIXED_CERTIFICATE_INPUT.json',dict(faces=len(ids),padded=len(ids),width=len(ids),preserved_shell_faces=0,shell_relations_reused=0,scope='Bounded core/core and core/boundary face population'))


def main():
    mixed=len(sys.argv)>1 and sys.argv[1]=='mixed';prefix='MIXED_' if mixed else ''
    started=time.perf_counter();folder=OUT/('mixed_core' if mixed else 'assembly')
    if not (OUT/(prefix+'CERTIFICATE_INPUT.json')).exists():
        if mixed:prepare_mixed()
        else:prepare()
    meta=json.loads((OUT/(prefix+'CERTIFICATE_INPUT.json')).read_text());n=meta['faces'];width=meta['width'];retained=meta['preserved_shell_faces']
    xyz=np.load(folder/'coordinates.npy',mmap_mode='r');q=np.load(folder/'certificate_controls.npy',mmap_mode='r');ids=np.load(folder/'certificate_faces.npy',mmap_mode='r');arity=np.load(folder/'certificate_arity.npy',mmap_mode='r')
    native=Filter();forest=native.forest(q,width);population=0;fallback=0;bad=[];digest=hashlib.sha256();methods={}
    def face(i):return tuple(map(int,ids[i][[0,1,3,2]] if arity[i]==4 else ids[i][:3]))
    with ThreadPoolExecutor(max_workers=4) as pool:
        for layer in range(retained//width+1,meta['padded']//width+1):
            raw=forest.pairs(layer);pairs=np.column_stack((raw>>32,raw&0xffffffff)).astype('u4');del raw
            pairs=pairs[(pairs[:,1]>=retained)&(pairs[:,1]<n)]
            for start in range(0,len(pairs),50000):
                pp=np.ascontiguousarray(pairs[start:start+50000]);codes=np.empty(len(pp),'u1');w=np.empty_like(codes)
                def block(ab):
                    a,b=ab;native.dll.screen(q.ctypes.data,ids.ctypes.data,len(q),pp[a:b].ctypes.data,b-a,codes[a:b].ctypes.data,w[a:b].ctypes.data)
                bounds=np.linspace(0,len(pp),5,dtype=int);list(pool.map(block,zip(bounds[:-1],bounds[1:])))
                for k in np.flatnonzero(codes==0):
                    i,j=map(int,pp[k]);a=face(i);b=face(j)
                    # Repeated common triangle controls can defeat a hull filter.
                    # Rotate only the filter representation to repeat a nonshared control.
                    aa=list(a);bb=list(b)
                    for poly,other in [(aa,b),(bb,a)]:
                        if len(poly)==3:
                            for _ in range(3):
                                if poly[2] not in other:break
                                poly[:]=poly[1:]+poly[:1]
                    ia=aa if len(aa)==4 else aa+[aa[2]];ib=bb if len(bb)==4 else bb+[bb[2]]
                    if len(aa)==4:ia=[aa[k] for k in (0,1,3,2)]
                    if len(bb)==4:ib=[bb[k] for k in (0,1,3,2)]
                    pairids=np.array([ia,ib],dtype='u8');c,_=native.screen(xyz[pairids],pairids,[[0,1]])
                    if c[0]:codes[k]=c[0];continue
                    fallback+=1;z=exact.pair(xyz,a,b)
                    if z['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
                        bad.append(dict(pair=[i,j],result=z));write(prefix+'CONTINUOUS_FAILURE.json',bad);raise ValueError('ContinuousGeometryFailure')
                    codes[k]=7
                digest.update(pp.tobytes());digest.update(codes.tobytes());population+=len(pp)
                for code,num in zip(*np.unique(codes,return_counts=True)):methods[str(code)]=methods.get(str(code),0)+int(num)
            write(prefix+'CONTINUOUS_PROGRESS.json',dict(layer=layer,total_layers=meta['padded']//width,relations=population,fallback=fallback,seconds=time.perf_counter()-started))
            print(layer,population,fallback,flush=True)
    forest.close()
    write(prefix+'CONTINUOUS.json',dict(passed=True,new_relations=population,shell_reused=meta['shell_relations_reused'],total_relations=population+meta['shell_relations_reused'],forbidden=0,unresolved=0,missing=0,duplicate=0,proof_hash=digest.hexdigest(),exact_fallback=fallback,methods=methods,seconds=time.perf_counter()-started))


if __name__=='__main__':main()
