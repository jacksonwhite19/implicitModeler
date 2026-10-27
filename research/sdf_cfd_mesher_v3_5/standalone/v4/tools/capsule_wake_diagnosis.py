"""Frozen-prism capsule wake diagnosis. Case-specific experiments only."""
import hashlib,json,shutil,subprocess,sys,time
import numpy as np
import capsule_sizing_control as control
import os
from types import SimpleNamespace
from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash,identity
def seal(r):return dict(r,identity=identity(r))
def verify(r):
    assert r['identity']==identity({k:v for k,v in r.items() if k!='identity'})
def durable(path,r):
    with open(path,'w') as f:json.dump(r,f,sort_keys=True,indent=2,allow_nan=False);f.flush();os.fsync(f.fileno())
h=SimpleNamespace(read=lambda p:json.loads(p.read_text()),array_hash=array_hash,file_hash=file_hash,seal=seal,verify_record=verify,durable_json=durable)
BASE=control.V4/'results/capsule_c1_core_qualification'
OUT=control.V4/'results/capsule_wake_diagnosis'
D=.08

def frozen():
    arrays={n:np.load(BASE/'assembly'/(n+'.npy')) for n in ('coordinates','prisms','wall')}
    manifest=h.read(BASE/'ASSEMBLY.json')
    for n,a in arrays.items():assert h.array_hash(a)==manifest['hashes'][n]
    x=arrays['coordinates'][:202740];prisms=arrays['prisms'];tri=arrays['wall']
    assert prisms.shape==(371646,6) and x.shape==(202740,3)
    assert h.array_hash(x[-16895:])=='e7a2493262644cc4bb29c76785aea1900e9eb8d2f09cec793853437287ce15c1'
    assert h.array_hash(tri)=='d14f00450f2f67f7b98ab3efa1a7946393b7b793ccd64d4bdcdbf516342de2db'
    return dict(prism_coordinates=h.array_hash(x),prism_connectivity=h.array_hash(prisms),L11_coordinates=h.array_hash(x[-16895:]),L11_topology=h.array_hash(tri),prism_count=len(prisms),prism_vertices=len(x),L11_vertices=16895,L11_faces=len(tri),changes=0)

def dist(a):
    a=np.asarray(a)
    return dict(n=int(a.size),min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max())) if a.size else dict(n=0)

def geometry():
    import capsule_pressure_diagnostic as prev
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()=='2b4eafbea355222a93343c4c55980100447ca113'
    OUT.mkdir(exist_ok=False)
    for path,expected in [(prev.OUT/'RESULT.json','b198c9b1311c9e713294ac2e807471bb06e2970973e083d93394f54838de9dc4'),(prev.d.OUT/'RESULT.json','fe735e7d14cc6e083fd54ceda7002be39aeaabf0f1663d978bf256ab8f0b39cb'),(prev.c.OUT/'RESULT.json','e6657f2659ce6f11133c5adfc431c4f7e6cce74ea31ed80bde498e3a08b84f6a')]:
        r=h.read(path);h.verify_record(r);assert r['identity']==expected
    m=prev.l.u.e.p.prior.m;m.validate(h.read(m.OUT/'AUTHORITY.json'),h.read(m.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    h.durable_json(OUT/'PREFLIGHT.json',dict(frozen=frozen(),free_bytes=shutil.disk_usage(OUT).free,status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()))
    (OUT/'.gitignore').write_text('W1/\nTERMINAL_PRISMS.npz\n')
    core=np.load(BASE/'C1/NETGEN.npz');p,t=core['points'],core['tets'];q=p[t];centers=q.mean(1)
    vol=np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0]))/6
    size=np.cbrt(6*np.sqrt(2)*vol);edges=q[:,[1,2,3,2,3,3]]-q[:,[0,0,0,1,1,2]];length=np.linalg.norm(edges,axis=2)
    radial=np.linalg.norm(centers[:,:2],axis=1);station=(centers[:,2]-.08)/D
    # Exact line/tetra clipping in barycentric coordinates; no re-tetrahedralization.
    inv=np.linalg.inv(np.stack([q[:,1]-q[:,0],q[:,2]-q[:,0],q[:,3]-q[:,0]],axis=2))
    def line_counts(origin,axis,lo,hi):
        b=np.einsum('nij,nj->ni',inv,np.asarray(origin)-q[:,0]);a=inv[:,:,axis]
        b=np.c_[1-b.sum(1),b];a=np.c_[-a.sum(1),a]
        low=np.full(len(t),lo);high=np.full(len(t),hi);valid=np.all((a!=0)|(b>=0),axis=1)
        for j in range(4):
            root=np.divide(-b[:,j],a[:,j],out=np.zeros(len(t)),where=a[:,j]!=0)
            low=np.where(a[:,j]>0,np.maximum(low,root),low);high=np.where(a[:,j]<0,np.minimum(high,root),high)
        return int(np.count_nonzero(valid&(high>low)))
    stations={}
    for s in (0,.5,1,2,4,6,8,10,12):
        mask=(abs(station-s)<=.25)&(radial<=D/2)
        stations[str(s)]=dict(slab_center_from_aft_D=s,centroid_cells=int(mask.sum()),h_volume=dist(size[mask]),edges=dist(length[mask]),D_over_median_h=float(D/np.median(size[mask])) if mask.any() else None,cells_across_D_x=line_counts([0,0,.08+s*D],0,-D/2,D/2),cells_across_D_y=line_counts([0,0,.08+s*D],1,-D/2,D/2))
    wake=(station>=0)&(station<=8)&(radial<=D)
    angle=np.arctan2(centers[:,1],centers[:,0]);az=np.floor((angle+np.pi)/(np.pi/4)).astype(int)%8
    sectors={str(i):dict(count=int(sum(wake&(az==i))),volume=dist(vol[wake&(az==i)]),h=dist(size[wake&(az==i)])) for i in range(8)}
    directions=edges[wake].reshape(-1,3);directions/=np.linalg.norm(directions,axis=1)[:,None]
    tensor=directions.T@directions/len(directions)
    from scipy.spatial import cKDTree
    tree=cKDTree(centers)
    probes=[]
    # Sample retained baseline finite-volume cell values at the nearest cell center.
    fields={n:np.empty((562033,d)) for n,d in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]};cc=np.empty((562033,3))
    for rank in range(6):
        folder=prev.c.OUT/'pilot'/f'processor{rank}';ids=prev.l.labels(folder/'constant/polyMesh/cellProcAddressing');cc[ids]=prev.l.u.e.read_internal(folder/'0/C',3)
        for n,d in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:fields[n][ids]=prev.l.u.e.read_internal(folder/'500'/n,d)
    flowtree=cKDTree(cc)
    for s in (.5,1,2,4,6,8,10,12):
        for r in (0,.25,.5,1):
            for theta in (np.arange(8)*np.pi/4 if r else [0]):
                point=[r*D*np.cos(theta),r*D*np.sin(theta),.08+s*D];distance,idx=flowtree.query(point)
                probes.append(dict(s_D=s,r_D=r,theta=float(theta),point=point,cell=int(idx),sample_distance=float(distance),**{n:fields[n][idx].tolist() for n in fields}))
    result=h.seal(dict(schema='FrozenPrismWakeGeometry/1',frozen=frozen(),core_vertices=len(p),tetrahedra=len(t),volume_size_definition='(6 sqrt(2) V)^(1/3)',core_size=dist(size),core_edges=dist(length),stations=stations,sectors=sectors,edge_direction_tensor=tensor.tolist(),edge_direction_eigenvalues=np.linalg.eigvalsh(tensor).tolist(),centerline_cells_0_to_8D=line_counts([0,0,0],2,.08,.72),wake_probe_method='Nearest finite-volume cell center; no interpolation; distances retained; steady iteration-500 values.',wake_probes=probes))
    h.durable_json(OUT/'BASELINE.json',result)
    print(json.dumps({k:result[k] for k in ('stations','sectors','edge_direction_tensor','centerline_cells_0_to_8D')},indent=2))

def plan():
    r=h.read(OUT/'BASELINE.json');h.verify_record(r)
    target=r['stations']['1']['h_volume']['median']/2
    h.durable_json(OUT/'WAKE_CONTRACT.json',h.seal(dict(schema='CapsuleWakeRefinement/1',case_specific=True,baseline=r['identity'],frozen=frozen(),candidate='W1',target=target,target_derivation='Half the measured median volume-derived tetra size in the one-diameter downstream station slab',radius=D,z_start=.04,z_end=.72,smoothing_length=target,g=.26236426446749106,h_max=.09470177661841601,field='min(baseline, target + g*(sqrt(distance_to_closed_cylinder^2+target^2)-target))',region_reason='Cylinder radius one capsule diameter, from aft hemisphere onset through eight downstream diameters; measured wake deficit persists there. Outer box and surface discretization remain baseline.',next_level='Only if W1 passes all hard gates and CFD evidence warrants another level; no automatic sweep.',stop_on_mesh_hard_failure=True)))
    control.write(OUT/'CANDIDATES.json',dict(candidates=[dict(id='W1',g=.26236426446749106,h_max=.09470177661841601)]))

def generate():
    from optimized_core_sizing import ExactCachedSizingField
    con=h.read(OUT/'WAKE_CONTRACT.json');h.verify_record(con);assert frozen()==con['frozen']
    class WakeField(ExactCachedSizingField):
        def evaluate_with_work(self,point):
            value,work=super().evaluate_with_work(point);x,y,z=point
            radial=max(0.,float(np.hypot(x,y))-con['radius']);axial=max(con['z_start']-z,0.,z-con['z_end'])
            distance=np.hypot(radial,axial);cap=con['target']+self.g*(np.hypot(distance,con['target'])-con['target'])
            return min(value,float(cap)),work
    if 'random_seed' in con:
        original_setup=control.setup;calls=[0]
        def seeded_setup(gmsh):
            original_setup(gmsh);calls[0]+=1
            if calls[0]>1:gmsh.option.setNumber('Mesh.RandomSeed',con['random_seed'])
        control.setup=seeded_setup
    control.OUT=OUT;control.CoreSizingField=WakeField
    result=control.run('W1');assert result['status']=='generated_interface_passed',result
    import capsule_control_optimize as opt
    opt.OUT=OUT;result=opt.run('W1');assert result['passed_boundary'],result
    assert frozen()==con['frozen']
    if 'random_seed' in con:
        old=np.load(OUT.parent/'W1/OUTER.npz');new=np.load(OUT/'W1/OUTER.npz')
        assert all(np.array_equal(old[n],new[n]) for n in old.files)
    h.durable_json(OUT/'POST_GENERATION_FREEZE.json',frozen())

def audit():
    import capsule_core_control_audit as a
    import capsule_user_domain_handoff as handoff
    import recursive_epoch_recovery as recovery
    from unittest.mock import patch
    a.OUT=OUT
    tick=time.perf_counter();record=recovery.discover(control.V4/'results/prism_assembly_optimization/epoch_access')
    values=list(handoff.verify_sources());values[0]=dict(values[0],reference=next(n['reference'] for n in record['nodes'] if n['source']==values[0]['reference']))
    def renewed():return tuple(values)
    renewed_root=renewed()[0]['reference']['root']
    with patch.object(handoff,'verify_sources',renewed),patch.dict(handoff.EXPECTED,access=renewed_root):a.prepare_prisms()
    h.durable_json(OUT/'ACCESS_REUSE.json',dict(existing_qualified_recovery=record['identity'] if 'identity' in record else 'CURRENT',seconds=time.perf_counter()-tick,new_renewal=False))
    result=a.audit('W1','NETGEN.npz');outer=a.outer_quality('W1')
    assert frozen()==h.read(OUT/'WAKE_CONTRACT.json')['frozen']
    h.durable_json(OUT/'POST_AUDIT_FREEZE.json',frozen())
    print(json.dumps(dict(failures=result['failures'],outer=outer['failures'],status=result['status'])))

def complete():
    import capsule_user_domain_handoff as metrics
    from certify_repaired_complete import unique_faces
    from optimized_mixed_certificate import certify
    assert not any(h.read(OUT/'W1/NETGEN_AUDIT.json')['failures'].values())
    frozen();folder=OUT/'assembly';folder.mkdir()
    x=np.load(BASE/'assembly/coordinates.npy')[:202740];prisms=np.load(BASE/'assembly/prisms.npy');wall=np.load(BASE/'assembly/wall.npy')
    core=np.load(OUT/'W1/NETGEN.npz');p,t=core['points'],core['tets'];front=x[-16895:];lookup={tuple(v):i for i,v in enumerate(p)};ci=np.array([lookup[tuple(v)] for v in front]);other=np.setdiff1d(np.arange(len(p)),ci)
    mapping=np.empty(len(p),dtype=np.int64);mapping[ci]=11*16895+np.arange(16895);mapping[other]=len(x)+np.arange(len(other))
    outer=np.load(OUT/'W1/OUTER.npz');oi=np.array([lookup[tuple(v)] for v in outer['points']])
    arrays=dict(coordinates=np.concatenate([x,p[other]]),prisms=prisms,tets=mapping[t],wall=wall,farfield=mapping[oi[outer['triangles']]])
    for n,a in arrays.items():np.save(folder/(n+'.npy'),a)
    assert np.array_equal(arrays['coordinates'][:202740],x) and h.array_hash(prisms)==h.read(OUT/'PREFLIGHT.json')['frozen']['prism_connectivity']
    x=arrays['coordinates'];t=arrays['tets'];quads=unique_faces(prisms,[[0,1,4,3],[1,2,5,4],[2,0,3,5]]);caps=unique_faces(prisms,[[0,2,1],[3,4,5]])
    corefaces=unique_faces(t,[[0,2,1],[0,1,3],[1,2,3],[2,0,3]]);oldcaps={tuple(sorted(f)) for f in caps};new=[tuple(map(int,f)) for f in corefaces if tuple(sorted(f)) not in oldcaps]
    faces=[tuple(map(int,f)) for f in quads]+[tuple(map(int,f)) for f in caps]+new;old=len(quads)+len(caps)
    tick=metrics.measured();r=certify(x,faces,range(old,len(faces)));proofs=r.pop('proofs');proofs.write_jsonl(OUT/'PROOFS.jsonl.gz');r['metrics']=metrics.metrics(*tick);r['immutable_prism_proofs_reused']=33336032
    h.durable_json(OUT/'CONTINUOUS.json',r);assert r['passed']
    h.durable_json(OUT/'ASSEMBLY.json',dict(hashes={n:h.array_hash(a) for n,a in arrays.items()},vertices=len(x),prisms=len(prisms),tets=len(t),frozen=frozen()))

def foundation():
    import ast
    source=control.V4/'tools/qualify_capsule_c1_core.py';tree=ast.parse(source.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='foundation')
    ns=dict(OUT=OUT,control=control,json=json);exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(source),'exec'),ns);ns['foundation']()



def prepare_cfd():
    verify_candidate_prisms()
    import re
    import capsule_pressure_diagnostic as prev
    assert h.read(OUT/'FOUNDATION.json')['passed'] and h.read(OUT/'CONTINUOUS.json')['passed']
    case=OUT/'pilot';assert not case.exists()
    source=prev.l.OUT/'controls';con=h.read(prev.l.OUT/'CONTRACT.json');h.verify_record(con)
    for n,sha in con['controls'].items():assert h.file_hash(source/n)==sha
    shutil.copytree(source,case)
    pm=case/'constant/polyMesh';shutil.copytree(OUT/'foundation/openfoam_case/constant/polyMesh',pm)
    b=pm/'boundary';text=b.read_text();match=re.search(r'\bwall\s*\{[^}]+\}',text);block=match.group();new=block.replace('type            patch;','type            wall;').replace('physicalType    patch;','physicalType    wall;');assert new!=block
    b.write_text(text[:match.start()]+new+text[match.end():],newline='\n')
    ctl=case/'system/controlDict';ctl.write_text(ctl.read_text().replace('endTime 200;','endTime 500;'),newline='\n')
    hashes={n:h.file_hash(case/n) for n in con['controls']}
    for n,sha in hashes.items():
        if n!='system/controlDict':assert sha==con['controls'][n]
    h.durable_json(OUT/'CFD_CONTRACT.json',h.seal(dict(status='NON_QUALIFYING_WAKE_MESH_DIAGNOSTIC',horizon=500,start=0,controls=hashes,initial={n:hashes['0/'+n] for n in ('U','p','k','omega','nut')},mesh=h.read(OUT/'ASSEMBLY.json'),comparison='Same original fields, accurate PCG/DIC and 500-iteration observation horizon. No post-launch tuning.',adapter=dict(before=block,after=new),frozen=frozen())))
    linux='/mnt/c/'+str(OUT.resolve())[3:].replace('\\','/')
    (OUT/'run_cfd.sh').write_text(f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun > EXECUTABLE.txt
checkMesh -constant -allTopology -allGeometry -case pilot > adapted_checkMesh.log 2>&1
grep -q 'Mesh OK.' adapted_checkMesh.log
decomposePar -case pilot > decompose.log 2>&1
set +e
/usr/bin/time -v -o pilot.resources mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot > pilot.log 2>&1
echo $? > pilot.exit
''',newline='\n')
    with open(OUT/'.gitignore','a') as f:f.write('assembly/\nfoundation/\npilot/\nPROOFS.jsonl.gz\n')



def core_metrics(path):
    c=np.load(path);q=c['points'][c['tets']];center=q.mean(1)
    v=np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0]))/6
    size=np.cbrt(6*np.sqrt(2)*v);r=np.linalg.norm(center[:,:2],axis=1);s=(center[:,2]-.08)/D
    az=np.floor((np.arctan2(center[:,1],center[:,0])+np.pi)/(np.pi/4)).astype(int)%8
    out={}
    for station in (0,.5,1,2,4,6,8,10,12):
        m=(abs(s-station)<=.25)&(r<=D/2)
        out[str(station)]=dict(h_volume=dist(size[m]),cells=int(m.sum()),sectors={str(i):dist(size[m&(az==i)]) for i in range(8)})
    return out

def analyze_cfd():
    verify_candidate_prisms()
    import capsule_pressure_diagnostic as prev
    from scipy.spatial import cKDTree
    con=h.read(OUT/'CFD_CONTRACT.json');h.verify_record(con)
    for n,sha in con['controls'].items():assert h.file_hash(OUT/'pilot'/n)==sha
    text=(OUT/'pilot.log').read_text();rows=prev.l.u.e.parse_history(text)
    assert prev.l.u.e.warning_audit(text)
    assert [r['iteration'] for r in rows]==list(range(1,501))
    assert int((OUT/'pilot.exit').read_text())==0
    h.durable_json(OUT/'HISTORY.json',dict(rows=rows))
    n=h.read(OUT/'ASSEMBLY.json')['tets']+371646
    x=np.load(OUT/'assembly/coordinates.npy');pr=np.load(OUT/'assembly/prisms.npy');tet=np.load(OUT/'assembly/tets.npy')
    centers=np.concatenate([x[pr].mean(1),x[tet].mean(1)])
    rankids=[prev.l.labels(OUT/'pilot'/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
    checkpoints={};terminal=None
    for step in (20,50,100,200,300,400,500):
        arrays={}
        for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
            a=np.empty((n,dim))
            for i,ids in enumerate(rankids):a[ids]=prev.l.u.e.read_internal(OUT/'pilot'/f'processor{i}'/str(step)/name,dim)
            arrays[name]=a
        idx=int(arrays['omega'].argmax());fields={}
        for name,a in arrays.items():
            val=np.linalg.norm(a,axis=1) if name=='U' else a[:,0]
            fields[name]=dict(min=float(val.min()),max=float(val.max()),negative=int((val<0).sum()),nonfinite=int((~np.isfinite(a)).sum()),sha256=hashlib.sha256(a.astype('<f8').tobytes()).hexdigest())
        # First prism layer owns the wall in the canonical cell ordering.
        checkpoints[str(step)]=dict(fields=fields,omega_location=dict(cell=idx,center=centers[idx].tolist(),wall_adjacent=idx<33786,omega=float(arrays['omega'][idx,0]),k=float(arrays['k'][idx,0]),nut=float(arrays['nut'][idx,0]),U=float(np.linalg.norm(arrays['U'][idx]))))
        terminal=arrays
    probes=[];tree=cKDTree(centers)
    for p in h.read(OUT/'BASELINE.json')['wake_probes']:
        distance,idx=tree.query(p['point']);probes.append(dict(point=p['point'],s_D=p['s_D'],r_D=p['r_D'],theta=p['theta'],distance=float(distance),cell=int(idx),**{n:a[idx].tolist() for n,a in terminal.items()}))
    def summary(rr):
        return dict(last=rr[-1],bounds=[dict(iteration=r['iteration'],**b) for r in rr for b in r['bounds']],flux_max=max(r['flux']['normalized'] for r in rr),wall_max=max(r['flux']['wall_sumMag'] for r in rr),linear_totals={eq:sum(r['residuals'][eq]['iterations'] for r in rr) for eq in rr[0]['residuals']},maxiter=[r['iteration'] for r in rr if any(v['iterations']>=5000 for v in r['residuals'].values())])
    baseline=h.read(prev.c.OUT/'HISTORY.json')['rows']
    result=h.seal(dict(schema='CapsuleWakeComparison/1',contract=con['identity'],baseline=dict(summary=summary(baseline),windows=prev.windows(baseline)),W1=dict(summary=summary(rows),windows=prev.windows(rows)),checkpoints=checkpoints,wake_probes=probes,core_resolution=dict(baseline=core_metrics(BASE/'C1/NETGEN.npz'),W1=core_metrics(OUT/'W1/NETGEN.npz')),history=h.file_hash(OUT/'HISTORY.json'),log=h.file_hash(OUT/'pilot.log'),frozen=frozen()))
    h.durable_json(OUT/'MEASUREMENTS.json',result)
    print(json.dumps(dict(identity=result['identity'],W1=result['W1']['summary'],terminal=checkpoints['500']),indent=2))



def extra_geometry():
    result={}
    for label,path in [('baseline',BASE/'C1/NETGEN.npz'),('W1',OUT/'W1/NETGEN.npz')]:
        z=np.load(path);q=z['points'][z['tets']];t=z['tets'];center=q.mean(1)
        vol=np.abs(np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0]))/6)
        faces=np.sort(t[:,[[0,1,2],[0,1,3],[0,2,3],[1,2,3]]].reshape(-1,3),axis=1)
        order=np.lexsort(faces.T[::-1]);sf=faces[order];same=np.all(sf[1:]==sf[:-1],axis=1);a=order[:-1][same]//4;b=order[1:][same]//4
        wake=(center[:,2]>=.08)&(center[:,2]<=.72)&(np.linalg.norm(center[:,:2],axis=1)<=D)
        ratio=np.maximum(vol[a],vol[b])/np.minimum(vol[a],vol[b]);chosen=wake[a]|wake[b]
        result[label]=dict(neighbour_volume_ratio=dist(ratio),wake_neighbour_linear_size_ratio=dist(np.cbrt(ratio[chosen])),wake_centroid_mean=center[wake].mean(0).tolist(),wake_centroid_covariance=np.cov(center[wake].T).tolist(),wake_volume_total=float(vol[wake].sum()))
    x=np.load(BASE/'assembly/coordinates.npy');far=np.load(BASE/'assembly/farfield.npy');tri=x[far];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);area=np.linalg.norm(normal,axis=1)/2
    sides={}
    for axis in range(3):
        for sign in (-1,1):
            m=(normal[:,axis]*sign>0)&(np.argmax(abs(normal),axis=1)==axis)
            sides[str((axis,sign))]=dict(faces=int(m.sum()),areas=dist(area[m]))
    result['baseline_outer_box']=sides
    front=x[11*16895:12*16895];angles=np.floor((np.arctan2(front[:,1],front[:,0])+np.pi)/(np.pi/4)).astype(int)%8
    result['frozen_L11_vertex_azimuth_counts']=np.bincount(angles,minlength=8).tolist()
    h.durable_json(OUT/'GEOMETRY_EXTRA.json',h.seal(result))



def seal_candidate():
    verify_candidate_prisms()
    f=h.read(OUT/'FOUNDATION.json');a=h.read(OUT/'ASSEMBLY.json');c=h.read(OUT/'CONTINUOUS.json')
    assert f['passed'] and c['passed'] and not c['failures']
    assert frozen()==a['frozen']
    for n,sha in a['hashes'].items():assert h.array_hash(np.load(OUT/'assembly'/(n+'.npy')))==sha
    record=h.seal(dict(schema='FrozenPrismWakeCoreCandidate/1',classification='V4CapsuleWakeRefinedCoreCandidateMeshGatesPassed',production_promotion=False,prism_scientific_identity='48f39ddfae1f782f724321e0d8882f9454928c5c0c2898652220dcfa6db3f1a1',prism_qualification='75d827a837748dfca5014b873a18f4076843752ef6789622565f6de6d2ba8e7b',L11_authority='356e5a357bbbdf923f7dd3a2df0642632ba3497518b0305c44f372d6fec7f9b3',frozen=frozen(),assembly=a,quality=f['summary'],continuous_pairs=c['required_pairs'],prism_proofs_reused=c['immutable_prism_proofs_reused'],evidence={n:h.file_hash(OUT/n) for n in ('WAKE_CONTRACT.json','ASSEMBLY.json','CONTINUOUS.json','PROOFS.jsonl.gz','FOUNDATION.json','W1/NETGEN.npz','W1/NETGEN_AUDIT.json','W1/OUTER_QUALITY.json','W1/OPTIMIZATION.json')}))
    h.durable_json(OUT/'CANDIDATE.json',record)



def realization_plan():
    parent=OUT;result=h.read(parent/'MEASUREMENTS.json');h.verify_record(result)
    child=parent/'R1';child.mkdir(exist_ok=False)
    for name in ('BASELINE.json','PREFLIGHT.json','CANDIDATES.json'):shutil.copyfile(parent/name,child/name)
    con=h.read(parent/'WAKE_CONTRACT.json');con.pop('identity');con.update(candidate='R1',random_seed=2,baseline_seed=1,question='Same W1 sizing and boundary, independent interior realization to separate directional selection from wake refinement.',seed_source='Gmsh 4.15.2 reference manual Mesh.RandomSeed: seed of pseudo-random number generator; default 1. https://gmsh.info/doc/texinfo/#Mesh-options',parent_measurements=result['identity'])
    h.durable_json(child/'WAKE_CONTRACT.json',h.seal(con))
    (child/'.gitignore').write_text('W1/\nTERMINAL_PRISMS.npz\nassembly/\nfoundation/\npilot/\nPROOFS.jsonl.gz\n')



def require_same_prisms(old_x,old_p,new_x,new_p,vertex_count):
    if not np.array_equal(old_x[:vertex_count],new_x[:vertex_count]):raise ValueError('FrozenPrismCoordinatesChanged')
    if not np.array_equal(old_p,new_p):raise ValueError('FrozenPrismConnectivityChanged')

def verify_candidate_prisms():
    require_same_prisms(np.load(BASE/'assembly/coordinates.npy'),np.load(BASE/'assembly/prisms.npy'),np.load(OUT/'assembly/coordinates.npy'),np.load(OUT/'assembly/prisms.npy'),202740)
    assert np.array_equal(np.load(BASE/'assembly/wall.npy'),np.load(OUT/'assembly/wall.npy'))
    return frozen()



def courant_probe():
    case=OUT/'courant_probe';shutil.copytree(OUT/'pilot/system',case/'system')
    shutil.copytree(OUT/'pilot/constant',case/'constant')
    for rank in range(6):
        for part in ('constant','500'):shutil.copytree(OUT/'pilot'/f'processor{rank}'/part,case/f'processor{rank}'/part)
    with open(OUT/'.gitignore','a') as f:f.write('courant_probe/\n')



def courant_analysis():
    import capsule_pressure_diagnostic as prev
    n=h.read(OUT/'FOUNDATION.json')['summary']['cells'];rates=np.empty(n)
    for i in range(6):
        folder=OUT/'courant_probe'/f'processor{i}'
        ids=prev.l.labels(folder/'constant/polyMesh/cellProcAddressing')
        rates[ids]=prev.l.u.e.read_internal(folder/'500/Co',1)[:,0]
    x=np.load(OUT/'assembly/coordinates.npy');pr=np.load(OUT/'assembly/prisms.npy');tet=np.load(OUT/'assembly/tets.npy');centers=np.concatenate([x[pr].mean(1),x[tet].mean(1)])
    j=int(rates.argmax());wake=(centers[:,2]>=.08)&(centers[:,2]<=.72)&(np.linalg.norm(centers[:,:2],axis=1)<=D)
    record=h.seal(dict(status='MeasuredCourantRatesOnlyNotURANSContract',formula='Co/deltaT = 0.5*surfaceSum(mag(phi))/V; actual retained conservative phi and Foundation cell volumes',runtime_registration='foamPostProcess -parallel -time 500 -fields (phi) -func CourantNo executed successfully',max_rate=float(rates[j]),max_cell=j,max_center=centers[j].tolist(),max_in_frozen_prism=j<371646,all=dist(rates),prism=dist(rates[:371646]),wake=dist(rates[wake]),D_over_U=D/33.528,domain_transit=2.5/33.528,illustrative_Co_point2=dict(dt=.2/float(rates[j]),steps_per_D=(D/33.528)/(.2/float(rates[j])),steps_for_10D=10*(D/33.528)/(.2/float(rates[j]))),warning='Resource estimate, not a frozen time-step/horizon or frequency claim. Co=0.2 is documented in shipped elipsekkLOmega, but its physics are not adopted.',canonical_rate_hash=h.array_hash(rates)))
    h.durable_json(OUT/'COURANT.json',record)
    sources=['src/functionObjects/field/CourantNo/CourantNo.C','tutorials/incompressibleFluid/cylinder/system/fvSolution','tutorials/incompressibleFluid/cylinder/system/fvSchemes','tutorials/incompressibleFluid/motorBike/lesFiles/fvSchemes','tutorials/incompressibleFluid/elipsekkLOmega/system/controlDict']
    folder=OUT/'transient_sources';folder.mkdir(exist_ok=True);manifest={}
    for i,name in enumerate(sources):
        raw=subprocess.check_output(['wsl','cat','/opt/openfoam13/'+name]);dest=folder/(str(i)+'_'+name.split('/')[-1]);dest.write_bytes(raw);manifest[name]=h.file_hash(dest)
    h.durable_json(folder/'MANIFEST.json',manifest)



def sampled_wake(probes):
    result={};edges=[0,.125,.375,.75,1.0];radii=[0,.25,.5,1]
    for station in sorted({p['s_D'] for p in probes}):
        pp=[p for p in probes if p['s_D']==station];xy=np.array([p['point'][:2] for p in pp]);u=np.array([p['U'] for p in pp]);weights=[]
        for p in pp:
            j=radii.index(p['r_D']);weights.append(np.pi*D*D*(edges[j+1]**2-edges[j]**2)/(1 if j==0 else 8))
        mass=np.asarray(weights)*np.maximum(33.528-u[:,2],0);total=mass.sum();center=(xy*mass[:,None]).sum(0)/total
        result[str(station)]=dict(center=center.tolist(),width_rms=float(np.sqrt((mass*((xy-center)**2).sum(1)).sum()/total)),sampled_deficit_integral=float(total),Uz=dist(u[:,2]),lateral_velocity=dist(np.linalg.norm(u[:,:2],axis=1)),note='Fixed 25-point disk quadrature, radius D; positive axial deficit; nearest-cell values. Diagnostic approximation, not a resolved wake integral.')
    return result



def build_report():
    m=h.read(OUT/'MEASUREMENTS.json');r=h.read(OUT/'R1/MEASUREMENTS.json')
    runs={'baseline':m['baseline'],'W1':m['W1'],'R1':r['W1']}
    probes={'baseline':h.read(OUT/'BASELINE.json')['wake_probes'],'W1':m['wake_probes'],'R1':r['wake_probes']}
    combined=h.seal(dict(schema='CapsuleWakeFamilyComparison/1',runs=runs,wake_disks={k:sampled_wake(v) for k,v in probes.items()},W1_measurements=m['identity'],R1_measurements=r['identity'],geometry=h.read(OUT/'GEOMETRY_EXTRA.json')['identity'],courant=h.read(OUT/'COURANT.json')['identity']))
    h.durable_json(OUT/'COMPARISON.json',combined)
    lines=['# Frozen-prism core and wake diagnosis','','All CFD runs are NON_QUALIFYING. No stopping contract, A/A qualification, production promotion or physical-frequency claim.','',
    'Baseline, W1 and R1 use the exact original prism shell. L11 coordinates and topology are unchanged. The box surface arrays are also exact. W1 changes the interior sizing field. R1 repeats the W1 field using Gmsh Mesh.RandomSeed=2 instead of default 1, only after outer surface generation. Gmsh 4.15.2 documents this as the pseudo-random generator seed: https://gmsh.info/doc/texinfo/#Mesh-options . Fixed six-rank partitioning method is retained; changed cell realization also changes partition membership. The test does not separate geometry sensitivity from roundoff amplification.','',
    'W1 target = 0.013784410412032215 m, half baseline median tetra volume-size at s/D=1. Cylinder radius 0.08 m, z from 0.04 to 0.72 m. h = min(h_baseline, target + g*(sqrt(distance_to_cylinder^2+target^2)-target)), g=0.26236426446749106, h_max=0.09470177661841601 m. R1 uses the same field. No universal wake rule or adequacy threshold is claimed.','',
    'CFD: air rho=1.225 kg/m3, mu=1.7894e-5 Pa s, nu=1.4607346938775508e-5 m2/s; U=(0,0,33.528) m/s; SST; kInf=3.060969551020408e-5, omegaInf=2095.5, nutInf=1.4607346938775508e-8. Original initial fields. PCG/DIC p tolerance1e-13 relTol0 maxIter5000; U/k/omega smoothSolver GaussSeidel tolerance1e-8 relTol0.1 nSweeps1 maxIter5000; SIMPLE consistent yes, momentumPredictor yes, nonorthogonal correctors0; U/k/omega relaxation0.9/0.5/0.5; U linearUpwindV, k/omega upwind. Wall and one-patch freestream BCs unchanged. Both new runs use 500 fixed outer iterations.','',
    'Exact command in each isolated folder: mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot. Baseline was 200+300 with a restart; new controls run continuously from iteration0. This is a remaining comparison limitation.','',
    '## Mesh evidence','','|Mesh|Points|Prisms|Tets|Min volume|Min determinant|Min weight|Min ratio|Max skew|Max aspect|','|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for name,folder in [('baseline',BASE),('W1',OUT),('R1',OUT/'R1')]:
        f=h.read(folder/'FOUNDATION.json');s=f['summary'];lines.append('|'+name+'|'+'|'.join(str(v) for v in [s['points'],371646,s['cells']-371646,s['minimum_cell_volume'],s['minimum_determinant'],s['minimum_interpolation_weight'],s['minimum_face_volume_ratio'],s['maximum_face_skewness'],s['maximum_cell_aspect_ratio']])+'|')
        if name!='baseline':
            c=h.read(folder/'CONTINUOUS.json');lines+=['',name+': Mesh OK.; exact Foundation conversion; all quality failures zero; '+str(c['required_pairs'])+' new pairs, '+str(c['immutable_prism_proofs_reused'])+' inherited prism proofs, continuous failures zero. Candidate '+h.read(folder/'CANDIDATE.json')['identity']+'.','']
    lines+=['','## Measured tetra volume-size and resolution','','s=(z-0.08)/0.08. Slab half-width0.25D, radius D/2. h=(6 sqrt(2) volume)^(1/3).','','|s/D|Baseline median h|W1 median h|R1 median h|Baseline centroid cells|W1 cells|R1 cells|','|---|---:|---:|---:|---:|---:|---:|']
    cores={'baseline':m['core_resolution']['baseline'],'W1':m['core_resolution']['W1'],'R1':r['core_resolution']['W1']}
    for s in cores['baseline']:
        lines.append('|'+s+'|'+'|'.join(str(cores[k][s]['h_volume'].get('median')) for k in cores)+'|'+'|'.join(str(cores[k][s]['cells']) for k in cores)+'|')
    lines+=['','## Steady histories and late statistics','','Initial residuals are used for outer convergence diagnostics. Every solve, force, moment and conservative flux is preserved in HISTORY.json. No new acceptance limits are introduced.','']
    for window in ('201-500','301-500','401-500','451-500'):
        lines+=['',window,'','|Mesh|Signal|Mean|Std|Range|Slope/iteration|','|---|---|---:|---:|---:|---:|']
        for name,v in runs.items():
            for signal,s in v['windows'][window]['signals'].items():lines.append('|'+name+'|'+signal+'|'+'|'.join(str(s[k]) for k in ('mean','std','range','slope'))+'|')
            lines+=['','Lateral vectors '+name+': '+json.dumps(v['windows'][window]['lateral'],sort_keys=True),'']
    lines+=['','## Terminal forces, health and conservation','']
    for name,v in runs.items():lines += [name+': '+json.dumps(v['summary'],sort_keys=True),'']
    lines+=['Conservation uses S=sum(phi), M=sum(abs(phi)), inflow=(M-S)/2, outflow=(M+S)/2, imbalance=abs(S)/max(inflow,outflow). Wall phi monitored independently. Force scale3.460917477391561 N; moment scale0.5537467963826498 N m.','', '## Fields and wall localization','']
    for name,measurement in [('W1',m),('R1',r)]:
        for step,c in measurement['checkpoints'].items():lines += [name+' iteration '+step+': '+json.dumps(c,sort_keys=True),'']
    lines+=['','## Wake sampling','','Identical 25 points per station, nearest finite-volume cell center, with sampling distances retained. Disk moments use fixed area-weighted positive axial-deficit quadrature out to radius D. These are coarse diagnostics, not interpolation-error bounds or exact cross-section integrals.','','|s/D|Baseline centerline Uz|W1 Uz|R1 Uz|','|---|---:|---:|---:|']
    for i,p in enumerate(probes['baseline']):
        if p['r_D']==0:lines.append('|'+str(p['s_D'])+'|'+'|'.join(str(probes[k][i]['U'][2]) for k in probes)+'|')
    lines+=['',json.dumps(combined['wake_disks'],sort_keys=True),'','## Courant and transient scope','',json.dumps(h.read(OUT/'COURANT.json'),sort_keys=True),'','No URANS solve has been performed. No startup discard, observation contract, stationary statistics, frequency in Hz, Strouhal number or cycle count is qualified. Solver-iteration spectral periods must not be interpreted as physical time.','']
    for name,folder in [('W1',OUT),('R1',OUT/'R1')]:lines+=['',name+' resources','```',(folder/'pilot.resources').read_text(),'```']
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')



def finalize():
    import capsule_pressure_diagnostic as prev
    m=h.read(OUT/'MEASUREMENTS.json');r=h.read(OUT/'R1/MEASUREMENTS.json')
    for folder,measurement in [(OUT,m),(OUT/'R1',r)]:
        h.verify_record(measurement)
        assert h.file_hash(folder/'HISTORY.json')==measurement['history']
        assert h.file_hash(folder/'pilot.log')==measurement['log']
        assert not measurement['W1']['summary']['bounds'] and not measurement['W1']['summary']['maxiter']
        assert all(f['nonfinite']==0 for c in measurement['checkpoints'].values() for f in c['fields'].values())
        for n in ('NETGEN_AUDIT.json','OUTER_QUALITY.json','OPTIMIZATION.json'):shutil.copyfile(folder/'W1'/n,folder/n)
    verify_candidate_prisms()
    require_same_prisms(np.load(BASE/'assembly/coordinates.npy'),np.load(BASE/'assembly/prisms.npy'),np.load(OUT/'R1/assembly/coordinates.npy'),np.load(OUT/'R1/assembly/prisms.npy'),202740)
    pre=h.read(OUT/'PREFLIGHT.json');status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=lambda ss:[s for s in ss if 'capsule_wake_diagnosis' not in s and 'wake_baseline_tmp' not in s]
    assert unrelated(status)==unrelated(pre['status']) and len(unrelated(status))==155
    baseline=h.read(prev.c.OUT/'MEASUREMENTS.json');h.verify_record(baseline)
    assert h.file_hash(prev.c.OUT/'HISTORY.json')==baseline['history_hash']
    prior=h.read(prev.d.OUT/'RESULT.json');h.verify_record(prior)
    for name,dim in [('U',3),('p',1),('k',1),('omega',1),('nut',1)]:
        a=np.concatenate([prev.l.u.e.read_internal(prev.c.OUT/'pilot'/f'processor{i}'/'500'/name,dim) for i in range(6)])
        assert hashlib.sha256(a.astype('<f8').tobytes()).hexdigest()==prior['terminal_verified'][name]
    classification='V4CapsuleCFDCoreRealizationChangesSteadyAzimuthWhileOffAxisLoadingPersists'
    conclusion='The baseline wake is sparsely represented, and targeted refinement changes the sampled downstream deficit. It does not remove drifting lateral loads. Same-sizing R1 changes the selected azimuth and late load statistics while both refined meshes remain healthy. This supports realization-sensitive steady trajectories, not a uniquely physical lateral direction or a fixed mesh-axis explanation. Geometry validity remains intact. Core resolution and realization are material numerical influences; physical unsteadiness and mesh independence remain unproved.'
    next_task='Run a bounded physical-time URANS comparison on W1 and R1, with a prospectively frozen Courant/inner-convergence and observation contract derived from the retained phi-rate evidence; compare time-mean loads and wake fields before interpreting any physical frequency.'
    decisions=dict(second_refinement_level='Not run. The controlled same-sizing realization is already a material source of steady-trajectory variation; more unconverged steady levels would not establish solution convergence. Near-separation resolution is not independently qualified.',independent_realization='R1 completed; same sizing, exact inner and outer boundaries; seed2 versus seed1. Partition rule unchanged but cell membership changes.',domain='Not run. Outer surface arrays remain identical across all comparisons. No isolated boundary-effect evidence justifies mixing a domain change into this result; domain independence remains open.',URANS='Not run. The principal cause of selected steady azimuth has been narrowed to numerical realization sensitivity. Physical-time stationarity is a separate major question. Measured Co rate implies about27049 steps for10D/U at Co0.2, before an inner-convergence or time-step study. No short under-resolved-in-time run is substituted.',promotion='No production-core promotion. W1 and R1 are separate mesh-gate-passed diagnostic candidates. Baseline authority remains unchanged.')
    storage=dict(start_free=pre['free_bytes'],end_free=shutil.disk_usage(OUT).free,retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),reclaimed_bytes=0,cleanup='Automatic approval review rejected task-created intermediate-directory deletion as blocked by policy. No deletion occurred; no alternative attempted.')
    linux='/mnt/c/'+str(OUT.resolve())[3:].replace('\\','/')
    storage['wsl_du_physical']=subprocess.check_output(['wsl','du','-s','-B1',linux],text=True).strip()
    notes=dict(epoch='Original legacy prism open required current access. Existing qualified epoch recovery reused; no new epoch renewal. Original access checks and CommittedState guard retained.',tooling_failures=['Initial WSL import required psutil through a heavy prior helper; replaced with lightweight task-local authority helpers, no tool installation.','Legacy access adapter attempts rejected stale access/guard mismatch before geometry audit; existing qualified recovery used with unchanged CommittedState guards.','First CFD launch ended in Foundation bashrc under set -e before any solve. Strict shell failure handling moved after environment setup.'],cleanup=storage['cleanup'])
    h.durable_json(OUT/'EXECUTION_NOTES.json',notes)
    report=OUT/'REPORT.md';report.write_text(report.read_text()+'\n## Decision\n\n'+classification+'\n\n'+conclusion+'\n\n'+json.dumps(decisions,indent=2)+'\n\nExactly one next task: '+next_task+'\n\nStorage: '+json.dumps(storage)+'\n',encoding='utf-8',newline='\n')
    record=h.seal(dict(schema='FrozenPrismWakeDiagnosisResult/1',classification=classification,starting_HEAD='2b4eafbea355222a93343c4c55980100447ca113',conclusion=conclusion,decisions=decisions,next_task=next_task,frozen=frozen(),prism_vertices_changed=0,prism_connectivity_changed=0,L11_vertices_changed=0,L11_topology_changed=0,baseline_mesh_authority='307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e',W1_candidate=h.read(OUT/'CANDIDATE.json')['identity'],R1_candidate=h.read(OUT/'R1/CANDIDATE.json')['identity'],comparison=h.read(OUT/'COMPARISON.json')['identity'],W1_measurements=m['identity'],R1_measurements=r['identity'],new_steady_iterations=1000,URANS_steps=0,Run_A=False,Run_A2=False,production_promotion=False,unrelated_dirty_entries=155,V3_modified=False,storage=storage,implementation=h.file_hash(__file__),tests=dict(regressions=h.file_hash(OUT/'TESTS.xml'),final_focused=h.file_hash(OUT/'TESTS_FINAL.xml')),evidence={n:h.file_hash(OUT/n) for n in ('REPORT.md','COMPARISON.json','BASELINE.json','GEOMETRY_EXTRA.json','COURANT.json','WAKE_CONTRACT.json','CFD_CONTRACT.json','R1/WAKE_CONTRACT.json','R1/CFD_CONTRACT.json','EXECUTION_NOTES.json')}))
    h.durable_json(OUT/'RESULT.json',record)
    print(json.dumps(record,indent=2))

if __name__=='__main__':
    if len(sys.argv)>2:
        assert sys.argv[2]=='R1';OUT=OUT/'R1'
    globals()[sys.argv[1]]()
