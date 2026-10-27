"""Bounded standard constrained tet generation on measured failing cavities."""
import sys,time,resource,traceback
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import gmsh
from fixed_core_study import *
from fixed_core_topology_repair import boundary,orient,invariant,patch_interface,better
from broader_geometry_core import extract_all_volumes
from repair_bulk_core import load,V3
from constrained_netgen_adapter import load_volume_classified

def run(src,interface,dest,seed_rank=0):
    start=time.perf_counter();dest.mkdir(parents=True,exist_ok=False);x,t,ci,iface=load_core(src,interface);q,m=evaluate(x,t,ci,iface)
    old=load('local_cavity_gmsh',V3/'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py')
    seeds=np.unique([np.argmin(m['determinant']),m['interface_owner'][np.argmin(m['interface_weight'])],components(m['bad'],m['owners'])[0][0]])
    trials=[]
    for seed in seeds[seed_rank:seed_rank+1]:
        if time.perf_counter()-start>105:break
        region={int(seed)}
        for depth in range(2):
            region.update(map(int,m['owners'][np.any(np.isin(m['owners'],list(region)),axis=1)].ravel()))
        patch,pci=patch_interface(x,t,region,iface,ci,m);ids=sorted(region);before,_=evaluate(x,t[ids],pci,patch);bf=boundary(t[ids]);vs=np.unique(bf)
        row=dict(seed=int(seed),removed_tets=len(ids),fixed_vertices=len(vs),boundary_faces=len(bf),before=before['failures']);tick=time.perf_counter()
        gmsh.initialize([],False)
        try:
            old.setup_gmsh(gmsh,1);gmsh.option.setNumber('General.Verbosity',2);gmsh.model.add('local_cavity');gmsh.model.addDiscreteEntity(2,1)
            gmsh.model.mesh.addNodes(2,1,(vs+1).tolist(),x[vs].ravel().tolist());gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(bf)+1).tolist(),(bf+1).ravel().tolist())
            gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True);gmsh.model.addDiscreteEntity(3,1,[1]);gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
            # Existing tet barycentre is inside the old cavity. An embedded
            # discrete point asks the standard constrained mesher to retain it.
            point=x[t[max(region,key=lambda j:m['volume'][j])]].mean(0)
            gmsh.model.addDiscreteEntity(0,2);gmsh.model.mesh.addNodes(0,2,[len(x)+1],point.tolist());gmsh.model.mesh.embed(0,[2],3,1)
            h=float(np.median(np.linalg.norm(x[bf]-x[np.roll(bf,1,axis=1)],axis=2)))
            gmsh.option.setNumber('Mesh.MeshSizeMin',h);gmsh.option.setNumber('Mesh.MeshSizeMax',h);gmsh.model.mesh.generate(3)
            # Netgen's Gmsh adapter omits embedded-entity vertices. Reclassify
            # valid generated interior nodes onto the volume before importing.
            generated=extract_all_volumes(gmsh,old);gx=generated['coordinates'];gt=orient(gx,generated['tets'])
            coord={p.tobytes():i for i,p in enumerate(gx)};allpoints=list(gx)
            for v in np.unique(t[ids]):
                if x[v].tobytes() not in coord:coord[x[v].tobytes()]=len(allpoints);allpoints.append(x[v])
            gx=np.array(allpoints)
            gb=np.array([[coord[x[v].tobytes()] for v in face] for face in bf])
            from constrained_cavity import invariants
            oldmap=np.array([[coord[x[v].tobytes()] for v in tet] for tet in t[ids]])
            if not invariants(gx,oldmap,gt)['passed']:raise ValueError('GeneratedCavityInvariantFailed')
            load_volume_classified(gmsh,gx,gt,gb);gmsh.model.mesh.optimize('Netgen',force=True)
            arrays=extract_all_volumes(gmsh,old);xx=arrays['coordinates'];tt=arrays['tets'];lookup={p.tobytes():int(j) for j,p in enumerate(x)}
            np.savez_compressed(dest/'RAW_PROPOSAL.npz',**arrays)
            mapping=[];added=[]
            for p in xx:
                key=p.tobytes()
                if key not in lookup:lookup[key]=len(x)+len(added);added.append(p)
                mapping.append(lookup[key])
            candidate_x=np.vstack((x,np.array(added).reshape(-1,3)));new=orient(candidate_x,np.array(mapping)[tt]);row.update(raw_nodes=len(xx),unique_coordinate_nodes=len(set(p.tobytes() for p in xx)),raw_tets=len(tt),duplicate_tets=len(new)-len(np.unique(np.sort(new,axis=1),axis=0)))
            # Standard mesher output can contain repeated identical cells in
            # a thin cavity. Preserve raw evidence, remove exact duplicates,
            # then still require all independent cavity invariants and gates.
            _,unique=np.unique(np.sort(new,axis=1),axis=0,return_index=True);new=new[np.sort(unique)]
            same=invariant(candidate_x,t[ids],new)
            row.update(replacement_tets=len(new),inserted_points=len(added),exact_cavity_invariant=same)
            if same:
                after,_=evaluate(candidate_x,new,pci,patch);row['after']=after['failures'];row['accepted']=better(before,after)
                if row['accepted']:
                    candidate_t=np.vstack((np.delete(t,ids,axis=0),new));qq,mm=evaluate(candidate_x,candidate_t,ci,iface)
                    row['global_quality']=qq
                    # Save independently; do not let a changed index silently
                    # invalidate the remaining representative seed inventory.
                    out=dest/str(int(seed));out.mkdir()
                    tags=np.arange(1,len(candidate_x)+1)
                    for k,z in dict(coordinates=candidate_x,tets=candidate_t,node_tags=tags,interface_tags=tags[ci],interface_owner=mm['interface_owner'],farfield_tags=np.empty(0,dtype='i8')).items():np.save(out/(k+'.npy'),z)
                    save(out/'QUALITY.json',qq)
            else:row['accepted']=False
        except Exception as e:row.update(error=str(e),traceback=traceback.format_exc(),accepted=False)
        finally:gmsh.finalize()
        row['seconds']=time.perf_counter()-tick;trials.append(row);save(dest/'TRIALS.json',trials);print(json.dumps(row),flush=True)
    save(dest/'RESULT.json',dict(seconds=time.perf_counter()-start,peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,trials=len(trials),accepted=sum(z['accepted'] for z in trials)))
if __name__=='__main__':run(*map(Path,sys.argv[1:4]),int(sys.argv[4]) if len(sys.argv)>4 else 0)
