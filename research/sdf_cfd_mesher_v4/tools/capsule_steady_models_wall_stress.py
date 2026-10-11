"""Native terminal wall stress and pressure diagnostics; zero CFD iterations."""
import re
import subprocess
import numpy as np
import capsule_steady_models as s
import capsule_steady_models_wall as w
h=s.h


def vector_patch(path,count):
    data=path.read_bytes();m=re.search(rb'\bwall\s*\{',data);assert m
    tail=data[m.end():];v=re.search(rb'\bvalue\s+',tail);assert v
    tail=tail[v.end():]
    u=re.match(rb'uniform\s*\(([^)]+)\)\s*;',tail)
    if u:return np.tile(np.fromstring(u[1].decode(),sep=' '),(count,1))
    a=re.match(rb'nonuniform\s+List<vector>\s+(\d+)\s*\(',tail);assert a and int(a[1])==count
    assert re.search(rb'format\s+binary\s*;',data[:m.start()])
    end=a.end()+24*count;assert tail[end:end+1]==b')'
    return np.frombuffer(tail[a.end():end],dtype='<f8').copy().reshape(count,3)


def run():
    source='/opt/openfoam13/src/functionObjects/field/wallShearStress/wallShearStress.C'
    snapshot=s.ROOT/'model_sources/wallShearStress.C'
    snapshot.write_bytes(subprocess.check_output(['wsl','cat',source]))
    registry=subprocess.check_output(['wsl','bash','/mnt/c/'+str((s.ROOT/'wall_registry.sh').resolve())[3:].replace('\\','/')],text=True)
    assert 'wallShearStress is in table' in registry and 'libfieldFunctionObjects.so' in registry
    results={}
    for model in ('SST','SA'):
        for mesh in ('GW1-S1','GW1-S2'):
            folder=s.ROOT/'wall_reference'/mesh if model=='SST' else s.ROOT/'model_runs'/('SA-'+mesh)
            if model=='SA':assert (folder/'pilot.exit').read_text().strip()=='0'
            case=folder/'pilot';before={str(p.relative_to(case)):h.file_hash(p) for rank in range(6) for p in (case/f'processor{rank}/500').rglob('*') if p.is_file() and p.name!='wallShearStress'}
            linux='/mnt/c/'+str(folder.resolve())[3:].replace('\\','/')
            script=folder/'wall_stress.sh'
            script.write_text('source /opt/openfoam13/etc/bashrc\nset -e\ncd '+linux+'\nmpirun -np 6 foamPostProcess -solver incompressibleFluid -parallel -case pilot -time 500 -func wallShearStress > wall_stress.log 2>&1\n',newline='\n')
            subprocess.run(['wsl','bash',linux+'/wall_stress.sh'],check=True)
            for rel,digest in before.items():assert h.file_hash(case/rel)==digest
            start,total=w.patch_layout(case/'constant/polyMesh/boundary');assert total==33786
            shear=np.full((total,3),np.nan);pressure=np.full(total,np.nan);seen=np.zeros(total,int)
            for rank in range(6):
                proc=case/f'processor{rank}';first,count=w.patch_layout(proc/'constant/polyMesh/boundary')
                ids=np.abs(w.labels(proc/'constant/polyMesh/faceProcAddressing')[first:first+count])-1-start
                assert np.all((ids>=0)&(ids<total));seen[ids]+=1
                shear[ids]=vector_patch(proc/'500/wallShearStress',count);pressure[ids]=w.pressure_patch(proc)
            assert np.all(seen==1) and np.isfinite(shear).all() and np.isfinite(pressure).all()
            geometry=np.load(s.ROOT/'wall_reference'/mesh/'WALL_YPLUS.npz');centers=geometry['centers'];areas=geometry['areas']
            normal=centers.copy();normal[:,2]-=np.clip(normal[:,2],-.04,.04);normal/=np.linalg.norm(normal,axis=1)[:,None]
            tangent=np.array([0.,0.,1.])-normal*normal[:,2,None];tangent/=np.linalg.norm(tangent,axis=1)[:,None]
            axial=np.sum(shear*tangent,axis=1);z=centers[:,2]
            mid=np.abs(z)<=.04;direction=np.sign(np.median(axial[mid]));assert direction!=0
            regions=dict(front_cap=z<-.04,mid_body=mid,rear_cap=z>.04)
            sector=np.floor((np.arctan2(centers[:,1],centers[:,0])+np.pi)/(2*np.pi)*8).astype(int)%8
            regions.update({f'rear_azimuth_{i}':(z>.04)&(sector==i) for i in range(8)})
            def stats(mask):
                return dict(faces=int(mask.sum()),Cp=s.f.statistics(2*pressure[mask]/33.528**2),shear_Pa=s.f.statistics(1.225*np.linalg.norm(shear[mask],axis=1)),meridional_shear=s.f.statistics(axial[mask]),opposite_midbody_sign_area_fraction=float(areas[mask&(axial*direction<0)].sum()/areas[mask].sum()))
            archive=folder/'WALL_STRESS.npz';np.savez_compressed(archive,centers=centers,areas=areas,stress=shear,p=pressure,meridional_shear=axial)
            results[model+'-'+mesh]=dict(global_statistics=stats(np.ones(total,bool)),regions={k:stats(v) for k,v in regions.items()},stress_array=h.array_hash(shear),pressure_array=h.array_hash(pressure),archive_sha256=h.file_hash(archive),preexisting_terminal_files=before,preexisting_terminal_unchanged=True,log=h.file_hash(folder/'wall_stress.log'),new_CFD_iterations=0)
    record=h.seal(dict(schema='NativeSteadyWallStressPressureComparison/1',source=source,source_sha256=h.file_hash(snapshot),registry=registry,runs=results,
        interpretation='Installed incompressible wallShearStress=-model.devSigma boundary value. Magnitude multiplied by rho gives Pa. Meridional direction is +Z projected onto exact capsule tangent. Opposite sign relative to midbody is a reverse-shear diagnostic, not a qualified separation line or transition location. Terminal iterative phase and mesh sensitivity remain.',qualifying=False))
    h.durable_json(s.ROOT/'WALL_STRESS_COMPARISON.json',record);print(record['identity'])


if __name__=='__main__':run()
