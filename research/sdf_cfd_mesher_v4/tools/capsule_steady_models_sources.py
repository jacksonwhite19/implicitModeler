"""Read-only installed Foundation source recovery; not model admission."""
import math,subprocess
import capsule_steady_models as s
h=s.h

def run():
    root=s.ROOT/'model_sources';root.mkdir(exist_ok=True)
    base='/opt/openfoam13/'
    rels=[]
    for model in ('kOmegaSST','kOmegaSSTLM','SpalartAllmaras'):
        for ext in ('C','H'):rels.append(f'src/MomentumTransportModels/momentumTransportModels/RAS/{model}/{model}.{ext}')
    rels+=['src/functionObjects/field/yPlus/yPlus.C','etc/caseDicts/functions/fields/yPlus']
    rels.extend('src/MomentumTransportModels/momentumTransportModels/Base/kOmegaSST/kOmegaSSTBase.'+ext for ext in ('C','H'))
    rels.extend('src/MomentumTransportModels/momentumTransportModels/derivedFvPatchFields/wallFunctions/nutWallFunctions/nutUSpaldingWallFunction/nutUSpaldingWallFunctionFvPatchScalarField.'+ext for ext in ('C','H'))
    for case,fields in [('T3A',('U','p','k','omega','nut','gammaInt','ReThetat')),('airFoil2D',('U','p','nuTilda','nut'))]:
        rels.extend(f'tutorials/incompressibleFluid/{case}/0/{field}' for field in fields)
        rels.extend(f'tutorials/incompressibleFluid/{case}/system/{file}' for file in ('fvSchemes','fvSolution','controlDict'))
        rels.append(f'tutorials/incompressibleFluid/{case}/constant/momentumTransport')
    files={}
    for rel in rels:
        data=subprocess.check_output(['wsl','cat',base+rel]);dest=root/rel;dest.parent.mkdir(parents=True,exist_ok=True)
        if dest.exists():assert dest.read_bytes()==data
        else:dest.write_bytes(data)
        files[base+rel]=h.file_hash(dest)
    nu=1.4607346938775508e-5;nut=1.4607346938775508e-8;Cv1=7.1
    def model(x):return x*(x/nu)**3/((x/nu)**3+Cv1**3)
    lo=0.;hi=nu
    while model(hi)<nut:hi*=2
    while math.nextafter(lo,math.inf)<hi:
        mid=(lo+hi)/2
        if model(mid)<nut:lo=mid
        else:hi=mid
    val=min((lo,hi),key=lambda x:abs(model(x)-nut))
    Tu=100*math.sqrt((2/3)*3.060969551020408e-5)/33.528;effective=max(Tu,.027)
    ret=1173.51-589.428*effective+.2196/effective**2
    result=h.seal(dict(schema='FoundationSteadyModelSourceRecovery/1',status='INPUT_DERIVATIONS_ONLY_WALL_AND_MESH_ADMISSION_PENDING',sources=files,SA=dict(equation='nut=nuTilda*chi^3/(chi^3+Cv1^3); chi=nuTilda/nu',Cv1=Cv1,nu=nu,nut_target=nut,nuTilda=val,nuTilda_hex=val.hex(),computed_nut=model(val),residual=model(val)-nut,bracket=[lo,hi],method='Monotone bisection to adjacent binary64 numbers; choose smaller absolute residual. No new physical input.'),SSTLM=dict(Tu_percent=Tu,implemented_Tu_floor_percent=.027,effective_Tu_percent=effective,uniform_dUsds=0,lambda_value=0,Flambda=1,ReThetat=ret,ReThetat_hex=ret.hex(),gammaInt=1,gamma_provenance='Installed steady T3A uniform and inlet gammaInt=1',ReThetat_provenance='Installed kOmegaSSTLM.C ReThetat0, zero-gradient uniform freestream branch, including model Tu floor; not the T3A case-specific number.'),model_constants_tuned=False,transient_runs=0))
    h.durable_json(s.ROOT/'MODEL_SOURCE_RECOVERY.json',result);print(result['identity']);print(result['SA']);print(result['SSTLM'])

if __name__=='__main__':run()
