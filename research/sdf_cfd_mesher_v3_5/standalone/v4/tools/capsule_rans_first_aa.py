"""Case-specific Foundation 13 capsule RANS contract and isolated mesh adapter."""
from pathlib import Path
import hashlib,json,math,re,shutil,subprocess
import formalize_capsule_complete_mesh as m
OUT=m.RESULTS/'capsule_rans_first_aa'
U=33.528; NU=1.7894e-5/1.225; RHO=1.225; LENGTH=.16
OMEGA=10*U/LENGTH; NUT=.001*NU; K=OMEGA*NUT
AREA=math.pi*.04**2

def write(p,s):p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(s.encode())
def header(name,cls='dictionary'):
    return f'FoamFile {{ format ascii; class {cls}; object {name}; }}\n'
def field(name,dimensions,value,wall,far):
    return header(name,'volVectorField' if name=='U' else 'volScalarField')+f'dimensions [{dimensions}];\ninternalField uniform {value};\nboundaryField\n{{\nwall {{ {wall} }}\nfarfield {{ {far} }}\n}}\n'

def controls():
    c={}
    c['0/U']=field('U','0 1 -1 0 0 0 0','(0 0 33.528)','type noSlip;','type freestreamVelocity; freestreamValue uniform (0 0 33.528); value uniform (0 0 33.528);')
    c['0/p']=field('p','0 2 -2 0 0 0 0','0','type zeroGradient;','type freestreamPressure; freestreamValue uniform 0; value uniform 0;')
    for name,val,dim,bc in [('k',K,'0 2 -2 0 0 0 0','kLowReWallFunction'),('omega',OMEGA,'0 0 -1 0 0 0 0','omegaWallFunction')]:
        extra='blended true;' if name=='omega' else ''
        c['0/'+name]=field(name,dim,repr(val),f'type {bc}; {extra} value uniform {val!r};',f'type freestream; freestreamValue uniform {val!r}; value uniform {val!r};')
    c['0/nut']=field('nut','0 2 -1 0 0 0 0',repr(NUT),'type nutUSpaldingWallFunction; value uniform 0;',f'type calculated; value uniform {NUT!r};')
    c['constant/physicalProperties']=header('physicalProperties')+f'viscosityModel constant;\nrho [1 -3 0 0 0 0 0] {RHO};\nnu [0 2 -1 0 0 0 0] {NU!r};\n'
    c['constant/momentumTransport']=header('momentumTransport')+'simulationType RAS;\nRAS { model kOmegaSST; turbulence on; printCoeffs on; }\n'
    c['system/fvSchemes']=header('fvSchemes')+'''ddtSchemes { default steadyState; }
gradSchemes { default Gauss linear; grad(U) cellLimited Gauss linear 1; }
divSchemes
{
 default none;
 div(phi,U) bounded Gauss linearUpwindV grad(U);
 div(phi,k) bounded Gauss linearUpwind grad(k);
 div(phi,omega) bounded Gauss linearUpwind grad(omega);
 div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear corrected; }
interpolationSchemes { default linear; }
snGradSchemes { default corrected; }
fluxRequired { default no; p; }
wallDist { method meshWave; }
'''
    c['system/fvSolution']=header('fvSolution')+'''solvers
{
 p { solver PCG; preconditioner DIC; tolerance 1e-13; relTol 0; maxIter 5000; }
 "(U|k|omega)" { solver smoothSolver; smoother GaussSeidel; tolerance 1e-8; relTol 0.1; nSweeps 1; maxIter 5000; }
}
SIMPLE { nNonOrthogonalCorrectors 0; consistent yes; momentumPredictor yes; }
relaxationFactors { equations { U 0.9; k 0.5; omega 0.5; } }
cache { grad(U); }
'''
    monitors='''forces
{
 type forces; libs ("libforces.so"); patches (wall);
 rho rhoInf; rhoInf 1.225; CofR (0 0 0); pRef 0;
 writeControl timeStep; writeInterval 1; writePrecision 17; log true;
}
fieldExtrema
{
 type fieldMinMax; libs ("libfieldFunctionObjects.so"); fields (U p k omega nut);
 writeControl timeStep; writeInterval 1; writePrecision 17; log true;
}
'''
    for patch in ('wall','farfield'):
        for op in ('sum','sumMag'):
            monitors+=f'''{patch}_{op}
{{
 type surfaceFieldValue; libs ("libfieldFunctionObjects.so");
 patch {patch}; operation {op}; fields (phi); writeFields false;
 writeControl timeStep; writeInterval 1; writePrecision 17; log true;
}}
'''
    c['system/controlDict']=header('controlDict')+'''application foamRun;
solver incompressibleFluid;
startFrom startTime; startTime 0; stopAt endTime; endTime 500; deltaT 1;
writeControl timeStep; writeInterval 500; purgeWrite 0;
writeFormat binary; writePrecision 17; writeCompression off;
timeFormat general; timePrecision 10; runTimeModifiable false;
functions
{
'''+monitors+'}\n'
    c['system/decomposeParDict']=header('decomposeParDict')+'numberOfSubdomains 6;\ndecomposer hierarchical;\ndistributor ptscotch;\nhierarchicalCoeffs { n (3 2 1); order xyz; }\n'
    return c

def adapted_boundary(original):
    match=re.search(r'(\bwall\s*\{)([^}]*)(\})',original)
    if not match:raise ValueError('MissingWall')
    block=match[2]
    if not re.search(r'\btype\s+patch;',block):raise ValueError('UnexpectedCanonicalWallType')
    block=re.sub(r'(\btype\s+)patch;',r'\1wall;',block)
    block=re.sub(r'(\bphysicalType\s+)patch;',r'\1wall;',block)
    return original[:match.start(2)]+block+original[match.end(2):]

def prepare():
    OUT.mkdir(exist_ok=True)
    h=m.h;tick=h.measured()
    h.eq(subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'d842be0c91310501dfb846fb477a88b6cfe932b8','WrongHEAD')
    h.eq(subprocess.check_output(['git','branch','--show-current'],text=True).strip(),'feature/sdf-cfd-mesher-v4-advancing-front','WrongBranch')
    a=h.read(m.OUT/'AUTHORITY.json');m.validate(a,h.read(m.OUT/'CFD_HANDOFF.json'),a['identity'])
    r=h.read(m.OUT/'ACCESS.json')
    with m.Store(r['directory']).open(r['root']):pass
    old=h.read(m.RESULTS/'capsule_cfd_first_aa_admission/AUDIT.json');h.verify_record(old)
    h.eq(old['identity'],'a330a6ae94e4c00766c05ffc52db43b777dff4f27b03a4ca06c28c19b9e146e0','WrongPriorAudit')
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    status=[s for s in status if 'capsule_rans_first_aa' not in s]
    h.eq(len(status),155,'WrongDirtyCount')
    h.durable_json(OUT/'PREFLIGHT.json',dict(status=status,free_bytes=shutil.disk_usage(V4 if False else m.V4).free,epoch_renewal=False))
    write(OUT/'.gitignore','A/\nA2/\n')
    for name,text in controls().items():write(OUT/'controls'/name,text)
    for run in ('A',):
        case=OUT/run
        if case.exists():raise ValueError('CaseAlreadyExists')
        shutil.copytree(OUT/'controls',case)
        pm=case/'constant/polyMesh';shutil.copytree(m.CORE/'foundation/openfoam_case/constant/polyMesh',pm)
        canonical=(pm/'boundary').read_text();write(pm/'boundary',adapted_boundary(canonical))
        exact={n:h.file_hash(pm/n)==v for n,v in a['openfoam']['members'].items() if n!='boundary'}
        h.need(all(exact.values()),'GeometryChanged')
        adapter=h.seal(dict(schema='V4CapsuleCaseWallAdapter/1',canonical_authority=a['identity'],unchanged=exact,
            original_boundary=a['openfoam']['members']['boundary'],adapted_boundary=h.file_hash(pm/'boundary'),
            patch_counts=[33786,5182],start_faces=[1290405,1324191],only_changes=['wall.type patch->wall','wall.physicalType patch->wall'],hardlinks=0))
        h.durable_json(OUT/'MESH_ADAPTER.json',adapter)
    contract=h.seal(dict(schema='V4CapsuleRANSCaseContract/1',status='FrozenBeforeRunA_NotYetNumericallyQualified',
        mesh=a['identity'],handoff=h.read(m.OUT/'CFD_HANDOFF.json')['identity'],adapter=adapter['identity'],
        physical=dict(rho=RHO,mu=1.7894e-5,nu=NU,Uinf=[0,0,U],Tref=288.15,radius=.04,length=LENGTH,Aref=AREA),
        turbulence=dict(model='kOmegaSST',omega=OMEGA,k=K,nut=NUT,omega_hex=OMEGA.hex(),k_hex=K.hex(),
            viscosity_ratio=.001,omega_factor=10,Lref=LENGTH,intensity=math.sqrt(2*K/3)/U,
            equivalent_Lt=math.sqrt(K)/(.09**.25*OMEGA),
            formula='omega=10*U/Lref; nut=0.001*nu; k=omega*nut; I=sqrt(2*k/3)/U',
            provenance='NASA Wind-US SST recommended freestream convention, https://www.grc.nasa.gov/www/winddocs/windus3.0/user/keywords/turbulence.html; explicit case-specific engineering adoption, not measured turbulence or universal default'),
        stopping=dict(maximum_iterations=500,residual_max=1e-5,window=50,consecutive_disjoint_windows=2,
            force_component_range_normalized_max=1e-5,moment_component_range_normalized_max=1e-5,
            force_normalization='.5*rho*Uinf^2*Aref',moment_normalization='.5*rho*Uinf^2*Aref*length',
            flux_normalization='max(total absolute inflow,total absolute outflow)',normalized_flux_max=1e-10,
            wall_absolute_flux_max=1e-12,linear_max_iter=5000,nonfinite_allowed=0,
            provenance='New prospectively frozen capsule budget: 1e-5 equation residual from airFoil2D; same 1e-5 nondimensional accuracy budget for force and moment ranges; 50-step observation windows from airFoil2D write cadence, two complete disjoint windows (100 iter) to exclude a lucky state; 500 cap from motorBikeSteady. Flux gates reused only as conservation identities on solver phi, independent of PDE; no Phi residual gates imported.'),
        pressure_reference='freestreamPressure mixed patch fixesValue() true; outlet-weighted zero pressure; no pRefCell necessary',
        provenance=dict(algorithm='B: motorBikeSteady steadyState SIMPLE consistent yes, relaxation U .9 k/omega .5, zero nonorthogonal correctors',
            pressure='A plus user authorization: PCG/DIC 1e-13,0,5000; pressure Laplacian symmetric; corrected nonorthogonal contribution explicit',
            transport='B: motorBikeSteady smoothSolver GaussSeidel 1e-8 relTol .1 one sweep',
            schemes='B: motorBikeSteady linearUpwindV U; airFoil2D linearUpwind turbulence adaptation replaces tutorial upwind before launch, no first-order downgrade',
            boundaries='B: airFoil2D freestream U/p; generic freestream scalar; installed wall profiles plus project selection',
            dimensional_inputs='C: user physics, NASA turbulence relations, direct capsule Lref and frontal area',
            observables='C: prospectively chosen shared nondimensional 1e-5 iteration-error budget; no measured trajectory used to choose thresholds'),
        controls={n:h.file_hash(OUT/'controls'/n) for n in controls()},execution='six fixed hierarchical ranks, no potential initialization; original initial fields independently for A/A2'))
    h.durable_json(OUT/'CONTRACT.json',contract)
    print(json.dumps(dict(contract=contract['identity'],adapter=adapter['identity'],k=K,omega=OMEGA)))

if __name__=='__main__':prepare()
