"""Prospective SA controls. No run or admission is implied by template generation."""
import difflib
import shutil
import sys
import re
import numpy as np
import capsule_steady_models as s
h=s.h

def binary_labels(path):
    data=path.read_bytes()
    header=data[:data.index(b'}')]
    assert re.search(rb'format\s+binary\s*;',header)
    # Foundation labelList output omits arch; the attested executable is DPInt32.
    # If an arch entry is present it must agree; byte count/closing token also bind width.
    if re.search(rb'\barch\s',header):assert b'LSB;label=32;scalar=64' in header
    m=re.search(rb'\n(\d+)\s*\(',data);assert m
    n=int(m[1]);end=m.end()+4*n
    assert data[end:end+1]==b')'
    return np.frombuffer(data[m.end():end],dtype='<i4').astype(np.int64)


def mesh_source(mesh):
    assert mesh in ('GW1-S1','GW1-S2','GC2-S1','GC2-S2','GC2F-S1','GC2F-S2')
    return (s.OLD if mesh.startswith('GW1-') else s.ROOT)/mesh

def controls():
    source=h.read(s.ROOT/'MODEL_SOURCE_RECOVERY.json');h.verify_record(source)
    baseline=s.ROOT/'GC2-S1/pilot'
    contract=h.read(s.ROOT/'GC2-S1/CFD_CONTRACT.json');h.verify_record(contract)
    out={name:(baseline/name).read_bytes() for name in contract['controls'] if name not in ('0/k','0/omega')}
    for name,data in out.items():assert h.file_hash(baseline/name)==contract['controls'][name]
    nu=source['SA']['nuTilda']
    out['0/nuTilda']=(f'FoamFile {{ format ascii; class volScalarField; object nuTilda; }}\n'
        'dimensions [0 2 -1 0 0 0 0];\n'
        f'internalField uniform {nu!r};\n'
        'boundaryField\n{\nwall { type fixedValue; value uniform 0; }\n'
        f'farfield {{ type freestream; freestreamValue uniform {nu!r}; value uniform {nu!r}; }}\n'
        '}\n').encode()
    changes={}
    replacements={
        'constant/momentumTransport':[('model kOmegaSST;','model SpalartAllmaras;')],
        'system/fvSolution':[('"(U|k|omega)"','"(U|nuTilda)"'),('k 0.5; omega 0.5;','nuTilda 0.5;')],
        'system/fvSchemes':[('div(phi,k) bounded Gauss upwind;','div(phi,nuTilda) bounded Gauss upwind;'),(' div(phi,omega) bounded Gauss upwind;\n','')],
    }
    for name,pairs in replacements.items():
        before=out[name].decode();after=before
        for old,new in pairs:
            assert after.count(old)==1,(name,old)
            after=after.replace(old,new)
        out[name]=after.encode()
        changes[name]=list(difflib.unified_diff(before.splitlines(),after.splitlines(),fromfile='SST/'+name,tofile='SA/'+name,lineterm=''))
    return out,h.seal(dict(schema='ProspectiveSteadySAControls/1',status='NOT_ADMITTED_NOT_LAUNCHED',source=source['identity'],baseline=contract['identity'],nuTilda=nu,nuTilda_hex=nu.hex(),changes=changes,
        common_momentum_pressure_monitoring_unchanged=True,steady=True,horizon=500,
        provenance=dict(wall_nuTilda='Installed airFoil2D: fixedValue zero',farfield_nuTilda='Installed airFoil2D: scalar freestream; value from exact installed viscosity-relation inversion',nut='Unchanged Spalding wall function; also installed airFoil2D practice',scalar_controls='Retain the existing capsule smoothSolver/GaussSeidel, nSweeps1, tolerance1e-8, relTol0.1, maxIter5000 and scalar relaxation0.5 to avoid adding a relaxation experiment. Installed SA tutorial confirms the same solver/smoother mechanism but uses nSweeps2 and relaxation0.7; these unrelated changes are not imported.',convection='Retain existing bounded Gauss upwind scalar mechanism; change transported-field name only'),new_model_coefficients=False))

def prepare(mesh):
    selection=h.read(s.ROOT/'MESH_SELECTION.json');h.verify_record(selection)
    admission=h.read(s.ROOT/'SA_ADMISSION.json');h.verify_record(admission)
    assert selection['model_comparison_supported'] and mesh in selection['pair']
    assert admission['admitted'] and admission['selection']==selection['identity']
    source=mesh_source(mesh)
    candidate=h.read(source/'CANDIDATE.json');h.verify_record(candidate)
    assert candidate['frozen']==s.f.prior.frozen()
    template,prospective=controls();assert admission['controls']==prospective['identity']
    folder=s.ROOT/'model_runs'/('SA-'+mesh);folder.mkdir(parents=True,exist_ok=False)
    case=folder/'pilot'
    for name,data in template.items():
        path=case/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
        archive=folder/'controls'/name;archive.parent.mkdir(parents=True,exist_ok=True);archive.write_bytes(data)
    original=source/'pilot/constant/polyMesh';copy=case/'constant/polyMesh'
    shutil.copytree(original,copy)
    geometry={p.name:h.file_hash(p) for p in original.iterdir() if p.is_file()}
    assert all(h.file_hash(copy/name)==digest for name,digest in geometry.items())
    assert all(p.stat().st_nlink==1 for p in copy.iterdir() if p.is_file())
    contract=h.seal(dict(schema='SteadySAComparisonCase/1',model='SpalartAllmaras',mesh=mesh,candidate=candidate['identity'],selection=selection['identity'],admission=admission['identity'],prospective=prospective['identity'],controls={name:h.file_hash(case/name) for name in template},mesh_files=geometry,start=0,horizon=500,ranks=6,steady=True,qualifying=False,terminal_mapping=False,prism_changes=0))
    h.durable_json(folder/'CFD_CONTRACT.json',contract)
    (folder/'.gitignore').write_text('pilot/\n')
    linux='/mnt/c/'+str(folder.resolve())[3:].replace('\\','/')
    (folder/'run_cfd.sh').write_text(f'''source /opt/openfoam13/etc/bashrc
set -e
cd {linux}
sha256sum /opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun > EXECUTABLE.txt
checkMesh -constant -allTopology -allGeometry -case pilot > adapted_checkMesh.log 2>&1
grep -q 'Mesh OK.' adapted_checkMesh.log
decomposePar -case pilot > decompose.log 2>&1
PYTHONPATH=../../../../src:../../../../tools python3 ../../../../tools/capsule_steady_models_sa.py audit SA-{mesh} > INITIAL_AUDIT.log
set +e
/usr/bin/time -v -o pilot.resources mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot > pilot.log 2>&1
echo $? > pilot.exit
''',newline='\n')
    print(contract['identity'])

def audit(name):
    folder=s.ROOT/'model_runs'/name;case=folder/'pilot'
    con=h.read(folder/'CFD_CONTRACT.json');h.verify_record(con)
    source=mesh_source(con['mesh'])
    for rel,digest in con['controls'].items():assert h.file_hash(case/rel)==digest
    for rel,digest in con['mesh_files'].items():assert h.file_hash(case/'constant/polyMesh'/rel)==digest
    initial={'U':[0.,0.,33.528],'p':[0.],'nut':[1.4607346938775508e-8],'nuTilda':[h.read(s.ROOT/'MODEL_SOURCE_RECOVERY.json')['SA']['nuTilda']]}
    records={}
    for rank in range(6):
        proc=case/f'processor{rank}'
        ids=binary_labels(proc/'constant/polyMesh/cellProcAddressing')
        assert np.array_equal(ids,binary_labels(source/'pilot'/f'processor{rank}/constant/polyMesh/cellProcAddressing'))
        for field,expected in initial.items():
            path=proc/'0'/field;data=path.read_bytes()
            m=re.search(rb'internalField\s+uniform\s+([^;]+);',data);assert m
            actual=list(map(float,m[1].decode().strip('() ').split()));assert actual==expected
            records[f'processor{rank}/0/{field}']=h.file_hash(path)
    assert 'f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e' in (folder/'EXECUTABLE.txt').read_text()
    result=h.seal(dict(schema='FreshSteadySAInitialAudit/1',contract=con['identity'],decomposition_exact=True,initial_uniform_exact=True,files=records,passed=True))
    h.durable_json(folder/'INITIAL_AUDIT.json',result);print(result['identity'])

if __name__=='__main__':
    assert sys.argv[1] in ('prepare','audit');globals()[sys.argv[1]](sys.argv[2])
