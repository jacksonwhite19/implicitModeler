"""Seal reproducible partial-operator evidence and fail closed on near-null inference."""
import ctypes
import json
import re
import shutil
import subprocess
from pathlib import Path
import numpy as np
import capsule_sa_operator_directions as d
o=d.o;c=d.c;h=d.h;a=d.a


def direction(name,label,key,dim):
    ids=[c.sa.binary_labels(o.ROOT/'scratch'/name/f'processor{i}/constant/polyMesh/cellProcAddressing') for i in range(6)]
    parts=[np.fromfile(o.ROOT/'directions'/name/label/str(i)/(key+'.bin'),dtype='<f8').reshape(-1,dim) for i in range(6)]
    return a.canonical(parts,ids)


def derive():
    jac=h.read(o.ROOT/'JACOBIAN.json');result={}
    for name in c.NAMES:
        data=np.load(o.ROOT/(name+'_1000_operators.npz'));V=data['V'];C=data['C']
        baseline={};bits=a.load(name,1000,'patchBits')[:,0].astype(int)
        for eq,dim in [('RU0',3),('RN0',1),('RC0',1)]:
            baseline[eq]={p:a.stats(data[eq],V,(bits&(1<<j))!=0,C) for j,p in enumerate(('xmin','xmax','ymin','ymax','zmin','zmax'))}
        stiffness={}
        for label in ('U_rotation','U_streamwise','U_lateral_strain','U_late_change','SA_rotation','SA_amplitude','SA_late_change','rotation_augmented'):
            for mode in (('fixed','velocity') if label=='rotation_augmented' else ('fixed',)):
                tag=label+'_'+mode;stiffness[tag]={}
                for field,dim,eq in [('dU',3,'JU'),('dN',1,'JN')]:
                    g=direction(name,label,field,dim)
                    if d.rms(g,V)==0:continue
                    j=np.load(o.ROOT/(name+'_'+tag+'_'+eq+'.npz'))['values']
                    stiffness[tag][eq]=dict(directional_stiffness=d.inner(g,j,V)/d.inner(g,g,V),
                        direction_response_cosine=d.inner(g,j,V)/(d.rms(g,V)*d.rms(j,V)),units='1/s; not an eigenvalue')
        eq=a.load(name,1000,'RU_equilibriumNut',3)
        constant=a.load(name,1000,'constantAction')
        rejected=a.load(name,1000,'constantResidualAPI')/V[:,None]
        result[name]=dict(patch_baseline=baseline,stiffness=stiffness,
            equilibrium_nut_momentum=a.stats(eq,V,centers=C),equilibrium_change=a.stats(eq-data['RU0'],V,centers=C),
            constant_matrix_action=a.stats(constant,V),rejected_constant_residual_API=a.stats(rejected,V))
    # Independent process replay of the full six-epsilon augmented velocity path.
    name=c.NAMES[0];matches={}
    for rank in range(6):
        for eq in ('JU','JN','JC'):
            for i in range(6):
                rel=f'{rank}/{eq}{i}.bin'
                x=o.ROOT/'outputs'/name/'rotation_augmented_velocity'/rel
                y=o.ROOT/'outputs'/name/'rotation_augmented_velocity_replay'/rel
                assert h.file_hash(x)==h.file_hash(y),rel
                matches[rel]=h.file_hash(x)
    result['independent_process_replay']=dict(exact=True,arrays=len(matches),hashes=matches)
    result['classification']=a.evidence_status(False,False,False)
    result['operator_status']={
        'momentum':'ExactMomentumOperatorEvaluatedWithRetainedAuxiliaryPhiAndNut',
        'continuity':'ExactContinuityOperatorEvaluated',
        'SA':'ExactSATransportOperatorEvaluatedWithRetainedConservativePhi',
        'constitutive':'RetainedNutComparedWithInstalledEquilibriumRelation; momentum also evaluated after native correctNut',
        'pressure':'PressureCorrectionOperatorNotEvaluated',
        'directional':'DiscreteJacobianDirectionalActionEvaluatedForDeclaredAugmentedPathsOnly',
        'full_rotational_jacobian':'NotQualified: generator disagreement; no conservative boundary-consistent rotational flux tangent; physical nut boundary held fixed',
        'azimuth_scan':'NotEvaluated: a scan of an unqualified remapping would mix reconstruction error with pinning'}
    result['next_task']='Qualify a conservative, boundary-consistent rotational tangent for the retained augmented SA state, then repeat the read-only directional operator test.'
    result['no_convergence_rule']=True;result['no_stopping_contract']=True;result['no_CFD_qualification']=True
    h.durable_json(o.ROOT/'DERIVED.json',h.seal(result))


def allocation():
    from ctypes import wintypes
    class StandardInfo(ctypes.Structure):
        _fields_=[('allocation',ctypes.c_longlong),('length',ctypes.c_longlong),('links',wintypes.DWORD),('delete_pending',wintypes.BOOLEAN),('directory',wintypes.BOOLEAN)]
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE]
    kernel.CreateFileW.restype=wintypes.HANDLE
    kernel.GetFileInformationByHandleEx.argtypes=[wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD]
    kernel.CloseHandle.argtypes=[wintypes.HANDLE]
    logical=physical=0;count=0
    for p in o.ROOT.rglob('*'):
        if not p.is_file():continue
        count+=1;logical+=p.stat().st_size
        handle=kernel.CreateFileW(str(p.resolve()),0x80,7,None,3,0,None)
        if handle==ctypes.c_void_p(-1).value:raise ctypes.WinError(ctypes.get_last_error())
        info=StandardInfo()
        try:
            if not kernel.GetFileInformationByHandleEx(handle,1,ctypes.byref(info),ctypes.sizeof(info)):raise ctypes.WinError(ctypes.get_last_error())
            physical+=info.allocation
        finally:kernel.CloseHandle(handle)
    return dict(files=count,logical_bytes=logical,physical_bytes=physical,allocation_method='Windows FILE_STANDARD_INFO.AllocationSize',free_bytes=shutil.disk_usage(o.ROOT).free,reclaimed_bytes=0)


def verify():
    audit=o.verify()
    scratch=h.read(o.ROOT/'SCRATCH_INPUTS.json')
    for rel,value in scratch['files'].items():assert h.file_hash(o.ROOT/'scratch'/rel)==value,rel
    current={str(p.relative_to(o.ROOT/'scratch')) for p in (o.ROOT/'scratch').rglob('*') if p.is_file()}
    assert current==set(scratch['files']),current-set(scratch['files'])
    src=Path(__file__).parent/'sa_operator/saReadOnlyOperator.C'
    assert h.file_hash(src)==h.file_hash(o.ROOT/'build/saReadOnlyOperator.C')
    code=re.sub(r'//[^\n]*','',src.read_text())
    assert not re.search(r'\bsolve\s*\(|runTime\.(run|loop|write)\s*\(|\+\+runTime|runTime\+\+',code)
    native=dict(source=h.file_hash(src),binary=h.file_hash(o.ROOT/'build/bin/saReadOnlyOperator'),no_solver_call=True,no_time_increment=True,no_field_writeback=True)
    prior=h.read(o.ROOT/'ADMISSION.json')
    for path,rec in prior['source_files'].items():
        assert subprocess.check_output(['wsl','sha256sum',path],text=True).split()[0]==rec['sha256']
    extra=h.read(o.ROOT/'EXTRA_SOURCE_PROVENANCE.json')
    for path,rec in extra.items():
        if path.startswith('/opt/'):
            assert subprocess.check_output(['wsl','sha256sum',path],text=True).split()[0]==rec['sha256']
    assert extra['published_binary']['sha256']==native['binary']
    audit.update(scratch_files_unchanged=len(current),native=native,storage=allocation(),new_CFD_iterations=0,field_mutations=0,mesh_changes=0,prism_changes=0,L11_changes=0)
    h.durable_json(o.ROOT/'FINAL_VERIFICATION.json',h.seal(audit))
    print('verified',len(current),'scratch files; all prior inputs unchanged',flush=True)


if __name__=='__main__':
    import sys
    {'derive':derive,'verify':verify}[sys.argv[1]]()
