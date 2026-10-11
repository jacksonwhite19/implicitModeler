"""Final evidence closure for the bounded family; never promotes production mesh."""
import ctypes
import json
import shutil
import subprocess
from pathlib import Path
import capsule_broad_global_family as f
h=f.h
TASK_NAMES=('capsule_broad_global','capsule_family_resources','capsule_family_health','capsule_family_seal','capsule_family_wake','capsule_family_decision','capsule_family_cleanup','broad_family_prepare.tmp')


def allocation(root):
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    class StandardInfo(ctypes.Structure):
        _fields_=[('allocation',ctypes.c_int64),('length',ctypes.c_int64),('links',ctypes.c_uint32),('delete_pending',ctypes.c_ubyte),('directory',ctypes.c_ubyte)]
    create=kernel.CreateFileW
    create.argtypes=[ctypes.c_wchar_p,ctypes.c_uint32,ctypes.c_uint32,ctypes.c_void_p,ctypes.c_uint32,ctypes.c_uint32,ctypes.c_void_p]
    create.restype=ctypes.c_void_p
    query=kernel.GetFileInformationByHandleEx
    query.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_uint32]
    query.restype=ctypes.c_int
    close=kernel.CloseHandle;close.argtypes=[ctypes.c_void_p]
    logical=physical=count=0
    for path in root.rglob('*'):
        if not path.is_file():continue
        handle=create(str(path.resolve()),0x80,7,None,3,0,None)
        if handle==ctypes.c_void_p(-1).value:raise ctypes.WinError(ctypes.get_last_error())
        try:
            info=StandardInfo()
            if not query(handle,1,ctypes.byref(info),ctypes.sizeof(info)):raise ctypes.WinError(ctypes.get_last_error())
            assert info.links==1,'Task must not create scientific hardlinks'
            logical+=info.length;physical+=info.allocation;count+=1
        finally:close(handle)
    return dict(files=count,logical_bytes=logical,physical_bytes=physical)


def storage(label):
    r=dict(free_bytes=shutil.disk_usage(f.ROOT).free,total=allocation(f.ROOT),variants={n:allocation(f.ROOT/n) for n in ('C2','C2-R','G1')},method='NTFS GetFileInformationByHandleEx FileStandardInfo AllocationSize for every regular file; NumberOfLinks=1 required.')
    h.durable_json(f.ROOT/('STORAGE_'+label+'.json'),r)
    print(json.dumps(r,indent=2))


def seal():
    pre=h.read(f.ROOT/'PREFLIGHT.json');decision=h.read(f.ROOT/'DECISION.json')
    h.verify_record(decision)
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==pre['head']
    comparison=h.read(f.ROOT/'COMPARISON.json');h.verify_record(comparison)
    dense=h.read(f.ROOT/'DENSE_WAKE.json');h.verify_record(dense)
    for name,v in dense['variants'].items():assert h.file_hash(f.ROOT/'wake'/(name+'.npz'))==v['archive_sha256']
    assert f.prior.frozen()==pre['frozen']
    evidence={};mesh_recheck={}
    for name in ('C2','C2-R','G1'):
        folder=f.ROOT/name
        candidate=h.read(folder/'CANDIDATE.json');h.verify_record(candidate)
        for rel,digest in candidate['evidence'].items():assert h.file_hash(folder/rel)==digest
        m=h.read(folder/'MEASUREMENTS.json');h.verify_record(m)
        assert h.file_hash(folder/'HISTORY.json')==m['history']
        assert h.file_hash(folder/'pilot.log')==m['log']
        assert int((folder/'pilot.exit').read_text())==0
        con=h.read(folder/'CFD_CONTRACT.json');h.verify_record(con)
        assert con['start']==0 and con['horizon']==500
        for rel,digest in con['controls'].items():assert h.file_hash(folder/'pilot'/rel)==digest
        source=folder/'foundation/openfoam_case/constant/polyMesh';mesh=folder/'pilot/constant/polyMesh'
        members={member:h.file_hash(source/member) for member in ('points','faces','owner','neighbour')}
        for member,digest in members.items():assert h.file_hash(mesh/member)==digest
        assert (source/'boundary').read_text().replace(con['adapter']['before'],con['adapter']['after'])==(mesh/'boundary').read_text()
        mesh_recheck[name]=dict(unchanged_geometry_members=members,boundary_sha256=h.file_hash(mesh/'boundary'),only_authorized_wall_metadata_changed=True)
        assert not m['W1']['summary']['maxiter']
        assert all(v['nonfinite']==0 for cp in m['checkpoints'].values() for v in cp['fields'].values())
        for rel in ('CONTRACT.json','CANDIDATE.json','CFD_CONTRACT.json','MEASUREMENTS.json','HISTORY.json','pilot.log','pilot.resources','adapted_checkMesh.log','EXECUTABLE.txt'):
            evidence[name+'/'+rel]=h.file_hash(folder/rel)
    current=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    unrelated=lambda rows:[r for r in rows if not any(k in r for k in TASK_NAMES)]
    assert unrelated(current)==unrelated(pre['status'])
    assert len(unrelated(current))==155
    for name,identity in pre['authorities'].items():
        record=h.read(f.OLD/name);h.verify_record(record);assert record['identity']==identity
    for rel in ('FAMILY_CONTRACT.json','COMPARISON.json','DECISION.json','REQUESTED_SIZING.json','REPORT.md','TESTS_PRIOR.txt','TESTS_PRIOR_ADDITIONAL.txt','TESTS_PRIOR_HANDOFF.txt','TESTS_FINAL.txt','STORAGE_before_cleanup.json','STORAGE_after_cleanup.json','CLEANUP.json','CLEANUP_PLAN.json','CLEANUP_PLAN_RAW.json','CLEANUP_EXECUTION_RAW.json','RESOURCE_SAMPLES.jsonl','SCHEDULING.json','HEALTH_CHECKPOINTS.json','DENSE_WAKE.json','GEOMETRY_REPLAY.json'):
        evidence[rel]=h.file_hash(f.ROOT/rel)
    for name in ('Baseline','W1','R1','C2','C2-R','G1'):
        evidence['resolution/'+name+'.json']=h.file_hash(f.ROOT/'resolution'/(name+'.json'))
    h.durable_json(f.ROOT/'CFD_MESH_RECHECK.json',h.seal(mesh_recheck))
    evidence['CFD_MESH_RECHECK.json']=h.file_hash(f.ROOT/'CFD_MESH_RECHECK.json')
    sources={str(path.relative_to(f.prior.control.V4)):h.file_hash(path) for path in sorted((f.prior.control.V4/'tools').glob('capsule*family*.py'))}
    sources['tools/capsule_broad_global_report.py']=h.file_hash(f.prior.control.V4/'tools/capsule_broad_global_report.py')
    sources['tests/test_capsule_broad_global_family.py']=h.file_hash(f.prior.control.V4/'tests/test_capsule_broad_global_family.py')
    sources['tools/capsule_family_cleanup.ps1']=h.file_hash(f.prior.control.V4/'tools/capsule_family_cleanup.ps1')
    result=h.seal(dict(schema='V4FrozenPrismBroadGlobalFamilyResult/1',classification=decision['classification'],starting_head=pre['head'],baseline_mesh='307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e',frozen=pre['frozen'],decision=decision,evidence=evidence,source_implementation=h.seal(sources),unrelated_dirty_entries=155,V3_changed=False,prism_changes=0,URANS_runs=0,Run_A=False,Run_A2=False,production_promotion=False))
    h.durable_json(f.ROOT/'RESULT.json',result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    import sys
    if sys.argv[1]=='seal':seal()
    else:storage(sys.argv[1])
