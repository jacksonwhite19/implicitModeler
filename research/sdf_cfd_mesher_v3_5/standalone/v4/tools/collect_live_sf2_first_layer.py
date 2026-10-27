"""Compact evidence collection and proportionate preservation, no meshing."""
import ctypes,datetime,json,os,shutil,subprocess,time
from pathlib import Path
from run_live_sf2_first_layer import OUT,ROOT,V4,read,save,sha

def allocation(path):
    if os.name!='nt':return path.stat().st_blocks*512
    from ctypes import wintypes
    api=ctypes.WinDLL('kernel32',use_last_error=True)
    class StandardInfo(ctypes.Structure):
        _fields_=[('allocation',ctypes.c_longlong),('end',ctypes.c_longlong),('links',wintypes.DWORD),('deleted',ctypes.c_ubyte),('directory',ctypes.c_ubyte)]
    api.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE];api.CreateFileW.restype=wintypes.HANDLE
    api.GetFileInformationByHandleEx.argtypes=[wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD];api.GetFileInformationByHandleEx.restype=wintypes.BOOL
    api.CloseHandle.argtypes=[wintypes.HANDLE]
    handle=api.CreateFileW(str(path.resolve()),0x80,7,None,3,0,None)
    if handle==wintypes.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
    try:
        info=StandardInfo()
        if not api.GetFileInformationByHandleEx(handle,1,ctypes.byref(info),ctypes.sizeof(info)):raise ctypes.WinError(ctypes.get_last_error())
        return info.allocation
    finally:api.CloseHandle(handle)

def main():
    base=read(OUT/'PRESERVATION_BASELINE.json');runs={}
    for label in ['A','B']:
        folder=OUT/label
        if not (folder/'RESULT.json').exists():continue
        r={name:read(folder/(name+'.json')) for name in ['RESULT','TRAJECTORY','DIAGNOSTICS','TOPOLOGY','CERTIFICATE_SUMMARY','COVERAGE_RECHECK'] if (folder/(name+'.json')).exists()}
        if (folder/'VOLUME.json').exists():
            v=read(folder/'VOLUME.json')
            r['VOLUME']={k:v[k] for k in ['boundary_volume','cell_volume','exact_volume_balance','positive_cells','conditional_nonoverlap','prerequisites','proof']}
            r['VOLUME']['interior_degree']=dict(passed=v['interior_degree']['passed'],unresolved=v['interior_degree']['unresolved'],witnesses=len(v['interior_degree']['values']))
        r['PROCESS']=read(OUT/(label+'.PROCESS.json'));runs[label]=r
    save(OUT/'RESULTS.json',runs)
    # Match the baseline's literal UTF-8 path presentation. Git's default
    # quotePath octal escaping is not a file-identity change.
    status=subprocess.check_output(['git','-c','core.quotePath=false','status','--porcelain=v1','--untracked-files=normal'],cwd=ROOT).decode('utf-8').splitlines()
    task_prefix=OUT.relative_to(ROOT).as_posix()+'/'
    owned={str(p.relative_to(ROOT)).replace('\\','/') for p in [V4/'LIVE_SF2_FIRST_LAYER_REPORT.md']+[V4/'tools'/n for n in ['live_spline_first_layer.py','run_live_sf2_first_layer.py','execute_live_sf2_first_layer.py','verify_live_sf2_first_layer.py','collect_live_sf2_first_layer.py','test_live_sf2_first_layer.py']]}
    unrelated=[r for r in status if r[3:] not in owned and not r[3:].startswith(task_prefix)]
    altered=[]
    for row in base['files']:
        p=ROOT/row['path']
        if not p.is_file() or p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:altered.append(row['path'])
    result=dict(status_exact=unrelated==base['status'],baseline_entries=len(base['status']),final_unrelated_entries=len(unrelated),protected_files=len(base['files']),altered_protected_files=altered,
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        status=status,coverage='Exact case-sensitive porcelain entries and hashes/sizes of the 245 selected protected files; not a repository-wide byte inventory. Nested repositories not entered.')
    save(OUT/'PRESERVATION_FINAL.json',result)
    if not result['status_exact'] or altered:raise ValueError('PreservationMismatch')
    manifest=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.name not in ['BULK_MANIFEST.json','RESOURCES.json','HASHES.json']:
            manifest.append(dict(path=p.relative_to(ROOT).as_posix(),bytes=p.stat().st_size,allocated_bytes=allocation(p),sha256=sha(p)))
    save(OUT/'BULK_MANIFEST.json',manifest)
    start=datetime.datetime.fromisoformat(base['time']);now=datetime.datetime.now(datetime.timezone.utc)
    save(OUT/'RESOURCES.json',dict(initial_free_c=base['initial_free_c'],initial_free_z=base['initial_free_z'],final_free_c=shutil.disk_usage(ROOT).free,
        final_free_z=int(subprocess.check_output(['wsl.exe','--','df','-B1','/'],text=True).splitlines()[1].split()[3]),elapsed_since_baseline_seconds=(now-start).total_seconds(),
        retained_logical_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),
        retained_allocation_bytes=sum(allocation(p) for p in OUT.rglob('*') if p.is_file()),
        source_report_logical_bytes=sum((ROOT/p).stat().st_size for p in owned),
        source_report_allocation_bytes=sum(allocation(ROOT/p) for p in owned),
        resource_scope='Task result directory; source/report sizes separately visible in Git. Volumes are aliases, not summed.',
        allocation_method='NTFS FileStandardInfo.AllocationSize per file; excludes directory/MFT and Git metadata. Z backing availability uses WSL df Available, matching baseline and PowerShell 7 Get-PSDrive.',
        cleanup='No scientific cleanup. Only synthetic unit-test TemporaryDirectory outputs were automatically removed.',
        worker_peak_memory=max(r['RESULT']['peak_working_set_bytes'] for r in runs.values()),whole_task_peak_memory='Not measured'))
    names=['AUTHORITY.json','REPLAY.json','IMPLEMENTATION.json','RESULTS.json','PRESERVATION_FINAL.json','BULK_MANIFEST.json','RESOURCES.json','REQUEST.json','TESTS.json']
    save(OUT/'HASHES.json',{name:sha(OUT/name) for name in names if (OUT/name).exists()})
    print(json.dumps(dict(preservation={k:v for k,v in result.items() if k!='status'},resources=read(OUT/'RESOURCES.json')),indent=2))

if __name__=='__main__':main()
