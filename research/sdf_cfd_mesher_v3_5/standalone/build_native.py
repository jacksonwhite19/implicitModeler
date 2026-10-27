"""Build the copied V3 consumers into this standalone tree."""
from pathlib import Path
import hashlib, json, os, subprocess, time

ROOT=Path(__file__).resolve().parent
NATIVE=ROOT/'v3/short_wing_displacement_contract_2026-09-06/native'
def linux(p):
    p=Path(p).resolve(); return '/mnt/'+p.drive[0].lower()+p.as_posix()[2:]

def build():
    logs=ROOT/'build_logs';logs.mkdir(exist_ok=True)
    binary=ROOT/'native_bin';binary.mkdir(exist_ok=True)
    results=[]
    for folder,name,source in [('direct','contractDirect','directAddPatchCellLayer.C'),('staged','contractStaged','stagedAddPatchCellLayer.C'),('audit','contractAudit','contractAudit.C')]:
        directory=NATIVE/folder
        (directory/'Make/files').write_text(source+'\n\nEXE = $(FOAM_USER_APPBIN)/'+name+'\n')
        for p in [directory/'Make/files',directory/'Make/options']:
            p.write_bytes(p.read_bytes().replace(b'\r\n',b'\n'))
        command=['wsl','-d','Ubuntu-22.04','--','bash','-lc',
            'source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; export FOAM_USER_APPBIN="'+linux(binary)+'"; cd "'+linux(directory)+'"; wmake']
        start=time.perf_counter()
        with (logs/(name+'.stdout')).open('w') as out,(logs/(name+'.stderr')).open('w') as err:
            p=subprocess.run(command,stdout=out,stderr=err,timeout=600)
        result=dict(name=name,command=command,seconds=time.perf_counter()-start,returncode=p.returncode)
        results.append(result)
        (logs/'NATIVE_BUILD.json').write_text(json.dumps(results,indent=2)+'\n')
        print(name,p.returncode,round(result['seconds'],3),flush=True)
        if p.returncode:raise RuntimeError('NativeBuildFailed:'+name)
        result['binary_sha256']=hashlib.sha256((binary/name).read_bytes()).hexdigest()
    (logs/'NATIVE_BUILD.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__': build()
