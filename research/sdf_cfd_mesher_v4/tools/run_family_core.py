"""Single qualified interface/core/assembly path for every family member."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
import argparse,json,time,subprocess
import psutil
import broader_geometry_assembly as assembly
from native_mixed_face_certificate import certify
from sdf_cfd_mesher_v4.common.authority import file_hash,identity

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/broader_geometry_family_qualification'


def wsl(p):
    p=p.resolve();return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')


def bound_core_directory(folder,base):
    inputs={p.name:file_hash(p) for p in sorted((folder/'interface').iterdir()) if p.is_file()}
    binding=dict(inputs=inputs,scientific=identity(inputs))
    destination=folder/base
    prior=destination/'CORE_INPUT.json'
    if destination.exists() and (not prior.exists() or json.loads(prior.read_text())!=binding):
        destination=folder/(base+'_'+binding['scientific'][:16])
    destination.mkdir(exist_ok=True)
    receipt=destination/'CORE_INPUT.json'
    if receipt.exists() and json.loads(receipt.read_text())!=binding:raise ValueError('CoreInputIdentityCollision')
    assembly.g.save(receipt,binding)
    return destination.name


def run(name,label,*,sealed_state=None):
    folder=OUT/name/label;assert (folder/'SHELL_RESULT.json').exists() or (folder/'PARTIAL_SHELL_RESULT.json').exists()
    tick=time.perf_counter();times={};trials=[];selected=None
    try:
        for algorithm,core_name in [(1,'core'),(10,'core_hxt')]:
            core_name=bound_core_directory(folder,core_name)
            started=time.perf_counter()
            if not (folder/core_name/'GENERATION.json').exists():
                with (folder/(core_name+'_GENERATION.log')).open('w') as log:
                    subprocess.run(['wsl','--','env','PYTHONPATH='+wsl(ROOT/'src'),'python3','-B',wsl(ROOT/'tools/broader_geometry_core.py'),wsl(folder/'interface'),wsl(folder/core_name),'--algorithm',str(algorithm)],stdout=log,stderr=subprocess.STDOUT,check=True)
            times[core_name+'_generation_or_reuse']=time.perf_counter()-started
            quality=json.loads((folder/core_name/'GENERATION.json').read_text())
            row=dict(core=core_name,algorithm=algorithm,initial_passed=quality['passed']);trials.append(row)
            if quality['passed']:selected=core_name;break
            from repair_family_core import run as repair
            started=time.perf_counter();destination='repaired_'+core_name
            try:
                repair(folder,core_name,destination);selected=destination;row['repair_passed']=True;break
            except ValueError as exc:row.update(repair_passed=False,reason=str(exc))
            finally:times[core_name+'_repair']=time.perf_counter()-started
        assembly.g.save(folder/'CORE_ATTEMPTS.json',trials)
        if selected is None:raise ValueError('PreservedCoreRoutesDidNotPassFrozenGates')
        assembly.g.save(folder/'CORE_SELECTED.json',dict(directory=selected))
        def measured(points,faces,active):
            started=time.perf_counter();result=certify(points,faces,active)
            times['complete_certificate']=time.perf_counter()-started
            return result
        started=time.perf_counter();assembly.run(folder,core_name=selected,certificate_provider=measured,sealed_state=sealed_state);times['assembly']=time.perf_counter()-started
    finally:
        times['total']=time.perf_counter()-tick;times['peak_rss']=psutil.Process().memory_info().peak_wset
        assembly.g.save(folder/'COMPLETE_PERFORMANCE.json',times)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--replay',default='Q');a=p.parse_args();run(a.case,a.replay)
