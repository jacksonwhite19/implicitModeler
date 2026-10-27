"""Publish only after frozen core and complete continuous gates pass."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import importlib.util,json,time,shlex,hashlib,re
import numpy as np

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def stream_hash(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def existing_stage(stage,name):
    text=(stage/(name+'.resource.txt')).read_text()
    code=int(re.search(r'Exit status:\s*(\d+)',text).group(1))
    elapsed=re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\):\s*(\S+)',text).group(1)
    seconds=0.
    for field in elapsed.split(':'):seconds=seconds*60+float(field)
    rss=int(re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)',text).group(1))*1024
    if code:raise ValueError('ExistingStageFailed:'+name)
    return dict(exit_code=code,runtime_seconds=seconds,peak_rss_bytes=rss,stdout_sha256=stream_hash(stage/(name+'.stdout')),stderr_sha256=stream_hash(stage/(name+'.stderr')),resume='Validated existing stage resources and exact conversion fidelity')

def main():
    import resource
    root=Path(__file__).resolve().parents[3];v3=root/'research/sdf_cfd_mesher_v3';out=root/'research/sdf_cfd_mesher_v4/results/fixed_interface_bulk_core_repair';stage=out/'foundation';stage.mkdir(exist_ok=True)
    cert=json.loads((out/'CONTINUOUS.json').read_text());quality=json.loads((out/'repaired_A/HARD_QUALITY.json').read_text());support=json.loads((out/'SUPPORT_INTERFACE.json').read_text())
    if not cert['passed'] or not quality['passed'] or any(support[k] for k in ['failed_weight','failed_ratio','failed_skew']):raise ValueError('PublicationGateFailed')
    complete=load('v4_readonly_complete_reference',v3/'farfield_core_transition/run_farfield_complete.py')
    bounded=load('v4_readonly_conversion_reference',v3/'pmp_failure_forensics/surface_acceptance_contract/bounded_downstream_integration.py')
    arrays={k:np.load(out/'assembly'/(k+'.npy'),mmap_mode='r') for k in ['coordinates','prisms','tets','wall','farfield']}
    x,p,t,w,f=[arrays[k] for k in ['coordinates','prisms','tets','wall','farfield']];start=time.perf_counter()
    resume='--resume' in sys.argv
    case=stage/'openfoam_case'
    quote=lambda p:shlex.quote(str(p.resolve()))
    prefix='source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; '
    if resume:
        transfer=dict(sha256=stream_hash(stage/'complete_transfer.msh'),bytes=(stage/'complete_transfer.msh').stat().st_size,nodes=len(x),elements=len(w)+len(f)+len(p)+len(t))
        conversion=existing_stage(stage,'gmshToFoam')
    else:
        transfer=complete.write_msh(stage/'complete_transfer.msh',x,w,f,p,t)
        complete.old.create_case(case)
        conversion=complete.timed(prefix+'gmshToFoam '+quote(stage/'complete_transfer.msh')+' -case '+quote(case),stage,'gmshToFoam',timeout=1200)
    if conversion['exit_code']:raise ValueError('ConversionFailed')
    audit_dir=stage/'foundation_audit';audit_dir.mkdir(exist_ok=True)
    audit=existing_stage(stage,'foundationAudit') if resume else complete.timed(prefix+quote(v3/'hxt_complete_mesh_preserved_core/bin/foundationCompleteAudit')+' -case '+quote(case)+' -outputDir '+quote(audit_dir),stage,'foundationAudit',timeout=1200)
    check=existing_stage(stage,'checkMesh') if resume else complete.timed(prefix+'checkMesh -constant -allTopology -allGeometry -case '+quote(case),stage,'checkMesh',timeout=1200)
    if audit['exit_code']:raise ValueError('FoundationAuditFailed')
    nc=len(w);shell_count=json.loads((out/'ASSEMBLY.json').read_text())['shell_prisms'];interface=p[shell_count-nc:shell_count,3:]
    fidelity=bounded.conversion_fidelity_bounded(audit_dir,case,x,p,t,w,f,interface)
    summary=json.loads((audit_dir/'foundation_audit_summary.json').read_text());text=(stage/'checkMesh.stdout').read_text();literal='Mesh OK.' in text and check['exit_code']==0
    hashes={path.name:stream_hash(path) for path in sorted((case/'constant/polyMesh').iterdir()) if path.is_file()}
    hard=bool(summary['minimum_cell_volume']>0 and summary['minimum_determinant']>=.0011 and summary['minimum_interpolation_weight']>=.055 and summary['minimum_face_volume_ratio']>=.01 and summary['maximum_cell_aspect_ratio']<=1000 and summary['maximum_face_skewness']<=3.63636 and summary['cells']==len(p)+len(t) and summary['points']==len(x))
    result=dict(literal_mesh_ok=literal,frozen_hard_gates=hard,conversion_fidelity=fidelity,summary=summary,convert=conversion,audit=audit,check=check,transfer=transfer,polyMesh_files=hashes,seconds=sum(row['runtime_seconds'] for row in (conversion,audit,check)),collection_seconds=time.perf_counter()-start,collection_resumed=resume,peak_rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,failed_lines=[line.strip() for line in text.splitlines() if 'Failed' in line or line.strip().startswith('***')])
    (out/'FOUNDATION.json').write_text(json.dumps(result,sort_keys=True,indent=2)+'\n')
    if not literal or not fidelity['pass'] or not hard:raise ValueError('CompleteFoundationGateFailed')

if __name__=='__main__':main()
