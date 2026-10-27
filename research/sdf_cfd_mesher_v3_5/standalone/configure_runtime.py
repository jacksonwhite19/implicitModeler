"""Install the already-built local canonical host; preserve upstream metadata."""
from pathlib import Path
import hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parent
def configure():
    runtime=ROOT/'v4/runtime';manifest=runtime/'MANIFEST.json'
    prior=json.loads(manifest.read_text())
    before=runtime/'UPSTREAM_MANIFEST.json'
    if not before.exists():shutil.copyfile(manifest,before)
    executable=ROOT/'v4/canonical_kernel/target/debug/canonical_kernel_host.exe'
    target=runtime/'bin/canonical_kernel_host.exe'
    if not target.exists() or hashlib.sha256(target.read_bytes()).digest()!=hashlib.sha256(executable.read_bytes()).digest():shutil.copyfile(executable,target)
    hello=subprocess.run([str(target)],input='',text=True,capture_output=True,timeout=10,check=True)
    source=json.loads(hello.stdout)['kernel_source_sha256']
    row=next(r for r in prior['files'] if r['path']=='runtime/bin/canonical_kernel_host.exe')
    row.update(bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest(),scope='V3.5 local canonical source build; infrastructure only')
    prior['native_host_installation']=dict(kernel_source_sha256=source,scope='V3.5 local copied kernel; no upstream scientific promotion',binary_sha256=row['sha256'])
    manifest.write_text(json.dumps(prior,indent=2)+'\n')
    print(json.dumps(dict(binary_sha256=row['sha256'],kernel_source_sha256=source)))
if __name__=='__main__':configure()
