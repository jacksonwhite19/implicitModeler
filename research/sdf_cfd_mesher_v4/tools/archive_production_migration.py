"""Lossless, verified storage of large task-owned checkpoints."""
import hashlib,json,subprocess,tarfile,sys
from pathlib import Path
from audit_production_migration import OUT

def digest(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def wsl(path):return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\','/')

def run():
    target=OUT/'EVIDENCE.tar.zst'
    if target.exists():raise ValueError('ArchiveAlreadyExists')
    paths=sorted(p for p in OUT.rglob('*') if p.is_file() and p.stat().st_size>1000000)
    index={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=digest(p),archive=target.name,format='tar.zst') for p in paths}
    process=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','--long=31','-o',wsl(target.resolve())],stdin=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdin,mode='w|') as tar:
        for p in paths:
            name=p.relative_to(OUT).as_posix();info=tarfile.TarInfo(name);info.size=index[name]['bytes'];info.mtime=0
            with p.open('rb') as f:tar.addfile(info,f)
    process.stdin.close()
    if process.wait()!=0:raise ValueError('ArchiveCompressionFailed')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=31','-c',wsl(target.resolve())],stdout=subprocess.PIPE)
    seen=set()
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if hashlib.file_digest(tar.extractfile(member),'sha256').hexdigest()!=index[member.name]['sha256']:raise ValueError('ArchiveBytesMismatch')
            seen.add(member.name)
    if process.wait()!=0 or seen!=set(index):raise ValueError('ArchivePopulationMismatch')
    (OUT/'ARTIFACT_INDEX.json').write_text(json.dumps(index,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')
    for p in paths:
        if not p.resolve().is_relative_to(OUT.resolve()) or digest(p)!=index[p.relative_to(OUT).as_posix()]['sha256']:raise ValueError('ArchiveRemovalGuard')
        p.unlink()
    print('ARCHIVED',len(paths),target.stat().st_size,digest(target),flush=True)

def restore(destination):
    destination=Path(destination).resolve();destination.mkdir(parents=True,exist_ok=True)
    index=json.loads((OUT/'ARTIFACT_INDEX.json').read_text());seen=set()
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=31','-c',wsl((OUT/'EVIDENCE.tar.zst').resolve())],stdout=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            row=index[member.name];path=(destination/member.name).resolve()
            if not member.isfile() or not path.is_relative_to(destination):raise ValueError('UnsafeArchiveMember')
            path.parent.mkdir(parents=True,exist_ok=True)
            if path.exists():
                if digest(path)!=row['sha256']:raise ValueError('ExistingFileDiffers')
            else:
                with path.open('xb') as out:
                    source=tar.extractfile(member)
                    while block:=source.read(1024*1024):out.write(block)
                if digest(path)!=row['sha256']:raise ValueError('RestoredBytesMismatch')
            seen.add(member.name)
    if process.wait()!=0 or seen!=set(index):raise ValueError('IncompleteRestore')

def repack():
    original=OUT/'EVIDENCE.tar.zst';target=OUT/'REPACK.tar.zst'
    decoder=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=31','-c',wsl(original.resolve())],stdout=subprocess.PIPE)
    encoder=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','--long=31','-o',wsl(target.resolve())],stdin=decoder.stdout)
    decoder.stdout.close()
    if encoder.wait()!=0 or decoder.wait()!=0:raise ValueError('RepackFailed')
    index=json.loads((OUT/'ARTIFACT_INDEX.json').read_text());seen=set()
    check=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=31','-c',wsl(target.resolve())],stdout=subprocess.PIPE)
    with tarfile.open(fileobj=check.stdout,mode='r|') as tar:
        for member in tar:
            if hashlib.file_digest(tar.extractfile(member),'sha256').hexdigest()!=index[member.name]['sha256']:raise ValueError('RepackBytesMismatch')
            seen.add(member.name)
    if check.wait()!=0 or seen!=set(index):raise ValueError('RepackPopulationMismatch')
    target.replace(original)
    print('REPACKED',original.stat().st_size,digest(original),flush=True)

if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--restore':restore(sys.argv[2])
    elif sys.argv[1:]==['--repack']:repack()
    elif len(sys.argv)==1:run()
    else:raise SystemExit('Usage: archive_production_migration.py [--restore DIRECTORY]')
