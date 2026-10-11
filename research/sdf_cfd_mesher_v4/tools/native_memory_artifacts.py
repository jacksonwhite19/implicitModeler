"""Lossless memory-task evidence sealing and hash-checked bounded restoration."""
from pathlib import Path
from contextlib import contextmanager
import tempfile,subprocess,tarfile,hashlib,json,shutil
from sdf_cfd_mesher_v4.common.authority import file_hash
OUT=Path(__file__).resolve().parents[1]/'results/native_runner_memory_profile'
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json

def wsl(path):
    path=Path(path).resolve();return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\','/')

def seal():
    archive=OUT/'EVIDENCE.tar.zst'
    if (OUT/'ARCHIVE.json').exists():raise ValueError('EvidenceAlreadySealed')
    keep={'SCIENTIFIC_COMPARISON.json','FINAL_NATIVE_AUDIT.json','EXECUTION.json','EVENT_TIMES.json','PROGRESS.json'}
    work={'fresh','baseline_first','general_regression','baseline_certificate','optimized_certificate'}
    def selected(p):
        parts=p.relative_to(OUT).parts
        if not p.is_file() or parts[0] not in work:return False
        if 'memory_profile' in parts or 'profile' in parts:return False
        if parts[0]=='fresh' and (p.name in keep or p.suffix=='.log' or p.name.startswith('COMMIT_')):return False
        if parts[0]=='baseline_first' and p.name=='RESULT.json':return False
        if parts[0]=='general_regression' and p.name in {'ROLLBACK_MANIFEST.json','VALIDATION_5.json','CORE_INGESTION.json'}:return False
        return True
    paths=sorted(p for p in OUT.rglob('*') if selected(p))
    index={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=file_hash(p)) for p in paths}
    if not archive.exists():
        process=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','--long=29','-o',wsl(archive)],stdin=subprocess.PIPE)
        with tarfile.open(fileobj=process.stdin,mode='w|') as tar:
            for p in paths:
                info=tarfile.TarInfo(p.relative_to(OUT).as_posix());info.size=p.stat().st_size;info.mtime=0
                with p.open('rb') as stream:tar.addfile(info,stream)
        process.stdin.close()
        if process.wait()!=0:raise ValueError('EvidenceCompressionFailed')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=29','-c',wsl(archive)],stdout=subprocess.PIPE)
    seen=set()
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if hashlib.file_digest(tar.extractfile(member),'sha256').hexdigest()!=index[member.name]['sha256']:raise ValueError('EvidenceArchiveMismatch')
            seen.add(member.name)
    # tar stops at its terminator; WSL still must flush trailing tar padding.
    # Waiting before draining that pipe can deadlock the Windows relay.
    while process.stdout.read(1024*1024):pass
    if process.wait()!=0 or seen!=set(index):raise ValueError('EvidenceArchiveIncomplete')
    archive_hash=file_hash(archive);parts=[]
    with archive.open('rb') as stream:
        while block:=stream.read(90*1024*1024):
            target=OUT/f'EVIDENCE.tar.zst.part{len(parts)+1:03}'
            target.write_bytes(block);parts.append(dict(file=target.name,sha256=file_hash(target),bytes=len(block)))
    combined=hashlib.sha256()
    for row in parts:
        with (OUT/row['file']).open('rb') as stream:
            while chunk:=stream.read(1024*1024):combined.update(chunk)
    if combined.hexdigest()!=archive_hash:raise ValueError('ArchivePartMismatch')
    atomic_json(OUT/'ARTIFACT_INDEX.json',index)
    atomic_json(OUT/'ARCHIVE.json',dict(format='tar.zst',window_log=29,sha256=archive_hash,parts=parts))
    for p in paths:
        if not p.resolve().is_relative_to(OUT.resolve()) or file_hash(p)!=index[p.relative_to(OUT).as_posix()]['sha256']:raise ValueError('ArchiveRemovalGuard')
        p.unlink()
    archive.unlink()

@contextmanager
def materialize(prefix):
    current=OUT/prefix
    if (current/'CURRENT.json').exists() or (current/'MANIFEST.json').exists():
        yield current;return
    index=json.loads((OUT/'ARTIFACT_INDEX.json').read_text());archive=json.loads((OUT/'ARCHIVE.json').read_text())
    with tempfile.TemporaryDirectory(prefix='v4-native-evidence-') as temporary:
        root=Path(temporary);joined=root/'evidence.tar.zst'
        with joined.open('wb') as target:
            for row in archive['parts']:
                p=OUT/row['file']
                if file_hash(p)!=row['sha256']:raise ValueError('EvidencePartCorrupt')
                with p.open('rb') as source:shutil.copyfileobj(source,target,1024*1024)
        if file_hash(joined)!=archive['sha256']:raise ValueError('EvidenceJoinCorrupt')
        process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=29','-c',wsl(joined)],stdout=subprocess.PIPE)
        with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
            for member in tar:
                if not member.name.startswith(prefix+'/'):continue
                p=(root/member.name).resolve()
                if not p.is_relative_to(root) or not member.isfile():raise ValueError('UnsafeEvidenceMember')
                p.parent.mkdir(parents=True,exist_ok=True)
                with p.open('wb') as target:shutil.copyfileobj(tar.extractfile(member),target,1024*1024)
                if file_hash(p)!=index[member.name]['sha256']:raise ValueError('RestoredEvidenceCorrupt')
        while process.stdout.read(1024*1024):pass
        if process.wait()!=0:raise ValueError('EvidenceDecodeFailed')
        yield root/prefix

if __name__=='__main__':seal()
