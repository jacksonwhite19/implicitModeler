"""Losslessly seal this task's numerical artifacts; preserve readable records."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import tarfile
from sdf_cfd_mesher_v4.common.authority import file_hash

ROOT=Path(__file__).resolve().parents[1]/'results/mixed_front_shell_to_core'


def linux(path):
    p=str(path.resolve())
    return '/mnt/'+p[0].lower()+p[2:].replace('\\','/')


def main():
    if (ROOT/'ARCHIVE.json').exists():
        raise ValueError('EvidenceAlreadySealed')
    paths=sorted(p for p in ROOT.rglob('*') if p.is_file() and (p.suffix in {'.npy','.msh'} or p.name=='INTERFACE.json'))
    index={p.relative_to(ROOT).as_posix():dict(sha256=file_hash(p),bytes=p.stat().st_size) for p in paths}
    archive=ROOT/'NUMERICAL.tar.zst'
    process=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','-o',linux(archive)],stdin=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdin,mode='w|') as tar:
        for p in paths:
            info=tarfile.TarInfo(p.relative_to(ROOT).as_posix());info.size=p.stat().st_size;info.mtime=0
            with p.open('rb') as stream:tar.addfile(info,stream)
    process.stdin.close()
    if process.wait():raise ValueError('CompressionFailed')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','-c',linux(archive)],stdout=subprocess.PIPE)
    seen=set()
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if hashlib.file_digest(tar.extractfile(member),'sha256').hexdigest()!=index[member.name]['sha256']:
                raise ValueError('ArchiveMismatch')
            seen.add(member.name)
    while process.stdout.read(1024*1024):pass
    if process.wait() or seen!=set(index):raise ValueError('ArchiveIncomplete')
    parts=[]
    with archive.open('rb') as stream:
        while block:=stream.read(90*1024*1024):
            target=ROOT/f'NUMERICAL.tar.zst.part{len(parts)+1:03}'
            target.write_bytes(block)
            parts.append(dict(file=target.name,sha256=file_hash(target),bytes=len(block)))
    (ROOT/'ARTIFACT_INDEX.json').write_text(json.dumps(index,sort_keys=True,indent=2)+'\n')
    (ROOT/'ARCHIVE.json').write_text(json.dumps(dict(format='tar.zst',sha256=file_hash(archive),parts=parts,restoration='Concatenate parts in listed order, verify SHA-256, zstd decompress then extract tar; verify every member against ARTIFACT_INDEX.json.'),sort_keys=True,indent=2)+'\n')
    for p in paths:
        if not p.resolve().is_relative_to(ROOT.resolve()) or file_hash(p)!=index[p.relative_to(ROOT).as_posix()]['sha256']:
            raise ValueError('RemovalGuard')
        p.unlink()
    archive.unlink()


if __name__=='__main__':main()
