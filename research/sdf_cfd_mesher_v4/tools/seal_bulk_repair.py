"""Seal task-owned numerical evidence only after successful qualification."""
import sys
sys.dont_write_bytecode=True
import json,hashlib,subprocess,tarfile,shutil
from pathlib import Path
from repair_bulk_core import OUT,write
from sdf_cfd_mesher_v4.common.authority import file_hash

def wsl(path):
    p=str(path.resolve());return '/mnt/'+p[0].lower()+p[2:].replace('\\','/')

def main():
    if not json.loads((OUT/'FOUNDATION.json').read_text())['literal_mesh_ok']:raise ValueError('NotQualified')
    archive=OUT/'EVIDENCE.tar.zst'
    if (OUT/'ARCHIVE.json').exists():raise ValueError('AlreadySealed')
    # Replays and restored immutable import sources are not duplicate authority.
    roots={'repaired_A','quality_arrays','assembly','foundation','mixed_core','mixed_owner_corrected'}
    paths=sorted(p for p in OUT.rglob('*') if p.is_file() and p.relative_to(OUT).parts[0] in roots and not p.name.startswith('certificate_'))
    index={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=file_hash(p)) for p in paths}
    process=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','-o',wsl(archive)],stdin=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdin,mode='w|') as tar:
        for p in paths:
            member=tarfile.TarInfo(p.relative_to(OUT).as_posix());member.size=p.stat().st_size;member.mtime=0
            with p.open('rb') as stream:tar.addfile(member,stream)
    process.stdin.close()
    if process.wait():raise ValueError('CompressionFailed')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','-c',wsl(archive)],stdout=subprocess.PIPE);seen=set()
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if hashlib.file_digest(tar.extractfile(member),'sha256').hexdigest()!=index[member.name]['sha256']:raise ValueError('ArchiveVerificationFailed')
            seen.add(member.name)
    while process.stdout.read(1024*1024):pass
    if process.wait() or seen!=set(index):raise ValueError('ArchiveIncomplete')
    parts=[]
    with archive.open('rb') as stream:
        while block:=stream.read(90*1024*1024):
            target=OUT/f'EVIDENCE.tar.zst.part{len(parts)+1:03}';target.write_bytes(block)
            parts.append(dict(file=target.name,bytes=len(block),sha256=file_hash(target)))
    write('ARTIFACT_INDEX.json',index);write('ARCHIVE.json',dict(format='tar.zst',sha256=file_hash(archive),parts=parts))
    # Preserve files in the workspace; Git commits the verified lossless archive.
    archive.unlink()

if __name__=='__main__':main()
