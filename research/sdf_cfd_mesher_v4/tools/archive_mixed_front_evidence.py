"""Lossless task-local evidence archive, verified before removing raw copies."""
from pathlib import Path
import hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/purpose_built_mixed_front_runner'

def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def run():
    index={};groups={}
    for path in sorted(OUT.rglob('*')):
        if not path.is_file() or path.suffix=='.zip' or path.stat().st_size<=1_000_000:continue
        relative=path.relative_to(OUT);parts=relative.parts
        group='/'.join(parts[:2]) if len(parts)>2 else parts[0]
        groups.setdefault(group,[]).append(path)
    for number,paths in enumerate(groups.values(),1):
        archive=OUT/f'EVIDENCE_{number:02}.zip'
        if archive.exists():raise ValueError('ArchiveAlreadyExists')
        members={}
        with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
            for path in paths:
                relative=path.relative_to(OUT).as_posix()
                members[relative]=dict(archive=archive.name,sha256=digest(path),bytes=path.stat().st_size)
                z.write(path,relative)
        with zipfile.ZipFile(archive) as z:
            for relative,record in members.items():
                with z.open(relative) as stream:
                    if hashlib.file_digest(stream,'sha256').hexdigest()!=record['sha256']:raise ValueError('ArchiveVerificationFailed')
        for path in paths:
            if not path.resolve().is_relative_to(OUT.resolve()):raise ValueError('OutsideTaskEvidence')
            path.unlink()
        index.update(members)
    (OUT/'ARTIFACT_INDEX.json').write_text(json.dumps(index,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')

def restore():
    index=json.loads((OUT/'ARTIFACT_INDEX.json').read_text())
    if index and all(r.get('format')=='tar.zst' for r in index.values()):
        import subprocess,tarfile
        from solid_mixed_front_evidence import wsl
        process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=29','-c',wsl(OUT/'EVIDENCE.tar.zst')],stdout=subprocess.PIPE)
        seen=set()
        with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
            for member in tar:
                record=index[member.name];target=(OUT/member.name).resolve()
                if not target.is_relative_to(OUT.resolve()):raise ValueError('OutsideTaskEvidence')
                data=tar.extractfile(member).read()
                if hashlib.sha256(data).hexdigest()!=record['sha256']:raise ValueError('ArchiveVerificationFailed')
                if target.exists():
                    if digest(target)!=record['sha256']:raise ValueError('ExistingEvidenceMismatch')
                else:target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
                seen.add(member.name)
        if process.wait()!=0 or seen!=set(index):raise ValueError('IncompleteSolidArchive')
        return
    for relative,record in index.items():
        target=(OUT/relative).resolve()
        if not target.is_relative_to(OUT.resolve()):raise ValueError('OutsideTaskEvidence')
        if target.exists():
            if digest(target)!=record['sha256']:raise ValueError('ExistingEvidenceMismatch')
            continue
        with zipfile.ZipFile(OUT/record['archive']) as z:data=z.read(relative)
        if hashlib.sha256(data).hexdigest()!=record['sha256']:raise ValueError('ArchiveVerificationFailed')
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)

if __name__=='__main__':
    import sys
    restore() if '--restore' in sys.argv else run()
