"""Standard solid zstd storage for byte-identical repeated checkpoint evidence."""
import hashlib,json,subprocess,tarfile,zipfile
from archive_mixed_front_evidence import OUT,digest

def wsl(path):return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\','/')

def run():
    index=json.loads((OUT/'ARTIFACT_INDEX.json').read_text());target=OUT/'EVIDENCE.tar.zst'
    if not target.exists():
        process=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','--long=29','-o',wsl(target)],stdin=subprocess.PIPE)
        with tarfile.open(fileobj=process.stdin,mode='w|') as tar:
            for name,row in sorted(index.items()):
                with zipfile.ZipFile(OUT/row['archive']) as z:
                    info=tarfile.TarInfo(name);info.size=row['bytes'];info.mtime=0
                    with z.open(name) as stream:tar.addfile(info,stream)
        process.stdin.close()
        if process.wait()!=0:raise ValueError('ZstdCompressionFailed')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=29','-c',wsl(target)],stdout=subprocess.PIPE)
    seen=set()
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            row=index[member.name];stream=tar.extractfile(member)
            if hashlib.file_digest(stream,'sha256').hexdigest()!=row['sha256']:raise ValueError('SolidArchiveVerificationFailed')
            seen.add(member.name)
    if process.wait()!=0 or seen!=set(index):raise ValueError('IncompleteSolidArchive')
    archives={row['archive'] for row in index.values()}
    for row in index.values():row['archive']=target.name;row['format']='tar.zst'
    (OUT/'ARTIFACT_INDEX.json').write_text(json.dumps(index,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')
    for name in archives:
        path=(OUT/name).resolve()
        if not path.is_relative_to(OUT.resolve()) or path.suffix!='.zip':raise ValueError('OutsideTaskArchive')
        path.unlink()
    print(target.name,target.stat().st_size,digest(target))

if __name__=='__main__':run()
