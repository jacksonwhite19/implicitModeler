"""Verify and restore the sealed rejected core, without legacy shell state."""
from pathlib import Path
import hashlib,json,shutil,subprocess,tarfile
from sdf_cfd_mesher_v4.common.authority import file_hash

BASE=Path(__file__).resolve().parents[1]/'results'
SOURCE=BASE/'mixed_front_shell_to_core'
OUT=BASE/'fixed_interface_bulk_core_repair'


def main():
    archive=json.loads((SOURCE/'ARCHIVE.json').read_text())
    index=json.loads((SOURCE/'ARTIFACT_INDEX.json').read_text())
    joined=OUT/'source.tar.zst'
    with joined.open('wb') as target:
        for row in archive['parts']:
            p=SOURCE/row['file']
            if file_hash(p)!=row['sha256']:raise ValueError('SourcePartCorrupt')
            with p.open('rb') as stream:shutil.copyfileobj(stream,target,1024*1024)
    if file_hash(joined)!=archive['sha256']:raise ValueError('SourceArchiveCorrupt')
    p=str(joined.resolve());linux='/mnt/'+p[0].lower()+p[2:].replace('\\','/')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','-c',linux],stdout=subprocess.PIPE)
    prefixes=('sf2_core/third_standard/','sf2_interface/','mixed_interface/')
    restored={}
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if not member.name.startswith(prefixes):continue
            dest=(OUT/'source'/member.name).resolve()
            if not dest.is_relative_to((OUT/'source').resolve()) or not member.isfile():raise ValueError('UnsafeMember')
            dest.parent.mkdir(parents=True,exist_ok=True)
            with dest.open('wb') as target:shutil.copyfileobj(tar.extractfile(member),target,1024*1024)
            if file_hash(dest)!=index[member.name]['sha256']:raise ValueError('RestoredMemberCorrupt')
            restored[member.name]=index[member.name]
    while process.stdout.read(1024*1024):pass
    if process.wait():raise ValueError('DecodeFailed')
    (OUT/'RESTORED.json').write_text(json.dumps(restored,sort_keys=True,indent=2)+'\n')
    joined.unlink()


if __name__=='__main__':main()
