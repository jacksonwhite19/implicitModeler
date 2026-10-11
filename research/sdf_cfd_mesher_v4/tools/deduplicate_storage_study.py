"""Byte-identical hardlink reuse in this task's disposable working copies.

No production authority shortcut: both complete files are hashed before an
atomic replacement. Sealed reference files are read-only. Pending files and
objects held open without delete sharing are left untouched.
"""
import argparse,hashlib,json,os,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]/'results/canonical_block_authenticated_storage'
PRIOR=ROOT.parent/'incremental_certificate_receipt_identity'

def run(directory):
    directory=Path(directory).resolve()
    if not directory.is_relative_to(ROOT.resolve()) or directory.name!='objects':raise ValueError('StudyDedupScope')
    references={}
    for folder in list(PRIOR.glob('*/**/checkpoints/objects'))+list(ROOT.glob('*/**/checkpoints/objects')):
        if folder.resolve()==directory:continue
        for p in folder.iterdir():
            if p.is_file():references.setdefault(p.name,p)
    rows=[];start=time.perf_counter()
    for target in directory.iterdir():
        source=references.get(target.name)
        if source is None or target.stat().st_nlink>1 or target.suffix=='.pending':continue
        if target.stat().st_size!=source.stat().st_size:continue
        with target.open('rb') as f:a=hashlib.file_digest(f,'sha256').hexdigest()
        with source.open('rb') as f:b=hashlib.file_digest(f,'sha256').hexdigest()
        if a!=b:continue
        temporary=target.with_name(target.name+'.dedup-link')
        row=dict(file=target.name,sha256=a,bytes=target.stat().st_size,source=str(source),reused=False)
        try:
            os.link(source,temporary);os.replace(temporary,target);row['reused']=True
        except OSError as exc:row['error']=str(exc)
        finally:temporary.unlink(missing_ok=True)
        rows.append(row)
    result=dict(rows=rows,seconds=time.perf_counter()-start,bytes_reused=sum(r['bytes'] for r in rows if r['reused']))
    log=ROOT/'WORKING_COMPONENT_DEDUP.jsonl'
    with log.open('a') as f:f.write(json.dumps(dict(directory=str(directory),**result))+'\n')
    print('WORKING_DEDUP '+json.dumps(result),flush=True)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);run(p.parse_args().directory)
