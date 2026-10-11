"""Lossless, content-addressed archive of large checkpoint and proof artifacts."""
from pathlib import Path
import hashlib,json,zipfile,argparse

ROOT=Path(__file__).resolve().parents[1]/'results/broader_mixed_geometry_end_to_end'


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()


def run(case):
    source=ROOT/case
    assert source.is_dir() and source.parent==ROOT
    files=sorted(p for p in source.rglob('*') if p.is_file() and
                 ('checkpoints' in p.parts or 'assembly' in p.parts))
    records={p.relative_to(ROOT).as_posix():dict(sha256=digest(p),bytes=p.stat().st_size) for p in files}
    dest=ROOT/'archives';dest.mkdir(exist_ok=True)
    path=dest/(case+'.zip')
    assert not path.exists(),'Preserve existing archive'
    unique={}
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED,compresslevel=3,allowZip64=True) as z:
        for p in files:
            key=records[p.relative_to(ROOT).as_posix()]['sha256']
            if key not in unique:
                z.write(p,key);unique[key]=p.stat().st_size
    # Verify every decompressed member, not just the ZIP CRC.
    with zipfile.ZipFile(path) as z:
        for key,size in unique.items():
            h=hashlib.sha256();count=0
            with z.open(key) as f:
                for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b);count+=len(b)
            assert h.hexdigest()==key and count==size
    parts=[]
    with path.open('rb') as f:
        while True:
            data=f.read(90*1024*1024)
            if not data:break
            part=path.with_name(path.name+f'.part{len(parts):03d}')
            assert not part.exists()
            part.write_bytes(data)
            parts.append(dict(path=part.relative_to(ROOT).as_posix(),bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))
    value=dict(files=records,parts=parts,archive_sha256=digest(path),archive_bytes=path.stat().st_size,
        raw_bytes=sum(r['bytes'] for r in records.values()),unique_bytes=sum(unique.values()),
        verified_decompressed_members=len(unique),lossless=True,
        restore='Concatenate ordered parts into ZIP. Extract members by SHA-256, then copy each member to its files-map relative path. Verify hashes. Raw local artifacts remain present; no deletion is performed.')
    (dest/(case+'.json')).write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')
    print(case,value['raw_bytes'],value['archive_bytes'],len(parts),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');run(p.parse_args().case)
