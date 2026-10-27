"""Restore losslessly archived task evidence before checkpoint replay."""
from pathlib import Path
import json,hashlib,zipfile
OUT=Path(__file__).resolve().parents[1]/'results/production_automatic_termination_remesh'

def restore():
    index=json.loads((OUT/'ARTIFACT_INDEX.json').read_text())
    with zipfile.ZipFile(OUT/index['archive']) as archive:
        for name,record in index['members'].items():
            path=(OUT/name).resolve()
            if not path.is_relative_to(OUT.resolve()):raise ValueError('ArchivePathOutsideTask')
            data=archive.read(name)
            if hashlib.sha256(data).hexdigest()!=record['sha256']:raise ValueError('ArchiveHashMismatch')
            if path.exists() and path.read_bytes()!=data:raise ValueError('RefuseToOverwriteChangedEvidence')
            path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    print('Restored',len(index['members']),'task evidence files')

if __name__=='__main__':restore()
