"""Bounded lookup worker control; never substitutes for scientific coverage."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import capsule_user_domain_handoff as h
from persistent_numeric_incidence import open_geometry

if __name__=='__main__':
    out=Path(__file__).resolve().parents[1]/'results/persistent_publication_optimization'
    parent=h.read(out/'PARENT_INCIDENCE_AUTHORITY.json');lookup=h.read(out/'PARENT_LOOKUP_AUTHORITY.json')
    ref=dict(directory=str(out/'parent_incidence'),root=parent['root'],content=parent['parent_content'],lookup=lookup['reference'])
    records=[];expected=None
    with open_geometry(ref,ref['content']) as index:
        hashes=index.lookup.hashes
        queries=[bytes(hashes[i]).decode() for i in range(0,len(hashes),max(1,len(hashes)//4096))][:4096]
        for key in queries[:64]:index.scientific_face(key)
        for workers in (1,2,4):
            tick=h.measured()
            if workers==1:rows=[index.scientific_face(key) for key in queries]
            else:
                with ThreadPoolExecutor(workers) as pool:rows=list(pool.map(index.scientific_face,queries))
            timing=h.metrics(*tick);digest=h.identity(rows)
            if expected is None:expected=digest
            assert digest==expected
            records.append(dict(workers=workers,queries=len(queries),metrics=timing,identity=digest))
    h.durable_json(out/'LOOKUP_SCALING.json',records)
