"""Measured full-size quality-codec controls; not production timing."""
from pathlib import Path
import hashlib
import capsule_user_domain_handoff as h
from critical_quality_codec import encoded
from profile_remaining_prism import memory_sample

if __name__ == '__main__':
    out=Path(__file__).resolve().parents[1]/'results/remaining_prism_critical_path'
    rows=h.read(out/'normal_candidate_confirm_prism/QUALITY_DELTA.json')
    records=[]; expected=None
    for mode, workers in (('serial',1),('thread',2),('process',2),('process',4)):
        tick=h.measured();digest=hashlib.sha256();count=0
        with memory_sample(out,'codec_'+mode+str(workers)):
            for inputs,payload in encoded(rows,workers,mode=='process'):
                for blob in (inputs,payload):
                    digest.update((-1 if blob is None else len(blob)).to_bytes(8,'little',signed=True))
                    if blob is not None:digest.update(blob)
                count+=1
        if expected is None:expected=digest.hexdigest()
        assert count==219609 and digest.hexdigest()==expected
        records.append(dict(mode=mode,workers=workers,rows=count,hash=expected,metrics=h.metrics(*tick)))
        h.durable_json(out/'QUALITY_CODEC_SCALING.json',records)
        print(mode,workers,records[-1]['metrics']['seconds'],flush=True)
