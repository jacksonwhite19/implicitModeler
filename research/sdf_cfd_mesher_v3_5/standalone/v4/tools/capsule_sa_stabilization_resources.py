"""Read-only process sampling. Does not change solver controls."""
import json
from pathlib import Path
import time
import capsule_steady_models_resources as r
r.ROOT=Path(__file__).resolve().parents[1]/'results/capsule_sa_stabilization'
r.NAMES=('SA-GW1-S1','SA-GW1-S2')
if __name__=='__main__':
    with (r.ROOT/'RESOURCE_SAMPLES.jsonl').open('a',buffering=1) as out:
        while True:
            out.write(json.dumps(r.sample(),sort_keys=True)+'\n')
            if all((r.ROOT/n/'continuation.exit').exists() for n in r.NAMES):break
            time.sleep(10)
