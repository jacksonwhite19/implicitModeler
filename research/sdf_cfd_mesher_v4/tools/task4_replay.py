"""Independent A2 execution from frozen wall, only after A finishes validation."""
from pathlib import Path
import os,sys,subprocess,json,time
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/task4'
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
from task3c_io import read
from sdf_cfd_mesher_v4.forecast.detailed.scientific import scientific_hash
from task4_certify_candidate import MemorySampler
from task4_compare_replay import compare

if __name__=='__main__':
    before=read(OUT/'FINAL_VALIDATION.json');assert before['layers']==20 and before['certificate']['complete']
    replay=OUT/'replay_A2';assert not replay.exists(),'A2 already exists: inspect before resuming'
    env=dict(os.environ,TASK4_OUTPUT_DIRECTORY=str(replay.resolve()))
    started=time.perf_counter()
    with MemorySampler() as memory:
        subprocess.run([sys.executable,'-B',str(ROOT/'tools/task4_build.py')],env=env,check=True)
    (OUT/'A2_PRODUCTION_TIMING.json').write_text(json.dumps(dict(wall_seconds=time.perf_counter()-started,
        observed_tree_peak_rss=memory.peak,memory_samples=memory.samples,memory_sample_interval_seconds=.1,
        scope='Fresh independent build including final qualification; no development pauses; RSS covers the replay coordinator and all descendants'),indent=2)+'\n')
    compare(expected_state_hash=before['transaction_state_hash'])
