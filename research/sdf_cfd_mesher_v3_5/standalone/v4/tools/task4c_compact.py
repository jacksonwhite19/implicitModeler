"""Use the existing deterministic archive format for large study-only tables."""
from task4c_common import *
if __name__=='__main__':
    for name in ('atlas_1_SUPPORT_MISMATCH_2.json','RUNNER_02.json','RUNNER_08.json','INITIAL_SCAN_REPLAY.json'):
        path=OUT/name;value=read(path);before=scientific_hash(value);write(path,value)
        assert scientific_hash(read(path))==before
    print('Four large tables compacted without scientific changes')
