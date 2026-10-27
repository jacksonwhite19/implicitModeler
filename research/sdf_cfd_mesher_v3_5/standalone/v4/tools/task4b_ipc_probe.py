"""Small separate measurement of Windows spawn and warm batch transport."""
from task4b_common import *
from concurrent.futures import ProcessPoolExecutor
def echo(payload):return len(payload[0]),len(payload[1])
if __name__=='__main__':
    payload=(np.zeros((16,4,3)),np.zeros((16,4),np.uint64));start=time.perf_counter()
    with ProcessPoolExecutor(max_workers=8) as pool:
        assert list(pool.map(echo,[payload]*8))==[(16,16)]*8
        startup=time.perf_counter()-start;start=time.perf_counter()
        assert list(pool.map(echo,[payload]*256))==[(16,16)]*256
        warm=time.perf_counter()-start
    save('IPC_PROBE.json',dict(workers=8,spawn_plus_first_eight_roundtrips_seconds=startup,warm_256_roundtrips_seconds=warm,
        input_array_bytes=sum(x.nbytes for x in payload),note='Transport-only synthetic payload with the same 8-pair maximum geometry size; no kernel work.'))
