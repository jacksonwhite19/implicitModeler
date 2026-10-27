"""Post-build exact comparison; never supplies geometry to the A2 runner."""
from task4e_common import *
from task4c_compare_formats import compare_formats

def run():
    started=time.perf_counter();a=read(BASE/'FINAL_VALIDATION.json');b=read(RUN/'FINAL_VALIDATION.json')
    result=compare_formats(BASE,RUN,a,b)
    with np.load(BASE/'FINAL_CONDITIONING.npz') as x,np.load(RUN/'FINAL_CONDITIONING.npz') as y:
        assert x.files==y.files
        for key in x.files:assert np.array_equal(x[key].view(np.uint8),y[key].view(np.uint8)),key
    result.update(conditioning_arrays_bitwise_equal=True,completed_before_cutoff=elapsed()<3300)
    save('FINAL_COMPARISON.json',result);save('FINAL_COMPARISON_TIMING.json',dict(seconds=time.perf_counter()-started,elapsed_task=elapsed()))
    assert result['passed'];print('EXACT SCIENTIFIC EQUALITY',result['semantic_A'],flush=True)

if __name__=='__main__':run()
