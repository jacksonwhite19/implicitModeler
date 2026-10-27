from task4j_common import *
from sdf_cfd_mesher_v4.certificate.compact_storage import write_compact
import tempfile
def run():
    rows=[]
    with tempfile.TemporaryDirectory(dir=OUT) as directory:
        for layer in (2,12,20):
            with np.load(BASE/f'certificate_layer_{layer:02d}/COMPACT.npz') as z:arrays={k:z[k] for k in z.files}
            for level in (6,1):
                path=Path(directory)/'test.npz';s=time.perf_counter();write_compact(path,arrays,level);write=time.perf_counter()-s
                h=file_hash(path);s=time.perf_counter()
                with np.load(path) as z:
                    for k,v in arrays.items():assert np.array_equal(z[k],v)
                load=time.perf_counter()-s;size=path.stat().st_size
                write_compact(path,arrays,level);assert file_hash(path)==h
                rows.append(dict(layer=layer,level=level,write_seconds=write,read_verify_seconds=load,bytes=size,deterministic=True));save('STORAGE_EXPERIMENT.json',rows)
if __name__=='__main__':run()
