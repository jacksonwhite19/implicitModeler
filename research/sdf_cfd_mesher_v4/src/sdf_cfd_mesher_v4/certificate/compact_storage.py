"""Canonical NPZ members; compression level changes storage only."""
import zipfile
import numpy as np

def write_compact(path,arrays,level=1):
    with zipfile.ZipFile(path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=level,allowZip64=True) as z:
        for name,value in arrays.items():
            info=zipfile.ZipInfo(name+'.npy',date_time=(1980,1,1,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED;info._compresslevel=level
            with z.open(info,'w',force_zip64=True) as f:
                np.lib.format.write_array(f,np.asanyarray(value),allow_pickle=False)
