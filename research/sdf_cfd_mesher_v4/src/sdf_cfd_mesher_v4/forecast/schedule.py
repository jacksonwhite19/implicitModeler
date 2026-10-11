"""Bind the preserved uniform nominal V3 increments; never round for use."""
from dataclasses import dataclass
import json
import numpy as np
from ..common.authority import ROOT,REPO,file_hash,array_hash,frozen

@dataclass(frozen=True)
class LayerSchedule:
    increments: np.ndarray
    targets: np.ndarray
    sha256: str
    source_sha256: str
    first_height: float
    growth: float
    def remaining(self,layer):
        if not 0<=layer<=len(self.targets):raise ValueError('InvalidCommittedLayer')
        return self.targets[layer:]

def load_schedule():
    rows=json.loads((ROOT/'authorities/TASK1_SOURCE_BINDING.json').read_text())['files']
    r=next(r for r in rows if r['path'].endswith('/input_state/increments.npy'))
    p=REPO/r['path'];assert file_hash(p)==r['sha256']
    data=np.load(p,mmap_mode='r');inc=np.array(data[0])
    if not np.array_equal(data,np.broadcast_to(inc,data.shape)):raise ValueError('NonuniformNominalAuthority')
    if len(inc)!=20 or inc[0]!=0.000166527069:raise ValueError('PhysicalContractMismatch')
    targets=np.cumsum(inc,dtype=np.float64)
    return LayerSchedule(frozen(inc,'<f8'),frozen(targets,'<f8'),array_hash(targets),r['sha256'],inc[0],1.163359674)
