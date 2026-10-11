"""Sparse persistent future-coordinate deltas against one frozen nominal cache."""
from pathlib import Path
import numpy as np
from ..common.authority import array_hash

def load_candidate(path,base):
    with np.load(path) as z:
        candidate=z['coordinates']
        if 'forecast' in z:return candidate,z['forecast'],z['normals']
        with np.load(base) as old:coordinates=old['coordinates'];normals=old['normals']
        assert str(z['base_coordinates_hash'])==array_hash(coordinates)
        ids=z['delta_vertices'];coordinates[:,ids]=z['delta_forecast'];normals[:,ids]=z['delta_normals']
        return candidate,coordinates,normals

def save_candidate(path,candidate,coordinates,normals,base):
    with np.load(base) as old:
        original=old['coordinates'];n=old['normals']
        ids=np.flatnonzero(np.any(coordinates.view(np.uint64)!=original.view(np.uint64),axis=(0,2))|
                           np.any(normals.view(np.uint64)!=n.view(np.uint64),axis=(0,2)))
        h=array_hash(original)
    np.savez_compressed(path,coordinates=candidate,delta_vertices=ids,delta_forecast=coordinates[:,ids],delta_normals=normals[:,ids],base_coordinates_hash=h)
    return ids
