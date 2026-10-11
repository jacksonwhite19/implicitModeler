"""Append-only exact certificate coverage, separate from committed mesh state."""
import copy,hashlib,json
from .continuous_prism_certificate import repaired_kernel as kernel
def digest(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
class ProofLedger:
    def __init__(self,topology,implementation):self.topology=topology;self.implementation=implementation;self.layers=[]
    def append(self,record):
        r=copy.deepcopy(record)
        if r['layer']!=len(self.layers)+1:raise ValueError('NonsequentialCertificateTransaction')
        if r['topology']!=self.topology or r['implementation']!=self.implementation or r['kernel']!=kernel.VERSION:raise ValueError('ProofAuthorityMismatch')
        if r['forbidden'] or r['unresolved'] or r['invalid_surfaces'] or r['completed_pairs']!=r['candidate_pairs']:raise ValueError('IncompleteCertificateTransaction')
        if self.layers and r['previous_geometry_hash']!=self.layers[-1]['geometry_hash']:raise ValueError('ChangedCommittedGeometry')
        self.layers.append(r)
    def snapshot(self):return copy.deepcopy(self.layers)
    def final(self,requested_layers):
        if len(self.layers)!=requested_layers:raise ValueError('IncompleteFinalCoverage')
        record=dict(version='V4ExactCoverage/1',topology=self.topology,implementation=self.implementation,kernel=kernel.VERSION,
            requested_layers=requested_layers,complete=True,layers=self.snapshot(),candidate_pairs=sum(r['candidate_pairs'] for r in self.layers),
            forbidden=0,unresolved=0,invalid_surfaces=0,
            universe='All distinct unordered side pairs; strict corner-AABB separation covers the complement of the enumerated population.',
            pair_ownership='Transaction max(layer_a,layer_b); canonical packed source-side IDs, unique within each transaction.')
        record['scientific_hash']=digest(record);return record
