"""Atomic complete-front Task-4 transactions and restartable hash chains.

Only CURRENT.json publishes a checkpoint. Prepared files alone never advance
the authority. Connectivity is the frozen triangle array with layer offsets.
"""
from dataclasses import dataclass
from pathlib import Path
import json,os,hashlib
import numpy as np
from ..common.authority import frozen,array_hash,identity,file_hash

VERSION='V4RollingCommit/1'

def connectivity(triangles,nvertices,layer):
    if layer<1:raise ValueError('NoPrismsAtWall')
    return np.concatenate((triangles+(layer-1)*nvertices,triangles+layer*nvertices),axis=1).astype('<i8')

def atomic_json(path,value):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.pending')
    with tmp.open('w') as f:json.dump(value,f,sort_keys=True,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)

def atomic_coordinates(path,coordinates):
    path=Path(path);tmp=path.with_suffix(path.suffix+'.pending')
    with tmp.open('wb') as f:np.save(f,coordinates);f.flush();os.fsync(f.fileno())
    os.replace(tmp,path)

@dataclass(frozen=True)
class RollingState:
    layer:int
    coordinates:np.ndarray
    topology_hash:str
    state_hash:str
    prism_hash:str
    side_hash:str
    certificate_hash:str
    forecast_key:str

class CommitStore:
    def __init__(self,path,front,top,schedule):
        self.path=Path(path);self.path.mkdir(parents=True,exist_ok=True)
        self.front=front;self.top=top;self.schedule=schedule

    def initialize(self):
        if (self.path/'CURRENT.json').exists():return self.restart()
        meta=dict(version=VERSION,layer=0,coordinate_hash=self.front.coordinate_hash,topology_hash=self.top.sha256,
            geometry=self.front.geometry_authority,surface=self.front.surface_authority,schedule=array_hash(self.schedule.targets),
            prism_hash=identity([]),side_hash=identity([]),certificate_hash=identity([]),forecast_key='',previous=None,warnings=[],corrections=[])
        meta['state_hash']=identity(meta);atomic_json(self.path/'LAYER_00.json',meta)
        atomic_json(self.path/'CURRENT.json',dict(layer=0,checkpoint='LAYER_00.json',sha256=file_hash(self.path/'LAYER_00.json')))
        return self.restart()

    def restart(self):
        pointer=json.loads((self.path/'CURRENT.json').read_text())
        if pointer.get('format',VERSION)!=VERSION:
            # Rectangular forecast/connectivity consumers cannot resume a
            # mixed topology until their adapters are qualified. Fail before
            # reading or rebuilding implicit prism connectivity.
            raise ValueError('MixedTopologyRequiresQualifiedRollingAdapter')
        p=self.path/pointer['checkpoint'];assert file_hash(p)==pointer['sha256']
        previous=None;old=self.front.coordinates
        for k in range(pointer['layer']+1):
            m=json.loads((self.path/f'LAYER_{k:02}.json').read_text());h=m.pop('state_hash');assert identity(m)==h;m['state_hash']=h
            assert m['layer']==k and m['topology_hash']==self.top.sha256 and m['schedule']==array_hash(self.schedule.targets)
            assert m['geometry']==self.front.geometry_authority and m['surface']==self.front.surface_authority
            if k==0:x=self.front.coordinates;assert m['previous'] is None
            else:
                assert m['previous']==previous['state_hash']
                assert file_hash(self.path/m['coordinate_file'])==m['coordinate_file_sha256'];x=np.load(self.path/m['coordinate_file'])
                assert m['prism_hash']==identity(dict(previous=previous['prism_hash'],layer_connectivity=array_hash(connectivity(self.top.triangles,len(x),k))))
                assert m['side_hash']==identity(dict(previous=previous['side_hash'],low=array_hash(old),high=array_hash(x),edges=array_hash(self.top.edges)))
                assert m['certificate_hash']==identity(dict(previous=previous['certificate_hash'],new=m['certificate']))
            assert array_hash(x)==m['coordinate_hash'];previous=m;old=x
        return RollingState(m['layer'],frozen(x,'<f8'),m['topology_hash'],h,m['prism_hash'],m['side_hash'],m['certificate_hash'],m['forecast_key'])

    def commit(self,state,coordinates,gate,exact,certificate,forecast_key,warnings,corrections):
        if self.restart().state_hash!=state.state_hash:raise ValueError('StaleTransaction')
        if state.layer>=len(self.schedule.targets):raise ValueError('RequestedLayerCountReached')
        if not gate['passed'] or not exact['passed'] or any(gate.get('violations',{}).values()) or exact.get('failures'):raise ValueError('ImmediateHardGate')
        if not certificate.get('complete') or any(certificate.get(k) for k in ('forbidden','unresolved','invalid_surfaces')):raise ValueError('ExactCertificateRequired')
        if certificate['candidate_pairs']!=certificate['completed_pairs']:raise ValueError('IncompleteExactCoverage')
        x=frozen(coordinates,'<f8');assert x.shape==state.coordinates.shape and np.isfinite(x).all()
        layer=state.layer+1;name=f'FRONT_{layer:02}.npy';atomic_coordinates(self.path/name,x)
        prisms=connectivity(self.top.triangles,len(x),layer)
        m=dict(version=VERSION,layer=layer,coordinate_hash=array_hash(x),coordinate_file=name,coordinate_file_sha256=file_hash(self.path/name),
            topology_hash=self.top.sha256,geometry=self.front.geometry_authority,surface=self.front.surface_authority,schedule=array_hash(self.schedule.targets),
            prism_hash=identity(dict(previous=state.prism_hash,layer_connectivity=array_hash(prisms))),
            side_hash=identity(dict(previous=state.side_hash,low=array_hash(state.coordinates),high=array_hash(x),edges=array_hash(self.top.edges))),
            certificate_hash=identity(dict(previous=state.certificate_hash,new=certificate)),forecast_key=forecast_key,previous=state.state_hash,
            warnings=warnings,corrections=corrections,gate=gate,exact=exact,certificate=certificate)
        m['state_hash']=identity(m);checkpoint=f'LAYER_{layer:02}.json';atomic_json(self.path/checkpoint,m)
        atomic_json(self.path/'CURRENT.json',dict(layer=layer,checkpoint=checkpoint,sha256=file_hash(self.path/checkpoint)))
        return self.restart()

def require_final_certificate(state,requested,run):
    if state.layer!=requested:raise ValueError('IncompleteRequestedShell')
    proof=run()
    if not proof['complete'] or any(proof[k] for k in ('forbidden','unresolved','invalid_surfaces')):raise ValueError('FinalShellCertificateFailed')
    return proof
