"""Production atlas and forecast providers over native committed entities.

The existing qualified descriptor and atlas algorithms remain the algorithms.
Only native object storage supplies coordinates, schedules and evidence.
"""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from ..common.authority import array_hash,file_hash,identity
from ..geometry.front import build_topology
from ..forecast.descriptors import edge_descriptors,triangle_descriptors
from .risk_atlas import RiskAtlas


def native_array(objects,row):
    path=Path(objects)/row['file']
    if Path(row['file']).name!=row['file'] or file_hash(path)!=row['sha256']:raise ValueError('NativeProviderBufferMismatch')
    result=np.load(path,mmap_mode='r',allow_pickle=False)
    if array_hash(result)!=row['scientific']:raise ValueError('NativeProviderScientificMismatch')
    return result


class NativeForecast:
    def __init__(self,state,objects):
        self.state=state;self.objects=Path(objects)
        self.coordinates=native_array(objects,state['forecast']['coordinates'])
        self.normals=native_array(objects,state['forecast']['normals'])
        self.backing=state['cells'].backing
        self.top=build_topology(self.backing.triangles,len(self.backing.coordinates(0)))
        self.layer=state['forecast']['current_layer']
        if self.layer!=self.backing.layer or self.top.sha256!=state['forecast']['topology']:raise ValueError('StaleNativeForecastTopology')
        if array_hash(np.asarray(state['local_schedule']['nominal'],dtype=np.float64))!=state['authority']['schedule']:raise ValueError('StaleNativeForecastSchedule')
        if any(values!=state['local_schedule']['nominal'] for values in state['local_schedule']['actual'].values()):raise ValueError('UseMixedTrajectoryProjectionForAdjustedSchedule')
        for k in range(1,self.layer+1):
            if not np.array_equal(self.coordinates[k-1].view(np.uint64),self.backing.coordinates(k).view(np.uint64)):raise ValueError('StaleNativeForecastGeometry')
        self.active=np.array([int(c) for c,row in state['columns'].items() if row['status']=='ActivePrismColumn'],np.int64)
        if set(map(str,self.active))!=set(state['prism_front']):raise ValueError('NativeActiveFrontMembershipMismatch')
        self.vertices=np.unique(self.top.triangles[self.active])
        self.edges=np.unique(self.top.triangle_edges[self.active])
    @staticmethod
    def project_mixed(state,project):
        """Variable-depth/adjusted source trajectories use the qualified projector.

        No rectangular backing is manufactured for terminated or transition
        entities. Existing mixed providers supply their geometry oracle.
        """
        from .mixed_schedule import project_active_futures
        result=project_active_futures(state,project)
        result['dependencies']=identity(dict(versions=state['versions'],schedule=state['local_schedule'],
            active_front=state['prism_front'],outer=state['outer']))
        return result
    def dependency_identity(self):
        s=self.state
        return identity(dict(coordinates=s['forecast']['coordinates']['scientific'],normals=s['forecast']['normals']['scientific'],
            ancestry=s['authority']['source_ids'],schedule=s['local_schedule'],versions=s['versions'],
            current_layer=self.layer,active_columns=array_hash(self.active),outer=s['outer']))
    def layer_state(self,layer):
        if not self.layer<layer<=len(self.coordinates):raise ValueError('NotFutureNativeLayer')
        current=self.coordinates[layer-1];previous=self.coordinates[layer-2] if layer>1 else self.backing.coordinates(0)
        base=self.backing.coordinates(self.layer)
        return dict(layer=layer,vertices=self.vertices,columns=self.active,edges=self.edges,
            coordinates=current[self.vertices],
            edge=edge_descriptors(current,previous,base,self.normals[layer-1],self.top.edges[self.edges]),
            triangle=triangle_descriptors(current,previous,base,self.top.triangles[self.active]),
            dependencies=dict(identity=self.dependency_identity(),detailed_columns=self.active,
                              collision_outer_faces=self.state['outer']))
    def scientific(self):
        records={}
        for layer in range(self.layer+1,len(self.coordinates)+1):
            row=self.layer_state(layer)
            records[str(layer)]=dict(coordinates=array_hash(row['coordinates']),
                edge={k:array_hash(v) for k,v in row['edge'].items()},
                triangle={k:array_hash(v) for k,v in row['triangle'].items()})
        return dict(dependency=self.dependency_identity(),layers=records,
                    active_columns=array_hash(self.active),warnings=self.state['forecast']['warning_evidence'])


class NativeRiskAtlas(RiskAtlas):
    def __init__(self,state,objects):
        self.native_state=state;self.native_forecast=NativeForecast(state,objects)
        forecast=self.native_forecast;b=forecast.backing
        wall=SimpleNamespace(coordinates=b.coordinates(0),triangles=b.triangles,
            geometry_authority=state['authority']['geometry'],surface_authority=state['authority']['surface'])
        targets=np.asarray(state['local_schedule']['nominal'],dtype=np.float64)
        schedule=SimpleNamespace(targets=targets)
        sdf=state['authority'].get('sdf') or state['quality']['layer:1']['receipt']['inputs']['sdf']
        super().__init__(wall,forecast.top,schedule,sdf,forecast.coordinates,forecast.normals,copy_buffers=False)
        stored=state['atlas'];snapshot=dict(stored['metadata'])
        snapshot.update({name:native_array(objects,row) for name,row in stored['arrays'].items()})
        snapshot['future_ids']=self.future_ids
        snapshot['retained']=[np.asarray(x,dtype='<i8') for x in snapshot['retained']]
        self.restore(snapshot)
        if self.scientific_hash()!=stored['scientific']:raise ValueError('NativeAtlasScientificMismatch')
        # Current warnings determine state; no layer/percentage trigger is used.
        expected=self.states.copy();self.refresh_complete(snapshot['events'])
        if not np.array_equal(expected,self.states):raise ValueError('NativeAtlasWarningStateMismatch')
    def partition(self):
        result=super().partition()
        result['active_columns']=np.intersect1d(result['active_columns'],self.native_forecast.active)
        result['active_column_fraction']=len(result['active_columns'])/len(self.top.triangles)
        return result
    def native_identity(self):
        return identity(dict(task4c=self.scientific_hash(),dependencies=self.native_forecast.dependency_identity()))
