"""Fresh prism construction provider for the native mixed rolling runner.

Reuses qualified correction kernels with an explicit native context. No
CommitStore or accepted historical correction input participates in execution.
"""
import json,time,gc
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from task3c_io import read,write
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front import VERSION,wall_authority,scientific_hash
from sdf_cfd_mesher_v4.advancing_front.native_prism_entities import prism_views,write_array
from sdf_cfd_mesher_v4.advancing_front.native_front_providers import NativeRiskAtlas,native_array
from sdf_cfd_mesher_v4.advancing_front.risk_atlas import RiskAtlas
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_layer,evaluate_cells,validate_admission_history
from sdf_cfd_mesher_v4.advancing_front.validation_evidence import input_identity,make_receipt
from sdf_cfd_mesher_v4.advancing_front.termination_policy import IMMEDIATE_CHECKS
from sdf_cfd_mesher_v4.certificate.production import certify_immediate
from sdf_cfd_mesher_v4.certificate.native_manifest import NativeManifest


class Provider:
    def __init__(self,output):
        if np.__version__!='1.26.4':
            raise RuntimeError('UnqualifiedNumericalRuntime: native SF2 construction requires the qualified NumPy 1.26.4 runtime')
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.objects=self.output/'checkpoints/objects';self.objects.mkdir(parents=True,exist_ok=True)
        self.schedule=load_schedule();self.sdf=NativeSdf();self.records=[];self.validated={}
    def atlas_record(self,atlas):
        snapshot=atlas.checkpoint()
        for key in ('states','generations','last_evaluated','future_ids'):snapshot.pop(key)
        snapshot['retained']=[x.tolist() for x in snapshot['retained']]
        return dict(metadata=snapshot,scientific=atlas.scientific_hash(),
                    arrays={k:write_array(self.objects,getattr(atlas,k)) for k in ('states','generations','last_evaluated')})
    def initial(self):
        from task4_initial_scan import initial_scan
        front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates))
        context=SimpleNamespace(front=front,top=top,sdf=self.sdf,schedule=self.schedule,output=self.output)
        initial_scan(context)
        with np.load(self.output/'INITIAL_FORECAST.npz') as z:coordinates=z['coordinates'];normals=z['normals']
        atlas=RiskAtlas(front,top,self.schedule,self.sdf.source_identity,coordinates,normals,copy_buffers=False)
        views=prism_views(self.objects,[front.coordinates],top.triangles,top.edges)
        return dict(version=VERSION,wall=wall_authority({str(i):x.tolist() for i,x in enumerate(front.coordinates)},
             {str(i):t.tolist() for i,t in enumerate(top.triangles)}),first_layer={},**views,
             outer=[f'cap:0:{i}' for i in range(len(top.triangles))],versions=dict(topology=0,geometry=0),
             authority=dict(geometry=front.geometry_authority,surface=front.surface_authority,sdf=self.sdf.source_identity,
                 source_ids=list(front.vertex_ids),source_ancestry=[list(x) for x in front.source_ancestry],schedule=array_hash(self.schedule.targets)),
             local_schedule=dict(nominal=self.schedule.targets.tolist(),actual={},default='nominal'),
             atlas=self.atlas_record(atlas),forecast=dict(coordinates=write_array(self.objects,coordinates),normals=write_array(self.objects,normals),
                 current_layer=0,source_vertices=list(front.vertex_ids),topology=top.sha256,warning_evidence=[],corrections=[]),
             quality={},certificate={},events={})
    def context(self,state):
        atlas=NativeRiskAtlas(state,self.objects);b=atlas.native_forecast.backing
        front=SimpleNamespace(coordinates=b.coordinates(0),triangles=b.triangles,committed_layer=0,
            target_distances=np.zeros(len(b.coordinates(0))),coordinate_hash=array_hash(b.coordinates(0)),
            topology_hash=atlas.top.sha256,geometry_authority=state['authority']['geometry'],surface_authority=state['authority']['surface'])
        return SimpleNamespace(front=front,top=atlas.top,sdf=self.sdf,schedule=self.schedule,output=self.output,atlas=atlas,
            current=SimpleNamespace(layer=b.layer,coordinates=b.coordinates(b.layer),forecast_key=state['forecast']['coordinates']['scientific']),
            prefix=tuple(b.coordinates(k) for k in range(1,b.layer+1)),
            forecast_coordinates=atlas.coordinates,forecast_normals=atlas.normals)
    def propose(self,state):
        from task4_correct_first import correct_first
        from task4_prepare_next import prepare
        from task4_warning_cache import initialize_warnings
        from task4_certify_candidate import exact_all
        tick=time.perf_counter();context=self.context(state);k=context.current.layer+1
        if k==1:correct_first(context);initialize_warnings(context)
        elif not prepare(k,context=context):raise ValueError('NativeOrdinaryCorrectionUnresolved')
        prepared=time.perf_counter()
        candidate,coordinates,normals=load_candidate(self.output/f'LAYER{k}_CANDIDATE.npz',self.output/'INITIAL_FORECAST.npz')
        decisions=read(self.output/('CORRECTION_HISTORY.json' if k==1 else f'LAYER{k}_SEARCH.json'))
        validate_admission_history(decisions['history'])
        prior=None if k==1 else evaluate_cells(context.top,context.front.coordinates if k==2 else context.prefix[-2],context.current.coordinates,k-1)
        numeric=evaluate_layer(context.top,context.current.coordinates,candidate,k,prior)
        if not numeric['gate']['passed']:raise ValueError('NativeImmediateQualityFailure')
        write(self.output/f'LAYER_{k:02}_METRIC_ARRAY_HASHES.json',{family:{key:array_hash(v) for key,v in numeric[family].items() if isinstance(v,np.ndarray)} for family in ('cells','sides','caps') if family in numeric})
        q=self.sdf.evaluate(candidate)
        if not np.all(abs(q.phi-self.schedule.targets[k-1])<=q.source_error_bound):raise ValueError('NativeSdfTargetFailure')
        sdf_record=dict(target=float(self.schedule.targets[k-1]),maximum_residual=float(abs(q.phi-self.schedule.targets[k-1]).max()),
            phi_hash=array_hash(q.phi),gradient_hash=array_hash(q.gradient),gradient_norm_min=float(q.gradient_norm.min()),gradient_norm_max=float(q.gradient_norm.max()))
        signs,sign_seconds=exact_all(context.top,context.current.coordinates,candidate,self.output/f'LAYER_{k:02}_EXACT_SIGNS.jsonl.gz')
        if not signs['passed']:raise ValueError('NativeImmediateExactFailure')
        numeric_done=time.perf_counter();prefix=[*context.prefix,candidate]
        proof,times=certify_immediate(context.front,context.top,prefix,self.output/f'certificate_layer_{k:02}',workers=8)
        if not proof['complete'] or any(proof[x] for x in ('forbidden','unresolved','invalid_surfaces')):raise ValueError('NativeContinuousFailure')
        manifest=NativeManifest.import_verified(self.output/'native_manifest',context.top.edges,[(self.output/f'certificate_layer_{k:02}',proof)],append=True)
        cert_done=time.perf_counter()
        views=prism_views(self.objects,[context.front.coordinates,*prefix],context.top.triangles,context.top.edges)
        replacement=dict(state);replacement.update(views)
        replacement['first_layer']=state['first_layer'] or {str(i):x.tolist() for i,x in enumerate(candidate)}
        replacement['outer']=[f'cap:{k}:{i}' for i in range(len(context.top.triangles))]
        replacement['versions']=dict(topology=0,geometry=k)
        warnings=read(self.output/f'LAYER{k}_WARNINGS.json')
        corrections=decisions['outcomes'] if k==1 else read(self.output/f'LAYER{k}_CORRECTIONS.json')
        replacement['forecast']=dict(state['forecast'],coordinates=write_array(self.objects,coordinates),normals=write_array(self.objects,normals),current_layer=k,warning_evidence=warnings,corrections=corrections)
        with np.load(self.output/f'LAYER{k}_ATLAS.npz') as z:
            replacement['atlas']=dict(metadata=json.loads(str(z['metadata'])),scientific=str(z['scientific_hash']),
                arrays={name:write_array(self.objects,z[name]) for name in ('states','generations','last_evaluated')})
        conditioning={name:numeric['cells'][name] for name in ('sigma','G_bound','G_bound_available')}
        inputs=input_identity(context.top,context.current.coordinates,candidate,None if k==1 else context.front.coordinates if k==2 else context.prefix[-2],k,self.schedule.targets,self.sdf.source_identity)
        receipt=make_receipt(context.top,inputs,numeric['gate'],sdf_record,signs,proof,conditioning)
        replacement['quality']=dict(state['quality']);replacement['quality'][f'layer:{k}']=dict(receipt=receipt,conditioning={name:write_array(self.objects,v) for name,v in conditioning.items()})
        replacement['certificate']=dict(state['certificate']);replacement['certificate'][f'layer:{k}']=manifest.metadata()['blocks'][-1]
        NativeRiskAtlas(replacement,self.objects) # reject inconsistent native atlas/forecast binding before publication
        self.records.append(dict(layer=k,prepare_seconds=prepared-tick,numeric_seconds=numeric_done-prepared,
            exact_sign_seconds=sign_seconds,certificate_seconds=cert_done-numeric_done,state_seconds=time.perf_counter()-cert_done,
            total_seconds=time.perf_counter()-tick,certificate_stages=times))
        digest=scientific_hash(replacement)
        self.validated[digest]=dict(state=digest,passed=True,checks=dict(hard_quality=numeric['gate']['passed'],
            exact_validity=signs['passed'],ownership=numeric['gate']['source_wall_ownership_valid'] and numeric['gate']['topology_valid'],
            continuous=proof['complete'] and not proof['forbidden'] and not proof['unresolved'],sdf_target=True),proof=identity(proof))
        return replacement
    def evidence(self,before,proposal):
        context=dict(base=scientific_hash(before),layer=proposal['forecast']['current_layer'],region='active-prism-front',
            topology=proposal['forecast']['topology'],schedule=proposal['authority']['schedule'])
        validated=self.validate(proposal,proposal,None)
        receipt=dict(context=context,proof=validated['proof'],checks=validated['checks'])
        return dict(context=context,support=dict(receipt,status='clean'),immediate=dict(receipt,status='valid',seed={}),
                    accepted=receipt,warnings=[],witnesses=[],attempts=[])
    def correct(self,before,proposal,decision):return proposal
    def validate(self,state,proposal,cavity):
        digest=scientific_hash(state)
        if digest!=scientific_hash(proposal) or digest not in self.validated:raise ValueError('UnvalidatedNativePrismCandidate')
        return dict(self.validated[digest])
    def rebuild(self,before,state,receipt):return state
