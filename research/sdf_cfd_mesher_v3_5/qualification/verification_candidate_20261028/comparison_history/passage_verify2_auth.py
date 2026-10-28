"""External read-only comparison contract. No project imports or live grants.

Stored roots authenticate each run independently. This does not reconstruct
the unretained ephemeral mesh-token preimage or transfer historical authority.
"""
from pathlib import Path
import copy
import hashlib
import json
import re
import os
import ctypes
from ctypes import wintypes
from contextlib import contextmanager
import numpy as np

BASE = Path(__file__).resolve().parents[1]
PKG = BASE / 'work/phase1a/research/sdf_cfd_mesher_v3_5/standalone'
ORIGINAL = Path('C:/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/research/sdf_cfd_mesher_v3_5/standalone')
AUDIT_ROOT = '14f841e6c24075a06e4a62ec7038ba9c7dd92dfaab0565b1ba23137ed7963e9b'
PREFLIGHT = 'a25380f5e1362fb24a890590c1df8e32bf75577c4995f0657f87ce5ef86d5228'
OLD_FAILURE = 'e317230d394b1a2bf2ebcf173c994598601258dcf729fc3b5e3a46f74d2a83f1'
LOADER = {'candidate': '2e6bb59671521059d44f1de1506c50898ffe08516f40129f276e0410b0e49346',
          'baseline': 'ccef4e411170b4c4f0d4fbe5abb6bd5e1c062fc10a4007ed42f4e88a0c791d12'}
BOUND = ('generator_type','implementation_identity','geometry_identity','configuration_identity',
         'sizing_field_identity','adaptation_identity','coordinate_hash','topology_hash','residual_hash')
RAW_NAMES = set('points.f64 point_ancestor.i32 point_level.i32 cell_ancestor.i32 cell_level.i32 cell_offsets.u64 cell_points.i32 cell_volumes.f64 cell_centres.f64 cell_determinants.f64 cell_aspect_ratios.f64 cell_openness.f64 face_offsets.u64 face_points.i32 owner.i32 neighbour.i32 face_centres.f64 face_areas.f64 face_skewness.f64 face_weights.f64 face_volume_ratios.f64 internal_mask.u8'.split())
UPSTREAM_FILES = ('CASE.json','CURRENT.json','MESH.npz','shell/ACCEPTED_SHELL.json',
    'first/REQUEST.json','first/source/ADMISSION.json','first/source/TRAJECTORY.json',
    'first/source/SOURCE_SEGMENTS.json','first/source/SOURCE_SEGMENTS.jsonl','first/source/SOURCE_SEGMENTS.checkpoint.json',
    'native/SEALED_PLAN.json','native/SEALED_PLAN.npz',
    'upstream/surface/A/fresh/SURFACE_AUTHORITY.json')
_held = None

class HeldInputs:
    """Windows read handles deny write/delete; no scientific authority issued."""
    def __init__(self,roots):
        require(os.name=='nt','UnqualifiedComparisonFilesystem')
        self.roots=tuple(Path(p).resolve() for p in roots);self.handles={};self.initial={}
        self.api=ctypes.WinDLL('kernel32',use_last_error=True)
        self.api.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE]
        self.api.CreateFileW.restype=wintypes.HANDLE
        self.api.CloseHandle.argtypes=[wintypes.HANDLE];self.api.CloseHandle.restype=wintypes.BOOL
    def hold(self,path):
        path=Path(path).resolve()
        if path in self.handles:return
        handle=self.api.CreateFileW(str(path),0x80000000,1,None,3,0,None)
        if handle==wintypes.HANDLE(-1).value:raise ctypes.WinError(ctypes.get_last_error())
        self.handles[path]=handle
        s=path.stat();self.initial[path]=(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)
    def enter(self):
        self.population={root:tuple(sorted(p.resolve() for p in root.rglob('*') if p.is_file())) for root in self.roots}
        for paths in self.population.values():
            for path in paths:self.hold(path)
    def verify(self):
        for root,paths in self.population.items():require(tuple(sorted(p.resolve() for p in root.rglob('*') if p.is_file()))==paths,'ComparisonInputPopulationChanged')
        for path,expected in self.initial.items():
            s=path.stat();require((s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns)==expected,'ComparisonInputChanged')
    def close(self):
        for h in self.handles.values():
            if not self.api.CloseHandle(h):raise ctypes.WinError(ctypes.get_last_error())
        self.handles.clear()

@contextmanager
def input_scope(roots,extra=()):
    global _held
    require(_held is None,'NestedComparisonScope')
    held=HeldInputs(roots);_held=held
    try:
        held.enter()
        for p in extra:held.hold(p)
        yield held
        held.verify()
    finally:
        held.close();_held=None

def read(p):
    if _held is not None:_held.hold(p)
    def pairs(items):
        result = {}
        for k,v in items:
            if k in result: raise ValueError('DuplicateJSONKey:'+k)
            result[k] = v
        return result
    return json.loads(Path(p).read_text(encoding='utf8'), object_pairs_hook=pairs,
                      parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))

def sha(p):
    if _held is not None:_held.hold(p)
    with Path(p).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()

def identity(v):
    return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def array_hash(v): return hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest()

def require(ok, why):
    if not ok: raise ValueError(why)

def exact_arrays(a,b):
    with np.load(a,allow_pickle=False) as x,np.load(b,allow_pickle=False) as y:
        require(set(x.files)==set(y.files),'ArrayPopulation:'+str(a))
        for k in x.files:
            v,w=x[k],y[k]
            require(v.dtype==w.dtype and v.shape==w.shape and v.tobytes()==w.tobytes(),'ArrayPayload:'+k)

def selected_case(case, mode):
    v=copy.deepcopy(case)
    if mode=='baseline':
        require('verification' not in v,'BaselinePolicyChanged')
    else:
        choice=v.pop('verification')
        expected=dict(schema='V35VerificationSelection/1',mode=mode,requested=mode,effective=mode,
            origin='explicit_cli',profile=None if mode=='audit' else dict(version='V35DirectProducerAcceptance/1',
            identity='56370ef77a4525ef2209cbdd62df8da7091cd3805238daf0c952e4df444c8d7b'),
            background_identity=identity(v['background']),stop_after='complete')
        require(choice==expected,'UnsupportedVerificationSelection')
    return v

def package_path(name, package):
    p=Path(name)
    require(p.is_absolute() and p.resolve().is_relative_to(package.resolve()),'UnknownSourceRoot')
    rel=p.resolve().relative_to(package.resolve()).as_posix()
    require('..' not in Path(name).parts,'SourcePathEscape')
    return rel

def acceptance_binding(a):
    v={k:a[k] for k in ('surface_assurance','acceptance_inputs','acceptance_contract','acceptance_custody_completed')}
    v['checks']={k:a[k] for k in ('topology','normal','sdf_residual','feature_ownership','source_coverage','continuous_geometry','final_point_qualification')}
    v['checks']['numerical_geometry']={k:x for k,x in a['numerical_geometry'].items() if k!='seconds'}
    v['checks']['self_intersection']={k:a['self_intersection'][k] for k in ('forbidden','unresolved','missing','duplicate','coverage','method')}
    return v

# Explicit measured leaves only. Everything else, including new fields, remains
# in the semantic comparison. Ignored leaves are still sealed within each run.
AUDIT_MEASUREMENTS = ('seconds','peak_rss_bytes','retained_rss_bytes','topology_seconds',
    'numerical_geometry.seconds','numerical_geometry.batch_statistics.seconds',
    'self_intersection.timing.seconds','self_intersection.timing.free_before_bytes',
    'self_intersection.timing.measured_input_bytes','self_intersection.timing.peak_rss_bytes')
GEN_MEASUREMENTS = ('generation_seconds','peak_rss_bytes','retained_rss_bytes','timing.contract_seconds',
    'timing.extraction.native.native_seconds','timing.extraction.point_identity.seconds',
    'timing.extraction.seed_orientation_seconds','timing.extraction.seed_projection_seconds',
    'timing.final_projection_seconds','timing.final_sizing_seconds','timing.mmgs.free_before_bytes',
    'timing.mmgs.peak_rss_bytes','timing.mmgs.seconds','timing.seed_orientation_check_seconds','timing.sizing_seconds')

def measurements(value, paths):
    v=copy.deepcopy(value)
    for path in paths:
        parts=path.split('.'); parent=v
        for key in parts[:-1]: parent=parent[key]
        item=parent[parts[-1]]
        require(type(item) in (int,float) and np.isfinite(item) and item>=0,'InvalidMeasurement:'+path)
        parent[parts[-1]]='<measured>'
    return v

def relocated_command(command, root, package):
    # Only the explicitly selected command operands call this function.
    pairs=[]
    for path,role in ((root,'<run>'),(package,'<package>')):
        win=str(path.resolve());posix=path.resolve().as_posix()
        wsl='/mnt/'+win[0].lower()+win[2:].replace('\\','/')
        pairs.extend(((win,role),(posix,role),(wsl,role)))
    result=[]
    for item in command:
        require(isinstance(item,str),'InvalidCommandOperand')
        for prefix,role in pairs: item=item.replace(prefix,role)
        result.append(item)
    return result

def surface_provenance(root, package, mode):
    folder=root/'upstream/surface/A/fresh'
    case=read(root/'CASE.json');physical=selected_case(case,mode)
    require(sha(folder/'CASE.json')==sha(root/'CASE.json'),'DetachedSurfaceCase')
    authority=read(folder/'SURFACE_AUTHORITY.json');impl=read(folder/'IMPLEMENTATION.json')
    audit=read(folder/'AUDIT.json');generation=read(folder/'GENERATION.json')
    expected={'SURFACE.npz','AUDIT.json','QUALITY.npz','NUMERICAL_GEOMETRY.json','GENERATION.json','REFERENCES.npz','IMPLEMENTATION.json','CASE.json'}
    require(set(authority['files'])==expected,'SurfaceFilePopulation')
    for name,digest in authority['files'].items():require(sha(folder/name)==digest,'SurfaceFileHash:'+name)
    require(identity(impl)==authority['implementation_identity'],'ImplementationIdentity')
    require(impl['case_sha256']==sha(root/'CASE.json'),'ImplementationCase')
    require(identity(acceptance_binding(audit))==authority['acceptance_identity'],'AcceptanceIdentity')
    bound={k:authority[k] for k in BOUND}
    for k in ('assurance','acceptance_identity'):
        if k in authority:bound[k]=authority[k]
    require(identity(bound)==authority['surface_identity'],'SurfaceIdentity')
    require(read(folder/'NUMERICAL_GEOMETRY.json')==audit['numerical_geometry'],'DetachedNumericalEvidence')
    with np.load(folder/'SURFACE.npz',allow_pickle=False) as z:
        for key,name in [('coordinate_hash','coordinates'),('topology_hash','triangles'),('sizing_field_identity','metric'),('residual_hash','residual')]:
            require(array_hash(z[name])==authority[key],'SurfaceArrayBinding:'+name)
    normalized_impl=copy.deepcopy(impl)
    for kind in ('dependencies','binaries'):
        for name,digest in impl[kind].items():
            require(name.startswith('standalone/') and '..' not in Path(name).parts,'ImplementationPathEscape')
            require(sha(package/name[len('standalone/'):])==digest,'ImplementationSource:'+name)
    key='standalone/v4/tools/standalone_case.py'
    require(impl['dependencies'][key]==LOADER['baseline' if mode=='baseline' else 'candidate'],'UnreviewedLoader')
    normalized_impl['dependencies'][key]='<reviewed_loader_pair>'
    normalized_impl['case_sha256']='<own_case>'
    require(audit['acceptance_contract']['limits']['source']==str(package/'v4/tools/general_source_contract.py'),'AcceptanceSourcePath')
    normalized_audit=measurements(audit,AUDIT_MEASUREMENTS)
    normalized_audit['acceptance_contract']['limits']['source']='<package>/v4/tools/general_source_contract.py'
    normalized_audit['self_intersection']['timing']['command']=relocated_command(audit['self_intersection']['timing']['command'],root,package)
    normalized_generation=measurements(generation,GEN_MEASUREMENTS)
    normalized_generation['timing']['mmgs']['command']=relocated_command(generation['timing']['mmgs']['command'],root,package)
    normalized_authority=copy.deepcopy(authority)
    for k in ('implementation_identity','acceptance_identity','surface_identity'):normalized_authority[k]='<authenticated_derived_identity>'
    for name in ('AUDIT.json','NUMERICAL_GEOMETRY.json','GENERATION.json','IMPLEMENTATION.json','CASE.json'):
        normalized_authority['files'][name]='<authenticated_provenance_or_measurements>'
    request=read(root/'first/REQUEST.json');admission=read(root/'first/source/ADMISSION.json')
    require(request['source_documents']==[dict(path=str(root/'CASE.json'),sha256=sha(root/'CASE.json'))],'RequestCaseDocument')
    require(request['surface_identity']==authority['surface_identity']==admission['admission']['surface_identity']==admission['authority']['production_surface']==admission['coverage']['surface_identity'],'DetachedSurfaceAdmission')
    require(identity(request)==admission['admission']['request_identity']==admission['authority']['request'],'RequestIdentity')
    require(sha(root/'first/REQUEST.json')==admission['admission']['request_file_sha256'],'RequestByteHash')
    require(identity(admission['branch'])==admission['authority']['source_branch'],'SourceBranchIdentity')
    sourcefiles={}
    for name,digest in admission['branch']['source_files'].items():
        rel=package_path(name,package)
        require(rel not in sourcefiles and sha(name)==digest,'SourcePathBijectionOrHash')
        sourcefiles[rel]=digest
    require(set(sourcefiles)=={'v4/src/sdf_cfd_mesher_v4/sdf/operations.py','v4/src/sdf_cfd_mesher_v4/sdf/regional.py','v4/src/sdf_cfd_mesher_v4/sdf/source_topology.py','v4/tools/general_sdf_authority.py'},'SourcePopulation')
    normalized_request=copy.deepcopy(request);normalized_request['source_documents']='<own_case>';normalized_request['surface_identity']='<own_surface>'
    normalized_admission=copy.deepcopy(admission)
    for k in ('request_identity','request_file_sha256','surface_identity'):normalized_admission['admission'][k]='<own_bound_reference>'
    for k in ('request','production_surface','source_branch'):normalized_admission['authority'][k]='<own_bound_reference>'
    normalized_admission['coverage']['surface_identity']='<own_surface>'
    normalized_admission['branch']['source_files']=sourcefiles
    trajectory=trajectory_provenance(root,package,admission,request,authority)
    return dict(case=physical,implementation=normalized_impl,authority=normalized_authority,audit=normalized_audit,
                generation=normalized_generation,request=normalized_request,admission=normalized_admission,trajectory=trajectory)

def trajectory_provenance(root,package,admission,request,authority):
    folder=root/'first/source';trajectory=read(folder/'TRAJECTORY.json')
    require(trajectory['branch']==admission['branch'] and trajectory['coverage']==admission['coverage'],'DetachedTrajectoryAdmission')
    proof=trajectory['source_normal_and_segments']
    require(read(folder/'SOURCE_SEGMENTS.json')==proof,'DetachedTrajectoryProof')
    desc=proof['owned_segment_receipts'];summary=proof['receipt_summary'];binding=desc['binding']
    require(summary['binding']==binding and desc['chain_sha256']==summary['chain_sha256'],'TrajectoryReceiptBinding')
    require(set(desc['files'])=={'SOURCE_SEGMENTS.checkpoint.json','SOURCE_SEGMENTS.jsonl'},'TrajectoryReceiptPopulation')
    for name,pin in desc['files'].items():require(sha(folder/name)==pin,'TrajectoryReceiptHash:'+name)
    expected=dict(geometry_identity=authority['geometry_identity'],request_identity=identity(request),
        request_file_sha256=sha(root/'first/REQUEST.json'),surface_identity=authority['surface_identity'],
        start=array_hash(np.load(root/'plan/wall.npy',allow_pickle=False)),
        end=array_hash(np.load(root/'plan/first_front.npy',allow_pickle=False)),topology_hash=authority['topology_hash'],
        source_release_sha256=sha(package/'v4/runtime/RELEASE.json'))
    for k,v in expected.items():require(binding[k]==v,'TrajectoryBinding:'+k)
    chain='0'*64;semantic=hashlib.sha256();rows=0;footer=False
    with (folder/'SOURCE_SEGMENTS.jsonl').open('rb') as stream:
        for index,raw in enumerate(stream):
            require(len(raw)<=2097152 and not footer,'TrajectoryRecordBounds')
            row=json.loads(raw);require(row.pop('previous_sha256')==chain,'TrajectoryHashOrder')
            chain=hashlib.sha256(bytes.fromhex(chain)+raw).hexdigest()
            if index==0:
                require(row['kind']=='header' and row['binding']==binding,'TrajectoryHeader')
                row=copy.deepcopy(row);row['binding']={k:v for k,v in binding.items() if k not in ('request_identity','request_file_sha256','surface_identity')}
            elif row['kind']=='row':
                require(row['index']==rows,'TrajectoryRowOrder');rows+=1
            else:
                require(row==dict(kind='complete',completed_rows=rows,expected_rows=binding['segments']),'TrajectoryFooter');footer=True
            semantic.update(identity(row).encode('ascii'))
    require(footer and rows==binding['segments']==summary['completed_rows']==summary['expected_rows'] and summary['state']=='complete' and chain==desc['chain_sha256'],'TrajectoryCompleteChain')
    checkpoint=read(folder/'SOURCE_SEGMENTS.checkpoint.json')
    require(checkpoint==dict(schema='V4SegmentCheckpoint/1',committed_bytes=(folder/'SOURCE_SEGMENTS.jsonl').stat().st_size,
        chain_sha256=chain,completed_rows=rows,state='complete',binding_sha256=identity(binding)),'TrajectoryCheckpoint')
    normalized=copy.deepcopy(trajectory)
    normalized['branch']['source_files']={package_path(k,package):v for k,v in trajectory['branch']['source_files'].items()}
    normalized['coverage']['surface_identity']='<own_surface>'
    owned=normalized['source_normal_and_segments']['owned_segment_receipts'];summ=normalized['source_normal_and_segments']['receipt_summary']
    for target in (owned['binding'],summ['binding']):
        for k in ('request_identity','request_file_sha256','surface_identity'):target[k]='<own_bound_reference>'
    for target in (owned,summ):target['chain_sha256']='<authenticated_chain>'
    owned['files']={k:'<authenticated_file>' for k in owned['files']}
    normalized['ledger_semantic_identity']=semantic.hexdigest()
    return normalized

def parent_chain(root):
    bg=root/'background';parent=read(bg/'PARENT.json');current=read(root/'CURRENT.json');shell=read(root/'shell/ACCEPTED_SHELL.json')
    require(parent==read(bg/'PARENT_EXPECTATIONS.json')['binding']==read(bg/'RESULT.json')['parent']==read(bg/'EVIDENCE_HANDOFF.json')['binding']==read(bg/'JOINED_RECONSTRUCTION.json')['provenance']['binding']==read(bg/'conditioning/CONDITIONING.json')['parent'],'OwnParentChain')
    require(shell['mesh']==current['mesh']==parent['parent_prism_mesh'],'OwnShellToken')
    require(shell['source']==parent['source_identity']==read(root/'first/source/ADMISSION.json')['admission']['geometry_identity'],'OwnSourceIdentity')
    require(shell['native']==parent['native_identity'],'OwnNativeIdentity')
    require(parent['case_sha256']==sha(root/'CASE.json'),'OwnCaseHash')
    require(parent['policy_identity']==identity(read(bg/'POLICY.json'))==identity(read(root/'CASE.json')['background']),'OwnPolicy')
    require(current['sha256']==sha(root/'MESH.npz'),'OwnMeshByteHash')
    positions=np.load(root/'plan/positions.npy',allow_pickle=False)
    require(array_hash(positions)==parent['actual_positions_sha256']==current['coordinate_hash'],'OwnCoordinates')
    require(sha(root/'native/SEALED_PLAN.npz')==read(root/'native/SEALED_PLAN.json')['sha256'],'OwnSealedPlan')
    with np.load(root/'MESH.npz',allow_pickle=False) as m,np.load(root/'native/SEALED_PLAN.npz',allow_pickle=False) as s,np.load(bg/'JOINED_EVIDENCE.npz',allow_pickle=False) as j:
        require(set(m.files)=={'points','cell_vertices','cell_ids','source_columns','layers','outer','wall'},'MeshPopulation')
        require(set(s.files)=={'positions','faces','delta'},'SealedPopulation')
        require(s['positions'].tobytes()==positions.tobytes()==m['points'].tobytes(),'SealedPositions')
        for a,b in [('cell_vertices','prisms'),('cell_ids','prism_ids'),('wall','wall'),('outer','cap')]:
            require(m[a].dtype==j[b].dtype and m[a].shape==j[b].shape and m[a].tobytes()==j[b].tobytes(),'JoinedPrefix:'+a)
        require(j['points'][:len(m['points'])].tobytes()==m['points'].tobytes(),'JoinedCoordinates')
    return parent

def manifest(root,expected):
    folder=root/'background'
    require(sha(folder/'MANIFEST.json')==expected,'TrustedManifestRoot')
    files=read(folder/'MANIFEST.json')['files'];resolved=set()
    for name,digest in files.items():
        path=(folder/name).resolve()
        require(path.is_relative_to(folder.resolve()) and path not in resolved,'ManifestEscapeOrAlias')
        resolved.add(path);require(sha(path)==digest,'ManifestMember:'+name)
    require({'PARENT.json','PARENT_EXPECTATIONS.json','EVIDENCE_HANDOFF.json','JOINED_RECONSTRUCTION.json','conditioning/CONDITIONING.json','COMPLETE_MESH.npz'}<=set(files),'MissingParentWitness')

def completion(root,expected,mode):
    require(sha(root/'VERIFICATION.json')==expected,'ChangedClosedVerificationRoot')
    value=read(root/'VERIFICATION.json');descriptor=value['finalization']
    names={'CASE.json','RESULT.json','background/RESULT.json'}
    if mode=='audit':names.add('background/READBACK.json')
    require(set(value['completion_files'])==names,'CompletionPopulation')
    for name,pin in value['completion_files'].items():require(sha(root/name)==pin,'CompletionFile:'+name)
    require(sha(root/'FINALIZATION_HANDOFF.json')==value['handoff_sha256'],'HandoffByteHash')
    request=read(root/'FINALIZATION_HANDOFF.json')
    require(request['files']==value['completion_files'] and request['finalization']==descriptor and request['verification_policy']==mode
        and request['schema']==('V35AuditGuardHandoff/1' if mode=='audit' else 'V35ProductionGuardHandoff/1'),'HandoffBinding')
    require(sha(root/'background/FINALIZATION.json')==descriptor['finalization_sha256'],'FinalizationByteHash')
    require(sha(root/'background/MANIFEST.json')==descriptor['manifest_sha256'],'VerificationManifestBinding')
    require(set(value['closure_records'])=={'source','supervisor'},'ClosureRecordPopulation')
    run=read(BASE/('outputs/PASSAGE_VERIFY1_'+mode.upper()+'_RUN.json'))
    closure={}
    for name,key in [('source','source_record'),('supervisor','supervisor_record')]:
        binding=value['closure_records'][name];path=Path(binding['path'])
        require(path==Path(run[key]) and path.resolve().is_relative_to((PKG/'run_logs').resolve()),'DetachedClosurePath:'+name)
        require(sha(path)==binding['sha256'],'ClosureByteHash:'+name)
        closure[name]=read(path)
    sources=closure['source'];supervisor=closure['supervisor']
    require(sources['state']=='closed' and sources['passed'] is True and not sources['changed'] and not sources['added'],'SourceClosureRefused')
    require(sources['identity']==supervisor['source_identity']==value['source_identity']==run['runtime_identity'],'ClosureSourceIdentity')
    require(supervisor['exit_code']==0 and supervisor['stop_reason'] is None and supervisor.get('monitor_error') is None,'SupervisorClosureRefused')
    if mode=='production':
        require(set(value['closure_files'])=={'closure/SOURCES.json','closure/SUPERVISOR.json'},'ClosureCopyPopulation')
        for name,key in [('closure/SOURCES.json','source'),('closure/SUPERVISOR.json','supervisor')]:
            require(sha(root/name)==value['closure_files'][name]==value['closure_records'][key]['sha256'] and read(root/name)==closure[key],'DetachedClosureCopy')
    return descriptor

def baseline_closure():
    pins={'source':('070bd53d3ebf.SOURCES.json','b96da92984311884bc23ccd6097570dab57012fb1b97e58c05bf8162031dfa93'),
          'supervisor':('070bd53d3ebf.json','3adbed228ea31fc9cd87bef85f578338d0bd97126ac86c23a678570ad4195bb2')}
    evidence=BASE/'outputs/NODAL5_COMPONENT_RESULT_REVIEW.json'
    require(sha(evidence)=='d744adf4027b0b88d1dac9682ddd3bc96b0067401bde6118b1b4753732b280f7','ChangedBaselineClosureReview')
    records={}
    for key,(name,digest) in pins.items():
        path=ORIGINAL/'run_logs'/name
        require(sha(path)==digest,'BaselineClosureByteHash:'+key);records[key]=read(path)
    s=records['source'];r=records['supervisor']
    require(s['state']=='closed' and s['passed'] is True and not s['changed'] and not s['added'],'BaselineSourceClosure')
    require(s['identity']==r['source_identity']=='5e38fe034bb6f90fd23f5fee854293960e3ead35c289450ebe8d4f5a935f6666','BaselineRuntimeClosureIdentity')
    require(r['exit_code']==0 and r['stop_reason'] is None and r.get('monitor_error') is None,'BaselineSupervisorClosure')
    return s['identity']

def native_raw(folder):
    require({p.name for p in folder.iterdir()}==RAW_NAMES,'NativeRawPopulation')
    result={}
    for name in sorted(RAW_NAMES):
        raw=(folder/name).read_bytes();dtype=np.dtype({'f64':'<f8','i32':'<i4','u64':'<u8','u8':'u1'}[name.split('.')[-1]])
        require(len(raw)%dtype.itemsize==0,'MalformedRaw:'+name)
        result[name]=np.frombuffer(raw,dtype=dtype)
    return result

def native_identity(root):
    raw=native_raw(root/'native/postProcessing/stagedLayer/prefix_5')
    with np.load(root/'native/SEALED_PLAN.npz',allow_pickle=False) as s,np.load(root/'MESH.npz',allow_pickle=False) as m:
        positions=s['positions'];points=raw['points.f64'].reshape(-1,3);level=raw['point_level.i32'];anc=raw['point_ancestor.i32'];n=positions.shape[1]
        pm=level.astype('<i8')*n+anc
        require(len(pm)==positions.shape[0]*n and np.array_equal(np.sort(pm),np.arange(len(pm))),'NativePointBijection')
        require(np.all((anc>=0)&(anc<n)&(level>=0)&(level<len(positions))),'NativePointRange')
        require(points.tobytes()==positions[level,anc].tobytes()==m['points'][pm].tobytes(),'NativeCoordinates')
        ordered=np.empty_like(positions);ordered[level,anc]=points
        cl=raw['cell_level.i32'];ca=raw['cell_ancestor.i32'];cp=raw['cell_points.i32'];co=raw['cell_offsets.u64']
        cell_vertices=m['cell_vertices'];cell_ids=m['cell_ids'];layers=m['layers'];columns=m['source_columns'];planned_faces=s['faces']
        lookup={tuple(map(int,k)):i for i,k in enumerate(zip(layers,columns))}
        require(len(lookup)==len(cell_ids)==len(cl)==len(ca),'CanonicalCellKeyPopulation')
        keys=list(zip(map(int,cl),map(int,ca)));require(len(set(keys))==len(keys) and set(keys)==set(lookup),'NativeCellBijection')
        require(np.all((cl>=1)&(cl<len(positions))&(ca>=0)&(ca<len(planned_faces))),'NativeCellAncestryRange')
        require(len(co)==len(cl)+1 and co[0]==0 and co[-1]==len(cp) and np.all(np.diff(co)==6) and np.all((cp>=0)&(cp<len(pm))),'NativeCellOffsets')
        metadata=read(root/'background/JOINED_RECONSTRUCTION.json')['cell_metadata'][:len(cell_ids)]
        require(len(metadata)==len(cell_ids),'PrismMetadataPopulation')
        for i,record in enumerate(metadata):
            require(record['kind']=='prism' and record['layer']==int(layers[i]) and record['column']==int(columns[i]),'CanonicalPrismMetadataID')
        for i,key in enumerate(keys):
            actual=set(pm[cp[int(co[i]):int(co[i+1])]])
            expected=set(np.concatenate(((cl[i]-1)*n+planned_faces[ca[i]],cl[i]*n+planned_faces[ca[i]])))
            require(actual==expected==set(cell_vertices[lookup[key]]),'NativeCellVertices')
        fo=raw['face_offsets.u64'];fp=raw['face_points.i32'];owner=raw['owner.i32'];neighbour=raw['neighbour.i32']
        require(len(fo)==len(owner)+1 and fo[0]==0 and fo[-1]==len(fp) and np.all(fo[1:]>=fo[:-1]) and np.all((fp>=0)&(fp<len(pm))),'NativeFaceOffsets')
        faces=[list(map(int,pm[fp[int(fo[i]):int(fo[i+1])]])) for i in range(len(owner))]
        boundary=(root/'native/constant/polyMesh/boundary').read_text(encoding='utf8')
        patches={}
        for name,body in re.findall(r'(\w+)\s*\{([^}]+)\}',boundary):
            if name=='FoamFile':continue
            require(name not in patches,'DuplicateNativePatch')
            def field(k,pattern):
                rows=re.findall(r'\b'+k+r'\s+('+pattern+r')\s*;',body)
                require(len(rows)==1,'NativePatchField:'+k);return rows[0]
            patches[name]=(int(field('startFace',r'\d+')),int(field('nFaces',r'\d+')),field('type',r'\w+'))
        F=len(s['faces']);require(patches=={'outerFront':(len(neighbour),F,'patch'),'physicalWall':(len(neighbour)+F,F,'wall')} and len(owner)==len(neighbour)+2*F,'NativePatchCoverage')
        digest=identity(dict(points=array_hash(points),positions=array_hash(ordered),faces=identity(faces),owners=array_hash(owner),neighbours=array_hash(neighbour),patches=patches))
        require(digest==read(root/'shell/ACCEPTED_SHELL.json')['native'],'NativePayloadIdentity')
        return digest

def authenticate_pair(a,b,mode):
    pre=read(BASE/'outputs/PASSAGE_VERIFY1_PREFLIGHT.json')
    require(sha(BASE/'outputs/PASSAGE_VERIFY1_PREFLIGHT.json')==PREFLIGHT,'ChangedPreflight')
    require(sha(BASE/'outputs/PASSAGE_VERIFY1_AUDIT_COMPARISON.json')==OLD_FAILURE,'ChangedFirstFailure')
    require(sha(BASE/'work/passage_verify1_compare.py')=='1f4a1f7da23d9aa3fbd83bbc2b344ed005a03d31cb50a779a272c287db8348d1','ChangedOriginalComparator')
    plan=BASE/'outputs/PASSAGE_VERIFY2_AUTHENTICATION_PLAN_REVIEW.json'
    require(sha(plan)=='39a8e0e6fa564ea0bb7439ee7feb3cc59e201475e6b101f40ed995297dfe2cdb','ChangedAuthenticationPlan')
    for name,digest in read(plan)['pins'].items():
        path=Path(name);path=path if path.is_absolute() else BASE/path
        require(sha(path)==digest,'ChangedCapturedPlanInput:'+name)
    require(sha(Path(pre['outputs']['audit'])/'VERIFICATION.json')==AUDIT_ROOT,'ChangedTrustedAuditRoot')
    # Baseline roots were retained independently before this comparison.
    for rel,pin in pre['baseline_pins'].items():require(sha(Path(pre['baseline'])/rel)==pin,'ChangedFrozenRoot:'+rel)
    baseline_closure()
    for root in (a,b):
        if root==Path(pre['baseline']):manifest(root,pre['baseline_pins']['background\\MANIFEST.json'])
        else:
            expected=AUDIT_ROOT if root==Path(pre['outputs']['audit']) else read(BASE/'outputs/PASSAGE_VERIFY1_PRODUCTION_RUN.json')['closed_verification_sha256']
            descriptor=completion(root,expected,'audit' if root==Path(pre['outputs']['audit']) else 'production')
            manifest(root,descriptor['manifest_sha256'])
            if root==Path(pre['outputs']['production']):
                retained=read(BASE/'outputs/PASSAGE_VERIFY1_PRODUCTION_RUN.json')['closed_upstream_sha256']
                require(set(retained)==set(UPSTREAM_FILES),'ClosedUpstreamPopulation')
                for name,digest in retained.items():require(sha(root/name)==digest,'ChangedClosedUpstream:'+name)
    pa,pb=parent_chain(a),parent_chain(b)
    require({k:v for k,v in pa.items() if k not in ('case_sha256','parent_prism_mesh')}=={k:v for k,v in pb.items() if k not in ('case_sha256','parent_prism_mesh')},'PhysicalParentDiffers')
    left=surface_provenance(a,PKG,mode)
    bmode='baseline' if b==Path(pre['baseline']) else 'audit'
    right=surface_provenance(b,ORIGINAL if bmode=='baseline' else PKG,bmode)
    for k in left:require(left[k]==right[k],'ProvenanceSemantics:'+k)
    for relative in ('MESH.npz','native/SEALED_PLAN.npz','background/JOINED_EVIDENCE.npz'):
        exact_arrays(a/relative,b/relative)
    # Full source-segment proofs are data equality only; no predicate rerun.
    require(read(a/'shell/SOURCE_SEGMENTS.json')==read(b/'shell/SOURCE_SEGMENTS.json'),'SourceSegmentProofs')
    for prefix in (2,3,4,5):
        ra=native_raw(a/f'native/postProcessing/stagedLayer/prefix_{prefix}');rb=native_raw(b/f'native/postProcessing/stagedLayer/prefix_{prefix}')
        for name in RAW_NAMES:require(ra[name].dtype==rb[name].dtype and ra[name].shape==rb[name].shape and ra[name].tobytes()==rb[name].tobytes(),'NativeRawDiffers:'+str(prefix)+'/'+name)
    require(native_identity(a)==native_identity(b),'NativeIdentityDiffers')
    return dict(own_parent_chains_authenticated=True,surface_request_source_provenance_authenticated=True,
        physical_parent_source_identity_exact=True,prism_shell_native_joined_arrays_exact=True,
        native_raw_files_exact=88,source_segment_proofs_exact=True,ephemeral_token_recomputed=False,
        authority_transfer=False,cross_run_exclusions=['parent.case_sha256: own CASE authenticated','parent.parent_prism_mesh: own trusted chain authenticated'],
        limitation='Stored roots and exact physical payloads only; full ephemeral token preimage was not retained. No new scientific or live acceptance.')
