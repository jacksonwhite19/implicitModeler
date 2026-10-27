"""Retained background diagnostics never grant live admission."""
from pathlib import Path
from datetime import datetime,timezone
import json,math,sys
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from finite_support.native.native_runtime import sha
from finite_support.retained_native_contract import path,digest
REQUEST='RetainedBackgroundDiagnosticRequest/1'
RECEIPT='RetainedBackgroundDiagnosticReceipt/1'
CONTEXT='RETAINED_BACKGROUND_DIAGNOSTIC'
ARRAYS=('points','triangles','seeds')
def validate(request,*,fresh=True):
    fields={'schema','context','policy','policy_identity','prefix_identity','premises','premises_identity',
        'source_pins','source_release_sha256','owner_bindings','owner_bindings_identity',
        'input_sha256','array_identities','backend_seconds','expires_utc','extent_reached'}
    if type(request) is not dict or set(request)!=fields or request['schema']!=REQUEST or request['context']!=CONTEXT:
        raise ValueError('ExactRetainedBackgroundRequestRequired')
    if request['extent_reached'] is not False:raise ValueError('UnchangedUnreachedExtentRequired')
    policy=request['policy']
    from finite_support.retained_native_contract import validate_request
    from finite_research.flexible_controls import validate as controls
    controls(policy['tet_transition'])
    if policy['schema']!='V4GradedBackground/6' or identity(policy)!=request['policy_identity']:
        raise ValueError('ChangedBackgroundPolicy')
    from background_backend_method import HXT
    if policy['backend_method']!=HXT or policy['interface_seed_policy']!='balanced_centroid_volume/1':
        raise ValueError('ChangedBackgroundMethodOrSeeds')
    if policy['bounds_m']!=[[-.07,.07]]*3:raise ValueError('ChangedAuthorizedWorld')
    for name in ('growth_per_m','maximum_size_m'):
        if type(policy[name]) not in (int,float) or not math.isfinite(policy[name]) or policy[name]<=0:raise ValueError('InvalidExplicitSizing')
    for name in ('input_sha256','source_release_sha256','prefix_identity','policy_identity'):
        if not digest(request[name]):raise ValueError('InvalidDiagnosticIdentity',name)
    if set(request['array_identities'])!=set(ARRAYS) or not all(digest(value) for value in request['array_identities'].values()):
        raise ValueError('ExactBackgroundArraysRequired')
    for field in ('premises','owner_bindings'):
        rows=request[field]
        if not rows or identity(rows)!=request[field+'_identity']:raise ValueError('ChangedDiagnosticBindings',field)
        for row in rows.values():
            if set(row)!={'path','sha256','bytes'} or not digest(row['sha256']) or type(row['bytes']) is not int:
                raise ValueError('InvalidDiagnosticReference')
    if set(request['owner_bindings'])!={'task','authorization','launch_review','input_pins'}:raise ValueError('ExactOwnerBindingsRequired')
    if not request['source_pins'] or not all(digest(value) for value in request['source_pins'].values()):raise ValueError('SourcePinsRequired')
    seconds=request['backend_seconds']
    if type(seconds) not in (int,float) or not math.isfinite(seconds) or not 0<seconds<=300:raise ValueError('BackendBudget')
    deadline=datetime.fromisoformat(request['expires_utc'])
    if deadline.tzinfo is None or (fresh and datetime.now(timezone.utc)>=deadline):raise ValueError('DiagnosticDeadline')
    return request
def source_authentication(request,source):
    from finite_research.inputs import native_sources
    if sys.platform not in ('win32','linux'):raise ValueError('UnsupportedDiagnosticHost')
    expected=native_sources()
    def canonical(values):
        result={}
        for name,value in values.items():
            key=str(path(name).resolve())
            if key in result and result[key]!=value:raise ValueError('ConflictingDiagnosticAlias')
            result[key]=value
        return result
    if canonical(request['source_pins'])!=canonical(expected):raise ValueError('IncompleteOrChangedDiagnosticSourceSet')
    binding_path=Path(source)/'runtime/FINITE_NATIVE_BINDING.json'
    binding=json.loads(binding_path.read_text())
    external={row['external_path']:row['sha256'] for row in binding['files'] if 'external_path' in row}
    if sys.platform=='linux' and str(Path(sys.executable).resolve())!=binding['executable']:
        raise ValueError('ChangedDiagnosticNativeInterpreter')
    verified=[];pending=[]
    for name,expected_hash in request['source_pins'].items():
        if sys.platform=='win32' and name in external:
            if expected_hash!=external[name]:raise ValueError('ChangedExternalNativePin')
            pending.append(name)
        else:
            if sha(path(name))!=expected_hash:raise ValueError('ChangedDiagnosticSource',name)
            verified.append(name)
    return dict(host=sys.platform,source_pins_identity=identity(request['source_pins']),
        native_binding_sha256=sha(binding_path),verified=sorted(verified),pending_linux=sorted(pending),
        complete=not pending)

def verify(request,arrays,folder,source,*,fresh=True):
    validate(request,fresh=fresh)
    if sha(Path(folder)/'INPUT.npz')!=request['input_sha256'] or sha(Path(source)/'runtime/RELEASE.json')!=request['source_release_sha256']:
        raise ValueError('ChangedDiagnosticInputOrRelease')
    if set(arrays)!=set(ARRAYS) or any(array_hash(arrays[name])!=request['array_identities'][name] for name in ARRAYS):
        raise ValueError('ChangedDiagnosticArray')
    for field in ('premises','owner_bindings'):
        for row in request[field].values():
            actual=path(row['path'])
            if actual.stat().st_size!=row['bytes'] or sha(actual)!=row['sha256']:raise ValueError('ChangedDiagnosticReference',field)
    return source_authentication(request,source)
def consume(folder,request):
    validate(request)
    with (Path(folder)/'DIAGNOSTIC_ENTRY_CONSUMED.json').open('x') as stream:json.dump(dict(context=CONTEXT,request_identity=identity(request)),stream)
def receipt_fields(request,request_sha):
    validate(request,fresh=False)
    return dict(schema=RECEIPT,context=CONTEXT,passed=False,diagnostic_passed=False,
        request_sha256=request_sha,input_sha256=request['input_sha256'],prefix_identity=request['prefix_identity'],
        policy_identity=request['policy_identity'],source_release_sha256=request['source_release_sha256'],
        premises_identity=request['premises_identity'],owner_bindings_identity=request['owner_bindings_identity'],
        array_identities=request['array_identities'],extent_reached=False,scientific_acceptance=False,
        live_parent=False,lease_issued=False,publication='NOT_RUN')
def verify_native_authentication(authentication,request):
    if (authentication.get('host')!='linux' or authentication.get('complete') is not True
        or authentication.get('pending_linux')!=[]
        or authentication.get('source_pins_identity')!=identity(request['source_pins'])
        or authentication.get('verified')!=sorted(request['source_pins'])):
        raise ValueError('IncompleteNativeDiagnosticAuthentication')
    from finite_research.inputs import V4
    if authentication.get('native_binding_sha256')!=sha(V4/'runtime/FINITE_NATIVE_BINDING.json'):
        raise ValueError('ChangedNativeAuthenticationBinding')

def verify_postreadback(record,request,request_sha,receipt_sha):
    validate(request,fresh=False)
    if (record.get('schema')!='RetainedBackgroundPostReadbackAuthentication/1'
        or record.get('request_sha256')!=request_sha or record.get('receipt_sha256')!=receipt_sha
        or record.get('request_identity')!=identity(request) or record.get('generation_calls')!=0):
        raise ValueError('ChangedPostReadbackAuthenticationBinding')
    verify_native_authentication(record.get('authentication',{}),request)
    return True

def verify_receipt(receipt,request,request_sha):
    expected=receipt_fields(request,request_sha);expected['diagnostic_passed']=True
    if any(receipt.get(name)!=value for name,value in expected.items()):raise ValueError('ChangedDiagnosticReceipt')
    if any(type(receipt.get(name)) is not bool for name in ('passed','diagnostic_passed','extent_reached','scientific_acceptance','live_parent','lease_issued')):
        raise ValueError('ExactDiagnosticBooleansRequired')
    if set(receipt.get('files',{}))!={'NETGEN.npz','OUTER.npz','CORE.npz','GMSH.npz'}:raise ValueError('MissingDiagnosticOutputs')
    if not all(digest(value) for value in receipt['files'].values()):raise ValueError('InvalidOutputDigest')
    if receipt.get('loaded_source_closure',{}).get('unpinned')!=[]:raise ValueError('UnpinnedDiagnosticSource')
    for phase in ('before_generation','after_generation'):
        authentication=receipt.get('source_authentication',{}).get(phase,{})
        verify_native_authentication(authentication,request)
    return receipt
def receipt_custody(receipt_path,expected):
    if sha(receipt_path)!=expected:raise ValueError('ChangedConsumedDiagnosticReceipt')
