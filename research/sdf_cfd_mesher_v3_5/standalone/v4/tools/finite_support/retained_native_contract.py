"""Separate retained initializer provenance; never live admission authority."""
from datetime import datetime,timezone
from pathlib import Path
import json,math
from sdf_cfd_mesher_v4.common.authority import identity,array_hash
from finite_support.native.native_runtime import sha as file_hash

REQUEST='RetainedDiagnosticInitializerRequest/1'
RECEIPT='RetainedDiagnosticInitializerReceipt/1'
CONTEXT='RETAINED_DIAGNOSTIC_INITIALIZER'
SCOPE='RETAINED_INITIALIZER_ONLY_NO_LIVE_OR_FLUID_AUTHORITY'
ARRAYS=('cap','faces','outer','seeds')


def digest(value):
    return type(value) is str and len(value)==64 and all(character in '0123456789abcdef' for character in value)

def path(value):
    value=str(value).replace('\\','/')
    return Path('/mnt/'+value[0].lower()+value[2:]) if value[1:2]==':' and Path('/mnt').exists() else Path(value)

def validate_request(request):
    fields={'schema','context','source_pins','input_sha256','growth_per_m','maximum_size_m','seed_policy',
        'input_points_hash','prism_centres_hash','prism_volumes_hash','seed_hash','boundary_admission_sha256',
        'control','policy','policy_identity','policy_role','array_identities','historical_inputs',
        'historical_inputs_identity','historical_source_release_sha256','source_release_sha256',
        'owner_bindings','owner_bindings_identity','backend_seconds','expires_utc','geometry_scope'}
    if type(request) is not dict or set(request)!=fields or request['schema']!=REQUEST:
        raise ValueError('ExactRetainedInitializerRequestRequired')
    if (request['context']!=CONTEXT or request['geometry_scope']!=SCOPE or request['control'] is not False
            or request['policy_role']!='SIZING_AND_TARGETS_ONLY_NOT_CONSTRUCTION_DISPATCH'):
        raise ValueError('RetainedInitializerScopeRequired')
    from finite_research.flexible_controls import validate
    policy=request['policy']
    if (type(policy) is not dict or set(policy)!={'schema','bounds_m','growth_per_m','maximum_size_m',
            'interface_seed_policy','assurance_mode','backend_method','interface_completion','tet_transition'}
            or policy['schema']!='V4GradedBackground/6'):
        raise ValueError('ExactRetainedInitializerPolicyRequired')
    validate(policy['tet_transition'])
    bounds=policy['bounds_m']
    if (type(bounds) is not list or len(bounds)!=3 or any(type(row) is not list or len(row)!=2
            or any(type(value) not in (int,float) or not math.isfinite(value) for value in row)
            or row[0]>=row[1] for row in bounds)):
        raise ValueError('InvalidRetainedInitializerBounds')
    for name in ('growth_per_m','maximum_size_m'):
        if type(policy[name]) not in (int,float) or not math.isfinite(policy[name]) or policy[name]<=0:
            raise ValueError('InvalidRetainedInitializerSizing')
    if policy['tet_transition']['envelope']['mode']!='nominal':raise ValueError('NominalDiagnosticRequired')
    if (identity(policy)!=request['policy_identity'] or request['growth_per_m']!=policy['growth_per_m']
            or request['maximum_size_m']!=policy['maximum_size_m'] or request['seed_policy']!=policy['interface_seed_policy']):
        raise ValueError('ChangedRetainedInitializerSizing')
    if set(request['array_identities'])!=set(ARRAYS) or any(not digest(value) for value in request['array_identities'].values()):
        raise ValueError('ExactRetainedInitializerArraysRequired')
    if request['seed_hash']!=request['array_identities']['seeds']:raise ValueError('ChangedRetainedInitializerSeeds')
    for name in ('input_sha256','input_points_hash','prism_centres_hash','prism_volumes_hash','seed_hash',
            'boundary_admission_sha256','policy_identity','historical_inputs_identity','historical_source_release_sha256',
            'source_release_sha256','owner_bindings_identity'):
        if not digest(request[name]):raise ValueError('InvalidRetainedInitializerIdentity',name)
    historical=request['historical_inputs']
    if type(historical) is not dict or not historical or identity(historical)!=request['historical_inputs_identity']:
        raise ValueError('ChangedRetainedInitializerHistory')
    for name,row in historical.items():
        if (type(name) is not str or type(row) is not dict or set(row)!={'path','sha256','bytes'}
                or type(row['path']) is not str or not digest(row['sha256']) or type(row['bytes']) is not int or row['bytes']<0):
            raise ValueError('InvalidRetainedInitializerHistoricalInput')
    owners=request['owner_bindings']
    if (type(owners) is not dict or set(owners)!={'task','authorization','launch_review','input_pins'}
            or identity(owners)!=request['owner_bindings_identity']):
        raise ValueError('ChangedRetainedInitializerOwner')
    for row in owners.values():
        if type(row) is not dict or set(row)!={'path','sha256'} or type(row['path']) is not str or not digest(row['sha256']):
            raise ValueError('InvalidRetainedInitializerOwnerBinding')
    if type(request['source_pins']) is not dict or not request['source_pins'] or any(not digest(value) for value in request['source_pins'].values()):
        raise ValueError('RetainedInitializerSourcePinsRequired')
    seconds=request['backend_seconds']
    if type(seconds) not in (int,float) or not math.isfinite(seconds) or not 0<seconds<=300:
        raise ValueError('RetainedInitializerBackendBudget')
    deadline=datetime.fromisoformat(request['expires_utc'])
    if deadline.tzinfo is None or datetime.now(timezone.utc)>=deadline:raise ValueError('RetainedInitializerDeadline')
    return request

def verify_inputs(request,arrays,folder,source):
    validate_request(request);folder=Path(folder);source=Path(source)
    if file_hash(folder/'INPUT.npz')!=request['input_sha256']:raise ValueError('ChangedRetainedInitializerInput')
    if file_hash(source/'runtime/RELEASE.json')!=request['source_release_sha256']:raise ValueError('ChangedRetainedInitializerRelease')
    if set(arrays)!=set(ARRAYS) or any(array_hash(arrays[name])!=request['array_identities'][name] for name in ARRAYS):
        raise ValueError('ChangedRetainedInitializerArray')
    for row in request['historical_inputs'].values():
        actual=path(row['path'])
        if actual.stat().st_size!=row['bytes'] or file_hash(actual)!=row['sha256']:raise ValueError('ChangedRetainedInitializerHistoricalBytes')
    for row in request['owner_bindings'].values():
        if file_hash(path(row['path']))!=row['sha256']:raise ValueError('ChangedRetainedInitializerOwnerBytes')
    if file_hash(folder.parent/'BOUNDARY_ADMISSION.json')!=request['boundary_admission_sha256']:
        raise ValueError('ChangedRetainedInitializerBoundary')
    return True

def consume(folder,request):
    validate_request(request)
    with (Path(folder)/'DIAGNOSTIC_ENTRY_CONSUMED.json').open('x',encoding='utf8') as stream:
        json.dump(dict(schema=CONTEXT,request_identity=identity(request)),stream)

def receipt_fields(request,request_sha256):
    validate_request(request)
    return dict(schema=RECEIPT,context=CONTEXT,geometry_scope=SCOPE,request_sha256=request_sha256,
        input_sha256=request['input_sha256'],source_release_sha256=request['source_release_sha256'],
        historical_inputs_identity=request['historical_inputs_identity'],owner_bindings_identity=request['owner_bindings_identity'],
        policy_identity=request['policy_identity'],array_identities=request['array_identities'],
        live_parent=False,scientific_acceptance=False,fluid_proof='NOT_PERFORMED',lease_issued=False)

def validate_receipt(receipt,request,request_sha256):
    expected=receipt_fields(request,request_sha256)
    if receipt.get('passed') is not True or any(receipt.get(name)!=value for name,value in expected.items()):
        raise ValueError('ChangedRetainedInitializerReceipt')
    if set(receipt.get('files',{}))!={'CORE.npz','OUTER.npz','GMSH.npz','NETGEN.npz'}:
        raise ValueError('IncompleteRetainedInitializerOutputs')
    if any(not digest(value) for value in receipt['files'].values()):raise ValueError('InvalidRetainedInitializerOutputIdentity')
    if receipt.get('loaded_source_closure',{}).get('unpinned')!=[]:raise ValueError('UnpinnedRetainedInitializerOutput')
    return receipt

def reject_production(receipt):
    if receipt.get('schema')==RECEIPT or receipt.get('context')==CONTEXT:
        raise ValueError('RetainedDiagnosticReceiptCannotAdmitProduction')
