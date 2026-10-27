"""Case/enrollment/dispatch boundary for the qualified ordinary-prism stack.

No advancing loop, meshing algorithm, projector or checkpoint format here.
Enrollment is independent of permission (or geometry capability) to advance.
"""
from contextlib import ExitStack, contextmanager
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import os
import numpy as np
import immutable_state_access as access
from path_preserving_deduplication import file_hash, durable_json
from sdf_cfd_mesher_v4.common.authority import identity, array_hash

PROFILE = 'V4RemainingPrismProductionCriticalPathOptimizationQualified'
CONTROLS = dict(V4_CRITICAL='1', V4_NUMERIC='1', V4_PERSISTENT='1', V4_LOOKUP='1',
    V4_FACE_KEYS='1', V4_DEGREE_OVERLAP='1', V4_DEGREE_PROCESS='1', V4_DEGREE_WORKERS='2',
    V4_INDEX_OVERLAP='1', V4_INDEX_PROCESS='1', V4_QUALITY_BATCH='1', V4_QUALITY_CODEC='4', V4_PROFILE='0')
_installed = False


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def install_qualified_stack():
    """The exact installer order in profile_remaining_prism.normal; process local."""
    global _installed
    for name, value in CONTROLS.items():
        if name in os.environ and os.environ[name] != value:
            raise ValueError('ConflictingQualifiedProfileControl:' + name)
    if not _installed:
        os.environ.update(CONTROLS)
        from critical_prism_install import install
        install()
        from optimized_admission import install
        install()
        from prism_persistent_hotpath import install
        install()
        from critical_index_pipeline import install
        install()
        import composable_publication
        from critical_transaction import Store
        composable_publication.Store = Store
        _installed = True
    import composable_admission, composable_transition, composable_assembly
    from recursive_production_publication import publish
    # Return the existing executors, not another stage loop.
    return dict(admission=composable_admission.admit, generation=composable_transition.produce,
                assembly=composable_assembly.assemble, publication=publish)


class ScalarOffsetGeometry:
    """One existing analytic owner for forecast, prediction and target checks."""
    semantics = 'ExistingAnalyticScalarOffset/1'

    def __init__(self, spec, geometry_identity, units='m'):
        from geometry_family_authority import Source
        if units != 'm':
            raise ValueError('ExecutionCoordinatesRequireMetres')
        self.spec = deepcopy(spec)
        self.geometry_identity = geometry_identity
        self.source = Source(self.spec)
        # Preserve the qualified forecast's existing cache/source identity.
        self.source_identity = self.source.source_identity
        self.spec_identity = identity(self.spec)

    def check(self):
        if identity(self.spec) != self.spec_identity or identity(self.source.spec) != self.spec_identity:
            raise ValueError('GeometryDefinitionMutation')

    def evaluate(self, points):
        self.check()
        return self.source.evaluate(points)

    def predict(self, points, target, *, iteration_cap):
        from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
        self.check()
        return predict_next_front(points, target, self.source, iteration_cap=iteration_cap)

    def validate_target(self, points, target):
        from geometry_family_authority import sdf_record
        self.check()
        # The existing geometry-owned validator internally creates its evaluator.
        # Retain that implementation and verify the exact same source definition.
        result = sdf_record(self.spec, points, target)
        if result['sdf_identity'] != self.source_identity:
            raise ValueError('TargetValidatorGeometryMismatch')
        return result

    def check_adapter(self, adapter, spec):
        self.check()
        if adapter.sdf.source is not self or adapter.geometry_binding is not self:
            raise ValueError('ForecastGenerationGeometryOwnerMismatch')
        if adapter.binding['geometry'] != self.geometry_identity or identity(spec) != self.spec_identity:
            raise ValueError('ForecastGenerationGeometryIdentityMismatch')


@contextmanager
def qualified_dispatch():
    """Qualified process-local installer plus the same bounded face-key scope."""
    executors=install_qualified_stack()
    from critical_face_keys import installed
    with installed():
        yield executors


class IncidentNormalFirstLayerGeometry:
    """Retain existing source-sheet semantics; no unqualified continuation recipe."""
    semantics = 'ExistingIncidentNormalFirstLayerOnly/1'

    def __init__(self, geometry_identity):
        self.geometry_identity = geometry_identity

    def compatibility(self, incident_normals):
        from live_spline_first_layer import existing_compatibility
        return existing_compatibility()(incident_normals)

    def predict(self, *args, **kwargs):
        raise ValueError('IncidentNormalContinuationNotQualified')

    def validate_target(self, *args, **kwargs):
        raise ValueError('IncidentNormalContinuationNotQualified')


def load_case(case):
    """Hash-bound caller inputs; no geometry name dispatch or guessed schedule."""
    from production_surface_authority import load_production_wall
    from production_prism_sizing import read_request
    if case['units'] != 'm':
        raise ValueError('ExecutionCoordinatesRequireMetres')
    for path, digest in case['sources'].items():
        if file_hash(path) != digest:
            raise ValueError('CaseSourceMismatch:' + path)
    wall, triangles, surface = load_production_wall(case['surface'])
    for key in ('geometry_identity', 'surface_identity'):
        if surface[key] != case[key]:
            raise ValueError('CaseAuthorityMismatch:' + key)
    request = read_request(case['request'], surface)
    if request['request_file_sha256'] != case['request_sha256']:
        raise ValueError('CaseRequestMismatch')
    return wall, triangles, surface, request


def inspect_front(state, wall, triangles, surface, request, expected):
    """Validate stable ordinary-prism columns without requesting a next layer."""
    if state['authority']['sdf'] != surface['geometry_identity']:
        raise ValueError('StateGeometryMismatch')
    for key, want in (('production_surface',surface['surface_identity']),
                      ('request',identity(request['declaration'])),('schedule',request['schedule_identity'])):
        if state['authority'].get(key) != want:
            raise ValueError('StateCaseMismatch:' + key)
    layer = expected['layer']; n = len(wall)
    if not 0 < layer <= len(request['increments']) or len(triangles) >= 1000000:
        raise ValueError('UnsupportedOrdinaryPrismScope')
    if array_hash(wall) != surface['coordinate_hash'] or array_hash(triangles) != surface['topology_hash']:
        raise ValueError('WallPayloadMismatch')
    if set(state['columns']) != {str(i) for i in range(len(triangles))}:
        raise ValueError('SourceColumnInventoryMismatch')
    if any(v['last_prism_layer'] != layer or v['status'] != 'ActivePrismColumn' for v in state['columns'].values()):
        raise ValueError('NonuniformOrInactiveFrontUnsupported')
    if set(state['prism_front']) != set(state['columns']):
        raise ValueError('SourceFrontInventoryMismatch')
    if len(state['vertices']) != (layer+1)*n or len(state['cells']) != layer*len(triangles):
        raise ValueError('UnsupportedOrdinaryPrismInventory')
    prefix = np.array([[state['vertices'][f'layer:{k}:source:{i}'] for i in range(n)] for k in range(layer+1)])
    if not np.array_equal(prefix[0],wall) or array_hash(prefix[-1]) != expected['front_coordinates']:
        raise ValueError('CommittedCoordinateMismatch')
    for i, tri in enumerate(triangles):
        face = state['faces'][state['prism_front'][str(i)]]
        if set(face['vertices']) != {f'layer:{layer}:source:{int(v)}' for v in tri}:
            raise ValueError('FrontSourceTopologyMismatch')
    return SimpleNamespace(front=SimpleNamespace(coordinates=prefix[-1]), committed_prefix=prefix,
        triangles=triangles, layer=layer, remaining_increments=request['increments'][layer:])


def enroll_native(case, directory):
    """Parameterization of the qualified native-access enrollment, not a checkpoint.

    Reuse native deep restart, immutable guards/attestation and the existing
    mapping schema. Only packed coordinate/front metadata is written. Proof and
    quality payloads remain at their original authenticated paths.
    """
    return _enroll_native(case,directory)


def publish_and_enroll_initial(case, directory, state, validate):
    """Fresh native publication and enrollment share its verified readback.

    The publisher still independently restores and authenticates every component
    before publishing CURRENT. Reuse that actual restored state, not the input
    candidate, for the unchanged enrollment checks. No caller-provided trust
    token, restart bypass for old checkpoints, or changed storage format.
    """
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    store=Path(case['checkpoint']).resolve()
    if store.exists() and any(store.iterdir()):raise ValueError('FreshInitialPublicationRequiresEmptyStore')
    if validate(state) is not True:raise ValueError('InitialCandidateValidationFailed')
    verified=[];restores=[]
    class ReadbackStore(NativeCheckpointStore):
        def _restore(self,pointer):
            restored=super()._restore(pointer)
            envelope=read(self.path/pointer['checkpoint'])
            restores.append((deepcopy(pointer),deepcopy(envelope['components'])))
            return restored
    def readback(restored):
        before=scientific_hash(restored)
        if validate(restored) is not True:raise ValueError('InitialPrepublicationValidationFailed')
        if before!=case['committed'] or scientific_hash(restored)!=before:raise ValueError('InitialReadbackScientificMismatch')
        verified.append(restored)
        return True
    try:
        scientific=ReadbackStore(store).publish(state,validate=readback)
        if scientific!=case['committed'] or len(verified)!=1 or len(restores)!=1:
            raise ValueError('InitialPublicationReadbackMismatch')
        pointer,components=restores[0]
        # Publication must actually install the EXACT independently restored
        # pointer. Capturing whatever CURRENT happens to say afterward is unsafe.
        if read(store/'CURRENT.json')!=pointer:raise ValueError('InitialReadbackPointerChanged')
        return _enroll_native(case,directory,initial=(verified[0],pointer,components))
    except Exception:
        # Same fresh-initialization recovery rule as MixedFrontRunner. Revoke
        # only our exact pointer, never a concurrent/replacement publication.
        if len(restores)==1:revoke_fresh_pointer(store,restores[0][0])
        raise


def revoke_fresh_pointer(store, expected):
    """Same conditional rollback, with one NTFS handle owning compare/delete.

    Deny writes and replacement across the comparison AND handle disposition.
    The exact opened file is removed on handle close, never a later pathname.
    Immutable objects and failure evidence are retained.
    """
    import ctypes
    from path_preserving_deduplication import api,W
    current=Path(store).resolve()/'CURRENT.json';k=api()
    h=k.CreateFileW(str(current),0x80010000,1,None,3,0,None)
    if h==W.HANDLE(-1).value:
        if ctypes.get_last_error() in (2,3):return False
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        if read_pointer_handle(h)!=expected:return False
        class Disposition(ctypes.Structure):_fields_=[('delete',W.BOOL)]
        d=Disposition(True)
        k.SetFileInformationByHandle.argtypes=[W.HANDLE,ctypes.c_int,ctypes.c_void_p,W.DWORD]
        if not k.SetFileInformationByHandle(h,4,ctypes.byref(d),ctypes.sizeof(d)):
            raise ctypes.WinError(ctypes.get_last_error())
        return True
    finally:k.CloseHandle(h)


def read_pointer_handle(handle):
    """Read the already exclusively pinned small pointer, not its pathname."""
    import ctypes
    from path_preserving_deduplication import api,W
    k=api();block=ctypes.create_string_buffer(4096);count=W.DWORD();parts=[]
    while True:
        if not k.ReadFile(handle,block,len(block),ctypes.byref(count),None):
            raise ctypes.WinError(ctypes.get_last_error())
        if not count.value:break
        parts.append(block.raw[:count.value])
    return json.loads(b''.join(parts).decode('utf-8'))


def _enroll_native(case, directory, initial=None):
    from rolling_layer_retention import checkpoint_closure
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore, scientific_digest
    from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
    directory = Path(directory)
    if directory.exists():
        raise ValueError('PreserveExistingEnrollment')
    store = Path(case['checkpoint']).resolve()
    seal_path=Path(case['front_qualification'])
    if case['sources'].get(str(seal_path.resolve())) != file_hash(seal_path):
        raise ValueError('UnboundFrontQualification')
    seal=read(seal_path)
    accepted=(seal.get('first_layer_qualified') is True if initial is None else
        seal.get('candidate_geometry_qualified') is True and seal.get('first_layer_qualified') is False
        and seal.get('publication_complete') is False and seal.get('classification')=='V4ProductionInitialCandidateQualified')
    if (not accepted or
        identity(seal['scientific']) != seal['first_layer_identity'] or
        seal['committed_front_identity'] != case['committed'] or
        seal['scientific']['surface'] != case['surface_identity'] or
        seal['geometry'] != case['geometry_identity']):
        raise ValueError('FrontQualificationMismatch')
    refs, envelope = checkpoint_closure(store)
    if envelope['scientific'] != case['committed']:
        raise ValueError('SealedStateMismatch')
    refs.update(case['sources'])
    with ExitStack() as held:
        for path in refs:
            held.enter_context(access.guard(path))
        wall, triangles, surface, request = load_case(case)
        if initial is None:
            state = NativeCheckpointStore(store).restart()
        else:
            state,pointer,components=initial
            if read(store/'CURRENT.json')!=pointer or envelope['components']!=components:
                raise ValueError('InitialReadbackPointerChanged')
        if scientific_hash(state) != case['committed']:
            raise ValueError('DeepSealedStateMismatch')
        expected = dict(committed=case['committed'],geometry_identity=surface['geometry_identity'],
            surface_identity=surface['surface_identity'],request=identity(request['declaration']),
            request_file=request['request_file_sha256'],schedule=request['schedule_identity'],
            layer=case['layer'],front_coordinates=case['front_coordinates'])
        inspected = inspect_front(state,wall,triangles,surface,request,expected)
        objects = directory/'objects'; objects.mkdir(parents=True,exist_ok=False)
        def object_file(name,value,array=False):
            path=objects/name
            if array:
                with path.open('wb') as stream:
                    np.save(stream,value,allow_pickle=False);stream.flush();os.fsync(stream.fileno())
            else:
                durable_json(path,value)
            return dict(path=str(path.resolve()),sha256=file_hash(path))
        keys=sorted(state['vertices']); values=np.asarray([state['vertices'][k] for k in keys],dtype='<f8')
        if scientific_digest(access.VertexMap(keys,values)) != scientific_digest(state['vertices']):
            raise ValueError('PackedCoordinateMismatch')
        hot=dict(vertex_keys=object_file('keys.json',keys),vertex_values=object_file('coordinates.npy',values,True),
            front_faces=object_file('front_faces.json',{k:state['faces'][k] for k in state['prism_front'].values()}))
        for row in hot.values():refs[row['path']]=row['sha256']
        pointer=read(store/'CURRENT.json')
        content=dict(schema='V4LegacyComponentMapping/1',legacy_scientific=case['committed'],
            legacy_checkpoint_sha256=pointer['sha256'],legacy_components=envelope['components'],
            hot={k:v['sha256'] for k,v in hot.items()},
            implementation={str(Path(access.__file__).resolve()):file_hash(access.__file__),
                            str(Path(__file__).resolve()):file_hash(__file__)},
            receipt_roles='Existing immutable native components; no proof re-encoding',
            quality='Original authenticated quality; enrollment grants no advance',case_identity=identity(case))
        body=dict(schema=access.SCHEMA,complete=True,epoch=access.epoch(),store=str(store),
            legacy_pointer=pointer,components=envelope['components'],hot=hot,
            files={p:access.attest(p,h) for p,h in refs.items()},content=content,content_root=identity(content),
            counts={k:len(state[k]) for k in ('vertices','cells','faces','quality','certificate')},expected=expected,
            binding=dict(case=identity(case),remaining_increments=len(inspected.remaining_increments),advance_authorized=False))
        root=access.publish_manifest(directory,body)
    reference=dict(kind='native',directory=str(directory.resolve()),root=root)
    from composable_state import CommittedState
    with CommittedState(reference) as reopened:
        if reopened.scientific != case['committed']:
            raise ValueError('EnrolledScientificMismatch')
        reopened.check()
    return reference


def build_access_indices(base, inspected, directory):
    """Reuse exact numeric topology, version closure, lookup and pair enrollment."""
    from build_persistent_incidence import build_for_state
    from audit_parent_version_closure import bind_for_state
    from persistent_numeric_incidence import PackedSegment
    from prism_source_lookup import write_lookup
    from incremental_pair_storage import build_incidence
    directory=Path(directory)
    record=build_for_state(base,inspected.triangles,len(inspected.front.coordinates),directory)
    version_path=directory/'VERSION.json'
    version_root=bind_for_state(base,record['root'],version_path)
    with PackedSegment(directory/'parent_incidence',record['root'],base.content_root) as index:
        lookup=write_lookup(directory/'lookup',index.ordered_items(),len(inspected.front.coordinates),record['root'],base.content_root)
    pair_root=build_incidence(base,directory/'pairs')
    return dict(directory=str((directory/'parent_incidence').resolve()),root=record['root'],
        version_path=str(version_path.resolve()),version_root=version_root,lookup=lookup,
        pair_path=str((directory/'pairs/MANIFEST.json').resolve()),pair_root=pair_root)


@contextmanager
def indexed_session(base, inspected, indices):
    from prism_persistent_hotpath import bind_bootstrap, geometry_session
    from incremental_pair_storage import PairIncidence
    bind_bootstrap(base,indices['directory'],indices['root'],indices['version_path'],indices['version_root'],indices['lookup'])
    with geometry_session(base,inspected) as geometry,PairIncidence(base,indices['pair_path'],indices['pair_root']) as pairs:
        yield geometry,pairs


def bind_forecast(base,case,geometry):
    """No forecast execution. Preserve the original schedule and cache identities."""
    wall,triangles,surface,request=load_case(case)
    if geometry.geometry_identity != surface['geometry_identity']:
        raise ValueError('BoundGeometryMismatch')
    if base.expected['committed'] != case['committed']:
        raise ValueError('SealedStateMismatch')
    if base.layer >= len(request['increments']):
        raise ValueError('NoPendingIncrement')
    if type(geometry) is not ScalarOffsetGeometry:
        raise ValueError('IncidentNormalContinuationNotQualified')
    definition_path=Path(case['geometry_definition'])
    if case['sources'].get(str(definition_path.resolve())) != file_hash(definition_path):
        raise ValueError('UnboundGeometryDefinition')
    definition=read(definition_path)
    if definition['geometry_identity'] != geometry.geometry_identity or geometry.spec != dict(components=[definition['definition']]):
        raise ValueError('GeometryDefinitionMismatch')
    adapter=base.initialize(wall,triangles,surface,case['request'],geometry)
    adapter.geometry_binding=geometry
    geometry.check_adapter(adapter,geometry.spec)
    return adapter


def predict_bound(adapter,spec,points,target,*,iteration_cap):
    """The generation connection; no default geometry on this explicit route."""
    geometry=adapter.geometry_binding
    geometry.check_adapter(adapter,spec)
    prediction=geometry.predict(points,target,iteration_cap=iteration_cap)
    return prediction,geometry.validate_target(prediction.coordinates,target)
