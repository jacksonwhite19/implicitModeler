"""Owned bounded first-front receipts; no geometry or new scientific predicate."""
from pathlib import Path
from contextlib import ExitStack,nullcontext
import json,os,hashlib
import numpy as np
from owned_runtime import V4
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,file_hash
from sdf_cfd_mesher_v4.sdf.segment_receipts import SegmentReceiptSink,verify,verify_checkpoint,ReceiptLimitError,_tree,validate_proof,rational,RECORD_BYTES
from path_preserving_deduplication import durable_json

SUMMARY='SOURCE_SEGMENTS.json'
LEDGER='SOURCE_SEGMENTS.jsonl'
CHECKPOINT='SOURCE_SEGMENTS.checkpoint.json'
FILES=(SUMMARY,LEDGER,CHECKPOINT)
ARCHIVE='SOURCE_SEGMENT_EVIDENCE'

def read(path):
    path=Path(path)
    if path.name not in (SUMMARY,'ARCHIVE.json'):return json.loads(path.read_text(encoding='utf-8'))
    with path.open('rb') as stream:data=stream.read(262145)
    if len(data)>262144:raise ReceiptLimitError('OwnedSegmentSummaryByteLimit')
    return json.loads(data)

def bounded_json(path,value):
    _tree(value,16384);path=Path(path);pending=path.with_suffix(path.suffix+'.pending');count=0
    with pending.open('xb') as stream:
        for token in json.JSONEncoder(sort_keys=True,separators=(',',':'),ensure_ascii=True,allow_nan=False).iterencode(value):
            data=token.encode('ascii')
            for start in range(0,len(data),8192):
                chunk=data[start:start+8192];count+=len(chunk)
                if count+1>262144:raise ReceiptLimitError('OwnedSegmentSummaryByteLimit')
                stream.write(chunk)
        stream.write(b'\n');stream.flush();os.fsync(stream.fileno())
    os.replace(pending,path)

def enabled():return read(V4/'runtime/RELEASE.json').get('owned_source_segment_receipts') is True

def fresh_required(provider):
    from general_sdf_authority import GeneralSdfAuthority
    return enabled() and type(provider.source) is GeneralSdfAuthority and provider.source.stock_prism_queries()

def binding(provider):
    from production_prism_sizing import read_request
    req=read_request(provider.request,provider.wall_authority)
    if float(req['increments'][0]).hex()!=provider.h1.hex():raise ValueError('OwnedSegmentRequestHeightMismatch')
    depth,boxes,bits=provider.source.queries._scope._budgets
    return dict(geometry_identity=provider.source.geometry_identity,start=array_hash(provider.x),end=array_hash(provider.high),
        segments=len(provider.x),first_height_m=provider.h1,
        source_expression_sha256=hashlib.sha256(provider.binding.qualification.target['expression'].encode('utf-8')).hexdigest(),
        source_release_sha256=file_hash(V4/'runtime/RELEASE.json'),maximum_depth=depth,maximum_boxes=boxes,precision_bits=bits,
        request_identity=identity(req['declaration']),request_file_sha256=file_hash(provider.request),
        surface_identity=provider.wall_authority['surface_identity'],topology_hash=array_hash(provider.tri),
        metres_per_model_unit=float(provider.source.queries._scope._scale))

def qualify(provider):
    """Called only inside existing stock query/trajectory scopes, once per front."""
    expected=binding(provider);sink=SegmentReceiptSink(provider.folder/LEDGER,expected)
    proof=provider.source.qualify_segments(provider.x,provider.high,first_height_m=provider.h1,receipt_sink=sink)
    if 'segment_proofs' in proof:raise ValueError('UnboundedOwnedSegmentProofList')
    summary=proof['receipt_summary']
    descriptor=dict(schema='V4OwnedSourceSegments/1',binding=expected,
        files={n:file_hash(provider.folder/n) for n in (LEDGER,CHECKPOINT)},chain_sha256=summary['chain_sha256'])
    proof=dict(proof,owned_segment_receipts=descriptor)
    bounded_json(provider.folder/SUMMARY,proof)
    verify_owned(provider.folder,expected)
    return proof

def verify_owned(folder,expected=None):
    """Offline readback only; guards/hash-chain authenticate the retained stream."""
    import immutable_state_access as access
    folder=Path(folder)
    with ExitStack() as held:
        for name in FILES:held.enter_context(access.guard(folder/name))
        proof=read(folder/SUMMARY);desc=proof.get('owned_segment_receipts',{})
        if desc.get('schema')!='V4OwnedSourceSegments/1' or set(desc.get('files',{}))!={LEDGER,CHECKPOINT}:
            raise ValueError('OwnedSegmentArtifactCoverage')
        bound=desc['binding']
        if expected is not None and bound!=expected:raise ValueError('OwnedSegmentBindingMismatch')
        for name,pin in desc['files'].items():
            if file_hash(folder/name)!=pin:raise ValueError('OwnedSegmentArtifactChanged',name)
        summary=proof.get('receipt_summary',{})
        if (proof.get('passed') is not True or proof.get('exact_field_progress') is not True or proof.get('positive_endpoints') is not True
            or type(proof.get('segments')) is not int or proof['segments']!=bound['segments']
            or proof.get('start')!=bound['start'] or proof.get('end')!=bound['end'] or proof.get('geometry_identity')!=bound['geometry_identity']
            or proof.get('first_source_normal_height_m')!=bound['first_height_m'] or proof.get('height_reserve_m')!=1e-12
            or type(proof.get('maximum_first_height_error_m')) not in (float,int) or not 0<=proof['maximum_first_height_error_m']<1e-12
            or summary.get('binding')!=bound or summary.get('state')!='complete' or summary.get('completed_rows')!=bound['segments']
            or summary.get('expected_rows')!=bound['segments'] or summary.get('secondary_errors') or summary.get('chain_sha256')!=desc['chain_sha256']):
            raise ValueError('IncompleteOwnedSourceSegments')
        replay=verify(folder/LEDGER,bound,require_complete=True,expected_chain=desc['chain_sha256'])
        checkpoint=verify_checkpoint(folder/LEDGER,bound,require_complete=True)
        if (replay['chain_sha256']!=checkpoint['chain_sha256'] or replay['bytes_read']!=summary['ledger_bytes']
            or replay['route_counts']!=summary['route_counts'] or replay['maximum_first_height_error_m']!=proof['maximum_first_height_error_m']):
            raise ValueError('OwnedSegmentSummaryMismatch')
        minimum=None;leaves=boxes=depth=maximum_record=0
        with (folder/LEDGER).open('rb') as stream:
            while True:
                raw=stream.readline(RECORD_BYTES+1)
                if not raw:break
                if len(raw)>RECORD_BYTES:raise ReceiptLimitError('OwnedSegmentReplayRecordLimit')
                maximum_record=max(maximum_record,len(raw));row=json.loads(raw)
                if row['kind']!='row':continue
                candidate=row['proof'];lower,_=validate_proof(candidate,bound['source_expression_sha256'])
                minimum=lower if minimum is None else min(minimum,lower)
                leaves+=candidate['segment_leaves'];boxes+=candidate['segment_boxes'];depth=max(depth,candidate['maximum_depth_used'])
        if (minimum!=rational(summary['minimum_lower_exact']) or leaves!=summary['segment_leaves'] or boxes!=summary['segment_boxes']
            or depth!=summary['maximum_depth_used'] or maximum_record!=summary['maximum_record_bytes']):
            raise ValueError('OwnedSegmentAggregateMismatch')
        return proof

def verify_provider(provider,*,required=False):
    proof=provider.assigned.get('source_normal_and_segments',{})
    if 'owned_segment_receipts' not in proof:
        if required:raise ValueError('OwnedSourceSegmentsRequired')
        return None
    verified=verify_owned(provider.folder,binding(provider))
    if proof!=verified:raise ValueError('DetachedOwnedSegmentTrajectory')
    return dict(folder=str(provider.folder),surface=str(provider.surface),request=str(provider.request),binding=verified['owned_segment_receipts']['binding'])

def extra_files(folder):
    trajectory=read(Path(folder)/'TRAJECTORY.json')
    return FILES if 'owned_segment_receipts' in trajectory.get('source_normal_and_segments',{}) else ()

def verify_retained(folder,request,surface,*,expected_release=None,case_path=None):
    """Match retained candidate arrays to its actual request/surface, no authority reopen."""
    folder=Path(folder)
    if not extra_files(folder):return None
    from production_prism_sizing import read_request
    surface=Path(surface);authority=read(surface/'SURFACE_AUTHORITY.json');req=read_request(request,authority)
    with np.load(folder/'CANDIDATE.npz',allow_pickle=False) as z:
        expected=dict(read(folder/SUMMARY)['owned_segment_receipts']['binding'])
        checked=dict(geometry_identity=authority['geometry_identity'],surface_identity=authority['surface_identity'],
            request_identity=identity(req['declaration']),request_file_sha256=file_hash(request),start=array_hash(z['wall']),end=array_hash(z['front']),
            topology_hash=array_hash(z['triangles']),segments=len(z['wall']),first_height_m=float(req['increments'][0]))
        if z['wall'].shape!=z['front'].shape or z['wall'].ndim!=2 or z['wall'].shape[1]!=3:raise ValueError('OwnedSegmentCandidateLayout')
        if array_hash(z['wall'])!=authority['coordinate_hash'] or array_hash(z['triangles'])!=authority['topology_hash']:raise ValueError('OwnedSegmentSurfaceMismatch')
        if any(expected.get(k)!=v for k,v in checked.items()):raise ValueError('OwnedSegmentFreshInputMismatch')
    if case_path is not None:
        case=read(case_path);audit=read(surface/'AUDIT.json');budgets=case['general_sdf']['proof']
        checks=dict(source_expression_sha256=hashlib.sha256(audit['acceptance_contract']['continuous_target']['expression'].encode('utf-8')).hexdigest(),
            metres_per_model_unit=case['geometry']['metres_per_model_unit'],maximum_depth=budgets['maximum_depth'],maximum_boxes=budgets['maximum_patches'],precision_bits=budgets['precision_bits'])
        if any(expected.get(k)!=v for k,v in checks.items()):raise ValueError('OwnedSegmentCaseSourceMismatch')
    if expected_release is not None and expected['source_release_sha256']!=expected_release:raise ValueError('OwnedSegmentReleaseMismatch')
    proof=verify_owned(folder,expected)
    if read(folder/'TRAJECTORY.json').get('source_normal_and_segments')!=proof:raise ValueError('DetachedOwnedSegmentTrajectory')
    return proof

def archive(context,output):
    """Preserve unique source evidence before the existing FastProd scratch retirement."""
    output=Path(output);folder=Path(context['folder']);target=output/'initial'/ARCHIVE
    verify_owned(folder,context['binding']);target.mkdir(exist_ok=False)
    for name in (*FILES,'CANDIDATE.npz','TRAJECTORY.json'):os.replace(folder/name,target/name)
    context_files={str(Path(context[k]).resolve()):file_hash(context[k]) for k in ('request',)}
    for path in (Path(context['surface'])/'SURFACE_AUTHORITY.json',Path(context['surface'])/'AUDIT.json',output/'CASE.json'):
        context_files[str(path.resolve())]=file_hash(path)
    record=dict(schema='V4OwnedSourceSegmentsArchive/1',files={n:file_hash(target/n) for n in (*FILES,'CANDIDATE.npz','TRAJECTORY.json')},
        request=str(Path(context['request']).resolve()),surface=str(Path(context['surface']).resolve()),context_files=context_files,binding=context['binding'])
    bounded_json(target/'ARCHIVE.json',record)
    verify_archive(output,context['binding']['source_release_sha256'])
    return dict(path='initial/'+ARCHIVE,archive_sha256=file_hash(target/'ARCHIVE.json'),chain_sha256=read(target/SUMMARY)['receipt_summary']['chain_sha256'])

def verify_archive(output,release,*,mesh_required=False):
    import immutable_state_access as access
    output=Path(output).resolve();folder=output/'initial'/ARCHIVE
    with ExitStack() as held:
        held.enter_context(access.guard(folder/'ARCHIVE.json'));record=read(folder/'ARCHIVE.json')
        required=set((*FILES,'CANDIDATE.npz','TRAJECTORY.json'))
        if record.get('schema')!='V4OwnedSourceSegmentsArchive/1' or set(record.get('files',{}))!=required:raise ValueError('OwnedSegmentArchiveCoverage')
        for name,pin in record['files'].items():
            held.enter_context(access.guard(folder/name))
            if file_hash(folder/name)!=pin:raise ValueError('OwnedSegmentArchiveChanged',name)
        request=Path(record['request']).resolve();surface=Path(record['surface']).resolve()
        if request!=output/'initial/REQUEST.json' or surface!=output/'surface/A/fresh':raise ValueError('OwnedSegmentArchiveContextPath')
        expected_paths={str(request),str(surface/'SURFACE_AUTHORITY.json'),str(surface/'AUDIT.json'),str(output/'CASE.json')}
        if set(record['context_files'])!=expected_paths:raise ValueError('OwnedSegmentArchiveContextCoverage')
        for path,pin in record['context_files'].items():
            held.enter_context(access.guard(path))
            if file_hash(path)!=pin:raise ValueError('OwnedSegmentArchiveContextChanged',path)
        proof=verify_retained(folder,request,surface,expected_release=release,case_path=output/'CASE.json')
        if proof is None or proof['owned_segment_receipts']['binding']!=record['binding']:raise ValueError('OwnedSegmentArchiveBinding')
        if mesh_required:
            for name in ('MESH.npz','CURRENT.json'):held.enter_context(access.guard(output/name))
            current=read(output/'CURRENT.json')
            with np.load(output/'MESH.npz',allow_pickle=False) as z:
                points=z['points'];n=record['binding']['segments']
                if (points.ndim!=2 or points.shape[1]!=3 or len(points)<2*n
                    or array_hash(points[:n])!=record['binding']['start'] or array_hash(points[n:2*n])!=record['binding']['end']
                    or current.get('coordinate_hash')!=array_hash(points) or current.get('sha256')!=file_hash(output/'MESH.npz')):
                    raise ValueError('OwnedSegmentMeshCoordinateMismatch')
        return proof


def complete_dependency(output,parent_current,points,*,expected=None,custody=None):
    """Authenticate the fresh archive against the preserved Prism prefix.

    Complete CURRENT is a different descriptor. Use the independently bound
    pre-publication Prism pointer and keep every dependency guarded through
    the caller's complete consumption/publication transaction.
    """
    import immutable_state_access as access
    output=Path(output).resolve()
    with (ExitStack() if custody is None else nullcontext(custody)) as held:
        folder=output/'initial'/ARCHIVE
        held.enter_context(access.guard(folder/'ARCHIVE.json'))
        record=read(folder/'ARCHIVE.json')
        paths=[folder/'ARCHIVE.json',output/'INITIAL_SUMMARY.json',output/'MESH.npz']
        paths.extend(folder/name for name in record['files'])
        paths.extend(Path(name) for name in record['context_files'])
        files={}
        for path in paths:
            path=path.resolve()
            if not path.is_relative_to(output):raise ValueError('CompleteSourceDependencyPath')
            held.enter_context(access.guard(path));files[path.relative_to(output).as_posix()]=file_hash(path)
        proof=verify_archive(output,file_hash(V4/'runtime/RELEASE.json'))
        summary=read(output/'INITIAL_SUMMARY.json').get('owned_source_segments',{})
        if (summary.get('path')!='initial/'+ARCHIVE or summary.get('archive_sha256')!=files['initial/'+ARCHIVE+'/ARCHIVE.json']
            or summary.get('chain_sha256')!=proof['receipt_summary']['chain_sha256']):raise ValueError('DetachedCompleteSourceArchive')
        with np.load(output/'MESH.npz',allow_pickle=False) as saved:
            prefix=saved['points'];n=record['binding']['segments']
            if (parent_current.get('schema')!='V4FastProdMeshOnly/1' or parent_current.get('sha256')!=files['MESH.npz']
                or parent_current.get('coordinate_hash')!=array_hash(prefix) or prefix.shape!=np.asarray(points).shape
                or not np.array_equal(prefix,points) or array_hash(prefix[:n])!=record['binding']['start']
                or array_hash(prefix[n:2*n])!=record['binding']['end']):raise ValueError('CompleteSourcePrismPrefixMismatch')
        value=dict(schema='V4CompleteSourceDependency/1',files=files,chain_sha256=proof['receipt_summary']['chain_sha256'],
            parent_prism_mesh=parent_current['mesh'],segments=n,source_release_sha256=file_hash(V4/'runtime/RELEASE.json'))
        if expected is not None and value!=expected:raise ValueError('ChangedCompleteSourceDependency')
        return value
