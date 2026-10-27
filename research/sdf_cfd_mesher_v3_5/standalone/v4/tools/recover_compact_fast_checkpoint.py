"""Recover current authority from preserved geometry and compact proofs.

Never bypass missing receipt validation: rebuild the quality digest, complete
certificate root, and whole scientific identity and require their old values.
Old result directories are read-only. No advancing layer is executed.
"""
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
import fast_milestone as m
from benchmark_production_fast import save
from production_scale_authority import cohort
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest,ENCODER,FORMAT
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs,face_key
from sdf_cfd_mesher_v4.common.authority import file_hash

def root_for(keys,values,auth):
    from sdf_cfd_mesher_v4.advancing_front import authenticated_map,canonical_blocks,authenticated_receipts
    if auth['authority']==canonical_blocks.AUTHORITY and auth['schema']==canonical_blocks.SCHEMA:
        return canonical_blocks.canonical_root(keys,values,auth['depth'])
    if auth['authority']==authenticated_receipts.AUTHORITY and auth['schema']==authenticated_map.SCHEMA:
        return authenticated_map.canonical_root(keys,values).hex()
    raise ValueError('RecoveryUnsupportedRootAuthority')


def geometry(source):
    store=NativeCheckpointStore(source);pointer=json.loads((source/'CURRENT.json').read_text())
    if pointer['format']!=FORMAT or Path(pointer['checkpoint']).name!=pointer['checkpoint']:raise ValueError('RecoveryPointerFormat')
    path=source/pointer['checkpoint']
    if file_hash(path)!=pointer['sha256']:raise ValueError('RecoveryCheckpointCorrupt')
    envelope=json.loads(path.read_text())
    if envelope['scientific']!=pointer['scientific']:raise ValueError('RecoveryEnvelopeIdentity')
    state={}
    for name,row in envelope['components'].items():
        if name in ('quality','certificate'):continue
        if row['encoding']!='json':raise ValueError('RecoveryRequiresPreservedJSONGeometry')
        state[name]=json.loads(store._object(row).read_text())
    if any(c['kind']!='prism' for c in state['cells'].values()):raise ValueError('RecoveryAllPrismScope')
    return state,envelope,pointer


def recover(source,spec,folder):
    start=time.perf_counter();state,envelope,pointer=geometry(source);times={}
    run=object.__new__(m.FastLayer);run.spec=spec;run.folder=folder;run.profile=m.ExecutionProfile.ProductionFast
    run.provider=m.AnalyticProvider(spec,folder)
    if any(state['authority'].get(k)!=v for k,v in run.provider.authority.items()):raise ValueError('RecoveryScientificImplementationChanged')
    fronts,levels=run.provider.geometry(state)
    rebuilt=run.provider.pack(fronts,levels,state)
    if any(rebuilt[k]!=state[k] for k in ('vertices','cells','faces','outer')):raise ValueError('RecoveryGeometryMismatch')
    import generalize_local_remesh as g
    from final_shell_incidence import check
    points,cells,outer=g.s.construct(run.provider.x,run.provider.tri,fronts,levels)
    wall=[tuple(reversed(f)) for f in run.provider.tri]
    owners,ownership=m.t.ownership(cells,outer,wall)
    if not ownership['passed']:raise ValueError('RecoveryOwnershipFailure')
    tick=time.perf_counter();q=m.t.quality(points,cells,owners,1);inc=check(points,cells,outer)
    if not q['passed'] or not inc['passed']:raise ValueError('RecoveryQualityFailure')
    run.points=points;run.outer=outer;run.wall=wall;run.topology=m.Topology(cells,owners)
    run.quality=m.Quality({r['id']:r for r in q['cells']},{m.key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in q['faces']},
                          {r['id']:r for r in q['exact']},{r['id']:r for r in inc['cells']})
    run.quality.bind_masks(run.topology,outer);run.volume=m.ExactVolume(points,cells,outer,run.provider.tri,run.quality.exact)
    run.degree_state=m.DegreeState(points,wall+list(outer),run.volume.witness)
    times['full_quality_volume_degree']=time.perf_counter()-tick
    run.tx=m.MemoryTransaction(dict(state,quality={},certificate={},_production_unsealed=True))
    run.inputs=geometry_inputs(state);run.vertex_incidence={}
    for k,row in state['faces'].items():
        for v in row['vertices']:run.vertex_incidence.setdefault(v,set()).add(k)
    n=len(run.provider.x)
    def source_key(v):return f'layer:{v//n}:source:{v%n}'
    faces=[owners[k][0][1] for k in sorted(owners)]
    run.face_keys=[face_key(tuple(source_key(v) for v in f)) for f in faces]
    indices={k:i for i,k in enumerate(run.face_keys)}
    descriptor=envelope['components']['certificate']
    if descriptor.get('proof_segments'):
        segments=[m.LoadedSegment(s,source/'objects',indices,True) for s in descriptor['proof_segments']]
        run.proofs=m.SegmentProofRows(segments,m.CompactProofRows())
        recovery_method='Preserved compact proofs; no exact proofs recomputed'
    else:
        # Missing historical storage is not permission to skip certification.
        # Re-certify this saved geometry once using the unchanged exact kernel;
        # continuation still requires the entire old scientific digest below.
        from native_mixed_face_certificate import certify
        tick=time.perf_counter();certificate=certify(points,faces,range(len(faces)))
        if not certificate['passed']:raise ValueError('RecoveryCompleteCertificateFailure')
        run.proofs=m.SegmentProofRows([],certificate['proofs'])
        times['saved_geometry_exact_recertification']=time.perf_counter()-tick
        recovery_method='Full exact re-certification of saved geometry; no layer construction'
    if len(run.proofs)!=descriptor['count'] or run.proofs.summary()[1]:raise ValueError('RecoveryProofPopulationFailure')
    bound=run.binding_state();tick=time.perf_counter()
    quality=scientific_digest(bound['quality']);expected=envelope['components']['quality']['scientific']
    save(folder/'QUALITY_COMPARISON.json',dict(actual=quality,expected=expected,passed=quality==expected))
    if quality!=expected:raise ValueError('RecoveryQualityReceiptDigestMismatch')
    times['quality_legacy_digest']=time.perf_counter()-tick
    root=None
    if 'authenticated' in descriptor:
        from sdf_cfd_mesher_v4.advancing_front.authenticated_receipts import buffers
        tick=time.perf_counter();keys,values,actions=buffers((k,ENCODER.encode(v),0) for k,v in bound['certificate'].items());del actions
        auth=descriptor['authenticated'];root=root_for(keys,values,auth)
        del keys,values
        save(folder/'ROOT_COMPARISON.json',dict(actual=root,expected=auth['root'],passed=root==auth['root'],count=len(run.proofs)))
        if root!=auth['root']:raise ValueError('RecoveryCertificateRootMismatch')
        times['independent_certificate_root']=time.perf_counter()-tick
        bound['certificate'].authenticated=auth
    tick=time.perf_counter()
    actual=scientific_hash(bound)
    save(folder/'WHOLE_SCIENTIFIC_COMPARISON.json',dict(actual=actual,expected=pointer['scientific'],passed=actual==pointer['scientific'],certificate_included=True))
    if actual!=pointer['scientific']:raise ValueError('RecoveryWholeScientificMismatch')
    times['whole_scientific_identity']=time.perf_counter()-tick
    from audit_capsule_scaling import final_coverage
    tick=time.perf_counter();coverage=final_coverage(points,faces,run.proofs);times['complete_coverage']=time.perf_counter()-tick
    checkpoint=folder/'checkpoint';record=m.save(run,checkpoint);restored=m.load(checkpoint,folder)
    if m.signature(restored)!=record['signature']:raise ValueError('RecoveryRestartMismatch')
    result=dict(passed=True,source=str(source),source_pointer=pointer,scientific=actual,quality_digest=quality,
                certificate_root=root,legacy_certificate_component=descriptor.get('scientific'),
                recovery_method=recovery_method,coverage=coverage,layer=record['layer'],checkpoint=str(checkpoint),
                no_layer_advanced=True,prior_files_modified=False,times=times,seconds=time.perf_counter()-start)
    save(folder/'RECOVERY.json',result);return result


def main():
    p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('source',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);started=time.perf_counter()
    try:print(json.dumps(recover(a.source,next(s for s in cohort() if s['name']==a.case),a.output)),flush=True)
    except BaseException as exc:
        save(a.output/'FAILURE.json',dict(error=str(exc),traceback=traceback.format_exc()));raise
    finally:save(a.output/'SESSION.json',dict(seconds=time.perf_counter()-started))

if __name__=='__main__':main()
