"""Preserve compact performance evidence before retiring private controls."""
from pathlib import Path
import shutil
import capsule_user_domain_handoff as h
from path_preserving_deduplication import metadata

OUT=Path(__file__).resolve().parents[1]/'results/prism_assembly_optimization'
MODES=('baseline','candidate','baseline_clean','candidate_replay','candidate_confirm')


def collect():
    controls={}
    for mode in MODES:
        folder=OUT/('normal_'+mode+'_prism')
        names=['NORMAL','REOPEN','GENERATION_PHASES','GENERATION','ASSEMBLY','INDEX_MEASURED',
               'INCIDENCE_MEASURED','QUALITY_SQL_PROFILE','SQL_COMMITS','PUBLICATION','REFERENCE']
        names += [stage+suffix for stage in ('admission','generation','assembly','publication')
                  for suffix in ('_MEASURED','_MEMORY')]
        record={name:h.read(folder/(name+'.json')) for name in names}
        volume=h.read(folder/'VOLUME_DELTA.json');degree=volume.pop('global_degree')
        record['volume']=dict(volume,degree_identity=h.identity(degree),witnesses=len(degree['values']),
                              unresolved=len(degree['unresolved']),degree_passed=degree['passed'])
        record['certificate']=h.read(folder/'FRESH_CERTIFICATE.json')
        record['admission']=h.read(folder/'ADMISSION.json')
        if mode in ('baseline_clean','candidate_replay','candidate_confirm'):
            comparison=h.read(OUT/(folder.name+'_EQUIVALENCE.json'))
            if not comparison['passed']:raise ValueError('UnqualifiedControl')
            record['comparison']=comparison
        controls[mode]=record
    if controls['candidate_replay']['NORMAL']['implementation']!=controls['candidate_confirm']['NORMAL']['implementation']:
        raise ValueError('ReplayImplementationMismatch')
    for name,digest in controls['candidate_replay']['NORMAL']['implementation'].items():
        if h.file_hash(Path(__file__).parent/name)!=digest:raise ValueError('ChangedQualifiedImplementation')
    h.durable_json(OUT/'FINAL_CONTROL_EVIDENCE.json',controls)
    for phase in ('admission','generation','assembly','publication'):
        for suffix in ('.json','.prof'):
            source=OUT/'current_optimized_prism'/(phase+'_PROFILE'+suffix)
            shutil.copyfile(source,OUT/('BASELINE_'+phase+'_PROFILE'+suffix))
    directories=['current_optimized_prism','raw_shared_optimized_core','quality_writer_baseline',
                 'quality_writer_v2_baseline','quality_writer_v2_adaptive']+['normal_'+m+'_prism' for m in MODES]
    targets=[OUT/name for name in ('baseline_proof_index.sqlite','candidate_proof_index.sqlite')]
    for name in directories:targets.extend(p for p in (OUT/name).rglob('*') if p.is_file())
    entries=[]
    for path in sorted(targets):
        path=path.resolve()
        if not path.is_relative_to(OUT.resolve()):raise ValueError('CleanupOutsideTask')
        m=metadata(path)
        if m['links']!=1:raise ValueError('UnexpectedControlHardlink')
        entries.append(dict(path=str(path.relative_to(OUT)),bytes=m['bytes'],allocation=m['cluster_bytes'],sha256=h.file_hash(path)))
    h.durable_json(OUT/'RETIREMENT_MANIFEST.json',dict(condition='Controls sealed, compared, fresh-reopened; production authorities untouched',
        directories=directories,files=entries,allocation=sum(r['allocation'] for r in entries),
        scientific_state='Private duplicate controls only; retained epoch metadata references original production ancestry'))


if __name__=='__main__':collect()
