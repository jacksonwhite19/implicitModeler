"""Reduce measured cohort evidence without treating unevaluated stages as passes."""
from pathlib import Path
import json
from broader_geometry_cohort import trigger_audit,bind

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/broader_mixed_geometry_end_to_end'


def read(path):
    return json.loads(path.read_text())


def save(path,value):
    path.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')


def run():
    cases=read(OUT/'COHORT_MANIFEST.json')['cases']; rows=[]
    for case in cases:
        name=case['name']; folder=OUT/name/'A'
        if name=='SF2':
            replay=read(OUT/name/'BOUNDED_REPLAY.json')
            assert replay['passed']
            assert 'Mesh OK.' in (OUT/name/'checkMesh.stdout').read_text()
            receipts=read(OUT/name/'SHELL_QUALITY_EXTREMA.json')
            rows.append(dict(name=name,scope=case['scope'],expected_remesh=False,
                remesh_count=0,shell_cells=replay['audit']['cells'],complete_cells=3776026,
                complete_passed=True,complete_scope='Sealed full authority plus fresh native replay and Foundation precheck',
                shell_relations=replay['audit']['certificate']['active'],complete_relations=226619419,
                replay_exact=True,foundation_mesh_ok=True,numerics='Sealed qualified A/A2 reused',
                shell_receipt_determinant_min=min(v['extrema']['determinant']['min'] for v in receipts.values()),
                shell_touching_weight_min=min(v['extrema']['weight']['min'] for v in receipts.values()),
                performance=replay))
            continue
        shell=read(folder/'SHELL_RESULT.json'); replay=read(folder/'BOUNDED_GEOMETRY_REPLAY.json')
        current=read(folder/'checkpoints/CURRENT.json')
        checkpoint=read(folder/'checkpoints'/current['checkpoint'])
        shell_relations=checkpoint['components']['certificate']['count']
        assert shell['restart_exact'] and replay['passed']
        core_folder=folder/('core_all_volumes' if (folder/'core_all_volumes').exists() else 'core')
        core=read(core_folder/'GENERATION.json')
        assembly=read(folder/'ASSEMBLY.json') if (folder/'ASSEMBLY.json').exists() else None
        failure=read(folder/'FIXED_SHELL_COUPLING_OBSTRUCTION.json') if (folder/'FIXED_SHELL_COUPLING_OBSTRUCTION.json').exists() else None
        assert assembly is not None or failure is not None,'Unexplained missing assembly'
        other=OUT/name/'B/SHELL_RESULT.json'
        fresh_replay=None
        if other.exists():
            fresh_replay=read(other)['final']==shell['final']
            assert fresh_replay
        core_replay=read(OUT/name/'CORE_REPLAY.json')
        assert all(core_replay.values())
        quality=read(folder/'assembly/QUALITY.json') if assembly else read(folder/'COMPLETE_QUALITY_FAILURE.json')
        shell_count=sum(shell['cells'].values())
        shell_ids=set(range(shell_count))
        row=dict(name=name,scope=case['scope'],expected_remesh='no remesh' not in case['roles'],
            remesh_count=shell['remesh_count'],shell_cells=shell_count,shell_cell_types=shell['cells'],
            column_states=shell['column_states'],interface_faces=len(shell['interface']['shell_owners']),
            scientific=shell['final'],replay_exact=True,fresh_shell_replay=fresh_replay,
            core_replay_exact=True,core=core,complete_passed=bool(assembly and assembly['passed']),
            complete_cells=assembly['total_cells'] if assembly else None,
            attempted_complete_cells=shell_count+core['tets'],
            shell_relations=shell_relations,
            complete_relations=assembly['total_relations'] if assembly else None,
            complete_shell_determinant_min=min(c['determinant'] for c in quality['cells'] if c['id'] in shell_ids),
            shell_touching_weight_min=min(f['weight'] for f in quality['faces'] if any(c in shell_ids for c in f['owners'])),
            assembly=assembly,blocker=failure,foundation_mesh_ok=None,
            numerics='Not evaluated: no authoritative case controls',
            performance=read(folder/'PERFORMANCE.json'))
        rows.append(row)
    audit=trigger_audit(rows)
    for row in rows:
        save(OUT/row['name']/'CASE_RESULT.json',row)
        operations=OUT/row['name']/'A/OPERATIONS.json'
        if operations.exists():
            save(OUT/row['name']/'TRIGGER_TRACE.json',[
                dict(operation=r['operation'],policy=r['policy']) for r in read(operations)])
    result=dict(classification='V4BroaderMixedGeometryEndToEndPartiallyQualified',cases=rows,
        trigger_audit=audit,cohort_count=len(rows),production_domains=1,bounded_domains=len(rows)-1,
        complete_geometry_passes=sum(r['complete_passed'] for r in rows),
        numerical_qualified_reused=1,numerical_fresh=0,numerical_not_evaluated=len(rows)-1,
        foundation_fresh_mesh_ok=1,compression_only_complete_control='Not established',
        adjusted_schedule_shell_passes=1,large_non_SF2_production_domain='Not established',
        claim='SF2 production authority is preserved. Three distinct mixed bounded shells and one ordinary bounded shell are complete-geometry qualified. General-SDF complete assembly is blocked by a frozen shell determinant. No broader CFD accuracy claim.',
        next_task='Resolve the frozen coupled-shell determinant under a separately authorized shell/remesh qualification; then qualify authoritative non-SF2 numerical cases.')
    resources=(OUT/'SF2/checkMesh.resources').read_text()
    peak_kib=int(next(line.split(':')[-1] for line in resources.splitlines() if 'Maximum resident set size' in line))
    peaks=[peak_kib*1024]
    for row in rows:
        for source in ('performance','core','assembly'):
            peaks.append((row.get(source) or {}).get('peak_rss_bytes',0))
    result['maximum_fresh_peak_rss_bytes']=max(peaks)
    result['largest_qualified_complete_cells']=max(r['complete_cells'] or 0 for r in rows)
    result['largest_qualified_complete_relations']=max(r['complete_relations'] or 0 for r in rows)
    result['fresh_case_stage_seconds']={r['name']:sum((r.get(k) or {}).get('seconds',0)
        for k in ('performance','core','assembly')) for r in rows}
    result['runtime_scope']='Fresh A shell/replay plus core plus successful assembly. SF2 precheck 100.35 s is additional. Rejected attempts and B replay have separate logs; no successful general-SDF assembly time exists.'
    prior_core=ROOT/'results/fixed_interface_bulk_core_repair/FINAL_CORE.json'
    core_authority=bind(prior_core,prior_core.parent/'AUTHORITY.json')
    core_minima=[read(prior_core)['core']['minimum']]+[r['core']['core']['minimum'] for r in rows if 'core' in r]
    result['extrema']=dict(
        worst_shell_determinant_including_failed_assembly=min(r.get('complete_shell_determinant_min',r.get('shell_receipt_determinant_min')) for r in rows),
        shell_touching_weight_min=min(r['shell_touching_weight_min'] for r in rows),
        core_determinant_min=min(r['determinant'] for r in core_minima),
        core_weight_min=min(r['interpolation_weight'] for r in core_minima),
        SF2_core_reused_authority=core_authority)
    save(OUT/'RESULTS.json',result)
    save(OUT/'TRIGGER_AUDIT.json',audit)
    save(OUT/'DETERMINISM.json',{r['name']:dict(native_restart=r['replay_exact'],
        fresh_shell_replay=r.get('fresh_shell_replay'),core_replay=r.get('core_replay_exact'),
        scope=r['complete_scope'] if r['name']=='SF2' else 'bounded geometry and native restart') for r in rows})
    return result


if __name__=='__main__':run()
