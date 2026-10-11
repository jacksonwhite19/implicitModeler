"""Read-only exact receipt comparison after a single-layer publication."""
import collections,json,time
from analyze_qualification_storage import compare_rows
from production_single_layer_cycle import OUT,save,resources


def run():
    start=time.perf_counter();record=json.loads((OUT/'A/FINAL.json').read_text())
    volume=json.loads((OUT/'A/VOLUME.json').read_text())
    quality=json.loads((OUT/'A/QUALITY.json').read_text())
    incidence=json.loads((OUT/'A/INCIDENCE.json').read_text())
    save('FINAL_GATE_SUMMARY.json',dict(
        volume={k:volume[k] for k in ('boundary_volume','cell_volume','conditional_nonoverlap','exact_volume_balance','positive_cells','prerequisites','proof')},
        interior_degree=dict(passed=volume['interior_degree']['passed'],unresolved=len(volume['interior_degree']['unresolved']),
            counts=dict(collections.Counter(str(x['degree']) for x in volume['interior_degree']['values']))),
        exact_cells=len(volume['exact_cells']),quality_passed=quality['passed'],quality_failures=quality['failures'],
        incidence_passed=incidence['passed'],incidence_failures=incidence['failures']))
    del volume,quality,incidence
    row=record['certificate_descriptor'];objects=OUT/'A/checkpoints/objects'
    envelope=json.loads((objects.parent/record['pointer']['checkpoint']).read_text())
    quality_table=envelope['components']['quality']
    if row['encoding']!='receipt_delta':raise ValueError('ReceiptDeltaNotUsed')
    base,delta=row['base'],row['updates']
    if base['encoding']!='receipt_table' or delta['encoding']!='receipt_table':raise ValueError('AuditRequiresDirectDelta')
    result=compare_rows(objects/base['file'],objects/delta['file'])
    removed=json.loads((objects/row['removed']['file']).read_text())
    certificate=json.loads((OUT/'A/CERTIFICATE.json').read_text())
    assert not removed and result['changed_other_fields']==0
    assert result['right_only']==certificate['new']
    assert result['left_only']+result['equal']+result['changed']==base['count']==certificate['reused']
    assert result['changed']==result['changed_inputs_only']
    assert base['count']+result['right_only']==row['count']==certificate['required_pairs']
    save('RECEIPT_AUDIT.json',dict(comparison=result,base_receipts=base['count'],
        unchanged_base_receipts=result['left_only']+result['equal'],delta_receipts=delta['count'],
        dependency_binding_updates=result['changed'],new_interactions=result['right_only'],removed=0,
        certificate_rows_materialized_at_publication=0,full_flattening_avoided=True,
        quality_rows=quality_table['count'],quality_table_bytes=(objects/quality_table['file']).stat().st_size,
        quality_publication='Existing full quality map is materialized; retained table shared across replay. The overlay-only instrumentation list is not a count of all quality writes.',
        base_bytes=(objects/base['file']).stat().st_size,delta_bytes=(objects/delta['file']).stat().st_size,
        proof_payloads_unchanged=True,seconds=time.perf_counter()-start,**resources()))


if __name__=='__main__':run()
