"""Exact final comparison, independent of a launcher's historical metadata copy."""
from task4_initial_scan import *
from task3c_io import read

def compare(expected_state_hash=None):
    a=read(OUT/'FINAL_VALIDATION.json');b=read(OUT/'replay_A2/FINAL_VALIDATION.json')
    assert (OUT/'A2_PRODUCTION_TIMING.json').exists(),'ReplayExecutionHasNotFinished'
    for result in (a,b):
        assert result['layers']==20 and result['certificate']['complete']
        assert not result['certificate']['forbidden'] and not result['certificate']['unresolved']
        assert all(row['passed'] and not any(row['violations'].values()) for row in result['gates'])
        assert result['shell_scientific_hash']==scientific_hash({k:v for k,v in result.items() if k!='shell_scientific_hash'})
        if expected_state_hash is not None and result is a:assert result['transaction_state_hash']==expected_state_hash
    passed=scientific_hash(a)==scientific_hash(b)
    result=dict(A=a['shell_scientific_hash'],A2=b['shell_scientific_hash'],passed=passed,independent_exact_proof_cache=True)
    if not passed:
        from task4c_compare_formats import compare_formats
        result=compare_formats(OUT,OUT/'replay_A2',a,b);passed=result['passed']
    if (OUT/'REPLAY_SCHEMA_REPRODUCER/REPRODUCER.json').exists():
        result['schema_normalization']=read(OUT/'REPLAY_SCHEMA_REPRODUCER/REPRODUCER.json')
    (OUT/'DETERMINISM_A_A2.json').write_text(json.dumps(result,sort_keys=True,indent=2)+'\n')
    assert passed,'IndependentReplayMismatch'
    print(json.dumps(dict(A=result['A'],A2=result['A2'],passed=True)))
    return result

if __name__=='__main__':compare()
