"""Qualification-only exact comparisons, outside normal layer timings."""
from pathlib import Path
import numpy as np
import capsule_user_domain_handoff as h

OUT=Path(__file__).resolve().parents[1]/'results/prism_assembly_optimization'
SEALED=OUT.parent/'recursive_layers9to11/L11'
EXPECTED='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870'


def compare(folder):
    tick=h.measured();checks={};hashes={}
    for name,ignored in [('FORECAST.json',()),('DIRECT.json',('seconds',)),('LOCALIZATION.json',('timings',))]:
        actual=h.read(folder/name);expected=h.read(SEALED/name)
        for value in (actual,expected):
            for key in ignored:value.pop(key,None)
        checks[name]=actual==expected
    admission=h.read(folder/'ADMISSION.json');expected_admission=h.read(SEALED/'ADMISSION.json')
    checks['admission_binding']=admission['evidence']['binding']==expected_admission['evidence']['binding']
    checks['admission_evidence']={k:v for k,v in admission['evidence'].items() if k!='authorities'}=={k:v for k,v in expected_admission['evidence'].items() if k!='authorities'}
    for value in (admission,expected_admission):value['decision'].pop('evidence_identity')
    checks['admission_decision']=admission['decision']==expected_admission['decision']
    for name in ('GEOMETRY_DELTA.json','CHANGED_INPUTS.json','NEW_QUALITY.json',
                 'AFFECTED_INCIDENCE.json','VOLUME_DELTA.json','QUALITY_DELTA.json',
                 'BINDING_PATCH.json','NEW_ATLAS.json','NEW_FORECAST_DEPENDENCIES.json',
                 'NEW_RECEIPT_INPUTS.json','PRIOR_DETERMINANT_COMPARISON.json'):
        checks[name]=h.read(folder/name)==h.read(SEALED/name)
        hashes[name]=h.file_hash(folder/name)
    for name in ('FORECAST.npz','GENERATED_NEXT.npz','LOCALIZATION.npz','SCREEN_ARRAYS.npz'):
        with np.load(folder/name) as actual,np.load(SEALED/name) as expected:
            checks[name]=set(actual.files)==set(expected.files) and all(
                actual[k].dtype==expected[k].dtype and actual[k].shape==expected[k].shape
                and actual[k].tobytes()==expected[k].tobytes() for k in actual.files)
    for name in ('GENERATION.json','FRESH_CERTIFICATE.json','ASSEMBLY.json'):
        actual=h.read(folder/name);expected=h.read(SEALED/name)
        if name=='GENERATION.json':
            # Access identity differs after explicit epoch renewal. Scientific
            # parent/content bindings and every other planner input must match.
            for value in (actual,expected):
                value.pop('seconds');value.pop('phases')
                value['transition_plan'].pop('base_root')
        elif name=='FRESH_CERTIFICATE.json':
            for value in (actual,expected):
                value.pop('seconds',None)
                value.pop('optimization',None)  # Timing/packed-buffer diagnostics only.
        else:
            for value in (actual,expected):
                value.pop('seconds')
                value['pair_accounting'].pop('seconds',None)
                value['pair_accounting'].pop('index_manifest')
                value['pair_accounting'].pop('index_root')
        checks[name]=actual==expected
    plan=h.read(folder/'CANDIDATE_PLAN.json');sealed=h.read(SEALED/'CANDIDATE_PLAN.json')
    checks['scientific_component_map']=plan['content']==sealed['content']
    checks['content_root']=plan['scientific']==EXPECTED==h.read(folder/'PUBLICATION.json')['content_root']
    checks['zero_prior_pair_scan']=h.read(folder/'ASSEMBLY.json')['qualification_compact_pair_rows_scanned']==0
    result=dict(passed=all(checks.values()),checks=checks,hashes=hashes,metrics=h.metrics(*tick),
                content=plan['scientific'],scope='Exact comparison after private candidate publication; no candidate repair from oracle')
    h.durable_json(OUT/(folder.name+'_EQUIVALENCE.json'),result)
    if not result['passed']:raise ValueError({k:v for k,v in checks.items() if not v})
    return result


if __name__=='__main__':
    import sys
    compare(OUT/sys.argv[1])
