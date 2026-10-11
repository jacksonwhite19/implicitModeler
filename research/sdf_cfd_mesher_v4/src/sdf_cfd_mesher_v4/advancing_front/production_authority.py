"""Single default runner selection, gated by sealed production qualification."""
import json
from ..common.authority import ROOT,file_hash
from .mixed_front_runner import MixedFrontRunner

CONFIG=ROOT/'authorities/PRODUCTION_RUNNER.json'

def configuration():
    if not CONFIG.exists():return dict(runner='legacy-reference',automatic_remesh=False)
    data=json.loads(CONFIG.read_text())
    if data['runner']!='native-mixed-front':raise ValueError('UnknownProductionAuthority')
    proof=ROOT/data['qualification']['path']
    if file_hash(proof)!=data['qualification']['sha256']:raise ValueError('ProductionQualificationChanged')
    result=json.loads(proof.read_text())
    if not result['passed'] or result['layers']!=20 or result['cells']!=2126040 or result['relations']!=56462864 or result['remesh_events']!=0:raise ValueError('IncompleteNativeProductionQualification')
    if data['automatic_remesh']:
        remesh=data['remesh_qualification']
        if file_hash(ROOT/remesh['path'])!=remesh['sha256']:raise ValueError('RemeshQualificationChanged')
    return data

def production_runner(path,provider):
    if configuration()['runner']!='native-mixed-front':raise ValueError('NativeProductionNotQualified')
    return MixedFrontRunner(path,provider)
