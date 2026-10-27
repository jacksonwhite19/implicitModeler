"""Synthetic control of the real native entry, stopped before generation."""
from pathlib import Path
from copy import deepcopy
import sys,json
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import background_diagnostic as backend
from finite_support import retained_background_contract as contract
from background_backend_method import request_method,validate_readback
import numpy as np
folder=Path(sys.argv[1]);request=json.loads((folder/'REQUEST.json').read_text())
with np.load(folder/'INPUT.npz',allow_pickle=False) as data:arrays={name:data[name] for name in data.files}
cases=[]
def require(value):
    if not value:raise ValueError('ControlAssertion')
def case(label,call,refuse=False):
    error=None
    try:call()
    except (ValueError,KeyError,TypeError,FileNotFoundError,FileExistsError) as failure:error=repr(failure)
    passed=(error is not None) if refuse else error is None
    row=dict(label=label,passed=passed,error=error);cases.append(row)
    (folder/('CASE_'+str(len(cases))+'.json')).write_text(json.dumps(row));require(passed)
case('actual_linux_python310',lambda:require(sys.platform=='linux' and sys.version_info[:2]==(3,10)))
external=next(name for name in request['source_pins'] if name.startswith('/etc/'))
for label,change in (
    ('linux_external_tampered_pin',lambda value:value['source_pins'].__setitem__(external,'0'*64)),
    ('linux_external_missing_pin',lambda value:value['source_pins'].pop(external)),
    ('missing_context',lambda value:value.pop('context')),
    ('stale_launch',lambda value:value.__setitem__('expires_utc','2020-01-01T00:00:00+00:00'))):
    bad=deepcopy(request);change(bad)
    case(label,lambda bad=bad:contract.verify(bad,arrays,folder,backend.V4),True)
case('production_rejects_diagnostic_request',lambda:request_method(request),True)
entries=[]
def tripwire(*args,**kwargs):
    entries.append(True)
    raise RuntimeError('ExpectedStopBeforeGeneration')
backend.DirectExecution=tripwire
case('actual_native_accepted_path',lambda:require(backend.run(folder)==1))
result=json.loads((folder/'BACKEND.json').read_text())
case('accepted_path_no_generation',lambda:require(len(entries)==1 and 'ExpectedStopBeforeGeneration' in result['error'] and not (folder/'generated').exists()))
authentication=result['source_authentication']['before_generation']
case('linux_authenticated_all_expected_pins',lambda:require(authentication['complete'] and authentication['pending_linux']==[] and external in authentication['verified'] and authentication['verified']==sorted(request['source_pins'])))
expired=deepcopy(request);expired['expires_utc']='2020-01-01T00:00:00+00:00'
case('post_expiry_integrity_not_launch',lambda:require(contract.verify(expired,arrays,folder,backend.V4,fresh=False)['complete']))
positive=contract.receipt_fields(request,contract.sha(folder/'REQUEST.json'))
positive.update(diagnostic_passed=True,files={name:'c'*64 for name in ('NETGEN.npz','OUTER.npz','CORE.npz','GMSH.npz')},
    loaded_source_closure=dict(unpinned=[]),source_authentication=dict(before_generation=authentication,after_generation=authentication))
case('synthetic_receipt_bound_readback',lambda:contract.verify_receipt(positive,expired,contract.sha(folder/'REQUEST.json')))
bad=deepcopy(positive);bad.pop('source_authentication')
case('missing_authentication_receipt_rejected',lambda:contract.verify_receipt(bad,request,contract.sha(folder/'REQUEST.json')),True)
case('stale_receipt_request_binding_rejected',lambda:contract.verify_receipt(positive,request,'b'*64),True)
case('production_rejects_diagnostic_receipt',lambda:validate_readback(positive,{}),True)
case('one_use_refuses_second_entry',lambda:contract.consume(folder,request),True)
(folder.parent/'NATIVE_RESULT.json').write_text(json.dumps(dict(passed=True,cases=cases,generation_calls=0,constructor_tripwire_calls=len(entries),source_authentication=authentication),indent=2))

