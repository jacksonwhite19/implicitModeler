"""Owned Linux source authentication after parent readback; no array decoding."""
from pathlib import Path
import sys,json
HERE=Path(__file__).resolve().parent;V4=HERE.parents[2]
sys.path[:0]=[str(V4/'tools'),str(V4/'src'),str(V4/'runtime/lib/gmsh')]
import native_runtime
native_runtime.verify()
from finite_support import retained_background_contract as contract
from sdf_cfd_mesher_v4.common.authority import identity
folder=Path(sys.argv[1]);request_sha=sys.argv[2];receipt_sha=sys.argv[3]
if contract.sha(folder/'REQUEST.json')!=request_sha or contract.sha(folder/'BACKEND.json')!=receipt_sha:
    raise ValueError('ChangedPostReadbackInputBinding')
request=json.loads((folder/'REQUEST.json').read_text());contract.validate(request,fresh=False)
authentication=contract.source_authentication(request,V4)
record=dict(schema='RetainedBackgroundPostReadbackAuthentication/1',request_sha256=request_sha,
    receipt_sha256=receipt_sha,request_identity=identity(request),generation_calls=0,authentication=authentication)
contract.verify_postreadback(record,request,request_sha,receipt_sha)
if contract.sha(folder/'REQUEST.json')!=request_sha or contract.sha(folder/'BACKEND.json')!=receipt_sha:
    raise ValueError('ChangedPostReadbackFinalCustody')
with (folder/'POST_READBACK_AUTHENTICATION.json').open('x') as stream:json.dump(record,stream,indent=2)

