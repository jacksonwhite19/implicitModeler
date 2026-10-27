"""Best-effort failure context only; never an acceptance or proof input."""
from fractions import Fraction
import hashlib,json,math
from pathlib import Path

ATTRIBUTE = '_outward_segment_diagnostic'

def encode(value):
    """Keep exact rationals and binary floats without JSON NaN/Infinity."""
    if value is None or type(value) in (str,bool,int):return value
    if isinstance(value,Fraction):return dict(type='rational',exact=str(value))
    if type(value) is float:
        return dict(type='binary64',hex=value.hex(),finite=math.isfinite(value))
    if type(value) in (list,tuple):return [encode(v) for v in value]
    if type(value) is dict:
        if any(type(k) is not str for k in value):raise TypeError('DiagnosticStringKeysRequired')
        return {k:encode(v) for k,v in value.items()}
    raise TypeError('UnsupportedDiagnosticValue:'+type(value).__name__)

def annotate(failure,key,factory):
    """May fail; every exception-path caller must guard this operation."""
    current=dict(getattr(failure,ATTRIBUTE,{}))
    current[key]=encode(factory())
    # Validate before attaching. No diagnostic exception replaces the owner error.
    json.dumps(current,sort_keys=True,allow_nan=False)
    setattr(failure,ATTRIBUTE,current)

def arithmetic_mode():
    from .regional import _filtered
    state=_filtered.get()
    return 'exact' if state is None or state['exact'] else 'filtered'

def fallback_context(first_failure,final_failure):
    return dict(first_attempt=getattr(first_failure,ATTRIBUTE,{}),
                final_attempt=getattr(final_failure,ATTRIBUTE,{}),
                exact_fallback_invoked=True)

def record(path,failure,context):
    """Write only a failure diagnostic. Caller always preserves its exception."""
    root=Path(__file__).resolve().parents[3]
    release=root/'runtime/RELEASE.json'
    value=dict(schema='V4OutwardSegmentFailureDiagnostic/1',diagnostic_only=True,
        exception_type=type(failure).__name__,exception_args=encode(failure.args),
        proof_context=getattr(failure,ATTRIBUTE,{}),context=encode(context),
        source_release_sha256=hashlib.sha256(release.read_bytes()).hexdigest(),
        source_root=str(root),
        interpretation='Unresolved bounds do not prove an invalid segment. No predicate, budget or acceptance decision is changed.')
    Path(path).write_text(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n',encoding='utf-8')
