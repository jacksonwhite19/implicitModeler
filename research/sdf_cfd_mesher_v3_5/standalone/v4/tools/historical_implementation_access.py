"""Reviewed, guard-held resolution of historical enrollment dependencies.

Original source pins/content are never replaced. Two bounded source relations
are supported: forecast-cache scope extension, and the reader's explicit
historical-source/lifetime hooks. This is not a general source-hash allowlist.
"""
import ast
from contextlib import ExitStack,contextmanager
from contextvars import ContextVar
from copy import deepcopy
from pathlib import Path
from path_preserving_deduplication import file_hash
from sdf_cfd_mesher_v4.common.authority import identity

_active=ContextVar('historical_implementation_access',default=None)
SCHEMA='V4HistoricalEnrollmentSourceAccess/1'


def _replace(text,old,new):
    if text.count(old)!=1:raise ValueError('UnsupportedHistoricalSourceRelation')
    return text.replace(old,new)


def source_relation(role,original,current):
    """Compare the entire executable AST after only the declared tiny extension."""
    if role=='forecast_cache_scope':
        replacements=(
            ('def __init__(self,adapter):','def __init__(self,adapter,*,last_layer=None):'),
            ('self.initial_key=cache_key(adapter.front,adapter.schedule,adapter.sdf)',
             'self.last_layer=last_layer\n        self.initial_key=cache_key(adapter.front,adapter.schedule,adapter.sdf,last_layer=last_layer)'),
            ('cache_key(adapter.front,adapter.schedule,adapter.sdf)!=self.initial_key',
             'cache_key(adapter.front,adapter.schedule,adapter.sdf,last_layer=self.last_layer)!=self.initial_key'),
            ('self.cache.forecast(adapter.front,adapter.topology,adapter.schedule,adapter.sdf)',
             'self.cache.forecast(adapter.front,adapter.topology,adapter.schedule,adapter.sdf,last_layer=self.last_layer)'))
    elif role=='reader_historical_access':
        replacements=(
            ('self._verified=set();self.closed=False','self._verified=set();self.closed=False;self._implementation_access=[]'),
            ("if file_hash(p)!=h:raise ValueError('AccessImplementationChanged')",
             "from historical_implementation_access import check_pinned\n                proof=check_pinned(p,h,self.body['content_root'],'AccessImplementationChanged')\n                if proof is not None and proof not in self._implementation_access:self._implementation_access.append(proof)"),
            ("def _live(self):\n        if self.closed:raise ValueError('ClosedImmutableAccess')",
             "def _live(self):\n        if self.closed:raise ValueError('ClosedImmutableAccess')\n        for proof in self._implementation_access:proof.require_open()"),
            ("def check(self):\n        self._live()",
             "def check(self):\n        self._live()\n        for proof in self._implementation_access:proof.check()"),
            ("def _verify_read(self,path):\n        if self.closed:raise ValueError('ClosedImmutableAccess')",
             "def _verify_read(self,path):\n        self._live()"),
            ("def __getitem__(self,key):\n        if self.closed:raise ValueError('ClosedImmutableAccess')",
             "def __getitem__(self,key):\n        self._live()"))
    else:raise ValueError('UnsupportedHistoricalSourceRole')
    for old,new in replacements:original=_replace(original,old,new)
    if ast.dump(ast.parse(original),include_attributes=False)!=ast.dump(ast.parse(current),include_attributes=False):
        raise ValueError('UnqualifiedHistoricalSourceDifference')
    return dict(role=role,whole_executable_ast_checked=True,
        scope='Enrollment adapter binding unchanged; current execution separately source-bound; sealed content/payloads unchanged')


def derive(native_reference,archives):
    """Instance-bound derivation; no production authorization or state writes."""
    import immutable_state_access as access
    import recursive_epoch_recovery as recovery
    import production_forecast_admission as forecast
    if set(archives)!={'reader_historical_access','forecast_cache_scope'}:
        raise ValueError('UnsupportedHistoricalSourceRole')
    with ExitStack() as held:
        old=recovery.sealed_body(native_reference,held)
        if native_reference['kind']!='native' or old['schema']!=access.SCHEMA or identity(old['content'])!=old['content_root']:
            raise ValueError('NativeEnrollmentSourceRequired')
        rows=[]
        for module,role in ((access,'reader_historical_access'),(forecast,'forecast_cache_scope')):
            current=Path(module.__file__).resolve();historical=Path(archives[role]).resolve()
            expected=old['content']['implementation'].get(str(current))
            for path in (current,historical):held.enter_context(access.guard(path))
            if expected is None or file_hash(historical)!=expected:raise ValueError('HistoricalSourcePinMismatch')
            relation=source_relation(role,historical.read_text(encoding='utf-8'),current.read_text(encoding='utf-8'))
            rows.append(dict(current=str(current),current_sha256=file_hash(current),historical=str(historical),
                historical_sha256=expected,relation=relation))
        if set(old['content']['implementation'])!={r['current'] for r in rows}:
            raise ValueError('UnsupportedHistoricalImplementationSet')
        root=Path(native_reference['directory'])/('ROOT_'+native_reference['root']+'.json')
        return dict(schema=SCHEMA,native_reference=deepcopy(native_reference),native_root_sha256=file_hash(root),
            content_root=old['content_root'],original_implementation=old['content']['implementation'],rows=rows,
            checker=str(Path(__file__).resolve()),checker_sha256=file_hash(__file__),
            semantics='Historical pins stay historical; current reader/enrollment binding qualified by whole-source relation; new execution retains current source identity')


class SourceAccess:
    def __init__(self,record):
        self.record=record;self.root=identity(record);self.closed=False
        self.rows={(r['current'],r['historical_sha256']):r for r in record['rows']}
        if len(self.rows)!=len(record['rows']):raise ValueError('DuplicateHistoricalSource')
        self.rows_identity=identity(sorted((p,h,r) for (p,h),r in self.rows.items()))

    def require_open(self):
        if self.closed:raise ValueError('ClosedHistoricalSourceAccess')

    def check(self):
        self.require_open()
        if identity(self.record)!=self.root:raise ValueError('AlteredHistoricalSourceAccess')
        if identity(sorted((p,h,r) for (p,h),r in self.rows.items()))!=self.rows_identity:
            raise ValueError('AlteredHistoricalSourceMapping')
        # All files are held under deny-write handles for the entire scope.

    def resolve(self,path,expected,actual,content):
        self.check()
        if content!=self.record['content_root']:raise ValueError('WrongHistoricalEnrollmentContent')
        row=self.rows.get((str(Path(path).resolve()),expected))
        if row is None or actual!=row['current_sha256']:raise ValueError('UnboundHistoricalSourceChange')
        return self


def check_pinned(path,expected,content,error):
    actual=file_hash(path)
    if actual==expected:return None
    proof=_active.get()
    if proof is None:raise ValueError(error)
    return proof.resolve(path,expected,actual,content)


@contextmanager
def qualified_source_access(record_path,record_identity,review_path,review_sha256):
    import immutable_state_access as access
    if _active.get() is not None:raise ValueError('NestedHistoricalSourceAccess')
    with ExitStack() as held:
        for p in (record_path,review_path,__file__):held.enter_context(access.guard(p))
        record=access.read(record_path);review=access.read(review_path)
        if (identity(record)!=record_identity or record.get('schema')!=SCHEMA
            or file_hash(review_path)!=review_sha256 or review.get('passed') is not True
            or not review.get('reviewer') or review.get('record')!=record_identity
            or review.get('record_sha256')!=file_hash(record_path)):
            raise ValueError('ReviewedHistoricalSourceAccessRequired')
        for row in record['rows']:
            for key,pin in (('current','current_sha256'),('historical','historical_sha256')):
                held.enter_context(access.guard(row[key]))
                if file_hash(row[key])!=row[pin]:raise ValueError('ChangedHistoricalSourceDependency')
        if record['checker']!=str(Path(__file__).resolve()) or record['checker_sha256']!=file_hash(__file__):
            raise ValueError('ChangedHistoricalSourceChecker')
        actual=derive(record['native_reference'],{r['relation']['role']:r['historical'] for r in record['rows']})
        if actual!=record:raise ValueError('AlteredHistoricalSourceDerivation')
        proof=SourceAccess(record);token=_active.set(proof)
        try:
            yield proof
            proof.check()
        finally:
            proof.closed=True;_active.reset(token)
