"""Immutable component transaction mechanics. Scientific validation is explicit.

This module never generates or certifies geometry. A pinned acceptance record
must bind the complete component plan and every changed record before publish.
"""
from contextlib import ExitStack,contextmanager
from pathlib import Path
from collections import Counter
import ctypes,hashlib,json,os,sqlite3,zlib
import immutable_state_access as access
from path_preserving_deduplication import api,W,durable_json,file_hash,atomic_replace
from sdf_cfd_mesher_v4.common.authority import identity

SCHEMA='V4IncrementalComponentTransaction/1'
def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()

@contextmanager
def writer(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    k=api();h=k.CreateFileW(str((directory/'WRITER.lock').resolve()),0xC0000000,0,None,4,0,None)
    if h==W.HANDLE(-1).value:raise ValueError('ConcurrentIncrementalWriter')
    try:yield
    finally:k.CloseHandle(h)

class Store:
    def __init__(self,path,object_directory=None):
        self.path=Path(path);self.objects=Path(object_directory) if object_directory is not None else self.path/'objects';self.stats=Counter()
    def current(self):return access.read(self.path/'CURRENT.json')
    def root(self,expected):
        p=self.current()
        if p['root']!=expected:raise ValueError('StaleIncrementalBase')
        if p['file']!='ROOT_'+expected+'.json':raise ValueError('WrongIncrementalRootPath')
        path=self.path/p['file']
        with access.guard(path):
            if file_hash(path)!=p['sha256']:raise ValueError('CorruptIncrementalRoot')
            body=access.read(path)
        if identity(body)!=expected or body['schema']!=SCHEMA:raise ValueError('WrongIncrementalRoot')
        return body
    def initialize(self,body):
        with writer(self.path):
            if (self.path/'CURRENT.json').exists():raise ValueError('IncrementalStoreAlreadyInitialized')
            self._switch(dict(body,schema=SCHEMA),None)
        return self.current()['root']
    def _switch(self,body,interrupt):
        root=identity(body);p=self.path/('ROOT_'+root+'.json')
        durable_json(p,body)
        with access.guard(p):
            if identity(access.read(p))!=root:raise ValueError('IncompleteNewRoot')
            if interrupt:interrupt('root_written')
            pointer=dict(root=root,file=p.name,sha256=file_hash(p))
            if interrupt:interrupt('before_pointer')
            durable_json(self.path/'CURRENT.json',pointer)
            if interrupt:interrupt('after_pointer')
        return root
    def publish(self,base,plan,quality_rows,binding_patch,acceptance,expected_acceptance,interrupt=None):
        """Commit only the delta, with original immutable objects referenced.

        acceptance is produced by a scientific authority, not manufactured by
        this writer. The control driver independently compares sealed L4/L5.
        """
        if identity(acceptance)!=expected_acceptance or acceptance.get('passed') is not True:raise ValueError('UnboundAcceptance')
        if acceptance['base']!=base or acceptance['plan']!=identity(plan):raise ValueError('WrongAcceptedPlan')
        if identity(quality_rows)!=acceptance['quality_delta'] or identity(binding_patch)!=acceptance['binding_patch']:
            raise ValueError('ChangedDeltaAfterAcceptance')
        self.stats['delta_canonical_hashes']+=2
        event=identity(dict(base=base,plan=identity(plan),acceptance=expected_acceptance))
        with writer(self.path),ExitStack() as held:
            pointer=self.current();existing=self.root(pointer['root'])
            if existing.get('event')==event:
                self._verify_components(existing,held);self.stats['idempotent_retries']+=1
                durable_json(self.path/('JOURNAL_'+event+'.json'),dict(event=event,base=base,
                    acceptance=expected_acceptance,stage='committed',root=pointer['root']))
                return pointer['root']
            if pointer['root']!=base:raise ValueError('StaleIncrementalBase')
            self._verify_components(existing,held)
            # Unchanged references carry prior authentication. New sealed input
            # references are independently bound by accepted control authority.
            for p,r in plan['references'].items():
                held.enter_context(access.guard(p))
                if access.token(p)!=r['token']:raise ValueError('ChangedInputReference')
                self.stats['reference_tokens']+=1
            if len(quality_rows)!=acceptance['quality_count']:raise ValueError('IncompleteQualityDelta')
            keys=[r['key'] for r in quality_rows]
            if len(set(keys))!=len(keys):raise ValueError('DuplicateQualityDelta')
            log=dict(event=event,base=base,acceptance=expected_acceptance,stage='prepared')
            durable_json(self.path/('JOURNAL_'+event+'.json'),log)
            if interrupt:interrupt('prepared')
            objects=self.objects;objects.mkdir(exist_ok=True)
            pending=objects/(event+'.pending')
            # Unpublished leftovers are retained for diagnosis. Retry uses a
            # fresh private pathname, never an incomplete prior object.
            import tempfile
            fd,name=tempfile.mkstemp(prefix=event+'-',suffix='.pending',dir=objects);os.close(fd);pending=Path(name)
            db=sqlite3.connect(pending)
            try:
                db.execute('CREATE TABLE updates(key TEXT PRIMARY KEY, inputs BLOB NOT NULL, payload BLOB) WITHOUT ROWID')
                for r in quality_rows:
                    db.execute('INSERT INTO updates VALUES(?,?,?)',(r['key'],zlib.compress(canonical(r['inputs'])),
                        None if r['payload'] is None else zlib.compress(canonical(r['payload']))))
                    self.stats['quality_rows_written']+=1
                    self.stats['quality_payloads_written']+=r['payload'] is not None
                    if interrupt and self.stats['quality_rows_written']==1:interrupt('component_partial')
                db.commit()
                assert db.execute('PRAGMA integrity_check').fetchone()[0]=='ok'
                # Verify every new row against the accepted changed input.
                supplied={r['key']:r for r in quality_rows}
                for key,inputs,payload in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'):
                    decoded=dict(key=key,inputs=json.loads(zlib.decompress(inputs)),payload=None if payload is None else json.loads(zlib.decompress(payload)))
                    if decoded!=supplied[key]:raise ValueError('ChangedQualityComponent')
                    self.stats['quality_rows_verified']+=1
            finally:db.close()
            with pending.open('r+b') as f:os.fsync(f.fileno())
            if interrupt:interrupt('component_written')
            h=file_hash(pending);target=objects/(h+'.sqlite')
            if target.exists():
                if file_hash(target)!=h:raise ValueError('CorruptExistingDelta')
                pending.unlink()
            else:atomic_replace(pending,target)
            held.enter_context(access.guard(target))
            db=sqlite3.connect(target.as_uri()+'?mode=ro&immutable=1',uri=True)
            try:
                verified=0
                for key,inputs,payload in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'):
                    decoded=dict(key=key,inputs=json.loads(zlib.decompress(inputs)),payload=None if payload is None else json.loads(zlib.decompress(payload)))
                    if decoded!=supplied[key]:raise ValueError('CorruptChangedComponent')
                    verified+=1
                    self.stats['quality_rows_reverified']+=1
                if verified!=len(supplied):raise ValueError('IncompleteChangedComponent')
            finally:db.close()
            binding=objects/('bindings_'+identity(binding_patch)+'.json')
            if not binding.exists():durable_json(binding,binding_patch)
            held.enter_context(access.guard(binding))
            if access.read(binding)!=binding_patch:raise ValueError('CorruptBindingPatch')
            produced={str(p.resolve()):access.attest(p,file_hash(p)) for p in (target,binding)}
            body=dict(schema=SCHEMA,parent=base,event=event,plan=plan,acceptance=acceptance,acceptance_identity=expected_acceptance,
                produced=produced,quality_delta=str(target.resolve()),binding_patch=str(binding.resolve()),
                scientific=plan['scientific'],epoch=access.epoch())
            self.stats['binding_entries_written']=len(binding_patch)
            self.stats['proof_payloads_written']=0
            self.stats['historical_receipt_rows_visited']=0
            self.stats['historical_quality_rows_visited']=0
            # Recheck the base at the commit boundary, under the writer lock.
            if self.current()['root']!=base:raise ValueError('ConcurrentBaseChanged')
            if identity(quality_rows)!=acceptance['quality_delta'] or identity(binding_patch)!=acceptance['binding_patch'] or identity(plan)!=acceptance['plan']:
                raise ValueError('MutableDeltaChangedBeforeCommit')
            self.stats['delta_canonical_hashes']+=2
            if interrupt:interrupt('validated')
            result=self._switch(body,interrupt)
            log['stage']='committed';log['root']=result;durable_json(self.path/('JOURNAL_'+event+'.json'),log)
            return result
    def _verify_components(self,body,held):
        if body.get('epoch')!=access.epoch():raise ValueError('IncrementalEpochChanged')
        for p,r in {**body.get('plan',{}).get('references',{}),**body.get('produced',{})}.items():
            held.enter_context(access.guard(p))
            if access.token(p)!=r['token']:raise ValueError('IncrementalComponentMutation')
    @contextmanager
    def open(self,expected):
        with ExitStack() as held:
            held.enter_context(access.guard(self.path/'CURRENT.json'))
            body=self.root(expected);held.enter_context(access.guard(self.path/('ROOT_'+expected+'.json')))
            self._verify_components(body,held)
            for p,r in body.get('plan',{}).get('references',{}).items():
                held.enter_context(access.guard(p))
                if access.token(p)!=r['token']:raise ValueError('IncrementalReferenceMutation')
            if 'acceptance' in body:
                a=body['acceptance']
                if identity(a)!=body['acceptance_identity'] or identity(body['plan'])!=a['plan'] or a['base']!=body['parent']:raise ValueError('IncompleteIncrementalRoot')
            yield body

class QualityView:
    """Exact old payload plus changed inputs, or a new complete value."""
    def __init__(self,base_path,delta_path):self.base_path=Path(base_path);self.delta_path=Path(delta_path)
    def __getitem__(self,key):
        db=sqlite3.connect(self.delta_path.as_uri()+'?mode=ro&immutable=1',uri=True)
        try:r=db.execute('SELECT inputs,payload FROM updates WHERE key=?',(key,)).fetchone()
        finally:db.close()
        if r is None or r[1] is None:
            db=sqlite3.connect(self.base_path.as_uri()+'?mode=ro&immutable=1',uri=True)
            try:old=db.execute('SELECT value FROM receipts WHERE key=?',(key,)).fetchone()
            finally:db.close()
            if old is None:raise KeyError(key)
            value=json.loads(old[0])
        else:value=json.loads(zlib.decompress(r[1]))
        if r is not None:value['inputs']=json.loads(zlib.decompress(r[0]))
        return value

class ReceiptView:
    def __init__(self,owner):self.owner=owner;self.patch=None;self.base=None;self.new=None
    def __len__(self):return self.owner.body['plan']['components']['certificate']['count']
    def __iter__(self):raise ValueError('ExplicitDeepReplayRequired')
    def __getitem__(self,key):
        self.owner.check_live()
        if self.base is None:
            from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
            plan=self.owner.body['plan']
            # Demanded cold files are fully checked before decoding, preserving
            # the qualified read-side corruption boundary.
            from rolling_layer_retention import descriptors
            for store,d in [(plan['base_store'],plan['base_components']['certificate']),
                            (plan['target_store'],plan['components']['certificate']['updates'])]:
                for r in descriptors(d):
                    p=Path(store)/'objects'/r['file']
                    if file_hash(p)!=r['sha256']:raise ValueError('CorruptColdProofComponent')
            self.base=NativeCheckpointStore(plan['base_store'])._receipt(plan['base_components']['certificate'],verify=False)
            self.new=NativeCheckpointStore(plan['target_store'])._receipt(plan['components']['certificate']['updates'],verify=False)
            self.patch=access.read(self.owner.body['binding_patch'])
        try:
            row=self.base[key];row['inputs']={k:self.patch.get(k,v) for k,v in row['inputs'].items()};return row
        except KeyError:return self.new[key]

class State:
    """Complete control state: original immutable geometry, effective deltas.

    This view supports the sealed-control protocol only. It does not invent
    scientific acceptance for an unknown future layer.
    """
    def __init__(self,path,root,access_directory):
        self.stack=ExitStack();self.closed=False;self.root=root;self.quality=None
        try:
            self.body=self.stack.enter_context(Store(path).open(root))
            self.original=self.stack.enter_context(access.ImmutableState(access_directory,self.body['plan']['backing_access_root']))
            if self.body['plan']['components']!=self.original.body['components']:raise ValueError('WrongControlMapping')
            if self.body['scientific']!=self.original.verified_legacy_identity():raise ValueError('WrongControlScientificIdentity')
            self.receipts=ReceiptView(self)
        except BaseException:self.close();raise
    def close(self):self.stack.close();self.closed=True
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def check_live(self):
        if self.closed:raise ValueError('ClosedIncrementalState')
    def check(self):
        self.check_live()
        if identity(self.body)!=self.root:raise ValueError('MutableIncrementalRoot')
        self.original.check()
    def verified_legacy_identity(self):self.check();return self.body['scientific']
    def __getitem__(self,key):
        self.check_live()
        if key=='certificate':return self.receipts
        if key=='quality':
            if self.quality is None:
                plan=self.body['plan'];d=plan['base_components']['quality'];p=Path(plan['base_store'])/'objects'/d['file']
                if file_hash(p)!=d['sha256']:raise ValueError('CorruptColdQualityBase')
                delta=Path(self.body['quality_delta'])
                if file_hash(delta)!=self.body['produced'][str(delta)]['sha256']:raise ValueError('CorruptColdQualityDelta')
                self.quality=QualityView(p,delta)
            return self.quality
        return self.original[key]
    def initialize(self,wall,triangles,surface,request_path,source,expected):
        from production_forecast_admission import ProductionForecastAdapter
        self.check();adapter=ProductionForecastAdapter.__new__(ProductionForecastAdapter)
        # Exact shared binding implementation; only the authenticated state
        # interface differs. No constructor hash bypass on an untrusted dict.
        adapter._bind(self,wall,triangles,surface,request_path,source,expected,self.verified_legacy_identity())
        self.check();return adapter
