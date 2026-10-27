"""Exact encoded-row verification for the existing durable delta transaction.

The writer retains the canonical compressed bytes actually derived from accepted
rows. Both independent SQLite readback passes compare every key and both blobs.
This is stricter than decoded equality: noncanonical encodings also fail closed.
Acceptance, duplicate detection, integrity_check, FULL journaling, fsync, guards,
CAS, interruption points, retry, and final mutable-input checks stay unchanged.
"""
from incremental_transaction import *
from incremental_transaction import Store as OriginalStore
from single_quality_component import quality_identity, verify_transaction_source, QualityRows


class Store(OriginalStore):
    def publish(self,base,plan,quality_rows,binding_patch,acceptance,expected_acceptance,interrupt=None):
        """Commit only the delta, with original immutable objects referenced.

        acceptance is produced by a scientific authority, not manufactured by
        this writer. The control driver independently compares sealed L4/L5.
        """
        if identity(acceptance)!=expected_acceptance or acceptance.get('passed') is not True:raise ValueError('UnboundAcceptance')
        if acceptance['base']!=base or acceptance['plan']!=identity(plan):raise ValueError('WrongAcceptedPlan')
        if quality_identity(quality_rows)!=acceptance['quality_delta'] or identity(binding_patch)!=acceptance['binding_patch']:
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
            single=verify_transaction_source(quality_rows,plan)
            if not single:
                keys=[r['key'] for r in quality_rows]
                if len(set(keys))!=len(keys):raise ValueError('DuplicateQualityDelta')
            log=dict(event=event,base=base,acceptance=expected_acceptance,stage='prepared')
            durable_json(self.path/('JOURNAL_'+event+'.json'),log)
            if interrupt:interrupt('prepared')
            objects=self.objects;objects.mkdir(exist_ok=True)
            if single:
                target=quality_rows.path
                # Same accepted logical rows. Both independent readbacks retain
                # canonical-blob, order, count and full logical-hash checks.
                quality_rows.verify_readback()
                self.stats['quality_rows_verified']+=len(quality_rows)
                if interrupt:interrupt('component_written')
                quality_rows.verify_readback()
                self.stats['quality_rows_reverified']+=len(quality_rows)
                self.stats['prepared_quality_payloads_reused']+=1
            else:
                target=self._write_quality_component(objects,event,quality_rows,interrupt,held)
            held.enter_context(access.guard(target))
            binding=objects/('bindings_'+identity(binding_patch)+'.json')
            if not binding.exists():durable_json(binding,binding_patch)
            held.enter_context(access.guard(binding))
            if access.read(binding)!=binding_patch:raise ValueError('CorruptBindingPatch')
            if single:
                # Newly produced dependency digests are computed inside the
                # unchanged immutable guard, once rather than twice. Both
                # independent row readbacks above remain compulsory.
                from preparation_reference import attest_content
                produced={str(p.resolve()):attest_content(p) for p in (target,binding)}
            else:
                produced={str(p.resolve()):access.attest(p,file_hash(p)) for p in (target,binding)}
            body=dict(schema=SCHEMA,parent=base,event=event,plan=plan,acceptance=acceptance,acceptance_identity=expected_acceptance,
                produced=produced,quality_delta=str(target.resolve()),binding_patch=str(binding.resolve()),
                scientific=plan['scientific'],epoch=access.epoch())
            self.stats['binding_entries_written']=len(binding_patch)
            self.stats['proof_payloads_written']=0
            self.stats['historical_receipt_rows_visited']=0
            self.stats['historical_quality_rows_visited']=0
            if self.current()['root']!=base:raise ValueError('ConcurrentBaseChanged')
            if quality_identity(quality_rows)!=acceptance['quality_delta'] or identity(binding_patch)!=acceptance['binding_patch'] or identity(plan)!=acceptance['plan']:
                raise ValueError('MutableDeltaChangedBeforeCommit')
            self.stats['delta_canonical_hashes']+=2
            if interrupt:interrupt('validated')
            result=self._switch(body,interrupt)
            log['stage']='committed';log['root']=result;durable_json(self.path/('JOURNAL_'+event+'.json'),log)
            return result

    def _write_quality_component(self,objects,event,quality_rows,interrupt,held):
            """Unchanged qualified legacy writer/readbacks; no format migration."""
            pending=objects/(event+'.pending')
            # Unpublished leftovers are retained for diagnosis. Retry uses a
            # fresh private pathname, never an incomplete prior object.
            import tempfile
            fd,name=tempfile.mkstemp(prefix=event+'-',suffix='.pending',dir=objects);os.close(fd);pending=Path(name)
            db=sqlite3.connect(pending)
            try:
                db.execute('CREATE TABLE updates(key TEXT PRIMARY KEY, inputs BLOB NOT NULL, payload BLOB) WITHOUT ROWID')
                encoded={}
                from critical_quality_codec import encoded as encode_rows
                from contextlib import closing
                workers=int(os.environ.get('V4_QUALITY_CODEC','1')) if len(quality_rows)>=2048 else 1
                with closing(encode_rows(quality_rows,workers,True)) as encoded_rows:
                    for r,blobs in zip(quality_rows,encoded_rows,strict=True):
                        encoded[r['key']]=blobs
                        db.execute('INSERT INTO updates VALUES(?,?,?)',(r['key'],*blobs))
                        self.stats['quality_rows_written']+=1
                        self.stats['quality_payloads_written']+=r['payload'] is not None
                        if interrupt and self.stats['quality_rows_written']==1:interrupt('component_partial')
                from critical_quality_codec import STATS
                durable_json(self.path.parent/'QUALITY_CODEC_METRICS.json',dict(STATS))
                db.commit()
                assert db.execute('PRAGMA integrity_check').fetchone()[0]=='ok'
                # Verify every new row against the accepted changed input.
                supplied={r['key']:r for r in quality_rows}
                for key,inputs,payload in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'):
                    if key not in encoded or (inputs,payload)!=encoded[key]:raise ValueError('ChangedQualityComponent')
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
                    if key not in encoded or (inputs,payload)!=encoded[key]:raise ValueError('CorruptChangedComponent')
                    verified+=1
                    self.stats['quality_rows_reverified']+=1
                if verified!=len(supplied):raise ValueError('IncompleteChangedComponent')
            finally:db.close()
            return target
