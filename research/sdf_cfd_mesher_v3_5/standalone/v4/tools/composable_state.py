"""Recursive committed-state access. Historical qualified readers stay pinned.

Small controls qualify representation mechanics, not a mesh advancement.
Every replacement names its previous record hash. Deletion is an explicit
tombstone; missing records never silently fall through a deletion.
"""
from collections import Counter, ChainMap
from collections.abc import Mapping
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import json
import sqlite3
import zlib

import immutable_state_access as access
from clean_sheet_candidate import Candidate
from incremental_transaction import Store
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER, NativeCheckpointStore, scientific_digest
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items

SCHEMA = 'V4RecursiveCommittedState/1'


def control_components(base,delta):
    result={k:dict(parent=base.content_root,role=k) for k in base.components}
    for kind,keys in [('certificate',delta['proofs']),('quality',{r['key'] for r in delta['quality']})]:
        added=sum(delta['before'][kind][key] is None for key in keys)
        result[kind].update(delta=identity(delta),count=len(base[kind])+added-len(delta['removed'].get(kind,[])))
    return result


class RecursiveFaces(Mapping):
    def __init__(self, owner): self.owner=owner
    def __len__(self): return len(self.owner.parent['faces'])+self.owner.plan['geometry_counts']['new_faces']
    def __iter__(self): return iter(self.owner.geometry('faces'))
    def __getitem__(self,key):
        self.owner.check_live()
        if key in self.owner.hot['faces']: return deepcopy(self.owner.hot['faces'][key])
        return deepcopy(self.owner.geometry('faces')[key])


class HotMap(Mapping):
    def __init__(self,owner,values): self.owner=owner; self.values=values
    def __len__(self): self.owner.check_live(); return len(self.values)
    def __iter__(self): self.owner.check_live(); return iter(self.values)
    def __getitem__(self,key): self.owner.check_live(); return deepcopy(self.values[key])


def merge_rows(old, changes, removed):
    """Sorted explicit deep traversal; point lookup never calls this."""
    newer = iter(sorted(changes.items())); b = next(newer, None)
    for key, value in old:
        while b is not None and b[0] < key:
            yield b; b = next(newer, None)
        if b is not None and b[0] == key:
            if key not in removed: yield b
            b = next(newer, None)
        elif key not in removed:
            yield key, value
    while b is not None:
        yield b; b = next(newer, None)


def merge_streams(old, new):
    a=next(old,None); b=next(new,None)
    while a is not None or b is not None:
        if a is not None and (b is None or a[0]<b[0]):
            yield a; a=next(old,None)
        elif b is not None and (a is None or b[0]<a[0]):
            yield b; b=next(new,None)
        else: raise ValueError('ImmutableProofConflict')


class Rows(Mapping):
    def __init__(self, owner, kind): self.owner=owner; self.kind=kind; self.deep=False
    def __len__(self): return self.owner.count(self.kind)
    def __iter__(self): raise ValueError('ExplicitDeepVerificationRequired')
    def __getitem__(self, key):
        o=self.owner; o.check_live(); o.stats[self.kind+'_lookups']+=1
        if o.kind != 'recursive':
            value=o.wrapped[self.kind][key]
            o.stats[self.kind+'_base_records_returned']+=1
            return deepcopy(value)
        d=o.delta
        if key in d['removed'].get(self.kind, []): raise KeyError(key)
        o.stats[self.kind+'_components_touched']+=1
        if self.kind=='certificate':
            if o.production:
                try: return o.new_proof(key)
                except KeyError:
                    value=deepcopy(o.parent[self.kind][key])
                    value['inputs']={k:d['bindings'].get(k,v) for k,v in value['inputs'].items()}
                    return value
            if key in d['proofs']: value=deepcopy(d['proofs'][key])
            else:
                o.stats[self.kind+'_parent_lookups']+=1
                value=deepcopy(o.parent[self.kind][key])
            for dep, edit in d['bindings'].items():
                if dep in value['inputs']:
                    if value['inputs'][dep] != edit['before']: raise ValueError('StaleProofBinding')
                    value['inputs'][dep]=edit['after']
            return value
        p=o.verify(Path(o.body['quality_delta']))
        db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
        try:
            o.stats['quality_point_queries']+=1
            row=db.execute('SELECT inputs,payload FROM updates WHERE key=?',(key,)).fetchone()
        finally: db.close()
        if row is not None: o.stats['quality_rows_visited']+=1
        if row is None or row[1] is None:
            o.stats['quality_parent_lookups']+=1
            value=deepcopy(o.parent['quality'][key])
        else: value=json.loads(zlib.decompress(row[1]))
        if row is not None: value['inputs']=json.loads(zlib.decompress(row[0]))
        return value
    def items_deep(self):
        o=self.owner
        if o.kind=='native':
            rows=NativeCheckpointStore(o.wrapped.store)._receipt(o.components[self.kind],verify=False)
            for k,v in raw_items(rows):
                o.stats['deep_'+self.kind+'_rows']+=1
                yield k,json.loads(v)
        elif o.kind=='clean_sheet':
            yield from o.wrapped[self.kind].deep_items()
        else:
            if o.production and self.kind=='certificate':
                def old_rows():
                    for key,value in o.parent[self.kind].items_deep():
                        value['inputs']={k:o.delta['bindings'].get(k,v) for k,v in value['inputs'].items()}
                        yield key,value
                yield from merge_streams(iter(old_rows()),iter(o.new_proof_items()))
                return
            if o.production and self.kind=='quality':
                old=iter(o.parent['quality'].items_deep()); a=next(old,None)
                p=o.verify(Path(o.body['quality_delta']))
                db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
                try:
                    for key,inputs,payload in db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'):
                        while a is not None and a[0]<key:
                            yield a; a=next(old,None)
                        if payload is None:
                            if a is None or a[0]!=key: raise ValueError('MissingInheritedQuality')
                            value=deepcopy(a[1])
                        else: value=json.loads(zlib.decompress(payload))
                        value['inputs']=json.loads(zlib.decompress(inputs)); yield key,value
                        if a is not None and a[0]==key: a=next(old,None)
                    if a is not None: yield a
                    yield from old
                finally: db.close()
                return
            # Only changed records are expanded, never the complete prefix.
            changes=dict(o.delta['proofs']) if self.kind=='certificate' else {}
            if self.kind=='quality':
                p=o.verify(Path(o.body['quality_delta']))
                db=sqlite3.connect(p.as_uri()+'?mode=ro&immutable=1',uri=True)
                try:
                    for k, in db.execute('SELECT key FROM updates ORDER BY key'): changes[k]=self[k]
                finally: db.close()
            old=o.parent[self.kind].items_deep()
            for k,value in merge_rows(old,changes,set(o.delta['removed'].get(self.kind,[]))):
                if self.kind=='certificate':
                    value=deepcopy(value)
                    for dep,edit in o.delta['bindings'].items():
                        if dep in value['inputs']:
                            if value['inputs'][dep]!=edit['before']: raise ValueError('StaleProofBinding')
                            value['inputs'][dep]=edit['after']
                o.stats['deep_'+self.kind+'_rows']+=1
                yield k,value
    def native_canonical_chunks(self):
        if not self.deep: raise ValueError('ExplicitDeepVerificationRequired')
        yield b'{'
        for i,(k,v) in enumerate(self.items_deep()):
            if i: yield b','
            yield ENCODER.encode(k).encode(); yield b':'; yield ENCODER.encode(v).encode()
        yield b'}'


class CommittedState(Mapping):
    """Factory-opened semantic interface; no unverified Mapping is admitted."""
    def __init__(self, reference, ancestry=()):
        self.reference=deepcopy(reference); self.stats=Counter(); self.stack=ExitStack(); self.closed=False; self.verified=set()
        self.kind=reference['kind']; self.root=reference['root']
        if self.root in ancestry: raise ValueError('AncestorCycle')
        try:
            if self.kind=='native':
                self.wrapped=self.stack.enter_context(access.ImmutableState(reference['directory'],self.root))
                self.components=self.wrapped.body['components']; self.content_root=self.wrapped.body['content_root']
                self.scientific=self.wrapped.verified_legacy_identity(); self.expected=self.wrapped.body['expected']
            elif self.kind=='clean_sheet':
                self.wrapped=self.stack.enter_context(Candidate(reference['directory'],self.root))
                seal=Path(reference['seal']); self.stack.enter_context(access.guard(seal))
                if file_hash(seal)!=reference['seal_sha256']: raise ValueError('WrongScientificSeal')
                s=access.read(seal)
                if s['root']!=self.root: raise ValueError('WrongScientificRoot')
                self.scientific=s['legacy_scientific']; self.components=self.wrapped.plan['components']
                self.content_root=self.wrapped.plan['scientific']; self.expected=self.wrapped.plan['expected']
            elif self.kind=='recursive':
                self.body=self.stack.enter_context(Store(reference['directory']).open(self.root)); p=self.body['plan']
                if p['schema']!=SCHEMA or p['implementation']!=file_hash(__file__): raise ValueError('WrongRecursiveImplementation')
                if identity(p['content'])!=p['scientific']: raise ValueError('WrongRecursiveContentRoot')
                self.parent=self.stack.enter_context(CommittedState(p['parent'],ancestry+(self.root,)))
                if p['content']['parent_content']!=self.parent.content_root: raise ValueError('ConflictingAncestorIdentity')
                if p['content']['parent_scientific']!=self.parent.scientific: raise ValueError('ConflictingScientificIdentity')
                if set(p['content']['components'])!=set(self.parent.components): raise ValueError('MissingInheritedComponent')
                self.production=p['scope']=='production'
                if self.production:
                    self.out=Path(p['output']); self.plan=p
                    self.hot=access.read(self.verify(Path(p['hot'])))
                    self.delta=dict(proofs={},removed={},bindings=access.read(self.verify(Path(self.body['binding_patch']))))
                    self.geometry_cache=None
                else:
                    self.delta=access.read(self.verify(Path(p['delta'])))
                    if identity(self.delta)!=p['content']['delta']: raise ValueError('WrongRecursiveDelta')
                    if p['content']['components']!=control_components(self.parent,self.delta): raise ValueError('ConflictingComponentIdentity')
                self.components=p['content']['components']; self.content_root=p['scientific']
                self.scientific=self.content_root; self.expected=p['expected']
            else: raise ValueError('UnsupportedCommittedStateKind')
            self.layer=int(self.expected['layer']); self.rows={k:Rows(self,k) for k in ('certificate','quality')}
        except BaseException: self.close(); raise
    def __enter__(self): return self
    def __exit__(self,*args): self.close()
    def close(self): self.stack.close(); self.closed=True
    def check_live(self):
        if self.closed: raise ValueError('ClosedCommittedState')
    def check(self):
        self.check_live()
        if self.kind=='recursive':
            if identity(self.body)!=self.root: raise ValueError('MutatedRecursiveRoot')
            self.parent.check()
        else: self.wrapped.check()
    def verify(self,p):
        self.check_live(); p=p.resolve(); records={**self.body['plan']['references'],**self.body['produced']}; r=records.get(str(p))
        if r is None: raise ValueError('UnboundRecursiveComponent')
        if access.token(p)!=r['token']: raise ValueError('RecursiveComponentMutation')
        if str(p) not in self.verified:
            if file_hash(p)!=r['sha256']: raise ValueError('CorruptRecursiveComponent')
            self.stats['demanded_component_bytes_hashed']+=p.stat().st_size; self.verified.add(str(p))
        return p
    def __iter__(self): return iter(self.components)
    def __len__(self): return len(self.components)
    def __getitem__(self,key):
        self.check_live()
        if key in self.rows: return self.rows[key]
        if self.kind=='recursive':
            if self.production:
                if key in ('vertices','columns','prism_front'): return ChainMap(HotMap(self,self.hot[key]),self.parent[key])
                if key=='outer': return sorted(self.hot['prism_front'].values())
                if key=='faces': return RecursiveFaces(self)
                if key in ('cells','versions'): return self.geometry(key)
                if key in ('atlas','forecast'):
                    name='NEW_ATLAS.json' if key=='atlas' else 'NEW_FORECAST_DEPENDENCIES.json'
                    return access.read(self.verify(self.out/name))
            return self.parent[key]
        return self.wrapped[key]
    def geometry(self,key):
        self.check_live()
        if self.geometry_cache is None: self.geometry_cache=access.read(self.verify(self.out/'GEOMETRY_DELTA.json'))
        return ChainMap(HotMap(self,self.geometry_cache[key]),self.parent[key])
    def count(self,kind):
        if self.kind=='recursive': return self.components[kind]['count']
        return len(self.wrapped[kind])
    def initialize(self,wall,tri,surface,request,source):
        if self.kind=='recursive' and self.body['plan']['scope']!='production': raise ValueError('NonGeometricControlNotAdvancementBase')
        from production_forecast_admission import ProductionForecastAdapter
        self.check(); adapter=ProductionForecastAdapter.__new__(ProductionForecastAdapter)
        adapter._bind(self,wall,tri,surface,request,source,self.expected,self.expected['committed'])
        self.check(); return adapter
    def proof_segments(self):
        """Return authenticated compact indices, never receipt payloads."""
        if self.kind=='native':
            for d in self.components['certificate']['proof_segments']:
                yield d, self.wrapped.store/'objects', self.wrapped._verify_read
        elif self.kind=='clean_sheet':
            b=self.wrapped.base
            for d in b.body['components']['certificate']['proof_segments']:
                yield d, b.store/'objects', b._verify_read
            yield self.components['certificate']['new_segment'], self.wrapped.out/'objects', self.wrapped.verify
        else:
            if not self.production: raise ValueError('NonGeometricControlHasNoGeometricCoverageAuthority')
            yield from self.parent.proof_segments()
            yield self.components['certificate']['new_segment'],self.out/'objects',self.verify
    def load_proof(self):
        if hasattr(self,'pairs'): return
        import numpy as np
        d=self.components['certificate']['new_segment']; objects=self.out/'objects'
        self.proof_faces=access.read(self.verify(objects/d['faces']['file']))
        self.payloads=access.read(self.verify(objects/d['payloads']['file']))
        self.pairs=np.load(self.verify(objects/d['pairs']['file']),mmap_mode='r',allow_pickle=False)
        self.codes=np.load(self.verify(objects/d['codes']['file']),mmap_mode='r',allow_pickle=False)
        self.inputs=access.read(self.verify(self.out/'NEW_RECEIPT_INPUTS.json'))
        self.index=self.verify(objects/'new-proof-key-index.sqlite')
    def proof_at(self,ordinal):
        a,b=self.pairs[ordinal]; faces=sorted((self.proof_faces[int(a)],self.proof_faces[int(b)]))
        deps=['face:'+f for f in faces]+['authority:certificate']
        if 'authority:certificate_filter' in self.inputs: deps.append('authority:certificate_filter')
        return dict(faces=faces,inputs={k:self.inputs[k] for k in deps},value=deepcopy(self.payloads[int(self.codes[ordinal])]))
    def new_proof(self,key):
        self.load_proof(); db=sqlite3.connect(self.index.as_uri()+'?mode=ro&immutable=1',uri=True)
        try:
            self.stats['proof_point_queries']+=1
            row=db.execute('SELECT ordinal FROM proof_keys WHERE key=?',(bytes.fromhex(key),)).fetchone()
        finally: db.close()
        if row is None: raise KeyError(key)
        self.stats['proof_rows_visited']+=1
        return self.proof_at(row[0])
    def new_proof_items(self):
        self.load_proof(); db=sqlite3.connect(self.index.as_uri()+'?mode=ro&immutable=1',uri=True)
        try:
            for key,ordinal in db.execute('SELECT hex(key),ordinal FROM proof_keys ORDER BY key'):
                yield key.lower(),self.proof_at(ordinal)
        finally: db.close()
    def deep_verify(self,expected=None):
        self.check()
        # Full content authentication is explicit, outside the normal path.
        if self.kind=='native': refs=self.wrapped.body['files']
        elif self.kind=='clean_sheet': refs={**self.wrapped.plan['references'],**self.wrapped.body['produced']}
        else:
            self.parent.deep_verify()
            refs={**self.body['plan']['references'],**self.body['produced']}
        for p,r in refs.items():
            if file_hash(p)!=r['sha256']: raise ValueError('DeepAncestorComponentCorrupt')
        for row in self.rows.values(): row.deep=True
        try: actual=scientific_digest(self)
        finally:
            for row in self.rows.values(): row.deep=False
        if expected is None and self.kind!='recursive': expected=self.scientific
        if expected is not None and actual!=expected: raise ValueError('DeepScientificMismatch')
        return actual


def plan_transition(base, adapter, next_layer=None):
    if type(base) is not CommittedState: raise ValueError('QualifiedCommittedStateRequired')
    base.check(); target=base.layer+1
    if next_layer is not None and next_layer!=target: raise ValueError('NonSequentialTransition')
    if adapter.binding['layer']!=base.layer or adapter.binding['next_layer']!=target or adapter.binding['committed']!=base.expected['committed']:
        raise ValueError('StaleTransitionBinding')
    if target>len(adapter.schedule.increments): raise ValueError('ScheduleComplete')
    n=len(adapter.front.coordinates); count=len(adapter.topology.triangles)
    if adapter.committed_prefix.shape!=(target,n,3): raise ValueError('PrefixShapeMismatch')
    return dict(base_layer=base.layer,target_layer=target,base_root=base.root,base_content=base.content_root,
        base_scientific=base.scientific,increment=float(adapter.schedule.increments[target-1]),
        increment_hex=float(adapter.schedule.increments[target-1]).hex(),schedule_index=target-1,
        forecast_horizon=list(range(target,len(adapter.schedule.increments)+1)),
        lower_vertex_offset=base.layer*n,upper_vertex_offset=target*n,
        first_cell_id=target*1000000,last_cell_id=target*1000000+count-1,
        vertices_per_front=n,prisms_per_layer=count,prefix_fronts=target,
        binding=deepcopy(adapter.binding),admission_run=False,forecast_run=False,geometry_generated=False)


def record_hash(state,kind,key):
    try: return identity(state[kind][key])
    except KeyError: return None


def publish_control(base,directory,delta,expected_delta,interrupt=None):
    """Bounded non-geometric child; never grants mesh advancement acceptance.

    delta = proofs, bindings, quality, removed, before. before authenticates
    every changed record against the current parent. Binding edits include
    before/after and explicit affected proof keys for this bounded control.
    """
    if identity(delta)!=expected_delta: raise ValueError('ChangedControlDelta')
    base.check(); directory=Path(directory); directory.mkdir(exist_ok=True,parents=True)
    if set(delta)!= {'proofs','bindings','quality','removed','before'}: raise ValueError('IncompleteControlDelta')
    rows=delta['quality']; keys=[r['key'] for r in rows]
    if len(keys)!=len(set(keys)): raise ValueError('QualityConflict')
    for kind,newkeys in [('certificate',set(delta['proofs'])),('quality',set(keys))]:
        removed=delta['removed'].get(kind,[])
        if len(removed)!=len(set(removed)) or set(removed)&newkeys: raise ValueError('ConflictingRemoval')
        expected=delta['before'].get(kind,{})
        if set(expected)!=newkeys|set(removed): raise ValueError('IncompleteChangePreconditions')
        for key,want in expected.items():
            if record_hash(base,kind,key)!=want: raise ValueError('StaleRecordPrecondition')
        for key in removed:
            if expected[key] is None: raise ValueError('RemoveMissingRecord')
    for key,value in delta['proofs'].items():
        if delta['before']['certificate'][key] is not None: raise ValueError('ImmutableProofConflict')
        if not isinstance(value,dict) or not {'inputs','value','faces'}<=set(value): raise ValueError('IncompleteProof')
    for dep,edit in delta['bindings'].items():
        if set(edit)!= {'before','after','proof_keys'} or not edit['proof_keys']: raise ValueError('UnboundBindingChange')
        for key in edit['proof_keys']:
            if base['certificate'][key]['inputs'].get(dep)!=edit['before']: raise ValueError('StaleBinding')
    for row in rows:
        if row['payload'] is None and delta['before']['quality'][row['key']] is None: raise ValueError('MissingInheritedQuality')
    path=directory/('DELTA_'+expected_delta+'.json')
    if path.exists():
        if access.read(path)!=delta: raise ValueError('CorruptExistingControlDelta')
    else: durable_json(path,delta)
    refs={str(path.resolve()):access.attest(path,file_hash(path))}
    components=control_components(base,delta)
    content=dict(parent_content=base.content_root,parent_scientific=base.scientific,delta=expected_delta,components=components)
    plan=dict(schema=SCHEMA,scope='non_geometric_composition_control',parent=base.reference,content=content,
        scientific=identity(content),expected=base.expected,delta=str(path.resolve()),references=refs,implementation=file_hash(__file__))
    store=Store(directory)
    initial_path=directory/'INITIAL.json'
    if initial_path.exists(): initial=access.read(initial_path)['root']
    else:
        initial=store.initialize(dict(epoch=access.epoch(),scientific=base.scientific,plan=dict(references={})))
        durable_json(initial_path,dict(root=initial))
    patch={k:v['after'] for k,v in delta['bindings'].items()}
    acceptance=dict(passed=True,base=initial,plan=identity(plan),quality_delta=identity(rows),quality_count=len(rows),binding_patch=identity(patch),scope=plan['scope'])
    def boundary(stage):
        if stage=='validated': base.check()
        if interrupt: interrupt(stage)
    root=store.publish(initial,plan,rows,patch,acceptance,identity(acceptance),boundary)
    return dict(kind='recursive',directory=str(directory.resolve()),root=root),dict(store.stats)
