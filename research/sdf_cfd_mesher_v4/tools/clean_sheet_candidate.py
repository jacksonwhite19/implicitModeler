"""Independent component-root publication and cold views; no target oracle.

The prior root is L4. New proof segments and factorized bindings are produced
by the guarded clean-sheet certifier. Legacy digest streaming is deep-audit
work and is never part of normal publication or next-increment binding.
"""
from collections import ChainMap, Counter
from collections.abc import Mapping
from contextlib import ExitStack
from pathlib import Path
import json, sqlite3, time, zlib
import numpy as np
import immutable_state_access as access
from incremental_transaction import Store, canonical
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity, array_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ENCODER, NativeCheckpointStore, scientific_digest
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items


def read(p):return json.loads(Path(p).read_text())


def prepare(base,adapter,out):
    """Seal independently produced inputs, not an external target snapshot."""
    started=time.perf_counter();info=read(out/'ASSEMBLY.json');g=read(out/'GENERATION.json')
    delta=read(out/'GEOMETRY_DELTA.json');cert=read(out/'FRESH_CERTIFICATE.json');objects=out/'objects'
    inputs=read(out/'CHANGED_INPUTS.json');prior_hashes=0
    fresh_faces=read(objects/cert['segment']['faces']['file'])
    for f in fresh_faces:
        dep='face:'+f
        if dep not in inputs:
            row=base['faces'][f]
            inputs[dep]=identity(dict(face=row,coordinates={v:base['vertices'][v] for v in row['vertices']}))
            prior_hashes+=1
    durable_json(out/'NEW_RECEIPT_INPUTS.json',inputs)
    pairs=np.load(objects/cert['segment']['pairs']['file'],mmap_mode='r',allow_pickle=False)
    index=objects/'new-proof-key-index.sqlite'
    if index.exists():raise ValueError('PreserveExistingProofIndex')
    db=sqlite3.connect(index)
    try:
        db.execute('CREATE TABLE proof_keys(key BLOB PRIMARY KEY, ordinal INTEGER NOT NULL UNIQUE) WITHOUT ROWID')
        for start in range(0,len(pairs),10000):
            values=[(bytes.fromhex(identity(sorted((fresh_faces[int(a)],fresh_faces[int(b)])))),start+j)
                    for j,(a,b) in enumerate(pairs[start:start+10000])]
            db.executemany('INSERT INTO proof_keys VALUES (?,?)',values)
        db.commit()
        if db.execute('SELECT COUNT(*) FROM proof_keys').fetchone()[0]!=cert['required_pairs']:raise ValueError('IncompleteNewProofIndex')
        if db.execute('PRAGMA integrity_check').fetchone()[0]!='ok':raise ValueError('CorruptNewProofIndex')
    finally:db.close()
    with np.load(out/'GENERATED_L5.npz',allow_pickle=False) as a:
        coordinates=a['coordinates'];triangles=a['topology']
    hot=dict(vertices=delta['vertices'],columns=delta['columns'],prism_front=delta['prism_front'],
             faces={k:delta['faces'][k] for k in delta['prism_front'].values()})
    durable_json(out/'CANDIDATE_HOT.json',hot)
    components={k:dict(base=base.body['content_root'],role=k) for k in base.body['components']}
    for k in delta:components[k]['overlay']=file_hash(out/'GEOMETRY_DELTA.json')
    components['outer']=dict(replace=identity(sorted(delta['prism_front'].values())))
    components['atlas']=dict(replace=file_hash(out/'NEW_ATLAS.json'))
    components['forecast']=dict(replace=file_hash(out/'NEW_FORECAST_DEPENDENCIES.json'))
    components['quality']=dict(base=base.body['components']['quality'],delta=file_hash(out/'QUALITY_DELTA.json'),
        count=len(base['quality'])+info['quality_new'])
    components['certificate']=dict(base=base.body['components']['certificate'],new_segment=cert['segment'],
        bindings=file_hash(out/'BINDING_PATCH.json'),new_inputs=file_hash(out/'NEW_RECEIPT_INPUTS.json'),
        lookup=file_hash(index),count=cert['combined_pairs'])
    content=dict(schema='V4CleanSheetScientificComponents/1',base_scientific=base.verified_legacy_identity(),
        base_content=base.body['content_root'],components=components)
    content_root=identity(content)
    names=['GEOMETRY_DELTA.json','QUALITY_DELTA.json','BINDING_PATCH.json','NEW_RECEIPT_INPUTS.json',
           'NEW_ATLAS.json','NEW_FORECAST_DEPENDENCIES.json','CANDIDATE_HOT.json','GENERATION.json',
           'ASSEMBLY.json','FRESH_CERTIFICATE.json','VOLUME_DELTA.json','ADMISSION.json','AFFECTED_INCIDENCE.json']
    refs=dict(base.body['files'])
    for p in [out/n for n in names]+[objects/cert['segment'][k]['file'] for k in ('faces','pairs','codes','payloads')]+[index]:
        refs[str(p.resolve())]=access.attest(p,file_hash(p))
    expected=dict(base.body['expected'],committed=content_root,layer=5,front_coordinates=array_hash(coordinates))
    plan=dict(schema='V4CleanSheetTransitionPlan/1',scientific=content_root,content=content,components=components,
        references=refs,base_access=str(base.directory),base_root=base._root,base_store=str(base.store),
        base_components=base.body['components'],output=str(out.resolve()),expected=expected,
        hot=str((out/'CANDIDATE_HOT.json').resolve()),no_oracle=True,geometry_counts=g['counts'],
        implementation={str(Path(__file__).resolve()):file_hash(__file__)})
    durable_json(out/'CANDIDATE_PLAN.json',plan)
    durable_json(out/'PREPARE_METRICS.json',dict(seconds=time.perf_counter()-started,new_proof_index_rows=len(pairs),
        old_proof_payloads_read=0,old_face_hashes_for_new_interaction_bindings=prior_hashes,
        content_root=content_root,complete_component_map=components))
    return plan


def publish(base,adapter,out):
    plan=prepare(base,adapter,out);store=Store(out/'candidate')
    initial=store.initialize(dict(epoch=access.epoch(),scientific=base.verified_legacy_identity(),
                                  plan=dict(references=base.body['files'])))
    quality=read(out/'QUALITY_DELTA.json');patch=read(out/'BINDING_PATCH.json')
    acceptance=dict(passed=True,base=initial,plan=identity(plan),quality_delta=identity(quality),
        binding_patch=identity(patch),quality_count=len(quality),
        authority='Independent guarded L4-to-L5 generation and complete changed certification',
        evidence={n:file_hash(out/n) for n in ('ADMISSION.json','GENERATION.json','ASSEMBLY.json','VOLUME_DELTA.json','FRESH_CERTIFICATE.json')})
    durable_json(out/'CANDIDATE_ACCEPTANCE.json',acceptance)
    tick=time.perf_counter()
    root=store.publish(initial,plan,quality,patch,acceptance,identity(acceptance))
    durable_json(out/'PUBLICATION.json',dict(root=root,content_root=plan['scientific'],parent=initial,
        seconds=time.perf_counter()-tick,stats=dict(store.stats),oracle_unlocked=False))
    return root


class EffectiveRows(Mapping):
    """Cold exact key lookup; complete traversal requires explicit deep mode."""
    def __init__(self,owner,kind):self.owner=owner;self.kind=kind;self.deep=False
    def __len__(self):return self.owner.plan['components'][self.kind]['count']
    def __iter__(self):raise ValueError('ExplicitDeepAuditRequired')
    def old(self):
        b=self.owner.base
        return NativeCheckpointStore(b.store)._receipt(b.body['components'][self.kind],verify=False)
    def new_proof(self,ordinal):
        o=self.owner;o.load_proof();a,b=o.pairs[ordinal]
        faces=sorted((o.proof_faces[int(a)],o.proof_faces[int(b)]))
        deps=['face:'+f for f in faces]+['authority:certificate']
        if 'authority:certificate_filter' in o.inputs:deps.append('authority:certificate_filter')
        return dict(faces=faces,inputs={k:o.inputs[k] for k in deps},value=o.payloads[int(o.codes[ordinal])])
    def __getitem__(self,key):
        o=self.owner;o.check_live()
        if self.kind=='certificate':
            o.load_proof()
            db=sqlite3.connect(o.index.as_uri()+'?mode=ro&immutable=1',uri=True)
            try:r=db.execute('SELECT ordinal FROM proof_keys WHERE key=?',(bytes.fromhex(key),)).fetchone()
            finally:db.close()
            if r is not None:return self.new_proof(r[0])
            r=o.base['certificate'][key];r['inputs']={k:o.patch.get(k,v) for k,v in r['inputs'].items()};return r
        from incremental_transaction import QualityView
        p=o.base._object_path(o.base.body['components']['quality']);o.base._verify_read(p)
        return QualityView(p,Path(o.body['quality_delta']))[key]
    def deep_items(self):
        o=self.owner;o.check_live()
        old=iter(raw_items(self.old()));a=next(old,None)
        if self.kind=='certificate':
            o.load_proof();db=sqlite3.connect(o.index.as_uri()+'?mode=ro&immutable=1',uri=True)
            cursor=iter(db.execute('SELECT hex(key),ordinal FROM proof_keys ORDER BY key'))
            def decode(r):return r[0].lower(),self.new_proof(r[1])
        else:
            db=sqlite3.connect(Path(o.body['quality_delta']).as_uri()+'?mode=ro&immutable=1',uri=True)
            cursor=iter(db.execute('SELECT key,inputs,payload FROM updates ORDER BY key'))
            def decode(r):return r[0],(json.loads(zlib.decompress(r[1])),None if r[2] is None else json.loads(zlib.decompress(r[2])))
        b=next(cursor,None)
        try:
            while a is not None or b is not None:
                bk=None if b is None else (b[0].lower() if self.kind=='certificate' else b[0])
                if a is not None and (b is None or a[0]<bk):
                    key,value=a[0],json.loads(a[1]);o.stats['deep_old_'+self.kind]+=1
                    if self.kind=='certificate':value['inputs']={k:o.patch.get(k,v) for k,v in value['inputs'].items()}
                    yield key,value;a=next(old,None)
                else:
                    key,value=decode(b)
                    if self.kind=='quality':
                        inputs,payload=value
                        if payload is None:
                            if a is None or a[0]!=key:raise ValueError('MissingBaseQualityPayload')
                            payload=json.loads(a[1])
                        value=dict(payload,inputs=inputs)
                    elif a is not None and a[0]==key:raise ValueError('DuplicateOldNewProof')
                    if a is not None and a[0]==key:a=next(old,None);o.stats['deep_old_'+self.kind]+=1
                    yield key,value;b=next(cursor,None);o.stats['deep_new_'+self.kind]+=1
        finally:db.close()
    def native_canonical_chunks(self):
        if not self.deep:raise ValueError('ExplicitDeepAuditRequired')
        yield b'{'
        for i,(k,v) in enumerate(self.deep_items()):
            if i:yield b','
            yield ENCODER.encode(k).encode();yield b':';yield ENCODER.encode(v).encode()
        yield b'}'


class FrontFaces(Mapping):
    def __init__(self,owner):self.owner=owner
    def __len__(self):return len(self.owner.base['faces'])+self.owner.plan['geometry_counts']['new_faces']
    def __iter__(self):return iter(self.owner.full_delta_map('faces'))
    def __getitem__(self,k):
        if k in self.owner.hot['faces']:return self.owner.hot['faces'][k]
        return self.owner.full_delta_map('faces')[k]


class Candidate(Mapping):
    def __init__(self,directory,root):
        self.stack=ExitStack();self.closed=False;self.root=root;self.cache={};self.stats=Counter()
        try:
            self.body=self.stack.enter_context(Store(directory).open(root));self.plan=self.body['plan']
            if self.plan['schema']!='V4CleanSheetTransitionPlan/1' or not self.plan['no_oracle']:raise ValueError('WrongCandidatePlan')
            if identity(self.plan['content'])!=self.plan['scientific']:raise ValueError('WrongContentRoot')
            if self.plan['components']!=self.plan['content']['components']:raise ValueError('CandidateComponentMappingMismatch')
            for p,h in self.plan['implementation'].items():
                if file_hash(p)!=h:raise ValueError('CandidateImplementationChanged')
            self.base=self.stack.enter_context(access.ImmutableState(self.plan['base_access'],self.plan['base_root']))
            if self.base.verified_legacy_identity()!=self.plan['content']['base_scientific']:raise ValueError('WrongScientificBase')
            if set(self.plan['components'])!=set(self.base.body['components']):raise ValueError('IncompleteCandidateComponents')
            self.out=Path(self.plan['output']);self.hot=self.load(Path(self.plan['hot']))
            self.patch=self.load(Path(self.body['binding_patch']))
            self.receipts=EffectiveRows(self,'certificate');self.quality=EffectiveRows(self,'quality')
        except BaseException:self.close();raise
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def close(self):self.stack.close();self.closed=True
    def check_live(self):
        if self.closed:raise ValueError('ClosedCandidate')
    def check(self):
        self.check_live();self.base.check()
        if identity(self.body)!=self.root:raise ValueError('CandidateRootMutation')
    def verify(self,p):
        p=Path(p).resolve();k=str(p);r={**self.plan['references'],**self.body['produced']}.get(k)
        if r is None:raise ValueError('UnboundCandidateRead')
        if access.token(p)!=r['token']:raise ValueError('ChangedCandidateComponent')
        if k not in self.cache:
            if file_hash(p)!=r['sha256']:raise ValueError('CorruptCandidateComponent')
            self.cache[k]=True;self.stats['bytes_hashed_on_demand']+=p.stat().st_size
        return p
    def load(self,p):return read(self.verify(p))
    def __iter__(self):return iter(self.plan['components'])
    def __len__(self):return len(self.plan['components'])
    def full_delta_map(self,k):
        if 'delta' not in self.cache:self.cache['delta']=self.load(self.out/'GEOMETRY_DELTA.json')
        return ChainMap(self.cache['delta'][k],self.base[k])
    def __getitem__(self,k):
        self.check_live()
        if k=='certificate':return self.receipts
        if k=='quality':return self.quality
        if k in ('vertices','columns','prism_front'):return ChainMap(self.hot[k],self.base[k])
        if k=='outer':return sorted(self.hot['prism_front'].values())
        if k in ('atlas','forecast'):
            name='NEW_ATLAS.json' if k=='atlas' else 'NEW_FORECAST_DEPENDENCIES.json'
            if k not in self.cache:self.cache[k]=self.load(self.out/name)
            return self.cache[k]
        if k=='faces':return FrontFaces(self)
        if k in ('cells','versions'):return self.full_delta_map(k)
        return self.base[k]
    def load_proof(self):
        if hasattr(self,'pairs'):return
        d=self.plan['components']['certificate']['new_segment'];objects=self.out/'objects'
        self.proof_faces=self.load(objects/d['faces']['file']);self.payloads=self.load(objects/d['payloads']['file'])
        self.pairs=np.load(self.verify(objects/d['pairs']['file']),mmap_mode='r',allow_pickle=False)
        self.codes=np.load(self.verify(objects/d['codes']['file']),mmap_mode='r',allow_pickle=False)
        self.inputs=self.load(self.out/'NEW_RECEIPT_INPUTS.json');self.index=self.verify(objects/'new-proof-key-index.sqlite')
    def initialize(self,wall,tri,surface,request,source):
        from production_forecast_admission import ProductionForecastAdapter
        self.check();a=ProductionForecastAdapter.__new__(ProductionForecastAdapter)
        a._bind(self,wall,tri,surface,request,source,self.plan['expected'],self.plan['scientific'])
        self.check();return a
    def deep_seal(self):
        """One qualification-only legacy digest; never a publication step."""
        self.check();self.receipts.deep=True;self.quality.deep=True
        try:return scientific_digest(self)
        finally:self.receipts.deep=False;self.quality.deep=False
