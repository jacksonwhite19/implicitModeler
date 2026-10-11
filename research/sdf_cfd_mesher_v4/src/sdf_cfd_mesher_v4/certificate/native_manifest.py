"""Native owned, memory-mapped scientific relation populations.

Packed integers are a storage index into an explicitly bound source-edge map.
They are never accepted without that map and the qualified geometry binding.
No intersection mathematics is implemented here.
"""
from pathlib import Path
import json, shutil
import numpy as np
from ..common.authority import identity, array_hash, file_hash
from ..advancing_front.rolling_state import atomic_json


class NativeManifest:
    VERSION='V4NativeRelationBlocks/1'
    def __init__(self,path):self.path=Path(path)
    def metadata(self):
        data=json.loads((self.path/'MANIFEST.json').read_text())
        body=dict(data);digest=body.pop('scientific')
        if identity(body)!=digest or body['version']!=self.VERSION:raise ValueError('NativeManifestIdentity')
        return data
    @classmethod
    def import_verified(cls,path,edges,layers,*,append=False):
        """layers: (source directory, qualified verified ledger) pairs.

        The caller must establish complete scientific input equality with the
        qualified verifier before calling. Payload hashes are checked again.
        """
        path=Path(path);path.mkdir(parents=True,exist_ok=True)
        blocks=[]
        if (path/'MANIFEST.json').exists():
            if not append:raise ValueError('NativeManifestAlreadyPublished')
            previous=cls(path).metadata()
            if previous['source_edges']!=array_hash(edges):raise ValueError('NativeManifestSourceChanged')
            blocks=previous['blocks']
        else:np.save(path/'source_edges.npy',edges,allow_pickle=False)
        for source,ledger in layers:
            source=Path(source);raw=json.loads((source/'MANIFEST.json').read_text())
            if raw!=ledger:raise ValueError('ManifestLedgerMismatch')
            number=raw['layer']
            if append and number<=len(blocks):
                # A crash may occur after proof publication but before the
                # geometry transaction commits. Exact retry is idempotent.
                if blocks[number-1]['ledger']!=raw:raise ValueError('NativeProofReplayChanged')
                for row in blocks[number-1]['arrays'].values():
                    if file_hash(path/row['file'])!=row['sha256']:raise ValueError('NativeProofReplayCorrupt')
                continue
            if number!=len(blocks)+1:raise ValueError('NativeProofLayerGap')
            if raw['layer']!=number or not raw['complete'] or raw['forbidden'] or raw['unresolved'] or raw['invalid_surfaces']:raise ValueError('UnqualifiedManifestImport')
            destination=path/f'block_{number:02}';destination.mkdir(exist_ok=True)
            hashes={}
            expected=dict(packed='pair_hash',codes='code_hash',witness='witness_hash',outcomes='outcome_hash',validity='validity_hash')
            with np.load(source/'COMPACT.npz') as z:
                for key,field in expected.items():
                    value=z[key]
                    if array_hash(value)!=raw[field]:raise ValueError('NativeProofImportHashMismatch')
                    target=destination/(key+'.npy');np.save(target,value,allow_pickle=False)
                    hashes[key]=dict(file=target.relative_to(path).as_posix(),sha256=file_hash(target),scientific=raw[field])
            # Witness storage is copied once. It is not expanded in memory.
            shutil.copyfile(source/'EXACT.json.gz',destination/'exact.json.gz')
            hashes['exact']=dict(file=(destination/'exact.json.gz').relative_to(path).as_posix(),sha256=file_hash(source/'EXACT.json.gz'))
            blocks.append(dict(layer=number,ledger=raw,arrays=hashes))
        data=dict(version=cls.VERSION,source_edges=array_hash(edges),edge_count=len(edges),blocks=blocks,
                  face_identity='(bilinear_side, layer, source_edge_endpoints); geometry/labels bound by block ledger',
                  relation_count=sum(b['ledger']['completed_pairs'] for b in blocks))
        atomic_json(path/'MANIFEST.json',dict(data,scientific=identity(data)))
        return cls(path)
    def array(self,block,key):
        row=block['arrays'][key];p=(self.path/row['file']).resolve()
        if not p.is_relative_to(self.path.resolve()):raise ValueError('NativeProofPath')
        return np.load(p,mmap_mode='r',allow_pickle=False)
    def audit(self,upto=None):
        data=self.metadata();edges=np.load(self.path/'source_edges.npy',mmap_mode='r')
        if array_hash(edges)!=data['source_edges']:raise ValueError('NativeSourceFaceMapMismatch')
        count=0
        blocks=data['blocks'] if upto is None else data['blocks'][:upto]
        for block in blocks:
            for row in block['arrays'].values():
                if file_hash(self.path/row['file'])!=row['sha256']:raise ValueError('NativeProofPayloadMismatch')
            packed=self.array(block,'packed');codes=self.array(block,'codes');outcomes=self.array(block,'outcomes')
            if len(packed)!=block['ledger']['completed_pairs'] or len(codes)!=len(packed) or len(outcomes)!=len(packed):raise ValueError('NativeRelationCoverageMismatch')
            if np.any(packed[1:]<=packed[:-1]):raise ValueError('NativeDuplicateOrUnorderedRelation')
            a=packed>>np.uint64(32);b=packed&np.uint64(0xffffffff)
            if np.any(a>=b) or np.any(np.maximum(a,b)//len(edges)+1!=block['layer']):raise ValueError('NativeRelationOwnership')
            # Original qualified outcome encoding: verify against the bound
            # scientific array hash rather than reinterpreting proof semantics.
            for key in ('packed','codes','witness','outcomes','validity'):
                if array_hash(self.array(block,key))!=block['arrays'][key]['scientific']:raise ValueError('NativeScientificProofMismatch')
            count+=len(packed)
        if count!=sum(b['ledger']['completed_pairs'] for b in blocks):raise ValueError('NativeMissingRelations')
        return dict(expected=count,active=count,missing=0,duplicates=0,forbidden=0,unresolved=0,recomputed=0,scientific=data['scientific'])
    def lookup(self,first,second):
        """Scientific faces are (layer, endpoint_a, endpoint_b)."""
        data=self.metadata();edges=np.load(self.path/'source_edges.npy',mmap_mode='r')
        def index(face):
            layer,a,b=face;edge=np.sort([a,b]);matches=np.flatnonzero(np.all(edges==edge,axis=1))
            if len(matches)!=1 or layer<1 or layer>len(data['blocks']):raise KeyError(face)
            return (layer-1)*len(edges)+int(matches[0])
        a,b=sorted((index(first),index(second)));owner=max(first[0],second[0])
        block=data['blocks'][owner-1];packed=self.array(block,'packed');key=np.uint64((a<<32)|b)
        position=int(np.searchsorted(packed,key))
        if position==len(packed) or packed[position]!=key:raise KeyError((first,second))
        return dict(faces=[first,second],proof_class=int(self.array(block,'codes')[position]),
                    witness_type=int(self.array(block,'witness')[position]),outcome=int(self.array(block,'outcomes')[position]),
                    geometry=block['ledger']['geometry_hash'],exact_witness_store=block['arrays']['exact'])
