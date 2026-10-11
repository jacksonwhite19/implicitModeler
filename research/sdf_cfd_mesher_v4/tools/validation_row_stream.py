"""Apply the existing JSON normalization one evidence row at a time."""
import json
import generalize_local_remesh as g


def rows(values):
    for value in values:
        yield json.loads(json.dumps(g.s.r.w.clean(value)))


class NormalizedProofRows:
    def __init__(self, values):
        self.source=values
        self.iterator=rows(values)
        self.native_delta=getattr(values,'native_delta',None)
    def __iter__(self):
        return self
    def __next__(self):
        return next(self.iterator)
    def publish_index(self,faces,directory):
        from native_proof_segments import persist_population
        return persist_population(self.source,faces,directory)


def normalized_validation(checked):
    # attach_validation reads these sequences once. Avoid the formerly live
    # whole validation clone and whole JSON serialization string during bind.
    result=dict(checked)
    result['quality']=dict(checked['quality'])
    for key in ('cells','faces','exact'):
        result['quality'][key]=rows(checked['quality'].get(key,()))
    result['certificate']=dict(checked['certificate'])
    result['certificate']['proofs']=NormalizedProofRows(checked['certificate']['proofs'])
    return result


def validation_digest(checked):
    """Exact legacy clean/JSON identity, without cloning the proof population."""
    import hashlib
    if checked.get('certificate',{}).get('authenticated_identity') is not None:
        checked=dict(checked);certificate=dict(checked['certificate'])
        certificate['proofs']=dict(authenticated_component=certificate['authenticated_identity'])
        checked['certificate']=certificate
        from sdf_cfd_mesher_v4.advancing_front.canonical_blocks import AUTHORITY
        checked['identity_schema']=('V4ValidationAuthenticatedBlocks/1' if certificate['authenticated_identity']['authority']==AUTHORITY else 'V4ValidationAuthenticatedComponents/1')
    digest=hashlib.sha256()
    def encoded(value):
        return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    def emit(value,path):
        if path in ((),('quality',),('certificate',)) and isinstance(value,dict):
            normalized={str(k):v for k,v in value.items()}
            digest.update(b'{')
            for index,key in enumerate(sorted(normalized)):
                if index:digest.update(b',')
                digest.update(encoded(key));digest.update(b':');emit(normalized[key],path+(key,))
            digest.update(b'}')
        elif path in (('quality','cells'),('quality','faces'),('quality','exact'),('certificate','proofs')) and isinstance(value,(list,tuple,g.np.ndarray)) and not (isinstance(value,g.np.ndarray) and value.ndim==0):
            digest.update(b'[')
            for index,row in enumerate(value):
                if index:digest.update(b',')
                digest.update(encoded(g.s.r.w.clean(row)))
            digest.update(b']')
        else:digest.update(encoded(g.s.r.w.clean(value)))
    emit(checked,())
    return digest.hexdigest()


def clean_rows_digest(values):
    """Canonical identity of the unchanged normalized ordered row sequence."""
    import hashlib
    digest=hashlib.sha256();digest.update(b'[')
    for index,value in enumerate(values):
        if index:digest.update(b',')
        digest.update(json.dumps(g.s.r.w.clean(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode())
    digest.update(b']');return digest.hexdigest()
