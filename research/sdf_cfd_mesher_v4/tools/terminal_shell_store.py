"""Content-addressed final terminal shell checkpoint; atomic pointer publication.

Only trusted local payloads are loaded, after byte and implementation checks.
The payload holds current native geometry, compact proof arrays and receipts.
No historical raw receipt overlays are required. Independent seal is mandatory.
"""
import gzip,pickle,os,json
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import file_hash,identity
SCHEMA='V4TerminalMixedShellCheckpoint/2'

def compact(data):
    if 'proofs' not in data:return data
    result=dict(data);proofs=result.pop('proofs')
    result['compact_proofs']=dict(segments=[{k:getattr(s,k) for k in ('faces','pairs','codes','payloads','mapping','descriptor')} for s in proofs.segments],fresh=vars(proofs.fresh))
    return result

def expand(data):
    if 'compact_proofs' not in data:return data
    from production_fast_layer import MemorySegment
    from native_proof_segments import SegmentProofRows
    from compact_proof_rows import CompactProofRows
    saved=data.pop('compact_proofs');segments=[]
    for values in saved['segments']:
        segment=object.__new__(MemorySegment);segment.__dict__.update(values);segments.append(segment)
    fresh=CompactProofRows();fresh.__dict__.update(saved['fresh'])
    data['proofs']=SegmentProofRows(segments,fresh);return data

def publish(path,data,validate):
    path=Path(path);path.mkdir(parents=True,exist_ok=True)
    if validate(data) is not True:raise ValueError('TerminalPublicationRejected')
    temp=path/'PAYLOAD.pending'
    with temp.open('wb') as raw:
        with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0,compresslevel=1) as f:pickle.dump(compact(data),f,protocol=5)
        raw.flush();os.fsync(raw.fileno())
    h=file_hash(temp);dest=path/(h+'.pickle.gz');os.replace(temp,dest)
    record=dict(schema=SCHEMA,payload=dest.name,sha256=h,bytes=dest.stat().st_size,
                scientific=data['scientific'],capacity=data['state']['local_schedule']['terminal_capacity'],
                implementation=file_hash(Path(__file__)),final_seal_required=True)
    pointer=path/'CURRENT.pending'
    with pointer.open('w') as f:
        f.write(json.dumps(record,sort_keys=True,indent=2));f.flush();os.fsync(f.fileno())
    os.replace(pointer,path/'CURRENT.json');return record

def restart(path):
    path=Path(path);r=json.loads((path/'CURRENT.json').read_text())
    if r['schema']!=SCHEMA or Path(r['payload']).name!=r['payload']:raise ValueError('TerminalCheckpointSchema')
    if file_hash(Path(__file__))!=r['implementation']:raise ValueError('TerminalCheckpointImplementation')
    if file_hash(path/r['payload'])!=r['sha256']:raise ValueError('TerminalCheckpointCorruption')
    with gzip.open(path/r['payload'],'rb') as f:d=expand(pickle.load(f))
    if d['scientific']!=r['scientific'] or d['state']['local_schedule']['terminal_capacity']!=r['capacity']:raise ValueError('TerminalCheckpointBinding')
    return d
