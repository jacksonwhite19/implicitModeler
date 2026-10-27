"""Canonical fixed-prefix authenticated blocks; active key/value digests only.

Partition and root depend only on schema, prefix depth and active content.
Storage layout and update order never enter scientific identity.
"""
import hashlib,os,struct,tempfile,time
from pathlib import Path
import numpy as np
from .verified_objects import verify as verify_file

SCHEMA='V4CanonicalSortedBlocks/1'
AUTHORITY='V4CertificateReceiptBlocks/1'
MAGIC=SCHEMA.encode()+b'\0'
HEADER=struct.Struct('>HIQ')
ROW=np.dtype([('key','S32'),('value','S32')])

def prefix(keys,depth):
    if not 1<=depth<=16:raise ValueError('BlockPrefixDepth')
    b=np.ascontiguousarray(keys).view('u1').reshape(-1,32)
    return ((b[:,0].astype('u4')<<8)|b[:,1])>>(16-depth)

def ordered(keys,values):
    if keys.dtype!=np.dtype('S32') or values.dtype!=np.dtype('S32') or len(keys)!=len(values):raise ValueError('BlockRowShape')
    if len(keys)>1 and np.any(keys[1:]<=keys[:-1]):raise ValueError('DuplicateOrUnorderedBlockKey')

def upper(blocks,depth):
    if not 1<=depth<=16:raise ValueError('BlockPrefixDepth')
    if any(not 0<=x['prefix']<(1<<depth) or x['count']<=0 for x in blocks):raise ValueError('InvalidBlockRange')
    count=0;nodes=[];by={x['prefix']:x for x in blocks}
    if len(by)!=len(blocks):raise ValueError('DuplicateBlockRange')
    for i in range(1<<depth):
        row=by.get(i);n=row['count'] if row else 0;count+=n
        content=bytes.fromhex(row['sha256']) if row else hashlib.sha256(MAGIC+HEADER.pack(depth,i,0)).digest()
        nodes.append(hashlib.sha256(MAGIC+b'L'+HEADER.pack(depth,i,n)+content).digest())
    levels=[nodes];level=0
    while len(nodes)>1:
        nodes=[hashlib.sha256(MAGIC+b'U'+struct.pack('>H',level)+nodes[i]+nodes[i+1]).digest() for i in range(0,len(nodes),2)]
        levels.append(nodes);level+=1
    root=hashlib.sha256(MAGIC+b'R'+struct.pack('>HQ',depth,count)+nodes[0]).hexdigest()
    return root,count,levels

def payload(keys,values,depth,part):
    ordered(keys,values)
    if len(keys) and np.any(prefix(keys,depth)!=part):raise ValueError('WrongCanonicalBlockRange')
    rows=np.empty(len(keys),dtype=ROW);rows['key']=keys;rows['value']=values
    return MAGIC+HEADER.pack(depth,part,len(keys))+rows.tobytes()

def store(directory,keys,values,depth,part,work):
    if not len(keys):return None
    tick=time.perf_counter();data=payload(keys,values,depth,part);h=hashlib.sha256(data).hexdigest()
    work['content_hash_seconds']+=time.perf_counter()-tick;path=Path(directory)/(h+'.cblock')
    if path.exists():
        verify_file(path,h);work['duplicate_block_hits']+=1;work['bytes_avoided']+=len(data)
    else:
        tick=time.perf_counter();fd,name=tempfile.mkstemp(prefix='block-',suffix='.pending',dir=directory)
        try:
            with os.fdopen(fd,'wb') as out:out.write(data);out.flush();os.fsync(out.fileno())
            os.replace(name,path)
        finally:
            if Path(name).exists():Path(name).unlink()
        work['write_seconds']+=time.perf_counter()-tick;work['bytes_written']+=len(data);work['block_writes']+=1
    return dict(prefix=part,count=len(keys),file=path.name,sha256=h,bytes=len(data))

def load(directory,row,depth):
    if Path(row['file']).name!=row['file']:raise ValueError('BlockObjectPath')
    path=verify_file(Path(directory)/row['file'],row['sha256']);data=path.read_bytes()
    offset=len(MAGIC)+HEADER.size
    if not data.startswith(MAGIC) or HEADER.unpack(data[len(MAGIC):offset])!=(depth,row['prefix'],row['count']):raise ValueError('BlockHeaderMismatch')
    if len(data)!=offset+64*row['count'] or len(data)!=row['bytes']:raise ValueError('BlockPayloadLength')
    rows=np.frombuffer(data,dtype=ROW,offset=offset);ordered(rows['key'],rows['value'])
    if len(rows) and np.any(prefix(rows['key'],depth)!=row['prefix']):raise ValueError('WrongCanonicalBlockRange')
    return rows['key'],rows['value']

def work_record():
    return dict(block_writes=0,bytes_written=0,duplicate_block_hits=0,bytes_avoided=0,content_hash_seconds=0.,write_seconds=0.)

def finish(blocks,depth,work,previous=None):
    tick=time.perf_counter();root,count,levels=upper(blocks,depth)
    old=upper(previous['blocks'],depth)[2] if previous else None
    work['upper_nodes_rewritten']=sum(sum(a!=b for a,b in zip(rows,old[i])) if old else len(rows) for i,rows in enumerate(levels))
    work['upper_internal_nodes_rewritten']=sum(sum(a!=b for a,b in zip(rows,old[i])) if old else len(rows) for i,rows in enumerate(levels) if i)
    work['upper_seconds']=time.perf_counter()-tick
    return dict(schema=SCHEMA,authority=AUTHORITY,depth=depth,root=root,count=count,blocks=blocks,
                files=[dict(file=x['file'],sha256=x['sha256']) for x in blocks],work=work)

def build(directory,keys,values,depth):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);ordered(keys,values)
    ids=prefix(keys,depth);cuts=np.searchsorted(ids,np.arange((1<<depth)+1));blocks=[];work=work_record()
    for part in range(1<<depth):
        a,b=map(int,cuts[part:part+2]);row=store(directory,keys[a:b],values[a:b],depth,part,work)
        if row:blocks.append(row)
    work.update(touched_blocks=len(blocks),untouched_blocks=0,bytes_reused=0)
    return finish(blocks,depth,work)

def update(directory,prior,keys,values,actions):
    ordered(keys,values)
    if prior['schema']!=SCHEMA or prior['authority']!=AUTHORITY:raise ValueError('BlockAuthorityVersion')
    if len(actions)!=len(keys) or np.any(actions>2) or np.any(actions<0):raise ValueError('BlockDeltaActions')
    prior_root,prior_count,_=upper(prior['blocks'],prior['depth'])
    if prior_root!=prior['root'] or prior_count!=prior['count']:raise ValueError('BlockPriorRootMismatch')
    depth=prior['depth'];ids=prefix(keys,depth);cuts=np.searchsorted(ids,np.arange((1<<depth)+1))
    old={x['prefix']:x for x in prior['blocks']};blocks=[];work=work_record();touched=0;reused=0
    work.update(insert_count=int(np.count_nonzero(actions==0)),replace_count=int(np.count_nonzero(actions==1)),retire_count=int(np.count_nonzero(actions==2)),delta_array_bytes=keys.nbytes+values.nbytes+actions.nbytes,maximum_block_array_bytes=0)
    rebound=set(map(int,ids[actions==1]));work['receipt_rebound_blocks']=len(rebound)
    work['old_block_bytes_in_rebound_ranges']=sum(old[i]['bytes'] for i in rebound if i in old)
    for part in range(1<<depth):
        a,b=map(int,cuts[part:part+2]);row=old.get(part)
        if a==b:
            if row:blocks.append(row);reused+=row['bytes']
            continue
        touched+=1;dk=keys[a:b];dv=values[a:b];action=actions[a:b]
        ok,ov=load(directory,row,depth) if row else (np.empty(0,dtype='S32'),np.empty(0,dtype='S32'))
        positions=np.searchsorted(ok,dk);present=positions<len(ok)
        present[present]=ok[positions[present]]==dk[present]
        if np.any((action==0)&present):raise ValueError('DuplicateBlockKey')
        if np.any((action!=0)&~present):raise ValueError('MissingBlockKey')
        keep=np.ones(len(ok),dtype=bool);keep[positions[action==2]]=False
        nk=np.union1d(ok[keep],dk[action==0]);nv=np.empty(len(nk),dtype='S32')
        nv[np.searchsorted(nk,ok[keep])]=ov[keep];live=action!=2
        nv[np.searchsorted(nk,dk[live])]=dv[live]
        work['maximum_block_array_bytes']=max(work['maximum_block_array_bytes'],sum(x.nbytes for x in (ok,ov,positions,present,keep,nk,nv,live)))
        new=store(directory,nk,nv,depth,part,work)
        if new:blocks.append(new)
    work.update(touched_blocks=touched,untouched_blocks=len(old)-sum(int(cuts[i]!=cuts[i+1]) for i in old),bytes_reused=reused)
    return finish(blocks,depth,work,prior)

def verify(descriptor,directory):
    if descriptor['schema']!=SCHEMA or descriptor['authority']!=AUTHORITY:raise ValueError('BlockAuthorityVersion')
    if descriptor['files']!=[dict(file=x['file'],sha256=x['sha256']) for x in descriptor['blocks']]:raise ValueError('BlockFileManifest')
    for row in descriptor['blocks']:load(directory,row,descriptor['depth'])
    root,count,_=upper(descriptor['blocks'],descriptor['depth'])
    if root!=descriptor['root'] or count!=descriptor['count']:raise ValueError('BlockRootMismatch')

def canonical_root(keys,values,depth):
    """Independent full-population construction; no stored block bytes read."""
    ordered(keys,values);ids=prefix(keys,depth);cuts=np.searchsorted(ids,np.arange((1<<depth)+1));blocks=[]
    for part in range(1<<depth):
        a,b=map(int,cuts[part:part+2])
        if a!=b:blocks.append(dict(prefix=part,count=b-a,sha256=hashlib.sha256(payload(keys[a:b],values[a:b],depth,part)).hexdigest()))
    return upper(blocks,depth)[0]
