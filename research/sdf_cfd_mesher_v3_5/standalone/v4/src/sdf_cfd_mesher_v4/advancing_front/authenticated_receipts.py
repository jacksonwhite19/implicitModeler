"""Versioned canonical receipt authority with authenticated delta updates.

The root binds scientific keys and complete canonical receipt values (proof
outcomes AND input dependencies). It is deliberately not the legacy JSON SHA.
Legacy scientific_digest remains available for independent transition audits.
"""
from pathlib import Path
from array import array
import hashlib
import numpy as np
from .authenticated_map import Nodes,EMPTY,update,key_digest,SCHEMA
from .native_receipt_delta import raw_items,ReceiptDelta
from ..common.authority import file_hash,identity

AUTHORITY='V4CertificateReceiptPatricia/1'


def storage_identity(table):
    if isinstance(table,ReceiptDelta):
        return identity(dict(base=storage_identity(table.base),updates=storage_identity(table.updates),removed=sorted(table.removed)))
    from .native_checkpoint import receipt_storage_signature
    signature=receipt_storage_signature(table)
    prior=getattr(table,'_content_hash',None)
    if prior is not None and prior[0]==signature:return prior[1]
    value=file_hash(table.path);table._content_hash=(signature,value)
    return value


def verify(table,directory):
    import sqlite3
    from contextlib import closing
    from .authenticated_map import digest,population
    from .verified_objects import verify as verify_file
    d=table.authenticated
    from . import canonical_blocks as blocks
    if d.get('authority')==blocks.AUTHORITY:
        if d['storage']!=storage_identity(table) or d['count']!=len(table):raise ValueError('AuthenticatedReceiptStorageMismatch')
        blocks.verify(d,directory)
        return
    if d.get('authority')!=AUTHORITY or d.get('schema')!=SCHEMA:raise ValueError('UnsupportedReceiptRootAuthority')
    if d['storage']!=storage_identity(table):raise ValueError('AuthenticatedReceiptStorageMismatch')
    root=bytes.fromhex(d['root']);found=root==EMPTY
    for row in reversed(d['files']):
        if Path(row['file']).name!=row['file']:raise ValueError('AuthenticatedReceiptObjectPath')
        path=verify_file(Path(directory)/row['file'],row['sha256'])
        if not found:
            with closing(sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)) as db:
                node=db.execute('SELECT data FROM nodes WHERE hash=?',(root,)).fetchone()
            if node is not None:
                if digest(node[0])!=root or population(node[0])!=len(table):raise ValueError('AuthenticatedReceiptRootMismatch')
                found=True
    if not found or d['count']!=len(table) or (root==EMPTY and len(table)):
        raise ValueError('AuthenticatedReceiptRootMissing')


def buffers(rows):
    keys=array('B');values=array('B');actions=array('B')
    for key,value,action in rows:
        keys.extend(key_digest(key));values.extend(hashlib.sha256(value.encode()).digest());actions.append(action)
    k=np.frombuffer(keys,dtype='S32');v=np.frombuffer(values,dtype='S32');a=np.frombuffer(actions,dtype='u1')
    return ordered_buffers(k,v,a)


def ordered_buffers(k,v,a):
    order=np.argsort(k,kind='stable')
    return k[order],v[order],a[order]


def bootstrap(table,directory):
    if hasattr(table,'authenticated'):return table.authenticated
    nodes=Nodes(directory)
    try:root=update(nodes,EMPTY,*buffers((k,v,0) for k,v in raw_items(table)))
    except BaseException:
        nodes.abort();raise
    descriptor=nodes.seal(root)
    if descriptor['count']!=len(table):raise ValueError('AuthenticatedBootstrapCount')
    descriptor.update(authority=AUTHORITY,legacy_scientific=table.scientific,storage=storage_identity(table))
    table.authenticated=descriptor;table.auth_objects=Path(directory)
    return descriptor


def advance(base,updates,removed,actions,directory):
    descriptor=base.authenticated
    from . import canonical_blocks as blocks
    if descriptor['authority']==blocks.AUTHORITY:
        result=blocks.update(directory,descriptor,*buffers(actions))
        table=ReceiptDelta(base,updates,removed,count=result['count'],scientific=result['root'])
        result['storage']=storage_identity(table)
        table.authenticated=result;table.auth_objects=Path(directory)
        return table
    if descriptor['authority']!=AUTHORITY or descriptor['schema']!=SCHEMA:
        raise ValueError('UnsupportedReceiptRootAuthority')
    nodes=Nodes(directory,descriptor['files'])
    try:root=update(nodes,bytes.fromhex(descriptor['root']),*buffers(actions))
    except BaseException:
        nodes.abort();raise
    result=nodes.seal(root);result['authority']=AUTHORITY
    table=ReceiptDelta(base,updates,removed,count=result['count'],scientific=result['root'])
    result['storage']=storage_identity(table)
    table.authenticated=result;table.auth_objects=Path(directory)
    return table


def component(table):
    descriptor=table.authenticated
    return dict(authority=descriptor['authority'],schema=descriptor['schema'],count=len(table),root=descriptor['root'])
