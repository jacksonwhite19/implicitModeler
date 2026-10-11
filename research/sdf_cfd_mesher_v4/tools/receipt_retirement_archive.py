"""Keep changed receipt identities on disk, not in accepted-step history."""
import json
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import identity,file_hash


def archive(folder, bound):
    folder=Path(folder)
    retired=bound['retired']
    name=identity(retired)+'.json'
    directory=folder/'RECEIPT_RETIREMENTS'
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/name
    temporary=directory/(name+'.pending')
    encoder=json.JSONEncoder(sort_keys=True,separators=(',',':'),allow_nan=False)
    with temporary.open('w',encoding='utf8',newline='\n') as stream:
        for block in encoder.iterencode(retired):stream.write(block)
        stream.write('\n')
    if path.exists():
        if file_hash(path)!=file_hash(temporary):raise ValueError('ReceiptRetirementArchiveMismatch')
        temporary.unlink()
    else:temporary.replace(path)
    result=dict(bound)
    result['retired']={k:dict(count=len(v),scientific=identity(v),
                            record='RECEIPT_RETIREMENTS/'+name,sha256=file_hash(path))
                       for k,v in retired.items()}
    return result


def retirement_count(value):
    return value['count'] if isinstance(value,dict) and 'record' in value else len(value)
