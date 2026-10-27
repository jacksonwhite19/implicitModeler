from pathlib import Path
import hashlib,json,zipfile
import numpy as np

def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()



def read(path):return json.loads(Path(path).read_text(encoding='utf8'))



def require(condition,label):
    if not condition:raise ValueError('RetainedDiagnosticMismatch',label)



def same(actual,expected,label):
    require(actual.dtype==expected.dtype and actual.shape==expected.shape and actual.tobytes()==expected.tobytes(),label)



def arrays(path,fields,check):
    check()
    with zipfile.ZipFile(path) as archive:
        members=archive.infolist()
        require({member.filename for member in members}=={name+'.npy' for name in fields}
            and len(members)==len(fields),'archive fields')
        require(sum(member.file_size for member in members)<=512*1024**2,'archive logical bound')
        for member in members:
            with archive.open(member) as stream:
                version=np.lib.format.read_magic(stream)
                require(version in ((1,0),(2,0)),'array version')
                parser=np.lib.format.read_array_header_1_0 if version==(1,0) else np.lib.format.read_array_header_2_0
                shape,fortran,dtype=parser(stream,max_header_size=10000)
                require(len(shape)<=2 and all(type(size) is int and 0<=size<=8000000 for size in shape),'array shape')
                count=1
                for size in shape:count*=size
                require(count<=8000000 and dtype.str==fields[member.filename[:-4]] and not dtype.hasobject,'array type/count')
                require(stream.tell()+count*dtype.itemsize==member.file_size,'array size')
    with np.load(path,allow_pickle=False) as archive:result={name:archive[name] for name in fields}
    for name,value in result.items():
        require(value.dtype.str==fields[name] and np.isfinite(value).all(),'decoded dtype/finite')
        value.flags.writeable=False
    check();return result
