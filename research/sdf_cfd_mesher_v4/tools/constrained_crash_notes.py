"""Extract ELF core signal/register/mapping evidence without loading its heap."""
import struct,json
from pathlib import Path
from constrained_cavity_audit import OUT,V4,digest,save

def notes(path):
    with path.open('rb') as f:
        h=f.read(64)
        if h[:6]!=b'\x7fELF\x02\x01':raise ValueError('Expected little-endian ELF64 core')
        phoff=struct.unpack_from('<Q',h,32)[0];size,num=struct.unpack_from('<HH',h,54);segments=[];result=[]
        for i in range(num):
            f.seek(phoff+i*size);p=struct.unpack('<IIQQQQQQ',f.read(56));segments.append(p)
        for p in segments:
            if p[0]!=4:continue
            f.seek(p[2]);raw=f.read(p[5]);pos=0
            while pos+12<=len(raw):
                ns,ds,kind=struct.unpack_from('<III',raw,pos);pos+=12;name=raw[pos:pos+ns].rstrip(b'\0');pos+=(ns+3)&~3;data=raw[pos:pos+ds];pos+=(ds+3)&~3
                result.append((name,kind,data))
    maps=[];threads=[];sig=None;info=None
    for name,kind,d in result:
        if name!=b'CORE':continue
        if kind==0x53494749:
            sig=dict(signal=struct.unpack_from('<i',d)[0],code=struct.unpack_from('<i',d,8)[0],fault_address=hex(struct.unpack_from('<Q',d,16)[0]))
        elif kind==1 and len(d)>=328:
            regs=struct.unpack_from('<27Q',d,112);threads.append(dict(pid=struct.unpack_from('<i',d,32)[0],signal=struct.unpack_from('<h',d,12)[0],rip=regs[16],rsp=regs[19],rbp=regs[4]))
        elif kind==3:info=d[40:120].split(b'\0')[0].decode(errors='replace')
        elif kind==0x46494c45:
            n,page=struct.unpack_from('<QQ',d);names=d[16+24*n:].split(b'\0')
            for i in range(n):
                a,b,off=struct.unpack_from('<QQQ',d,16+24*i);maps.append(dict(start=a,end=b,file_offset=off*page,path=names[i].decode(errors='replace')))
    for r in threads:
        match=next((m for m in maps if m['start']<=r['rip']<m['end']),None)
        if match:r.update(mapping=match['path'],instruction_file_offset=hex(r['rip']-match['start']+match['file_offset']))
    return dict(signal=sig,process=info,threads=threads,mappings=maps,note_types=[dict(name=n.decode(errors='replace'),type=k,bytes=len(d)) for n,k,d in result],heap_extracted=False,unique_heap_state_preserved_in_dump=True)

def run():
    rows=[]
    prior=json.loads((V4/'results/bent_terminal_shell_fixed_interface_core/NATIVE_FAILURES.json').read_text())
    for r in prior['crash_dumps']:
        p=Path(r['path']);q=notes(p);rows.append(dict(**r,evidence=q,deleted=False))
        print(json.dumps(dict(path=str(p),signal=q['signal'],threads=q['threads'])),flush=True)
    save(OUT/'CRASH_NOTES.json',rows)

if __name__=='__main__':run()
