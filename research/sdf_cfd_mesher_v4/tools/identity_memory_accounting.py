"""Measured process and CPython allocator accounting at phase boundaries."""
import os,re,sys,tempfile
import psutil


def snapshot():
    process=psutil.Process();result=dict(process=process.memory_info()._asdict())
    # Windows memory_maps reports mapped region sizes, not page residency.
    # Keep this distinct from working-set RSS and private committed bytes.
    maps=process.memory_maps(grouped=True)
    result['mapped_region_count']=len(maps)
    result['mapped_region_bytes_total']=sum(m.rss for m in maps)
    result['mapped_region_bytes']={m.path:m.rss for m in sorted(maps,key=lambda m:m.rss,reverse=True)[:20]}
    result['mapped_region_scope']='OS mapped region sizes; NOT resident pages and not additive to RSS'
    if hasattr(sys,'_debugmallocstats'):
        with tempfile.TemporaryFile(mode='w+b') as target:
            saved=os.dup(2)
            try:
                os.dup2(target.fileno(),2);sys._debugmallocstats()
            finally:os.dup2(saved,2);os.close(saved)
            target.seek(0);text=target.read().decode(errors='replace')
        result['cpython_allocator_text']=text
        result['cpython_allocator_counters']={k.strip():int(v.replace(',',''))
            for k,v in re.findall(r'^# (.*?)\s*=\s*([\d,]+)\s*$',text,re.M)}
    return result
