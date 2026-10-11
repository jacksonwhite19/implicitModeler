"""Compact numerical tables sourced only from realized diagnostic records."""
from production_bounded_layers import *

def render():
    rows=[];keys=('min','p1','p5','median','p95','p99','max')
    def walk(value,prefix):
        if not isinstance(value,dict):return
        if all(k in value for k in keys):
            rows.append(prefix+': '+', '.join(format(value[k],'.9g') for k in keys))
        else:
            for k,v in value.items():walk(v,prefix+'/'+k)
    for layer in (4,5):
        d=read(OUT/f'L{layer}/A/DIAGNOSTICS.json')
        for name in ('height','directions','quality','nonorthogonality','matched'):
            walk(d[name],f'L{layer}/{name}')
        rows.append(f'L{layer}/forecast: '+json.dumps(d['forecast'],sort_keys=True))
    d=read(OUT/'L5/A/DIAGNOSTICS.json')
    walk(d['fronts'],'fronts');walk(d['conditioning'],'conditioning')
    (OUT/'TERMINAL_METRICS.txt').write_text('\n'.join(rows)+'\n',encoding='utf-8')

if __name__=='__main__':render()
