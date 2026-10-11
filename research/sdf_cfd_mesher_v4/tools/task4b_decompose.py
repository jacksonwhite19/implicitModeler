"""Reduce measured profiles. Cumulative times are nested, never additive."""
from task4b_common import *
import collections

def categories(rows):
    out=collections.Counter()
    for r in rows:
        f=r['file'].replace('\\','/');n=r['function'];s=r['self_seconds']
        if any(k in f for k in ('rootisolation.py','rootoftools.py')):key='root_isolation_self'
        elif '/printing/' in f:key='symbolic_string_rendering_self'
        elif '/polys/' in f:key='other_polynomial_arithmetic_self'
        elif '/fractions.py' in f or 'math.gcd' in n:key='rational_arithmetic_self'
        elif '/json/' in f:key='json_self'
        elif any(k in f for k in ('gzip.py','zlib')) or 'compress' in n:key='compression_self'
        elif n in ('write','read','flush') or '_io.' in n:key='file_io_self'
        elif 'hash' in n or 'hashlib' in f:key='hashing_self'
        else:key='other_dispatch_and_python_self'
        out[key]+=s
    selected=[]
    names={'certify','canonical_pair','valid','rank','supporting_planes','cert','bounds','refine','compare','unit_roots','polynomial_record',
           'polynomial_interval','projection','projection_polynomials','sign','is_zero','primitive','root_record','put','get','candidates','conservative_pairs','identity'}
    for r in rows:
        if r['function'] in names and ('V4' in r['file']):selected.append(r)
    return dict(exclusive_self_seconds=dict(out),selected_nested_cumulative_functions=selected)

if __name__=='__main__':
    result={}
    for p in sorted(OUT.glob('profile_*_functions.json')):result[p.stem]=categories(json.loads(p.read_text()))
    for p in sorted(OUT.glob('CASE_*.json')):result[p.stem]=categories(json.loads(p.read_text())['functions'])
    save('PROFILE_DECOMPOSITION.json',dict(note='Self buckets are additive. Cumulative call times overlap. File I/O is Python-observed, not a physical storage latency measurement.',profiles=result))
