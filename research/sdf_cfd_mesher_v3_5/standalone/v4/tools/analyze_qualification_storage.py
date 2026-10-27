"""Analyze audited bytes and immutable receipt rows; no geometry or publication."""
import collections,json,sqlite3,time
from pathlib import Path
from contextlib import closing
from audit_qualification_storage import ROOT,V4,OUT,L1,L2,save,digest

def compare_rows(a,b):
    """Sorted exact key/value comparison, bounded to two rows in memory."""
    with closing(sqlite3.connect(a.resolve().as_uri()+'?mode=ro',uri=True)) as x, closing(sqlite3.connect(b.resolve().as_uri()+'?mode=ro',uri=True)) as y:
        left=iter(x.execute('select key,value from receipts order by key'))
        right=iter(y.execute('select key,value from receipts order by key'))
        a=next(left,None);b=next(right,None);counts=collections.Counter()
        while a is not None or b is not None:
            if b is None or (a is not None and a[0]<b[0]):counts['left_only']+=1;a=next(left,None)
            elif a is None or b[0]<a[0]:counts['right_only']+=1;b=next(right,None)
            else:
                counts['equal' if a[1]==b[1] else 'changed']+=1
                if a[1]!=b[1]:
                    av=json.loads(a[1]);bv=json.loads(b[1])
                    counts['changed_inputs_only' if {k:v for k,v in av.items() if k!='inputs'}=={k:v for k,v in bv.items() if k!='inputs'} else 'changed_other_fields']+=1
                a=next(left,None);b=next(right,None)
        return {k:counts[k] for k in ('equal','changed','left_only','right_only','changed_inputs_only','changed_other_fields')}

def main():
    start=time.perf_counter();rows=json.loads((OUT/'FILES.json').read_text());index={r['path']:r for r in rows}
    attribution=collections.Counter();types=collections.Counter();roots={};checks=[]
    for r in rows:
        p=ROOT/r['path']
        if p.is_relative_to(L2):
            rel=p.relative_to(L2);attribution[rel.parts[0] if len(rel.parts)>1 else 'root_records']+=r['bytes']
            category=('checkpoint_objects' if 'objects' in rel.parts else p.name if p.name in ('QUALITY.json','RAW_QUALITY.json','INCIDENCE.json','VOLUME.json') else 'receipt_retirements' if 'RECEIPT_RETIREMENTS' in rel.parts else 'other')
            types[category]+=r['bytes']
    for name,base in [('L1A',L1/'A/checkpoints'),('L2A',L2/'A/checkpoints'),('L2B',L2/'B/checkpoints')]:
        pointer=json.loads((base/'CURRENT.json').read_text());envelope_path=base/pointer['checkpoint'];env=json.loads(envelope_path.read_text())
        assert digest(envelope_path)==pointer['sha256'];assert env['scientific']==pointer['scientific']
        refs=[]
        def walk(value):
            if isinstance(value,dict):
                if 'file' in value and 'sha256' in value:
                    p=base/'objects'/value['file'];r=index[p.relative_to(ROOT).as_posix()]
                    assert r['sha256']==value['sha256'];refs.append(r['path'])
                for v in value.values():walk(v)
            elif isinstance(value,list):
                for v in value:walk(v)
        walk(env['components'])
        # These current envelopes contain flat JSON components and receipt tables.
        assert all(v['encoding'] in ('json','receipt_table') for v in env['components'].values())
        roots[name]=dict(scientific=pointer['scientific'],components=env['components'],reachable=sorted(set(refs)),
            referenced_bytes=sum(index[p]['bytes'] for p in set(refs)),pointer_sha256=digest(base/'CURRENT.json'))
    save('CHECKPOINT_REACHABILITY.json',roots)
    for name,base in [('L1',L1),('L2',L2)]:
        authority=json.loads((base/'AUTHORITY.json').read_text());n=0
        evidence=authority.get('evidence',{'report.md':authority.get('report_sha256')})
        for rel,h in evidence.items():
            p=base/rel;key=p.relative_to(ROOT).as_posix()
            assert index[key]['sha256']==h,(name,rel);n+=1
        checks.append(dict(authority=name,sha256=digest(base/'AUTHORITY.json'),evidence_files_verified=n))
    duplicate=json.loads((OUT/'DUPLICATES.json').read_text())
    l2rows=[r for r in rows if (ROOT/r['path']).is_relative_to(L2)]
    ids={};contents={}
    for r in l2rows:ids[(r['device'],r['inode'])]=r['bytes'];contents[(r['bytes'],r['sha256'])]=r['bytes']
    summary=dict(attribution=attribution,categories=types,l2_file_count=len(l2rows),l2_logical_bytes=sum(r['bytes'] for r in l2rows),
        l2_unique_file_id_payload_bytes=sum(ids.values()),l2_unique_content_bytes=sum(contents.values()),
        l2_duplicate_payload_upper_bound=sum(ids.values())-sum(contents.values()),authority_checks=checks,
        physical_allocation='NotMeasured; file-ID payload totals exclude hardlink double counting, not filesystem allocation')
    save('SUMMARY.json',summary);print(json.dumps(summary),flush=True)
    objects=L2/'A/checkpoints/objects';cert=objects/roots['L2A']['components']['certificate']['file']
    update=objects/'56cff164272e65e16987ed0afe874590b6efb60d3bee5de8e9b89e987c9ff16a.sqlite'
    prior=L1/'A/checkpoints/objects'/roots['L1A']['components']['certificate']['file']
    comparisons={}
    for name,a in [('intermediate_to_final',update),('l1_to_final',prior)]:
        tick=time.perf_counter();comparisons[name]=dict(counts=compare_rows(a,cert),seconds=time.perf_counter()-tick)
        print(name,comparisons[name],flush=True)
    save('ROW_COMPARISON.json',comparisons)
    import psutil
    save('ANALYSIS_RESOURCES.json',dict(seconds=time.perf_counter()-start,peak_rss=psutil.Process().memory_info().peak_wset))

if __name__=='__main__':main()
