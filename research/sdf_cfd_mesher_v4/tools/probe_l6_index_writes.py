"""Isolated sealed-input index writer measurement; no production publication.

Normal committed-state access is deliberately not bypassed. This probe verifies
the individual input hashes under read guards and constructs only an index.
"""
from pathlib import Path
from contextlib import ExitStack
import json,time,sys,sqlite3,hashlib,subprocess,shutil
import numpy as np
import psutil
import immutable_state_access as access
from path_preserving_deduplication import durable_json,file_hash,metadata
from sdf_cfd_mesher_v4.common.authority import identity
from clean_sheet_l4_bootstrap import metrics

V4=Path(__file__).resolve().parents[1]
SEALED=V4/'results/recursive_layer6'
OUT=V4/'results/l6_write_pair_scaling'
def read(p):return json.loads(p.read_text())
def tick():return time.perf_counter(),time.process_time(),psutil.Process().io_counters()
def semantic(db):
    digest=hashlib.sha256();count=0
    for key,ordinal in db.execute('SELECT key,ordinal FROM proof_keys ORDER BY key'):
        digest.update(key);digest.update(int(ordinal).to_bytes(8,'little'));count+=1
    return dict(count=count,sha256=digest.hexdigest())

def run(mode):
    OUT.mkdir(exist_ok=True)
    if (OUT/'AUTHORITY.json').exists():raise ValueError('PreserveSealedStudy')
    if mode=='preflight':
        a=read(SEALED/'AUTHORITY.json')
        for n,h in a['files'].items():assert file_hash(SEALED/n)==h,n
        ca=V4/'results/composable_incremental_state'
        for n,h in read(ca/'AUTHORITY.json')['files'].items():assert file_hash(ca/n)==h,n
        for n,h in read(ca/'IMPLEMENTATION.json')['files'].items():assert file_hash(V4/n)==h,n
        ref=a['reference'];rootfile=Path(ref['directory'])/('ROOT_'+ref['root']+'.json');body=read(rootfile)
        assert file_hash(rootfile)=='a4b40f3b0a071ab18b2cc117cc195ca030eab943d112556f9b78833f37b35007'
        from composable_state import CommittedState
        try:
            with CommittedState(ref):pass
        except ValueError as e:
            assert str(e)=='IncrementalEpochChanged';failure=str(e)
        else:raise ValueError('ExpectedObservedEpochBlockerChanged')
        old={n:read(SEALED/(n+'_METRICS.json'))['metrics'] for n in ('ADMISSION','GENERATION','ASSEMBLY','PUBLICATION')}
        total=sum(v['io']['write_bytes'] for v in old.values());assert total==20539570632
        status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
        entries=[x for x in status.split(b'\0') if x and b'probe_l6_index_writes.py' not in x and b'results/l6_write_pair_scaling' not in x]
        assert len(entries)==155
        result=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),branch=subprocess.check_output(['git','branch','--show-current'],text=True).strip(),status_hex=b'\0'.join(entries).hex(),
            verified_authority=file_hash(SEALED/'AUTHORITY.json'),checkpoint=file_hash(rootfile),root=ref['root'],scientific=a['scientific'],
            blocked=failure,sealed_epoch=body['epoch'],current_epoch=access.epoch(),free=shutil.disk_usage(V4).free,
            historical_phases=old,historical_total_writes=total,per_file_historical_attribution='Unavailable: original run retained phase process counters, not per-file OS traces')
        durable_json(OUT/'PREFLIGHT.json',result);print(json.dumps(result,indent=2));return
    assert mode in ('baseline','cache_control')
    path=OUT/(mode+'.sqlite')
    if path.exists():raise ValueError('PreserveExistingControl')
    cert=read(SEALED/'FRESH_CERTIFICATE.json');desc=cert['segment'];phases={};samples=[]
    with ExitStack() as held:
        for name in ('faces','pairs'):
            p=SEALED/'objects'/desc[name]['file'];held.enter_context(access.guard(p))
            assert file_hash(p)==desc[name]['sha256']
        faces=read(SEALED/'objects'/desc['faces']['file']);pairs=np.load(SEALED/'objects'/desc['pairs']['file'],mmap_mode='r',allow_pickle=False)
        assert len(pairs)==3576991
        start=tick();db=sqlite3.connect(path)
        if mode=='cache_control':db.execute('PRAGMA cache_size='+str(read(OUT/'BASELINE.json')['page_count']))
        pragmas={k:db.execute('PRAGMA '+k).fetchone()[0] for k in ('cache_size','cache_spill','journal_mode','synchronous','temp_store','page_size')}
        t=tick();db.execute('CREATE TABLE proof_keys(key BLOB PRIMARY KEY, ordinal INTEGER NOT NULL UNIQUE) WITHOUT ROWID');phases['schema']=metrics(*t)
        t=tick()
        for offset in range(0,len(pairs),10000):
            values=[(bytes.fromhex(identity(sorted((faces[int(a)],faces[int(b)])))),offset+j) for j,(a,b) in enumerate(pairs[offset:offset+10000])]
            db.executemany('INSERT INTO proof_keys VALUES (?,?)',values)
            files=[dict(path=str(p),logical=p.stat().st_size) for p in OUT.glob(mode+'.sqlite*')]
            samples.append(dict(rows=min(offset+10000,len(pairs)),files=files))
        phases['insert_new_pairs']=metrics(*t)
        t=tick();db.commit();phases['commit']=metrics(*t)
        t=tick();assert db.execute('SELECT COUNT(*) FROM proof_keys').fetchone()[0]==len(pairs)
        assert db.execute('PRAGMA integrity_check').fetchone()[0]=='ok';phases['integrity']=metrics(*t)
        page_count=db.execute('PRAGMA page_count').fetchone()[0]
        db.close();construction=metrics(*start)
    # Oracle comparison starts only after candidate construction and close.
    t=tick()
    with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True) as db:actual=semantic(db)
    oracle=SEALED/'objects/new-proof-key-index.sqlite'
    plan=read(SEALED/'CANDIDATE_PLAN.json');assert file_hash(oracle)==plan['references'][str(oracle.resolve())]['sha256']
    with sqlite3.connect(oracle.as_uri()+'?mode=ro&immutable=1',uri=True) as db:expected=semantic(db)
    assert actual==expected
    result=dict(mode=mode,scope='Exact qualified new-proof key index only; not a transaction or complete scientific-state control',
        construction=construction,phases=phases,pragmas=pragmas,page_count=page_count,file=metadata(path),file_sha256=file_hash(path),
        semantic=actual,oracle_equal=True,oracle_comparison=metrics(*t),samples=samples,
        new_pair_rows=len(pairs),historical_pair_rows=0,old_proof_payloads=0,quality_rows=0,full_prefix_hashes=0,deep_copies=0,
        atomic_publication_exercised=False,production_changes=False)
    durable_json(OUT/(mode.upper()+'.json'),result)
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2),flush=True)
if __name__=='__main__':run(sys.argv[1])
