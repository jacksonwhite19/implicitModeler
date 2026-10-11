"""Post-seal comparison only. Never imported by candidate construction."""
from pathlib import Path
from datetime import datetime, timezone
import json, os, sys, time


def require_seal(out):
    def read(n):return json.loads((out/n).read_text())
    seal=read('CANDIDATE_SEAL.json');published=read('PUBLICATION.json');reopen=read('TRUSTED_REOPEN.json')
    if seal['root']!=published['root'] or reopen['root']!=seal['root']:
        raise ValueError('CandidateNotPublishedReopenedAndSealed')
    if seal['oracle_unlocked'] or reopen['forecast_invoked']:
        raise ValueError('InvalidPreOracleScope')
    if not seal.get('legacy_scientific') or not seal.get('completed_utc'):
        raise ValueError('IncompleteCandidateSeal')
    return seal


def compare(out):
    from collections import Counter
    import psutil
    from contextlib import ExitStack
    import immutable_state_access as access
    from clean_sheet_candidate import Candidate
    from path_preserving_deduplication import durable_json,file_hash
    from rolling_layer_retention import descriptors
    from sdf_cfd_mesher_v4.common.authority import identity
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
    seal=require_seal(out)
    result_dir=out/'oracle';result_dir.mkdir(exist_ok=True)
    if (result_dir/'UNLOCK.json').exists():raise ValueError('OracleComparisonAlreadyStartedNoRepairAllowed')
    start=time.perf_counter();cpu=time.process_time();proc=psutil.Process();io=proc.io_counters();reads=Counter()
    def audit(event,args):
        if event=='open' and not isinstance(args[0],int):
            p=Path(os.fsdecode(args[0])).resolve();write=bool(args[2] & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND))
            if write and not p.is_relative_to(result_dir):raise ValueError('PostOracleCandidateWriteForbidden:'+str(p))
            if not write:reads[str(p)]+=1
    sys.addaudithook(audit)
    with Candidate(out/'candidate',seal['root']) as candidate,ExitStack() as held:
        # Holding the complete candidate closure prevents file mutation during
        # comparison. No oracle pathname is opened above this point.
        unlocked=datetime.now(timezone.utc).isoformat()
        if unlocked<=seal['completed_utc']:raise ValueError('OracleUnlockPrecedesSeal')
        durable_json(result_dir/'UNLOCK.json',dict(candidate_seal=file_hash(out/'CANDIDATE_SEAL.json'),
            candidate_root=seal['root'],candidate_scientific=seal['legacy_scientific'],
            seal_completed_utc=seal['completed_utc'],oracle_unlock_utc=unlocked))
        v4=Path(__file__).resolve().parents[1]
        oracle=v4/'results/production_adaptive_capsule_layers4_to5/L5/A/checkpoints'
        pointer=access.read(oracle/'CURRENT.json')
        expected='686c220a55965e0ee2868f2ab8502c6cda3ef6b6db8e97fca4806cf04afc0ec7'
        checkpoint_hash='c623ccf811899964fb9e8e537339730bfb2c037f9dad99b5be1ce39bfd60f98b'
        held.enter_context(access.guard(oracle/'CURRENT.json'))
        checkpoint=oracle/pointer['checkpoint'];held.enter_context(access.guard(checkpoint))
        if pointer['scientific']!=expected or pointer['sha256']!=checkpoint_hash or file_hash(checkpoint)!=checkpoint_hash:
            raise ValueError('HistoricalOracleAuthorityMismatch')
        envelope=access.read(checkpoint)
        if envelope['scientific']!=expected:raise ValueError('WrongOracleState')
        checked={};hashed_bytes=0
        for d in descriptors(envelope['components']):
            p=oracle/'objects'/d['file'];held.enter_context(access.guard(p));s=p.stat();key=(s.st_dev,s.st_ino)
            if key not in checked:checked[key]=file_hash(p);hashed_bytes+=s.st_size
            if checked[key]!=d['sha256']:raise ValueError('CorruptHistoricalOracleObject')
        results={};candidate.receipts.deep=True;candidate.quality.deep=True
        for k in sorted(envelope['components']):
            tick=time.perf_counter();actual=scientific_digest(candidate[k]);expected_hash=envelope['components'][k]['scientific']
            results[k]=dict(candidate=actual,oracle=expected_hash,equal=actual==expected_hash,seconds=time.perf_counter()-tick)
            durable_json(result_dir/'COMPONENT_COMPARISON.json',results)
            print('ORACLE COMPONENT',k,actual==expected_hash,results[k]['seconds'],flush=True)
        candidate.check()
        result=dict(passed=all(r['equal'] for r in results.values()) and seal['legacy_scientific']==expected,
            scientific_equal=seal['legacy_scientific']==expected,candidate_scientific=seal['legacy_scientific'],oracle_scientific=expected,
            components=results,oracle_checkpoint_sha256=checkpoint_hash,oracle_unique_objects_hashed=len(checked),oracle_bytes_hashed=hashed_bytes,
            comparison='Exact canonical component equality, including all quality and receipt proof/binding fields; oracle components authenticated by sealed checkpoint and complete byte hashes',
            candidate_mutated=False,seal_completed_utc=seal['completed_utc'],oracle_unlock_utc=unlocked,
            completed_utc=datetime.now(timezone.utc).isoformat(),seconds=time.perf_counter()-start,cpu=time.process_time()-cpu,
            peak_rss=proc.memory_info().peak_wset,retained_rss=proc.memory_info().rss,
            io={k:getattr(proc.io_counters(),k)-getattr(io,k) for k in io._fields},candidate_deep_rows=dict(candidate.stats))
        durable_json(result_dir/'COMPARISON.json',result)
    durable_json(result_dir/'READ_LOG.json',dict(reads=[dict(path=p,count=n) for p,n in sorted(reads.items())],
        oracle_access='Only after UNLOCK.json and sealed/published/reopened candidate',candidate_writes=0))


if __name__=='__main__':
    compare(Path(__file__).resolve().parents[1]/'results/clean_sheet_incremental_l4_to_l5/resume')
