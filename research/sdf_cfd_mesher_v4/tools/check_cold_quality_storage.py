"""Retained native quality control. No layer, proof or checkpoint generation."""
import argparse,json,shutil,sqlite3,time
from contextlib import closing
from pathlib import Path
import immutable_state_access as access
from compact_initial_quality import bind_shared,parts
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import ReceiptTable,NativeCheckpointStore,scientific_digest,receipt_table_from_rows
from path_preserving_deduplication import durable_json,file_hash
from check_current_front_encoding import allocation


def live_allocation(directory):
    """Telemetry permits a live SQLite writer; it is not authentication."""
    from collect_live_sf2_first_layer import allocation as allocated_bytes
    seen=set();total=0
    for path in Path(directory).rglob('*'):
        try:
            if not path.is_file():continue
            stat=path.stat();key=(stat.st_dev,stat.st_ino)
            if key not in seen:
                size=allocated_bytes(path)
                seen.add(key);total+=size
        except FileNotFoundError:
            # Atomic pending-file renames do not imply retained allocation.
            continue
    return total


def run(store,out,report,minimum_free_bytes,rows_per_population):
    store=Path(store).resolve();out=Path(out).resolve();report=Path(report).resolve()
    if out.exists() or report.exists():raise ValueError('PreserveExistingQualityControl')
    pointer=store/'CURRENT.json';start=time.perf_counter();times={}
    with access.guard(pointer):
        pin=file_hash(pointer);body=json.loads(pointer.read_text(encoding='utf-8'))
        envelope=store/body['checkpoint']
        with access.guard(envelope):
            if file_hash(envelope)!=body['sha256']:raise ValueError('NativeCheckpointCorrupt')
            component=json.loads(envelope.read_text(encoding='utf-8'))['components']['quality']
            source=store/'objects'/component['file']
            with access.guard(source):
                if file_hash(source)!=component['sha256']:raise ValueError('NativeQualityCorrupt')
                before_token=access.token(source)
                old=ReceiptTable(source,component['count'],component['scientific'])
                tick=time.perf_counter()
                if scientific_digest(old)!=old.scientific:raise ValueError('NativeQualityIdentity')
                times['original_canonical_read']=time.perf_counter()-tick
                # All actual row values, including the complete aggregate, must
                # have an exact reconstruction. This is not a sampling claim.
                tick=time.perf_counter();all_rows=0
                with closing(old._connect()) as db:
                    for key,encoded in db.execute('SELECT key,value FROM receipts ORDER BY key'):
                        prefix,members,suffix=parts(json.loads(encoded))
                        if prefix+','.join(members)+suffix!=encoded:raise ValueError('ColdQualityPartsMismatch',key)
                        all_rows+=1
                times['all_population_parts_exact']=time.perf_counter()-tick
                if rows_per_population:
                    def bounded_rows():
                        with closing(old._connect()) as db:
                            for kind in ('cell:','face:'):
                                for key,value in db.execute('SELECT key,value FROM receipts WHERE key>=? AND key<? ORDER BY key LIMIT ?',
                                                           (kind,kind+'\uffff',rows_per_population)):
                                    yield key,json.loads(value)
                    rows=bounded_rows
                    # Bounded diagnostics only; the sealed aggregate remains in
                    # the all-population exact reconstruction above, not a copy.
                    with closing(old._connect()) as db:
                        raw_bytes=sum(db.execute('SELECT sum(length(value)) FROM (SELECT value FROM receipts WHERE key>=? AND key<? ORDER BY key LIMIT ?)',
                            (kind,kind+'\uffff',rows_per_population)).fetchone()[0] or 0 for kind in ('cell:','face:'))
                    budget=raw_bytes*4 # Operational allowance for two SQLite representations, measured row content.
                else:
                    rows=old.items;budget=source.stat().st_size
                free_before=shutil.disk_usage(out.parent).free
                # At most the original quality component's logical size; this is
                # a diagnostic operational bound, not a scientific threshold.
                if free_before-minimum_free_bytes<budget:raise ValueError('BoundedQualityStorageReserve')
                out.mkdir()
                peak_logical=0
                def monitor():
                    nonlocal peak_logical
                    size=live_allocation(out)
                    peak_logical=max(peak_logical,size)
                    if size>budget or shutil.disk_usage(out).free<minimum_free_bytes:
                        raise ValueError('BoundedQualityStorageResourceLimit')
                tick=time.perf_counter()
                new=bind_shared(out/'objects',rows(),progress=monitor)
                times['shared_write_and_independent_readback']=time.perf_counter()-tick
                compared=old
                if rows_per_population:
                    tick=time.perf_counter();compared=receipt_table_from_rows(out/'legacy',rows())
                    times['legacy_subset_write_and_digest']=time.perf_counter()-tick
                    monitor()
                if new.scientific!=compared.scientific or len(new)!=len(compared):raise ValueError('ColdQualityScientificMismatch')
                # Compare every byte, not just hashes or samples, with a new
                # connection on both source and destination.
                tick=time.perf_counter();count=0
                with closing(compared._connect()) as a,closing(new._connect()) as b:
                    for x,y in zip(a.execute('SELECT key,value FROM receipts ORDER BY key'),
                                   b.execute('SELECT key,value FROM receipts ORDER BY key'),strict=True):
                        if x!=y:raise ValueError('ColdQualityByteMismatch',x[0])
                        count+=1
                    plan=b.execute('EXPLAIN QUERY PLAN SELECT key,value FROM receipts ORDER BY key').fetchall()
                    interned=b.execute('SELECT count(*) FROM dependency_members').fetchone()[0]
                    uses=b.execute('SELECT count(*) FROM dependency_uses').fetchone()[0]
                times['all_rows_exact_comparison']=time.perf_counter()-tick
                # Actual unchanged native import/reader. It hardlinks the
                # immutable table; never reconstructs expanded quality JSON.
                tick=time.perf_counter();native=NativeCheckpointStore(out/'native')
                descriptor=native._table(new);reopened=native._receipt(descriptor)
                if reopened.scientific!=compared.scientific or len(reopened)!=count:raise ValueError('ColdQualityNativeReadback')
                times['native_import_and_independent_readback']=time.perf_counter()-tick
                monitor()
                if file_hash(source)!=component['sha256'] or access.token(source)!=before_token or file_hash(pointer)!=pin:
                    raise ValueError('ChangedRetainedQualitySource')
                result=dict(classification='V4GenericColdQualityCanonicalStorageVerified',native_state=body['scientific'],
                    sealed_pointer_sha256=pin,source_sha256=component['sha256'],quality_scientific=old.scientific,
                    full_population_exact_parts=all_rows,physical_comparison_rows=count,rows_per_population=rows_per_population,
                    compared_quality_scientific=compared.scientific,canonical_byte_differences=0,dependency_members=interned,dependency_uses=uses,
                    sealed_quality_logical=source.stat().st_size,compared_original_logical=compared.path.stat().st_size,new_logical=new.path.stat().st_size,
                    new_path=str(new.path),new_sha256=file_hash(new.path),native_descriptor=descriptor,
                    query_plan=plan,times=times,seconds=time.perf_counter()-start,peak_diagnostic_logical=peak_logical,
                    output_budget=budget,minimum_free_bytes=minimum_free_bytes,free_before=free_before,
                    free_after=shutil.disk_usage(out).free,allocation=allocation([p for p in out.rglob('*') if p.is_file()]),
                    sealed_sources_changed=False,layer_generated=False,checkpoint_published=False)
                durable_json(report,result)
                print(json.dumps({k:v for k,v in result.items() if k not in ('allocation','query_plan')}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--store',required=True);p.add_argument('--out',required=True)
    p.add_argument('--report',required=True);p.add_argument('--minimum-free-bytes',required=True,type=int)
    p.add_argument('--rows-per-population',type=int,default=0)
    a=p.parse_args();run(a.store,a.out,a.report,a.minimum_free_bytes,a.rows_per_population)
