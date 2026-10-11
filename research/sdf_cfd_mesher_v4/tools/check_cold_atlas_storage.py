"""Full retained atlas storage/readback control; no geometry or proof generation."""
import argparse,json,shutil,sqlite3,time
from contextlib import ExitStack,closing
from pathlib import Path
import psutil
import immutable_state_access as access
from compact_initial_atlas import bind_state
from path_preserving_deduplication import durable_json,file_hash
from check_current_front_encoding import allocation
from check_cold_quality_storage import live_allocation
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,scientific_digest,ENCODER


def run(store,out,report,minimum_free_bytes):
    store=Path(store).resolve();out=Path(out).resolve();report=Path(report).resolve()
    if out.exists() or report.exists():raise ValueError('PreserveExistingAtlasControl')
    start=time.perf_counter();times={};process=psutil.Process()
    with ExitStack() as held:
        pointer=store/'CURRENT.json';held.enter_context(access.guard(pointer));saved=access.read(pointer)
        envelope=store/saved['checkpoint'];held.enter_context(access.guard(envelope))
        if file_hash(envelope)!=saved['sha256']:raise ValueError('NativeCheckpointCorrupt')
        row=access.read(envelope)['components']['atlas']
        if row['encoding']!='json':raise ValueError('RetainedNativeJSONAtlasRequired')
        source=store/'objects'/row['file'];held.enter_context(access.guard(source));token=access.token(source)
        if file_hash(source)!=row['sha256']:raise ValueError('ChangedRetainedAtlas')
        tick=time.perf_counter();values=access.read(source);times['original_json_decode']=time.perf_counter()-tick
        tick=time.perf_counter();original=scientific_digest(values);times['original_canonical_read']=time.perf_counter()-tick
        if original!=row['scientific']:raise ValueError('OriginalNativeAtlasIdentity')
        if shutil.disk_usage(out.parent).free-minimum_free_bytes<source.stat().st_size:
            raise ValueError('AtlasStorageReserve')
        out.mkdir();peak_allocation=0
        def monitor():
            nonlocal peak_allocation
            size=live_allocation(out)
            peak_allocation=max(peak_allocation,size)
            if size>source.stat().st_size or shutil.disk_usage(out).free<minimum_free_bytes:
                raise ValueError('AtlasStorageReserve')
        state={'atlas':values};tick=time.perf_counter()
        bind_state(state,out/'objects',progress=monitor);table=state['atlas']
        times['validated_native_binding_and_readbacks']=time.perf_counter()-tick;monitor()
        tick=time.perf_counter();count=0
        with closing(table._connect()) as db:
            for key,text in db.execute('SELECT key,value FROM receipts ORDER BY key'):
                if text!=ENCODER.encode(values[key]):raise ValueError('AtlasCanonicalByteMismatch',key)
                count+=1
            members=db.execute('SELECT count(*) FROM dependency_members').fetchone()[0]
            uses=db.execute('SELECT count(*) FROM dependency_uses').fetchone()[0]
        if count!=len(values):raise ValueError('AtlasIncompleteCoverage')
        times['complete_original_SQL_comparison']=time.perf_counter()-tick
        tick=time.perf_counter();native=NativeCheckpointStore(out/'native')
        descriptor=native._component(table);restored=native._receipt(descriptor)
        if restored.scientific!=original or len(restored)!=len(values):raise ValueError('NativeAtlasReadbackMismatch')
        times['native_import_independent_readback']=time.perf_counter()-tick;monitor()
        tick=time.perf_counter();read_digest=scientific_digest(restored)
        times['new_canonical_read']=time.perf_counter()-tick
        if read_digest!=original:raise ValueError('ChangedAtlasCanonicalStream')
        if access.token(source)!=token or access.read(pointer)!=saved:raise ValueError('ChangedSealedAtlasState')
        result=dict(classification='V4ColdAtlasFullPopulationCanonicalStorageVerified',
            scientific_state=saved['scientific'],atlas_scientific=original,source_sha256=row['sha256'],
            rows=count,canonical_byte_differences=0,original_bytes=source.stat().st_size,new_bytes=table.path.stat().st_size,
            dependency_members=members,dependency_uses=uses,native_descriptor=descriptor,
            times=times,seconds=time.perf_counter()-start,peak_working_set_bytes=process.memory_info().peak_wset,
            minimum_free_bytes=minimum_free_bytes,final_free=shutil.disk_usage(out).free,peak_unique_allocation=peak_allocation,
            allocation=allocation([p for p in out.rglob('*') if p.is_file()]),
            new_sha256=file_hash(table.path),source_changed=False,layers_generated=0,proofs_generated=0,
            scope='All original atlas rows and full native component import/readback, not full checkpoint publication/enrollment or a speed benchmark')
        durable_json(report,result);print(json.dumps({k:v for k,v in result.items() if k!='allocation'}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--store',required=True);p.add_argument('--out',required=True)
    p.add_argument('--report',required=True);p.add_argument('--minimum-free-bytes',type=int,required=True)
    a=p.parse_args();run(a.store,a.out,a.report,a.minimum_free_bytes)
