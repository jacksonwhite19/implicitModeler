"""Read-only sealed checkpoint import and native persistence memory probe."""
import copy,gc,json,shutil,subprocess,tarfile,time,sys
from pathlib import Path
import psutil
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.common.authority import file_hash

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/native_mixed_production_migration'
OLD=ROOT/'results/mixed_front_production_migration'

def save(name,value):
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/name).write_text(json.dumps(value,sort_keys=True,indent=2)+'\n',encoding='utf8',newline='\n')

def extract():
    pointer=json.loads((OLD/'general_native/checkpoints/CURRENT.json').read_text())
    name='general_native/checkpoints/'+pointer['checkpoint']
    dest=OUT/'import_source';dest.mkdir(parents=True,exist_ok=True)
    archive=(OLD/'EVIDENCE.tar.zst').resolve()
    wsl='/mnt/'+archive.drive[0].lower()+str(archive)[2:].replace('\\','/')
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','--long=31','-c',wsl],stdout=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if member.name==name:
                with (dest/pointer['checkpoint']).open('wb') as stream:shutil.copyfileobj(tar.extractfile(member),stream,1024*1024)
    if process.wait()!=0:raise ValueError('ArchiveDecodeFailed')
    if file_hash(dest/pointer['checkpoint'])!=pointer['sha256']:raise ValueError('SealedCheckpointMismatch')
    shutil.copyfile(OLD/'general_native/checkpoints/CURRENT.json',dest/'CURRENT.json')

def probe():
    process=psutil.Process();start=time.perf_counter();rows=[]
    def mark(stage):
        gc.collect();info=process.memory_info()
        rows.append(dict(stage=stage,seconds=time.perf_counter()-start,rss=info.rss,peak_rss=info.peak_wset))
        print(stage,rows[-1],flush=True)
    mark('empty_process')
    store=NativeCheckpointStore(OUT/'native_general_post6')
    state=store.import_mixed_checkpoint(OUT/'import_source');mark('import_complete_legacy_released')
    expected=json.loads((OUT/'import_source/CURRENT.json').read_text())['scientific']
    assert scientific_hash(state)==expected
    private=copy.deepcopy(state);mark('transaction_private_copy')
    assert private['certificate'].base is state['certificate']
    assert scientific_hash(private)==expected
    del private;gc.collect();mark('private_released')
    counts={k:len(state[k]) for k in ('cells','quality','certificate','columns')}
    del state;gc.collect();mark('all_state_released')
    state=store.restart();mark('native_restart')
    assert scientific_hash(state)==expected
    save('STORAGE_PROBE.json',dict(scientific=expected,counts=counts,exact_replay=True,stages=rows,
        scope='Sealed general-SDF post-layer-6 checkpoint storage only; no geometry regeneration, atlas or forecast qualification.',
        prior_full_migration_peak_rss=5189218304,prior_peak_comparable=False))

def restart_probe(native):
    start=time.perf_counter()
    if native:state=NativeCheckpointStore(OUT/'native_general_post6').restart()
    else:
        pointer=json.loads((OUT/'import_source/CURRENT.json').read_text())
        state=json.loads((OUT/'import_source'/pointer['checkpoint']).read_text())['state']
        # Match the historical hash allocation, not the new streaming hash.
        from sdf_cfd_mesher_v4.common.authority import identity
        assert identity(state)==pointer['scientific']
    digest=scientific_hash(state);gc.collect();info=psutil.Process().memory_info()
    save(('NATIVE' if native else 'LEGACY')+'_RESTART_MEMORY.json',dict(seconds=time.perf_counter()-start,
        rss=info.rss,peak_rss=info.peak_wset,scientific=digest,cells=len(state['cells']),relations=len(state['certificate'])))

def archive_storage():
    destination=OUT/'NATIVE_CHECKPOINT.tar.zst'
    if destination.exists():raise ValueError('ArchiveAlreadyExists')
    paths=sorted(p for p in (OUT/'native_general_post6').rglob('*') if p.is_file())
    index={p.relative_to(OUT).as_posix():dict(bytes=p.stat().st_size,sha256=file_hash(p)) for p in paths}
    absolute=destination.resolve();wsl='/mnt/'+absolute.drive[0].lower()+str(absolute)[2:].replace('\\','/')
    process=subprocess.Popen(['wsl','--','zstd','-q','-6','-T2','-o',wsl],stdin=subprocess.PIPE)
    with tarfile.open(fileobj=process.stdin,mode='w|') as tar:
        for p in paths:
            member=tarfile.TarInfo(p.relative_to(OUT).as_posix());member.size=p.stat().st_size;member.mtime=0
            with p.open('rb') as stream:tar.addfile(member,stream)
    process.stdin.close()
    if process.wait()!=0:raise ValueError('ArchiveFailed')
    import hashlib
    process=subprocess.Popen(['wsl','--','zstd','-q','-d','-c',wsl],stdout=subprocess.PIPE);seen=set()
    with tarfile.open(fileobj=process.stdout,mode='r|') as tar:
        for member in tar:
            if hashlib.file_digest(tar.extractfile(member),'sha256').hexdigest()!=index[member.name]['sha256']:raise ValueError('ArchiveVerificationFailed')
            seen.add(member.name)
    if process.wait()!=0 or seen!=set(index):raise ValueError('ArchiveIncomplete')
    save('ARTIFACT_INDEX.json',index)
    for p in paths:
        if not p.resolve().is_relative_to((OUT/'native_general_post6').resolve()) or file_hash(p)!=index[p.relative_to(OUT).as_posix()]['sha256']:raise ValueError('RemovalGuard')
        p.unlink()
    pointer=json.loads((OUT/'import_source/CURRENT.json').read_text())
    source=OUT/'import_source'/pointer['checkpoint']
    if not source.resolve().is_relative_to((OUT/'import_source').resolve()) or file_hash(source)!=pointer['sha256']:raise ValueError('SourceRemovalGuard')
    save('SOURCE_IMPORT.json',dict(authority=str(OLD.relative_to(ROOT)),pointer=pointer,source_unchanged=True))
    source.unlink();(OUT/'import_source/CURRENT.json').unlink()

if __name__=='__main__':
    if sys.argv[1:]==['extract']:extract()
    elif sys.argv[1:]==['probe']:probe()
    elif sys.argv[1:]==['legacy-restart']:restart_probe(False)
    elif sys.argv[1:]==['native-restart']:restart_probe(True)
    elif sys.argv[1:]==['archive']:archive_storage()
    else:raise SystemExit('extract | probe | legacy-restart | native-restart')
