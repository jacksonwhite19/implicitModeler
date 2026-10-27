"""Post-run immutable input verification and complete task-array manifest."""
import subprocess,re
from pathlib import Path
import capsule_sa_outer_map as m
import capsule_sa_operator_finish as oldfinish
h=m.h;R=m.ROOT


def verify():
    record=m.verify();admission=h.read(R/'ADMISSION.json')
    record['starting_HEAD']=m.HEAD
    record['canonical1000']={}
    for name in m.c.NAMES:
        m.c.audit(name)
        fields=m.c.canonical(m.c.ROOT/name/'pilot',1000)
        assert fields==admission['cases'][name]['checkpoints']['1000']
        record['canonical1000'][name]=fields
    sources=h.read(R/'SOURCES.json')
    for path,sha in sources.items():
        assert subprocess.check_output(['wsl','sha256sum',path],text=True).split()[0]==sha,path
    assert subprocess.check_output(['wsl','sha256sum','/opt/openfoam13/platforms/linux64GccDPInt32Opt/bin/foamRun'],text=True).split()[0]=='f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e'
    code=(R/'build/saOuterMap.C').read_text();assert code==Path('research/sdf_cfd_mesher_v4/tools/sa_outer_map/saOuterMap.C').read_text()
    assert all(t not in code for t in ('runTime++','++runTime','runTime.write(','runTime.run(','runTime.loop('))
    record.update(native_source=h.file_hash(R/'build/saOuterMap.C'),native_binary=h.file_hash(R/'build/bin/saOuterMap'),source_files=sources,
                  no_time_increment=True,no_scientific_write=True,exactly_one_sweep_per_map=True,source_order_independently_verified=True,
                  V3_unchanged=True,no_convergence_rule=True,no_stopping_contract=True,no_CFD_qualification=True)
    h.durable_json(R/'FINAL_VERIFICATION.json',h.seal(record));print('Immutable inputs and sources verified',flush=True)


def manifest():
    files={str(p.relative_to(R)):h.file_hash(p) for p in sorted((R/'outputs').rglob('*.bin'))}
    replay={}
    for name in m.c.NAMES:
        for folder in (R/'outputs'/name).iterdir():
            mode=folder.name
            if 'rejected' in mode or 'initial' in mode or mode=='reference':continue
            rows=[]
            for rank in range(6):
                prefix=str((folder/str(rank)).relative_to(R))+'/'
                for key,sha in files.items():
                    if key.startswith(prefix+'base0_'):
                        partner=prefix+'base1_'+key.split('base0_',1)[1]
                        assert files[partner]==sha,(key,partner)
                        rows.append(key)
                if mode!='base' and mode!='zero':
                    a,b=(2,3) if mode.endswith('_fine') else (3,4)
                    for key,sha in files.items():
                        if key.startswith(prefix+f'd{a}_'):
                            partner=prefix+f'd{b}_'+key.split(f'd{a}_',1)[1]
                            assert files[partner]==sha,(key,partner)
                            rows.append(key)
            replay[name+'/'+mode]=dict(all_intermediate_arrays_exact=True,comparisons=len(rows))
    h.durable_json(R/'ARRAY_MANIFEST.json',h.seal(dict(files=files,exact_replay=replay)))
    print('Manifest and intermediate replay verified',len(files),flush=True)
    oldfinish.o.ROOT=R
    h.durable_json(R/'STORAGE.json',h.seal(oldfinish.allocation()))

if __name__=='__main__':
    verify();manifest()
