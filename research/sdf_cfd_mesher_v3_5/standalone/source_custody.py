"""Read-only source custody for one worker; no historical release approval."""
from pathlib import Path
from contextlib import contextmanager,ExitStack
import os,json,hashlib
ROOT=Path(__file__).resolve().parent
SKIP={'target','results','build_logs','run_logs','__pycache__','.git','linux64GccDPInt32Opt'}
EXT={'.py','.rs','.C','.H','.cpp','.h','.toml','.lock','.json','.rhai','.wgsl','.dll','.exe','.so','.pyd','.zip'}
def selected(output):
    paths=[]
    for directory,dirs,files in os.walk(ROOT):
        dirs[:]=[d for d in dirs if d not in SKIP and not (Path(directory)/d).resolve().is_relative_to(output)]
        for name in files:
            p=Path(directory)/name
            if p.suffix in EXT or p.parent.name in ('native_bin','Make') or p.is_relative_to(ROOT/'v4/runtime'):paths.append(p)
    return sorted(paths)
def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
@contextmanager
def held(output,record):
    from immutable_state_access import guard
    paths=selected(output)
    with ExitStack() as scope:
        for p in paths:scope.enter_context(guard(p))
        rows=[dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p),bytes=p.stat().st_size) for p in paths]
        identity=hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        value=dict(schema='V35WorkerSourceCustody/1',identity=identity,files=rows,state='held',scope='Local source/native/runtime bytes protected for this worker; not upstream release review or numerical qualification')
        record.write_text(json.dumps(value,indent=2)+'\n')
        try:yield value
        finally:
            changed=[]
            for r,p in zip(rows,paths):
                try:unchanged=sha(p)==r['sha256']
                except OSError:unchanged=False
                if not unchanged:changed.append(r['path'])
            added=[p.relative_to(ROOT).as_posix() for p in set(selected(output))-set(paths)]
            value.update(state='closed',changed=changed,added=added,passed=not changed and not added)
            record.write_text(json.dumps(value,indent=2)+'\n')
