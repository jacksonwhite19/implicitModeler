"""Two exact historical Brotli files in a candidate-only Linux loader scope.

Original manifests and digests remain mandatory. This profile changes paths,
never hashes, and grants no historical environment or mesh qualification.
"""
from pathlib import Path
import hashlib,json,os,re,shlex,subprocess,sys,time
ROOT=Path(__file__).resolve().parent
VERSION='V35IsolatedBrotliRuntime/1'
MANIFEST='EXTERNAL_RUNTIME_PROFILE.json'
ORIGINALS=('v4/runtime/FINITE_NATIVE_BINDING.json','v4/tools/finite_research/direct_evidence_modules/LINUX_TOOLCHAIN.json')
RELOCATIONS={'/usr/lib/x86_64-linux-gnu/libbrotlicommon.so.1.0.9':'v4/runtime/isolated_brotli/libbrotlicommon.so.1',
    '/usr/lib/x86_64-linux-gnu/libbrotlidec.so.1.0.9':'v4/runtime/isolated_brotli/libbrotlidec.so.1'}
def identity(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
def read(p):return json.loads(Path(p).read_text())

def baseline(root=ROOT):
    root=Path(root);tool=read(root/ORIGINALS[1]);binding=read(root/ORIGINALS[0]);pins={}
    def add(name,pin):
        if name in pins and pins[name]!=pin:raise ValueError('ConflictingDirectRuntimePin',name)
        pins[name]=pin
    old='/mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/'
    for name,pin in tool['files'].items():
        if not name.startswith(old):add(name,pin)
    for row in binding['files']:
        if 'external_path' in row:add(row['external_path'],row['sha256'])
    return pins

def manifest(root=ROOT):
    root=Path(root);v=read(root/MANIFEST);pins=baseline(root)
    if (v.get('schema')!=VERSION or v.get('decision')!='restore_original_bytes_in_candidate_only'
        or v.get('historical_environment_qualification_inherited') is not False
        or v.get('baseline_identity')!=identity(pins) or v.get('baseline_files')!=len(pins)
        or v.get('original_manifests')!={name:sha(root/name) for name in ORIGINALS}):raise ValueError('UnsupportedExternalRuntimeProfile')
    rows=v.get('libraries',[])
    if len(rows)!=2 or {r.get('original_path') for r in rows}!=set(RELOCATIONS):raise ValueError('InvalidExternalRuntimeRelocations')
    for row in rows:
        name=row['original_path'];relative=RELOCATIONS[name];path=root/relative
        if (set(row)!={'original_path','relative_path','sha256'} or row['relative_path']!=relative or row['sha256']!=pins[name]
            or path.is_symlink() or path.resolve()!=root.resolve()/relative):raise ValueError('InvalidExternalRuntimeRelocation',name)
    return v

def profile(root=ROOT):
    v=manifest(root);return dict(version=VERSION,identity=identity(v))

def effective_pins(root=ROOT):
    root=Path(root);v=manifest(root);base=baseline(root);mapped={r['original_path']:root/r['relative_path'] for r in v['libraries']};pins={}
    for original,pin in base.items():
        path=(linux(mapped[original]) if original in mapped else original) if os.name=='nt' else str(Path(mapped.get(original,original)).resolve())
        if path in pins:raise ValueError('ExternalRuntimeCanonicalPathCollision',path)
        pins[path]=pin
    if len(pins)!=len(base):raise ValueError('IncompleteExternalRuntimePopulation')
    return pins

def dependency_paths(text):
    paths=set()
    for line in text.splitlines():
        value=line.split('=>',1)[-1].strip().rsplit(' (0x',1)[0].strip()
        if value.startswith('/'):
            path=Path(value)
            if not path.is_file():raise ValueError('MissingExternalLoaderPath',value)
            paths.add(str(path.resolve()))
    return sorted(paths)

def loader_check(root=ROOT,*,reader=sha):
    pins=effective_pins(root);binary=Path(root)/'v4/runtime/lib/gmsh/libgmsh.so.4.15'
    approved=read(Path(root)/'v4/runtime/MANIFEST.json')['files']
    expected=next(row['sha256'] for row in approved if row['path']=='runtime/lib/gmsh/libgmsh.so.4.15')
    if reader(binary)!=expected:raise ValueError('ChangedPreflightGmshLibrary')
    p=subprocess.run(['/usr/bin/ldd',str(binary)],capture_output=True,text=True,timeout=15)
    if p.returncode or 'not found' in p.stdout:raise ValueError('ExternalRuntimeLoaderFailure',p.stderr)
    paths=dependency_paths(p.stdout)
    for name in paths:
        if name not in pins or reader(Path(name))!=pins[name]:raise ValueError('UnpinnedPreflightLoaderDependency',name)
    restored={str((Path(root)/r['relative_path']).resolve()) for r in manifest(root)['libraries']}
    if not restored.issubset(paths):raise ValueError('WrongPreflightBrotliResolution')
    return dict(passed=True,dependencies=paths,restored_files=sorted(restored),scope='ldd resolution and byte pins; actual process mappings remain mandatory in the late gate')

def audit(root=ROOT,*,reader=sha):
    started=time.monotonic();pins=effective_pins(root);mismatches=[]
    if os.environ.get('LD_PRELOAD'):raise ValueError('ExternalRuntimePreloadConflict')
    for name,pin in pins.items():
        try:actual=reader(Path(name))
        except OSError:actual=None
        if actual!=pin:mismatches.append(dict(path=name,expected_sha256=pin,actual_sha256=actual))
    loader=loader_check(root,reader=reader) if not mismatches else dict(passed=False,status='not_run_due_byte_refusal')
    return dict(schema='V35ExternalRuntimePreflight/1',passed=not mismatches and loader['passed'],profile=profile(root),checked_files=len(pins),
        baseline_identity=identity(baseline(root)),effective_identity=identity(pins),restored_files=2,
        mismatches=mismatches,loader=loader,seconds=time.monotonic()-started,scope='Complete effective external byte/ldd closure before geometry; no native mesh or source predicates')

def require_mappings(root=ROOT,*,maps=None):
    """After Gmsh import, verify actual decoder/common mappings, not just ldd."""
    if os.environ.get('LD_PRELOAD'):raise ValueError('ExternalRuntimePreloadConflict')
    v=manifest(root);expected={str((Path(root)/r['relative_path']).resolve()):r['sha256'] for r in v['libraries']};seen=set()
    text=Path('/proc/self/maps').read_text() if maps is None else maps
    for line in text.splitlines():
        parts=line.split(maxsplit=5)
        if len(parts)<6:continue
        name=re.sub(r'\\([0-7]{3})',lambda m:chr(int(m.group(1),8)),parts[5])
        if not Path(name).name.startswith('libbrotli'):continue
        if name.endswith(' (deleted)') or name not in expected:raise ValueError('UnexpectedBrotliProcessMapping',name)
        st=Path(name).stat();device=parts[3].split(':')
        if int(parts[4])!=st.st_ino or os.makedev(int(device[0],16),int(device[1],16))!=st.st_dev:raise ValueError('ChangedBrotliMappingFileIdentity',name)
        if sha(Path(name))!=expected[name]:raise ValueError('ChangedMappedBrotliLibrary',name)
        seen.add(name)
    if seen!=set(expected):raise ValueError('IncompleteBrotliProcessMappings')
    return dict(passed=True,files=sorted(seen),scope='Actual current process maps; both exact original decoder/common files required')

def linux(path):
    path=Path(path).resolve()
    return '/mnt/'+path.drive[0].lower()+path.as_posix()[2:] if os.name=='nt' else str(path)

def launch_prefix(root=ROOT):
    manifest(root);directory=linux(Path(root)/'v4/runtime/isolated_brotli')
    return ('if [ -n "${LD_PRELOAD:-}" ]; then echo ExternalRuntimePreloadConflict >&2; exit 1; fi\n'
        'export LD_LIBRARY_PATH='+shlex.quote(directory)+'${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n')

def preflight(root=ROOT,*,seconds=60):
    root=Path(root);prefix=launch_prefix(root);interpreter=read(root/ORIGINALS[0])['executable']
    command=['wsl','-d','Ubuntu-22.04','--','bash','-lc','source /opt/openfoam13/etc/bashrc >/dev/null 2>&1;\n'+prefix+
        'exec '+shlex.quote(interpreter)+' -B '+shlex.quote(linux(root/'runtime_environment.py'))+' --audit']
    p=subprocess.run(command,capture_output=True,text=True,timeout=seconds)
    if p.returncode not in (0,1):raise ValueError('ExternalRuntimePreflightProcessFailed',p.returncode,p.stdout[:512],p.stderr[:512])
    try:value=json.loads(p.stdout)
    except ValueError:raise ValueError('ExternalRuntimePreflightInvalidResult',p.returncode,p.stderr)
    pins=effective_pins(root)
    if (value.get('schema')!='V35ExternalRuntimePreflight/1' or value.get('profile')!=profile(root)
        or value.get('baseline_identity')!=identity(baseline(root)) or value.get('checked_files')!=len(pins) or value.get('effective_identity')!=identity(pins)
        or value.get('restored_files')!=2 or type(value.get('passed')) is not bool or not isinstance(value.get('mismatches'),list)
        or bool(value.get('mismatches'))==value['passed'] or (value['passed'] and value.get('loader',{}).get('passed') is not True)
        or (p.returncode==0)!=bool(value.get('passed'))):raise ValueError('ExternalRuntimePreflightBindingMismatch')
    return value

if __name__=='__main__':
    if sys.argv[1:]!=['--audit']:raise SystemExit('Only --audit is supported')
    try:
        value=audit();print(json.dumps(value));raise SystemExit(0 if value['passed'] else 1)
    except Exception as error:print(json.dumps(dict(schema='V35ExternalRuntimePreflight/1',passed=False,error=repr(error))));raise SystemExit(2)
