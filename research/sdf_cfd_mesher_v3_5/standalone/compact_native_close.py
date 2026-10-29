"""Linux-only bounded retirement/inspection of this fresh output's recorded groups."""
from pathlib import Path
import json,os,signal,sys,time
root=Path(sys.argv[1]).resolve();normal=sys.argv[2]=='inspect'
assert root.name.startswith('.v35-compact-scratch-') and len(root.name)==53
marker=json.loads((root/'OWNER.json').read_text())
assert marker['nonce']==sys.argv[3] and marker['schema']=='V35NewCompactScratchOwner/1'
def inside(s):
    # One path operand (option=value supported), never a substring or sibling.
    candidate=s.split('=',1)[-1] if '=' in s else s
    if not candidate.startswith('/'):return False
    return Path(candidate).resolve().is_relative_to(root)
def process(pid):
    p=Path('/proc')/str(pid)
    try:
        fields=(p/'stat').read_text().rsplit(')',1)[1].split()
        argv=[os.fsdecode(x) for x in (p/'cmdline').read_bytes().rstrip(b'\0').split(b'\0')]
        try:cwd=str((p/'cwd').resolve(strict=True))
        except (FileNotFoundError,PermissionError):cwd=''
        return dict(pid=pid,pgid=int(fields[2]),birth=int(fields[19]),argv=argv,cwd=cwd)
    except (FileNotFoundError,ProcessLookupError):return None
def census():
    return [v for p in Path('/proc').iterdir() if p.name.isdigit() and int(p.name)!=os.getpid() and (v:=process(int(p.name))) is not None]
scopes=[];actions=[];unknown=[];unexpected=[]
for f in sorted(root.rglob('*.linux_scope.json')):
    v=json.loads(f.read_text());assert type(v['pid']) is int and v['pid']>1 and type(v['start_ticks']) is int
    assert isinstance(v.get('argv'),list),'InvalidNativeScopeArgv'
    if not normal:assert any(inside(a) for a in v['argv']),'UnboundNativeScope'
    scopes.append(dict(path=str(f),**v))
before=census()
def ownership(scope,members):
    pid=scope['pid'];leader=next((p for p in members if p['pid']==pid),None)
    pending=str(scope['path']).removesuffix('.linux_scope.json')+'.launch.sh'
    exact=leader is not None and leader['birth']==scope['start_ticks'] and (pending in leader['argv'] or any(leader['argv'][i:i+len(scope['argv'])]==scope['argv'] for i in range(len(leader['argv']))))
    orphan=leader is None and any(any(inside(a) for a in p['argv']) or inside(p['cwd']) for p in members) and all(p['birth']>=scope['start_ticks'] for p in members)
    return exact,orphan
for scope in scopes:
    pid=scope['pid'];members=[p for p in before if p['pgid']==pid]
    if not members:continue
    unexpected.append(dict(scope=scope,members=members))
    if normal:continue  # Success inspection never signals any group.
    exact,orphan_bound=ownership(scope,members)
    if not (exact or orphan_bound):unknown.append(dict(scope=scope,reason='GroupOwnershipUnresolved',members=members));continue
    for sig in (signal.SIGTERM,signal.SIGKILL):
        current=[p for p in census() if p['pgid']==pid]
        if not current:break
        exact,orphan_bound=ownership(scope,current)
        anchors={(p['pid'],p['birth']) for p in members}
        if not(exact or orphan_bound) or not any((p['pid'],p['birth']) in anchors for p in current):
            unknown.append(dict(scope=scope,reason='GroupOwnershipChangedBeforeSignal',members=current));break
        try:os.killpg(pid,sig);actions.append(dict(pgid=pid,signal=int(sig),orphan_group=orphan_bound))
        except ProcessLookupError:break
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            if not any(p['pgid']==pid for p in census()):break
            time.sleep(.05)
        if not any(p['pgid']==pid for p in census()):break
after=census();groups={s['pid'] for s in scopes}
remaining=[p for p in after if p['pgid'] in groups or any(inside(a) for a in p['argv']) or inside(p['cwd'])]
result=dict(schema='V35CompactNativeClosure/1',passed=not unknown and not remaining and not(normal and unexpected),normal_root_exit=normal,scope_count=len(scopes),scopes=scopes,unexpected_live_groups=unexpected,actions=actions,unknown=unknown,remaining=remaining,scope='Recorded birth/argv/PGID or root-bound orphan groups; exact fresh-output argv/cwd census. Does not promise cleanup after hard death of this external controller.')
print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
