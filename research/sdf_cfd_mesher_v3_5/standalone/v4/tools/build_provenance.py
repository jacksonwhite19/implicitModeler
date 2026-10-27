"""Fixed, release-owned relocation of preserved build provenance. No new build."""
import stage_observations as obs
from pathlib import Path, PurePosixPath
import hashlib,json,os
from owned_runtime import V4

MAP=V4/'runtime/BUILD_RELOCATION.json'
MAP_SHA256='3bf3a04e923e35fcc381e0ffbda6de481cdd89cfc1f12af000b07d5cdcb32b65'

@obs.observed('provenance.digest','source_lock')
def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):h.update(obs.byte_value(block))
    return h.hexdigest()

@obs.observed('provenance.table','source_lock')
def table():
    with obs.span('map_hash','io_hash'):
        if digest(MAP)!=MAP_SHA256:raise ValueError('ChangedOwnedBuildRelocation')
    with obs.span('map_read_decode','io_serialization'):
        value=json.loads(MAP.read_text(encoding='utf8'))
    if value.get('schema')!='OwnedBuildRelocation/1':raise ValueError('InvalidOwnedBuildRelocation')
    with obs.span('owned_path_validation','source_path'):
        seen=set()
        for old,row in value['owned_paths'].items():
            relative=PurePosixPath(row['relative'])
            if (relative.is_absolute() or '..' in relative.parts or str(relative)!=row['relative']
                    or '\\' in row['relative'] or row['platform'] not in ('windows','posix')
                    or row['kind'] not in ('file','intermediate')):raise ValueError('UnsafeOwnedBuildRelocation')
            path=obs.resolved(V4/row['relative'])
            if obs.resolved(V4) not in path.parents:raise ValueError('EscapedOwnedBuildRelocation')
            key=(row['platform'],str(path))
            if key in seen:raise ValueError('AmbiguousOwnedBuildRelocation')
            seen.add(key)
        if set(value['owned_paths']) & set(value['external_paths']):raise ValueError('AmbiguousExternalBuildRelocation')
    return value

@obs.observed('provenance.original','source_lock')
def original(path):
    path=obs.resolved(Path(path))
    if obs.resolved(V4) not in path.parents:raise ValueError('UnknownOwnedBuildOperand',str(path))
    relative=path.relative_to(obs.resolved(V4)).as_posix();platform='windows' if os.name=='nt' else 'posix'
    rows=[old for old,row in table()['owned_paths'].items() if row['relative']==relative and row['platform']==platform]
    if len(rows)!=1:raise ValueError('UnknownOwnedBuildOperand',relative,platform)
    return rows[0]

@obs.observed('provenance.resolve','source_lock')
def resolve(old):
    value=table();row=value['owned_paths'].get(old)
    if row is not None:
        path=obs.resolved(V4/row['relative'])
        if row['kind']!='file':raise ValueError('IntermediateBuildOperandIsNotInput',old)
        if digest(path)!=row['sha256']:raise ValueError('ChangedMappedBuildInput',old)
        return path
    if old not in value['external_paths']:raise ValueError('UnknownBuildDependency',old)
    return Path(old)

@obs.observed('provenance.receipt','source_lock')
def receipt(path):
    path=obs.resolved(Path(path));relative=path.relative_to(obs.resolved(V4)).as_posix()
    if table()['records'].get(relative)!=digest(path):raise ValueError('ChangedOriginalBuildReceipt',relative)
    return json.loads(path.read_text(encoding='utf8'))

def hold(stack):
    from immutable_state_access import guard
    for path in (Path(__file__),MAP):stack.enter_context(guard(path))
    table()
