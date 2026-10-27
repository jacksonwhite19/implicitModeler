"""Strict records for the owned query and direct-callback libraries.

Building is an explicit tool action. Runtime admission never builds or chooses
an alternate binary. The existing qualified scalar sizing leaf stays unchanged.
"""
import stage_observations as obs
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

from owned_runtime import V4
import build_provenance as provenance
from background_sizing import arithmetic_runtime

ROOT=V4/'runtime/source/background/direct'
FLAGS=['-std=c++17','-shared','-fPIC','-O3','-fno-fast-math',
    '-fno-associative-math','-ffp-contract=off']
DIRECT_FLAGS=[*FLAGS,'-pthread']

@obs.observed('query_build.digest','source_lock')
def digest(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):value.update(obs.byte_value(chunk))
    return value.hexdigest()

@obs.observed('query_build.spec','source_lock')
def spec(kind):
    if kind not in ('query','direct'):raise ValueError('UnknownOwnedQueryBuild')
    leaf=V4/'runtime/source/background/sizing_values.cpp'
    source=ROOT/('query.cpp' if kind=='query' else 'direct_callback.cpp')
    sources=[source,leaf] if kind=='query' else [source,ROOT/'query.cpp',leaf]
    return dict(source=source,sources={str(p):digest(p) for p in sources},
        flags=FLAGS if kind=='query' else DIRECT_FLAGS,
        schema='V4OwnedBackground'+('Query' if kind=='query' else 'DirectCallback')+'Build/1',
        record=ROOT/('QUERY_BUILD.json' if kind=='query' else 'DIRECT_BUILD.json'))

def recipe(configuration,compiler,runtime):
    return dict(sources=configuration['sources'],flags=configuration['flags'],
        compiler=compiler,arithmetic_runtime=runtime)

def library_name(kind,configuration,compiler,runtime):
    pin=hashlib.sha256(json.dumps(recipe(configuration,compiler,runtime),sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return kind+'_'+pin+'.so'

@obs.observed('query_build.record','source_lock')
def record(kind):
    obs.units('query_records' if kind=='query' else 'direct_records',1)
    configuration=spec(kind)
    # Admission retains the original recipe key namespace and library filename.
    configuration['sources']={provenance.original(Path(p)):pin for p,pin in configuration['sources'].items()}
    metadata=provenance.receipt(configuration['record'])
    compiler=metadata.get('compiler',{});runtime=arithmetic_runtime()
    name=library_name(kind,configuration,compiler,runtime);library=ROOT/name
    expected=['/usr/bin/c++',*configuration['flags'],provenance.original(configuration['source']),'-o',provenance.original(library)]
    if (metadata.get('schema')!=configuration['schema'] or metadata.get('exit_code')!=0
        or metadata.get('sources')!=configuration['sources'] or metadata.get('library')!=name
        or metadata.get('command')!=expected or metadata.get('arithmetic_runtime')!=runtime
        or compiler.get('path')!=str(Path('/usr/bin/c++').resolve())
        or compiler.get('sha256')!=digest(compiler['path'])
        or metadata.get('binary_sha256')!=digest(library)):
        raise ValueError('UnreviewedOwnedQueryBuild',kind)
    with obs.span('dependency_closure','source_lock'):
        for group in ('dependencies','shared_libraries'):
            bindings=metadata.get(group)
            if not isinstance(bindings,dict) or not bindings:raise ValueError('MissingOwnedQueryBuildClosure',kind)
            for p,pin in bindings.items():
                if digest(provenance.resolve(p))!=pin:raise ValueError('ChangedOwnedQueryBuildClosure',kind,p)
    if not set(configuration['sources'])<=set(metadata['dependencies']):
        raise ValueError('MissingOwnedQuerySourceClosure',kind)
    return metadata,library

def build(kind):
    configuration=spec(kind);compiler=Path('/usr/bin/c++').resolve()
    compiler_record=dict(path=str(compiler),sha256=digest(compiler),
        version=subprocess.check_output(['/usr/bin/c++','--version'],text=True))
    runtime=arithmetic_runtime();library=ROOT/library_name(kind,configuration,compiler_record,runtime)
    target=configuration['record']
    if target.exists() or library.exists():raise ValueError('ExistingOwnedQueryBuildPreserved',kind)
    source=configuration['source'];dependency_flags=['-std=c++17']+(['-pthread'] if kind=='direct' else [])
    text=subprocess.check_output(['/usr/bin/c++',*dependency_flags,'-M',str(source)],text=True)
    paths=shlex.split(text.replace('\\\n',' ').split(':',1)[1])
    dependencies={str(Path(p).resolve()):digest(p) for p in paths}
    command=['/usr/bin/c++',*configuration['flags'],str(source),'-o',str(library)]
    metadata=dict(schema=configuration['schema'],library=library.name,command=command,
        compiler=compiler_record,sources=configuration['sources'],dependencies=dependencies,
        arithmetic_runtime=runtime,scope='Owned primitive build only; no mesh, release selection or performance claim.')
    try:
        completed=subprocess.run(command,capture_output=True,text=True)
        metadata.update(exit_code=completed.returncode,stdout=completed.stdout,stderr=completed.stderr)
        if completed.returncode:raise ValueError('OwnedQueryBuildFailed',kind)
        linked=[]
        for line in subprocess.check_output(['ldd',str(library)],text=True).splitlines():
            tokens=line.strip().split();tokens=tokens[2:] if '=>' in tokens else tokens
            if tokens and tokens[0].startswith('/'):linked.append(Path(tokens[0]).resolve())
        metadata.update(binary_sha256=digest(library),shared_libraries={str(p):digest(p) for p in linked})
        if digest(compiler)!=compiler_record['sha256'] or any(digest(p)!=pin for p,pin in dependencies.items()):
            raise ValueError('ChangedOwnedQueryBuildInputs',kind)
    finally:target.write_text(json.dumps(metadata,sort_keys=True,indent=2)+'\n',encoding='utf-8')
    record(kind)
    return dict(passed=True,kind=kind,library=library.name,binary_sha256=metadata['binary_sha256'])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('kind',choices=('query','direct'))
    print(json.dumps(build(parser.parse_args().kind),sort_keys=True))
