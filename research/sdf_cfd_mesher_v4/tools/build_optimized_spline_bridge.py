"""Recompile only the frozen spline owner using existing Rust dependencies."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

V4 = Path(__file__).resolve().parents[1]
REPO = V4.parents[1]

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def historical_path():
    return REPO/'research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09/native_spline_acceleration_20260910/BUILD.json'

def expected_commands():
    historical=historical_path()
    if sha(historical)!='9d6c534f2d7185fb13b880b3b489410db05b83de5502bb35b550b4356f663766':
        raise ValueError('HistoricalBuildMismatch')
    command=json.loads(historical.read_text(encoding='utf-8'))['command'].copy()
    command[2]=str(V4/'runtime/source/spline_bridge_optimized.rs')
    command[command.index('-o')+1]=str(V4/'runtime/bin/spline_bridge_optimized.dll')
    index=next(i for i,x in enumerate(command) if x.startswith('implicit_cad='))
    del command[index-1:index+1]
    library=V4/'runtime/lib/libsdf_frozen_query.rlib'
    query=['rustc','--edition=2024',str(V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/naca_spline.rs'),
        '--crate-name','sdf_frozen_query','--crate-type','rlib','-C','opt-level=3',
        '-C','debuginfo=0','-o',str(library)]
    for item in command:
        if item.startswith(('glam=','serde=')): query.extend(['--extern',item])
        if item.startswith('dependency='): query.extend(['-L',item])
    command.extend(['--extern','sdf_frozen_query='+str(library)])
    return command,query

def expected_build_inputs():
    command,_=expected_commands()
    paths=[Path(x.split('=',1)[1]) for x in command if x.startswith(('glam=','serde=','serde_json='))]
    paths += [historical_path(),V4/'runtime/source/spline_bridge_optimized.rs',
        V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/naca_spline.rs',
        V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/spline_bridge.rs']
    return {str(p):sha(p) for p in paths}

def validate_options(command, query_command):
    # Complete command allowlist: joined flags, duplicate overrides and extra
    # extern/link arguments cannot bypass validation.
    expected,query=expected_commands()
    if command!=expected or query_command!=query:
        raise ValueError('UnsupportedCompilerOptionsOrLinkage')


def main():
    study = REPO/'research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09'
    historical = study/'native_spline_acceleration_20260910/BUILD.json'
    if sha(historical)!='9d6c534f2d7185fb13b880b3b489410db05b83de5502bb35b550b4356f663766':
        raise ValueError('HistoricalBuildMismatch')
    old = json.loads(historical.read_text(encoding='utf-8'))
    cmd = old['command'].copy()
    cmd[2] = str(V4/'runtime/source/spline_bridge_optimized.rs')
    # Keep FFI curvature powi/arithmetic compiled exactly as the reference.
    # Optimizing it also changed returned curvature by one ULP in controls.
    output = V4/'runtime/bin/spline_bridge_optimized.dll'
    cmd[cmd.index('-o')+1] = str(output)
    # No root application library is linked: compile the exact frozen module.
    index = next(i for i,x in enumerate(cmd) if x.startswith('implicit_cad='))
    del cmd[index-1:index+1]
    paths = [Path(x.split('=',1)[1]) for x in cmd if x.startswith(('glam=','serde=','serde_json='))]
    pins={'glam':'424b0e9ef08fb83f8ccb4bb8e359883736b200f428fd79fc2eea3b6c5448d088',
          'serde_json':'9fd775d75e34aa16a65e0b06abd746deb0718cde698ffba114d806b79cf2db19',
          'serde':'26883d437873433f493893d2883e6ad6370b6e9eee7159d9e56fc36722196291'}
    for item in cmd:
        if item.startswith(('glam=','serde=','serde_json=')):
            name,p=item.split('=',1)
            if sha(p)!=pins[name]:raise ValueError('CompilerDependencyMismatch:'+name)
    ffi=V4/'runtime/source/spline_bridge_optimized.rs'
    frozen=V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/naca_spline.rs'
    reference=V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/spline_bridge.rs'
    if sha(frozen)!='4df18567b3cbcbae6a1dd7a940ccb558ac4ed3a249f4e6ecdf1c2b5645140b8b':
        raise ValueError('FrozenQuerySourceMismatch')
    start='use glam::DVec2;'
    original=reference.read_text(encoding='utf-8');new=ffi.read_text(encoding='utf-8')
    if original[original.index(start):]!=new[new.index(start):]:raise ValueError('ChangedFFIArithmetic')
    before = {str(p):sha(p) for p in paths+[historical,ffi,frozen,reference]}
    t = time.perf_counter()
    library = V4/'runtime/lib/libsdf_frozen_query.rlib'
    frozen = V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/naca_spline.rs'
    query_command = ['rustc','--edition=2024',str(frozen),'--crate-name','sdf_frozen_query',
        '--crate-type','rlib','-C','opt-level=3','-C','debuginfo=0','-o',str(library)]
    for index,item in enumerate(cmd):
        if item.startswith(('glam=','serde=')):
            query_command.extend(['--extern',item])
        if item.startswith('dependency='):
            query_command.extend(['-L',item])
    cmd.extend(['--extern','sdf_frozen_query='+str(library)])
    validate_options(cmd,query_command)
    query_build=subprocess.run(query_command,cwd=REPO,capture_output=True,text=True,timeout=120)
    query_build.check_returncode()
    run = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, timeout=120)
    after={p:sha(p) for p in before}
    if before!=after:raise ValueError('BuildInputChanged')
    record = dict(command=cmd,query_command=query_command,query_stderr=query_build.stderr,
        frozen_sha256=sha(frozen),inputs=before,inputs_after=after,
        query_library_sha256=sha(library),ffi_sha256=sha(ffi),
        seconds=time.perf_counter()-t,exit_code=run.returncode,
        compiler=subprocess.run(['rustc','-vV'],capture_output=True,text=True,check=True).stdout,
        dependencies=before,stdout=run.stdout,stderr=run.stderr,
        binary_sha256=hashlib.sha256(output.read_bytes()).hexdigest() if run.returncode==0 else None)
    out = V4/'results/surface_certificate_speed'
    out.mkdir(parents=True,exist_ok=True)
    (out/'BUILD.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(record),flush=True)
    run.check_returncode()


if __name__ == '__main__': main()
