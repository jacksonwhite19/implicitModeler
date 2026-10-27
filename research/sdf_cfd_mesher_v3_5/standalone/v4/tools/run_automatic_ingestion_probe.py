"""Bounded audit runner; no extraction, conditioning or production modifications."""
import argparse, hashlib, json, subprocess, sys, time
from pathlib import Path
import psutil
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from replay_surface_dispatch import verify_kernel_sources
from historical_capsule_surface import save,sha


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--build',action='store_true');args=parser.parse_args()
    out=V4/'results/automatic_field_ingestion';out.mkdir(exist_ok=True)
    source,_=verify_kernel_sources()
    deps=ROOT/'target/debug/deps';exe=ROOT/'target/debug/automatic_ingestion_probe.exe'
    libraries={'v4_canonical_kernel':'libv4_canonical_kernel-cf12d17021cc7566.rlib',
        'implicit_cad':'libimplicit_cad-8e729cfd79f47752.rlib','glam':'libglam-e2f4f49b3c45c908.rlib',
        'serde_json':'libserde_json-2a71de8854bd2702.rlib'}
    command=['rustc','--edition=2024',str(V4/'tools/automatic_ingestion_probe.rs'),
        '--crate-name','automatic_ingestion_probe','-L','dependency='+str(deps)]
    registry=Path.home()/'.cargo/registry/src/index.crates.io-1949cf8c6b5b557f'
    for version in ('0.53.1','0.52.6'):
        command+=['-L','native='+str(registry/('windows_x86_64_msvc-'+version)/'lib')]
    for name,path in libraries.items():command+=['--extern',name+'='+str(deps/path)]
    command+=['-C','debuginfo=0','-o',str(exe)]
    if args.build:subprocess.run(command,check=True,timeout=60,cwd=ROOT)
    t=time.perf_counter();process=subprocess.Popen([str(exe),str(V4/'results/generic_canonical_bridge/DIRECT.json')],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,cwd=ROOT)
    peak=0
    while True:
        try:peak=max(peak,psutil.Process(process.pid).memory_info().peak_wset)
        except psutil.NoSuchProcess:pass
        try:stdout,stderr=process.communicate(timeout=.02);break
        except subprocess.TimeoutExpired:
            if time.perf_counter()-t>60:process.kill();raise RuntimeError('ProbeDeadlineExceeded')
    elapsed=time.perf_counter()-t
    if process.returncode:raise RuntimeError(stderr)
    raw=json.loads(stdout);save(out/'QUERY_RESULTS.json',raw)
    projections=raw['projections'];batch=raw['feature_batch'];evals=raw['evaluations']
    cap=[p for p in raw['polygon_midpoints'] if p['section_a'][0]!=p['section_b'][0]
         and p['result']['normal_sector']['stable_id'].endswith('trailing_edge.cap_sector')]
    failures=[p for p in projections if not p['finite'] or abs(p['residual_model'])>=1e-4]
    summary=dict(geometry_identity=raw['geometry_identity'],source=source,build_command=command,
        linked_libraries=[dict(path=str(deps/v),sha256=sha(deps/v)) for v in libraries.values()],
        executable_sha256=sha(exe),executable_bytes=exe.stat().st_size,
        geometry_descriptor_equal_to_retained=True,scalar_samples=raw['scalar_count'],scalar_bits_equal=raw['scalar_bits_equal'],
        features=len(raw['inventory']['features']),sectors=len(raw['inventory']['sectors']),junctions=len(raw['inventory']['junctions']),
        evaluations=len(evals),evaluation_none=sum(e['result'] is None for e in evals),
        maximum_feature_residual_model=max(abs(e['result']['diagnostics']['feature_to_sdf_residual']) for e in evals if e['result']),
        feature_batch=batch['summary'],feature_ambiguities=sum(bool(r['diagnostics']['ambiguity']) for r in batch['results']),
        repeated_feature_results_equal=raw['repeat_results_equal'],
        stored_section_points=raw['stored_section_points'],polygon_midpoint_queries=len(raw['polygon_midpoints']),
        nonvertical_section_edges_labelled_trailing_cap=len(cap),examples_nonvertical_cap=cap[:2],
        projection_samples=len(projections),projection_nonfinite=sum(not p['finite'] for p in projections),
        projection_zero_normal=sum(p['zero_normal'] for p in projections),
        projection_residual_not_below_existing_break=len(failures),projection_failures=failures,
        maximum_projection_residual_model=max(abs(p['residual_model']) for p in projections),
        query_options=raw['query_options'],load_seconds=raw['load_seconds'],projection_seconds=raw['projection_seconds'],
        feature_seconds=raw['feature_seconds'],timed_scalar_queries=raw['timed_scalar_queries'],
        timed_scalar_seconds=raw['timed_scalar_seconds'],wall_seconds=elapsed,
        sampled_peak_working_set_bytes=peak,memory_scope='Polled process lifetime peak counter; lower bound if final peak missed',
        units='model millimetres; divide model residuals by 1000 for metres',precision='binary32 kernel; binary64 transport',
        stderr=stderr,conditioning_invoked=False,mesh_generated=False,
        initial_link_failure='LNK1181 windows.0.53.0.lib; resolved by existing cached native library search paths, no downloads')
    save(out/'PROBE_SUMMARY.json',summary)
    print(json.dumps({k:v for k,v in summary.items() if k not in ('build_command','linked_libraries','examples_nonvertical_cap','projection_failures')}))


if __name__=='__main__':main()
