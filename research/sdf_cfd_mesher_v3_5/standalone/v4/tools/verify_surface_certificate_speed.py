"""Compare optimized build with sealed scientific inputs; no mesh generation."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time
if sys.flags.optimize: raise RuntimeError('OptimizedPythonUnsupportedForVerification')
from unittest.mock import patch
import numpy as np
import psutil

V4 = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V4/'src'),str(V4/'tools')]
sys.dont_write_bytecode = True
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
from surface_binding import bind_canonical
from standalone_case import bind
import canonical_spline_authority as owner
from audit_adaptive_surface import audit_directory
from historical_capsule_surface import write_off
from continuous_geometry_certifier import certify_face
from build_optimized_spline_bridge import validate_options, expected_build_inputs

PROFILE=V4/'results/surface_certificate_speed/baseline_exact_case/PROFILE.json'
BUILD=V4/'results/surface_certificate_speed/BUILD.json'


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path, record): path.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def expected_inputs(retained,case,candidate):
    paths=(case,V4/'runtime/bin/spline_bridge.dll',candidate,PROFILE,BUILD,
        V4/'runtime/bin/canonical_kernel_host.exe',Path(__file__),
        V4/'tools/profile_surface_certificate.py',V4/'tools/build_optimized_spline_bridge.py',
        V4/'tools/canonical_spline_authority.py',V4/'runtime/source/spline_bridge_optimized.rs',
        V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/naca_spline.rs',V4/'runtime/lib/libsdf_frozen_query.rlib',
        retained/'SURFACE.npz',retained/'REFERENCES.npz',retained/'GEOMETRY_CERTIFICATE.npz',
        retained/'QUALITY.npz',retained/'AUDIT.json',retained/'IMPLEMENTATION.json')
    return {str(p.resolve()):sha(p) for p in paths}

def validate_build(candidate):
    build=json.loads(BUILD.read_text(encoding='utf-8'))
    validate_options(build['command'],build['query_command'])
    if (build['exit_code']!=0 or build['binary_sha256']!=sha(candidate) or
            build['query_library_sha256']!=sha(V4/'runtime/lib/libsdf_frozen_query.rlib') or
            build['inputs']!=build['inputs_after'] or build['inputs']!=expected_build_inputs() or
            build['dependencies']!=build['inputs'] or
            build['ffi_sha256']!=sha(V4/'runtime/source/spline_bridge_optimized.rs') or
            build['frozen_sha256']!=sha(V4/'src/sdf_cfd_mesher_v4/sdf/_frozen/naca_spline.rs')):
        raise ValueError('BuildRecordMismatch')

def expected_profile_inputs():
    retained=V4/'results/live_sf2_surface/replay_20260929/authority'
    paths=(V4/'tools/profile_surface_certificate.py',V4/'cases/standalone_smooth_development.json',V4/'runtime/bin/spline_bridge.dll',
        V4/'runtime/bin/canonical_kernel_host.exe',V4/'runtime/surface/continuous_geometry_certifier.py',
        V4/'tools/canonical_spline_authority.py',retained/'SURFACE.npz',retained/'REFERENCES.npz')
    return {str(p.resolve()):sha(p) for p in paths}

def validate_profile(profile,payload):
    if (profile['schema']!='V4SurfaceCertificateProfile/1' or
            profile['dll_sha256']!=sha(V4/'runtime/bin/spline_bridge.dll') or
            profile['dll_path']!=str(V4/'runtime/bin/spline_bridge.dll') or
            profile['producer_sha256']!=sha(V4/'tools/profile_surface_certificate.py') or
            profile['payload_sha256']!=owner.digest(payload) or
            profile['inputs']!=expected_profile_inputs()):
        raise ValueError('ProfileRecordMismatch')

def validate_checks(record,candidate,inputs_hash,profile):
    required_negative=['nonfinite_native_rejected']*3+['incompatible_profile_rejected',
        'changed_dll_rejected','missing_dll_rejected','degenerate_rejected','inverted_rejected']
    if (record.get('candidate_sha256')!=sha(candidate) or record.get('inputs_sha256')!=inputs_hash or
            record.get('query_count')!=947 or record.get('query_values_bit_identical') is not True or
            record.get('face_sample_count')!=35 or record.get('face_samples_bit_identical') is not True or
            record.get('normal_initialization_verified') is not True or
            record.get('repeated_profile_verified') is not True or record.get('negative')!=required_negative or
            record.get('payload_sha256')!=profile['payload_sha256'] or
            record.get('geometry_identity')!=profile['geometry_identity'] or record.get('generated_mesh') is not False):
        raise ValueError('ChecksRecordMismatch')

def checked_rows(actual,expected):
    if len(actual)!=len(expected):raise ValueError('FaceSampleCountMismatch')
    for a,b in zip(actual,expected):
        if a[0]!=b[0] or a[3]!=b[3] or np.asarray(a[1:3],dtype='<f8').tobytes()!=np.asarray(b[1:3],dtype='<f8').tobytes():
            raise ValueError('FaceSampleBitMismatch')


def load_library(path,payload):
    lib = ctypes.CDLL(str(path))
    lib.spline_load.argtypes = [ctypes.c_char_p,ctypes.c_size_t]
    lib.spline_load.restype = ctypes.c_int
    lib.spline_query.argtypes = [ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_double)]
    lib.spline_query.restype = ctypes.c_int
    raw = json.dumps(payload,separators=(',',':'),allow_nan=False).encode()
    assert lib.spline_load(raw,len(raw)) == 0
    def query(point):
        out = (ctypes.c_double*8)()
        code = lib.spline_query(float(point[0]),float(point[1]),out)
        return code,np.asarray(tuple(out),dtype='<f8').tobytes()
    return query


def without_measurements(value):
    if isinstance(value,dict):
        return {k:without_measurements(v) for k,v in value.items() if k not in
            ('seconds','timing','peak_rss_bytes','retained_rss_bytes','topology_seconds')}
    if isinstance(value,list): return [without_measurements(v) for v in value]
    return value


def compare_saved_audit(directory,retained):
    result=json.loads((directory/'AUDIT.json').read_text(encoding='utf-8'))
    prior=json.loads((retained/'AUDIT.json').read_text(encoding='utf-8'))
    assert result['passed'], 'IndependentAuditFailure'
    for filename in ('GEOMETRY_CERTIFICATE.npz','QUALITY.npz'):
        with np.load(directory/filename) as new,np.load(retained/filename) as old:
            assert sorted(new.files)==sorted(old.files)
            for key in old.files:
                assert new[key].dtype==old[key].dtype and new[key].shape==old[key].shape
                assert new[key].tobytes()==old[key].tobytes(), 'ScientificArrayMismatch:'+filename+':'+key
    left,right=without_measurements(result),without_measurements(prior)
    texts=[]
    # Explicit case-description differences, not excluded scientific blocks.
    # Every other field, including source coverage/corner/incidence outcomes,
    # remains an exact comparison. Preserve both descriptions for review.
    expected_texts = (
        ('branch_continuity','analytic_fact',
         'SF2 stored cubic source charts, complete physical incidence, exact corners, native spline branches and derivative-hull curvature; adaptive signed-distance Hessian and finite-curve certificates',
         'Stored cubic source charts, physical incidence, exact corners, native branches and derivative-hull curvature'),
        ('source_coverage','finite_domain_basis',
         'source-labelled spline charts, preserved incidence and source projection; historical gate has no separate arbitrary-triangle finite-cap membership predicate',
         'source-labelled spline charts and preserved incidence; no arbitrary-triangle finite-cap predicate'))
    for section,key,old_text,new_text in expected_texts:
        assert right[section][key]==old_text and left[section][key]==new_text,'UnexpectedQualificationDescription'
        texts.append(dict(section=section,key=key,reference=right[section].pop(key),candidate=left[section].pop(key)))
    assert left==right, 'ScientificAuditMismatch'
    return result,texts


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--full',action='store_true')
    parser.add_argument('--candidate',type=Path,default=V4/'runtime/bin/spline_bridge_optimized.dll')
    parser.add_argument('--collect',action='store_true',help='Compare completed audit without rerunning qualification')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    t=time.perf_counter()
    retained=V4/'results/live_sf2_surface/replay_20260929/authority'
    case=V4/'cases/standalone_smooth_development.json'
    candidate=args.candidate.resolve()
    reference=V4/'runtime/bin/spline_bridge.dll'
    if args.collect:
        inputs=json.loads((args.output/'INPUTS.json').read_text(encoding='utf-8'))
        assert inputs==expected_inputs(retained,case,candidate),'IncompleteOrStaleInputMap'
        validate_build(candidate)
        completion=json.loads((args.output/'COMPLETED.json').read_text(encoding='utf-8'))
        assert completion['inputs_sha256']==sha(args.output/'INPUTS.json')
        assert completion['checks_sha256']==sha(args.output/'CHECKS.json')
        assert completion['candidate_sha256']==sha(candidate)
        assert completion['audit_files']=={p.name:sha(p) for p in (args.output/'audit').iterdir() if p.is_file()}
        result,texts=compare_saved_audit(args.output/'audit',retained)
        record=json.loads((args.output/'CHECKS.json').read_text(encoding='utf-8'))
        profile=json.loads(PROFILE.read_text(encoding='utf-8'))
        assert profile['inputs']==expected_profile_inputs(),'IncompleteOrStaleProfileMap'
        validate_checks(record,candidate,sha(args.output/'INPUTS.json'),profile)
        record.update(independent_full_audit=True,faces=result['triangles'],certificate_arrays_bit_identical=True,
            quality_arrays_bit_identical=True,case_description_differences=texts,
            audit_seconds=result['seconds'],face_seconds=result['continuous_geometry']['seconds'],
            bounds=result['continuous_geometry'],feature=result['feature_ownership'],
            intersections=result['self_intersection'],peak_audit_rss=result['peak_rss_bytes'],
            collection_seconds=time.perf_counter()-t)
        save(args.output/'RESULT.json',record)
        print(json.dumps(record),flush=True)
        return
    validate_build(candidate)
    inputs=expected_inputs(retained,case,candidate)
    save(args.output/'INPUTS.json',inputs)
    with CanonicalKernel([str(V4/'runtime/bin/canonical_kernel_host.exe')],timeout=60) as host:
        script=json.loads(case.read_text(encoding='utf-8'))['geometry']['rhai']
        geometry=host.load_rhai(script,1.)
        payload=geometry.stored_spline()['data']['sections'][0]['spline']
        # Each library has independent OnceLock storage. The reference authority
        # does not use this CDLL in this process; no repeated loads are hidden.
        rq=load_library(reference,payload)
        # Exercise NORMAL production initialization; do not inject private state.
        if candidate!=owner.DLL:raise ValueError('CandidateNotProductionBinding')
        binding=bind(bind_canonical(geometry),case);a=binding.authority
        def cq(point):
            out=(ctypes.c_double*8)()
            code=owner._native.spline_query(float(point[0]),float(point[1]),out)
            return code,np.asarray(tuple(out),dtype='<f8').tobytes()
        with np.load(retained/'SURFACE.npz') as z,np.load(retained/'REFERENCES.npz') as r:
            x,f,fr=z['coordinates'],z['triangles'],r['face_refs']
        ids=np.unique(np.r_[np.linspace(0,len(x)-1,512,dtype=int),np.linspace(0,len(f)-1,512,dtype=int)%len(x)])
        points=x[ids][:,[0,2]]
        points=np.r_[points,[[0.,0.],[1.,0.],[1.,.0012600078480318189],[1.,-.0012600078480318189],
            [-.1,.1],[1.1,-.1],[.5,0.],[-0.,-0.]]]
        ts=time.perf_counter();rb=[rq(p) for p in points];reference_seconds=time.perf_counter()-ts
        ts=time.perf_counter();cb=[cq(p) for p in points];candidate_seconds=time.perf_counter()-ts
        assert all(code==0 for code,_ in rb+cb),'FiniteNativeQueryFailure'
        differences=[]
        for i,(left,right) in enumerate(zip(rb,cb)):
            if left!=right:
                l=np.frombuffer(left[1],dtype='<f8');r=np.frombuffer(right[1],dtype='<f8')
                differences.append(dict(index=i,point=points[i].tolist(),reference_status=left[0],
                    candidate_status=right[0],reference=l.tolist(),candidate=r.tolist(),
                    differing_fields=np.flatnonzero(l.view('u8')!=r.view('u8')).tolist(),
                    maximum_difference=float(np.max(abs(l-r)))))
        if differences:
            save(args.output/'QUERY_MISMATCH.json',dict(count=len(differences),first=differences[:20],
                total=len(points),reference_sha256=sha(reference),candidate_sha256=sha(candidate)))
            raise AssertionError('NativeQueryBitMismatch; preserved QUERY_MISMATCH.json')
        negative=[]
        for p in ((float('nan'),0.),(0.,float('inf')),(-float('inf'),0.)):
            assert rq(p)[0]==cq(p)[0]==2
            negative.append('nonfinite_native_rejected')
        again=owner.existing_authority(a.source)
        assert again.source==a.unit.source
        altered=json.loads(json.dumps(payload));altered['spans'][0]['control'][0][0]+=.1
        def rejects(label,fn):
            try:fn()
            except (ValueError,FileNotFoundError,RuntimeError):negative.append(label)
            else:raise ValueError('RequiredRejectionAbsent:'+label)
        rejects('incompatible_profile_rejected',lambda:owner._library(altered))
        with patch.object(owner,'LOCKS',{**owner.LOCKS,candidate:'0'*64}):
            rejects('changed_dll_rejected',lambda:owner._library(payload))
        with patch.object(owner,'LOCKS',{args.output/'missing.dll':'0'*64}):
            rejects('missing_dll_rejected',lambda:owner._library(payload))
        profile=json.loads(PROFILE.read_text(encoding='utf-8'));validate_profile(profile,payload)
        sample=[certify_face(a,int(fr[i]),x[f[i]]) for i in profile['sample_ids']]
        checked_rows(sample,profile['baseline'])
        skin=a.unit.skin
        tri=x[f[profile['sample_ids'][0]]]
        assert certify_face(a,skin,np.repeat(tri[:1],3,axis=0))[3]=='degenerate'
        assert certify_face(a,int(fr[0]),x[f[0]][::-1])[3]=='orientation'
        negative.extend(('degenerate_rejected','inverted_rejected'))
        record=dict(query_count=len(points),query_values_bit_identical=True,
            reference_seconds=reference_seconds,candidate_seconds=candidate_seconds,
            query_speedup=reference_seconds/candidate_seconds,face_sample_count=len(sample),
            face_samples_bit_identical=True,negative=negative,geometry_identity=geometry.identity,
            independent_full_audit=False,generated_mesh=False,candidate_sha256=sha(candidate),
            inputs_sha256=sha(args.output/'INPUTS.json'),normal_initialization_verified=True,
            repeated_profile_verified=True,payload_sha256=owner.digest(payload))
        validate_checks(record,candidate,sha(args.output/'INPUTS.json'),profile)
        save(args.output/'CHECKS.json',record)
        print(json.dumps(record),flush=True)
        if args.full:
            directory=args.output/'audit';directory.mkdir(exist_ok=False)
            for name in ('SURFACE.npz','REFERENCES.npz'):
                shutil.copyfile(retained/name,directory/name)
                assert sha(directory/name)==sha(retained/name)
            write_off(directory/'surface.off',x,f)
            result=audit_directory(directory,a,binding.qualification)
            assert result['passed'], 'IndependentAuditFailure'
            with np.load(directory/'GEOMETRY_CERTIFICATE.npz') as new,np.load(retained/'GEOMETRY_CERTIFICATE.npz') as old:
                for key in old.files:
                    assert new[key].dtype==old[key].dtype and new[key].shape==old[key].shape and new[key].tobytes()==old[key].tobytes(), 'FullCertificateMismatch:'+key
            with np.load(directory/'QUALITY.npz') as new,np.load(retained/'QUALITY.npz') as old:
                assert sorted(new.files)==sorted(old.files)
                for key in old.files: assert new[key].tobytes()==old[key].tobytes(), 'QualityMismatch:'+key
            result,texts=compare_saved_audit(directory,retained)
            record.update(independent_full_audit=True,faces=len(f),certificate_arrays_bit_identical=True,
                quality_arrays_bit_identical=True,case_description_differences=texts,
                audit_seconds=result['seconds'],face_seconds=result['continuous_geometry']['seconds'],
                bounds=result['continuous_geometry'],feature=result['feature_ownership'],intersections=result['self_intersection'])
        assert inputs==expected_inputs(retained,case,candidate),'InputChanged'
        if args.full:
            save(args.output/'COMPLETED.json',dict(candidate_sha256=sha(candidate),
                inputs_sha256=sha(args.output/'INPUTS.json'),checks_sha256=sha(args.output/'CHECKS.json'),
                audit_files={p.name:sha(p) for p in directory.iterdir() if p.is_file()}))
        record.update(seconds=time.perf_counter()-t,peak_rss=psutil.Process().memory_info().peak_wset)
        save(args.output/'RESULT.json',record)
        print(json.dumps(record),flush=True)


if __name__=='__main__': main()
