"""Bounded policy/written-tensor check on retained inputs. No mesh generation."""
import ast
import copy
import dataclasses
from fractions import Fraction as Q
import json
from pathlib import Path
import struct
import sys
import time
from unittest.mock import patch
import numpy as np
import psutil

V4=Path(__file__).resolve().parents[1]; REPO=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel, GeometryFailure
from surface_binding import bind_canonical
from sf2_surface_policy import bind_sf2_policy, load_case, ROOT, CASE
from historical_capsule_surface import provider, SeedVertex, sha
from procedural_authorities import SeedFace, SeedEdge
from adaptive_sdf_carrier import _spd_edge_ratio
from shape_aware_metric import generic_geometry_metric_from_differential, apply_shape_metric
from prism_host_metric import intersect_tangent_metric
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate import kernel
from surface_execution import write_surface_contract


def retained_reader(authority):
    source=ROOT/'SF2/A/wing/verify_written_metric.py'
    tree=ast.parse(source.read_text())
    outer=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='verify')
    fn=next(n for n in outer.body if isinstance(n,ast.FunctionDef) and n.name=='unpack')
    ns=dict(struct=struct,np=np,SeedVertex=SeedVertex,SeedFace=SeedFace,SeedEdge=SeedEdge,
        a=authority.unit,vp=struct.Struct('<ddd i BB dddddd'),fp=struct.Struct('<iiii'),ep=struct.Struct('<iiiBB'))
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
    return ns['unpack']


def exact_majorant(actual, requirement):
    # Independent all-direction check of *computed* binary64 tensors, not a
    # certificate of the mathematical geometry or eigensolver. Principal minors
    # characterize PSD for symmetric 3x3; use the existing locked exact signs.
    exact=kernel.module('rank_geometry')
    m=[[Q(float(actual[i,j]))-Q(float(requirement[i,j])) for j in range(3)] for i in range(3)]
    if any(m[i][j]!=m[j][i] for i in range(3) for j in range(3)):return False
    minors=[m[i][i] for i in range(3)]
    minors += [m[i][i]*m[j][j]-m[i][j]*m[j][i] for i,j in ((0,1),(0,2),(1,2))]
    minors += [exact.dot(m[0],exact.cross(m[1],m[2]))]
    return all(exact.sign(v)>=0 for v in minors)


def main():
    start=time.perf_counter();out=V4/'results/sf2_coupled_policy';out.mkdir(parents=True,exist_ok=True)
    def save(name,value): (out/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',newline='\n')
    failures=[]
    def rejects(label,fn):
        try:fn()
        except (ValueError,GeometryFailure) as e:failures.append(dict(test=label,error=str(e)))
        else:raise AssertionError('Missing rejection: '+label)
    case=load_case()
    h=case['policy']['first_height_m']
    schedule=np.load(ROOT/'SF2/A/wing/planner/input_state/increments.npy',mmap_mode='r')
    assert np.all(schedule[:,0]==h)
    minimum_later=float(schedule[:,1:].min())
    assert case['policy']['minimum_owned_layer_thickness_m']<=minimum_later
    with CanonicalKernel([str(REPO/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
        g=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(),1.)
        b=bind_canonical(g);before=b.inspect();t=time.perf_counter();bind_sf2_policy(b);bind_seconds=time.perf_counter()-t
        after=b.inspect();assert after['executable'];a=b.authority;c=b.metric_connection
        unpack=retained_reader(a)
        oldpath=ROOT/'SF2/A/wing/surface/A/v3mmg1_A.bin'
        v,f,e,historical=unpack(oldpath)
        base=unpack(ROOT/'SF2/A/wing/surface/A/metric_unscaled_verification.bin')[3]
        assert sha(oldpath)==load_case()['source_files'][str(oldpath.relative_to(REPO)).replace('\\','/')]
        t=time.perf_counter();writer_timing,stats=write_surface_contract(b,out,v,f,e);write_seconds=time.perf_counter()-t
        actual=unpack(out/'metric.bin')[3]
        assert np.array_equal(actual,historical)
        assert np.array_equal(actual,base/(c.length_scale*c.length_scale))
        curves=[set() for _ in v]
        for edge in e:
            for i in edge.vertices:curves[i].add(edge.curve_ref)
        diffs=[];carrier_diffs=[];ratios=[];psd_counts={k:0 for k in ('geometry','feature','geometry_feature','prism','shape')}
        totals={k:0 for k in psd_counts};bad=[];t=time.perf_counter()
        for i,vertex in enumerate(v):
            composed,row=c.evaluate(vertex.point,vertex.sheet_ref,curves=curves[i],trace=True)
            row['shape']=apply_shape_metric(row['prism'],row['principal'],c.shape)[0]
            # Binary writer stores the symmetric upper triangle; compare the
            # identical six stored entries, not unused lower-triangle rounding.
            diffs.append(float(np.max(np.abs(composed[np.triu_indices(3)]-actual[i][np.triu_indices(3)]))))
            for key in totals:
                requirement=row[key]
                if requirement is None:continue
                totals[key]+=1
                requirement=np.triu(requirement)+np.triu(requirement,1).T
                passed=exact_majorant(actual[i],requirement)
                psd_counts[key]+=int(passed)
                if not passed:bad.append(dict(vertex=i,term=key))
                ratios.append(_spd_edge_ratio(requirement,actual[i],a.exact_normal(vertex.sheet_ref,vertex.point)))
            # Independently reproduce the actual historical carrier sequence.
            m,p=generic_geometry_metric_from_differential(a,vertex,1.)
            m,_=intersect_tangent_metric(m,a.exact_normal(vertex.sheet_ref,vertex.point),c.prism)
            m,_=apply_shape_metric(m,p,c.shape);m=m/(c.length_scale*c.length_scale)
            carrier_diffs.append(float(np.max(np.abs(c.carrier(vertex.point,vertex.sheet_ref)-m))))
        assert max(diffs)==0 and max(carrier_diffs)==0
        assert not bad,bad[:10]
        metrics_seconds=time.perf_counter()-t
        rejects('weaker metric override',lambda:c.write_contract(out/'forbidden.bin',v,f,e,metric_override=[np.eye(3)]*len(v)))
        rejects('stronger metric override is not an implicit policy',lambda:c.write_contract(out/'forbidden.bin',v,f,e,metric_override=[np.eye(3)*1e8]*len(v)))
        rejects('minimum-size conflict',lambda:c.bounds(np.eye(3)*1e15))
        rejects('maximum-size conflict',lambda:c.bounds(np.eye(3)))
        rejects('non-SPD',lambda:c.bounds(-np.eye(3)))
        rejects('nonfinite',lambda:c.bounds(np.eye(3)*float('nan')))
        rejects('unknown curve metric reference',lambda:c.evaluate(v[0].point,v[0].sheet_ref,curves={999}))
        original=c.prism;c.prism=dataclasses.replace(original,first_height_m=original.first_height_m*2)
        rejects('altered prism input',lambda:b.inspect());c.prism=original
        original_scale=c.length_scale;c.length_scale=.8
        rejects('altered family scale',lambda:b.inspect());c.length_scale=original_scale
        b.metric_connection=None
        rejects('detached complete composition',lambda:b.inspect())
        b.metric_connection=c
        for label,mutate in [('height',lambda d:d['policy'].update(first_height_m=.001)),
                             ('source',lambda d:d['source_files'].update({next(iter(d['source_files'])):'0'*64})),
                             ('unsupported policy',lambda d:d.update(schema='unknown'))]:
            data=copy.deepcopy(case);mutate(data);p=out/('invalid_'+label.replace(' ','_')+'.json');save(p.name,data)
            rejects(label,lambda:load_case(p))
        old=b.geometry_identity;b.geometry_identity='altered';rejects('geometry identity',b.inspect);b.geometry_identity=old
        old=b.units;b.units='mm';rejects('units',b.inspect);b.units=old
        # Existing source-corner check on the same retained labelled carrier
        # ledger; no full-surface certification or intersection campaign.
        x=np.array([vertex.point for vertex in v]);tri=np.array([face.vertices for face in f])
        fr=np.array([face.sheet_ref for face in f]);ce=np.array([(*edge.vertices,edge.curve_ref) for edge in e])
        cr=np.array([vertex.corner_ref for vertex in v])
        coverage=b.qualification.check_source_coverage(a,x,tri,fr,ce,cr)
        rejects('missing source curve',lambda:b.qualification.check_source_coverage(a,x,tri,fr,ce[ce[:,2]!=ce[0,2]],cr))
        missing=cr.copy();missing[:]=0
        rejects('missing corners',lambda:b.qualification.check_source_coverage(a,x,tri,fr,ce,missing))
        altered=x.copy();i=int(np.flatnonzero(cr)[0]);altered[i,0]=np.nextafter(altered[i,0],np.inf)
        rejects('nonexact corner',lambda:b.qualification.check_source_coverage(a,altered,tri,fr,ce,cr))
        # Reach this SAME source-coverage hook through the real file-backed
        # qualifier, stopping explicitly before any full geometry audit.
        import audit_adaptive_surface as audit
        probe=out/'qualification_entry';probe.mkdir(exist_ok=True)
        np.savez_compressed(probe/'SURFACE.npz',coordinates=x,triangles=tri,metric=actual)
        np.savez_compressed(probe/'REFERENCES.npz',face_refs=fr,
            vertex_refs=np.array([vv.sheet_ref for vv in v]),curve_edges=ce,corner_refs=cr)
        class BoundaryReached(Exception):pass
        with patch.object(audit,'audit_arrays',side_effect=BoundaryReached):
            try:audit.audit_directory(probe,a,b.qualification)
            except BoundaryReached:pass
            else:raise AssertionError('Expected bounded audit stop')
        save('ADMISSION.json',after);save('WRITER.json',stats)
        result=dict(classification='V4LiveSplineSF2CoupledMetricBindingVerifiedOnRetainedInputs',
            geometry_identity=g.identity,spline_identity=a.spline_packet['spline_identity'],policy_identity=c.source_identity,
            first_height_m=h,first_height_hex=h.hex(),first_height_meaning=case['policy']['first_height_meaning'],
            stored_first_increment_equal_count=int(len(schedule)),minimum_nominal_later_increment_m=minimum_later,
            declared_owned_floor_m=c.prism.minimum_owned_layer_thickness_m,
            owned_floor_below_nominal_by_m=minimum_later-c.prism.minimum_owned_layer_thickness_m,
            vertices=len(v),faces=len(f),feature_edges=len(e),feature_vertices=sum(bool(x) for x in curves),
            historical_tensor_bitwise_equal=True,scaled_once_exact=True,contract_bytes_equal=(out/'metric.bin').read_bytes()==oldpath.read_bytes(),
            maximum_written_difference=max(diffs),maximum_historical_carrier_difference=max(carrier_diffs),
            positive_eigenvalue_minimum=float(np.linalg.eigvalsh(actual).min()),
            maximum_eigenvalue=float(np.linalg.eigvalsh(actual).max()),
            exact_computed_tensor_majorants=psd_counts,applicable_contribution_counts=totals,
            failed_exact_majorants=bad,maximum_required_over_written_tangent_ratio=max(ratios),
            all_direction_check_scope='Exact rational PSD of serialized tensor minus each computed unscaled input term, including feature where applicable. Does not certify curvature or actual prism trajectories.',
            normal_predictor_scope='Historical direct differential 2.50 reserve; no curve ledger in carrier callback; all curve constraints intersected by final writer.',
            source_coverage=coverage,remaining_proof_scope=case['qualification']['finite_domain_basis'],
            admission_executable=after['executable'],negative_checks=failures,
            actual_file_backed_coverage_hook_reached=True,
            audit_stop='intentional test sentinel before audit_arrays/continuous/intersections; no full audit',
            source_contract_sha256=sha(oldpath),written_sha256=sha(out/'metric.bin'),
            timings=dict(binding_seconds=bind_seconds,writer_seconds=write_seconds,all_point_checks_seconds=metrics_seconds,total_seconds=time.perf_counter()-start),
            python_peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
            no_mesh_or_prism_generation=True,no_qualification_or_publication=True)
        save('RESULTS.json',result);print(json.dumps(result,indent=2))


if __name__=='__main__':main()
