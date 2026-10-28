"""Focused external comparison-contract checks; retained input files immutable."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import passage_verify3_auth as c

PRE=c.read(c.BASE/'outputs/PASSAGE_VERIFY1_PREFLIGHT.json')
A=Path(PRE['outputs']['audit']);B=Path(PRE['baseline'])

class ContractTests(unittest.TestCase):
    def test_saved_pair(self):
        self.assertEqual(c.authenticate_pair(A,B,'audit')['native_raw_files_exact'],88)

    def changed_read(self,root,rel,mutate,call):
        original=c.read
        def changed(p):
            value=original(p)
            if Path(p)==root/rel:mutate(value)
            return value
        with patch.object(c,'read',side_effect=changed):
            with self.assertRaises((ValueError,KeyError)):call()

    def test_parent_chain_tampering(self):
        for rel in ('background/PARENT.json','background/PARENT_EXPECTATIONS.json','background/EVIDENCE_HANDOFF.json','background/RESULT.json','background/JOINED_RECONSTRUCTION.json','background/conditioning/CONDITIONING.json'):
            def mutate(v):
                if rel.endswith('PARENT.json'):p=v
                elif rel.endswith(('PARENT_EXPECTATIONS.json','EVIDENCE_HANDOFF.json')):p=v['binding']
                elif rel.endswith('JOINED_RECONSTRUCTION.json'):p=v['provenance']['binding']
                else:p=v['parent']
                p['parent_prism_mesh']='0'*64
            with self.subTest(rel=rel):self.changed_read(A,rel,mutate,lambda:c.parent_chain(A))

    def test_shell_current_source_policy(self):
        for rel,key in [('shell/ACCEPTED_SHELL.json','mesh'),('CURRENT.json','mesh'),('CURRENT.json','sha256'),('CURRENT.json','coordinate_hash'),('shell/ACCEPTED_SHELL.json','native'),('shell/ACCEPTED_SHELL.json','source'),('background/POLICY.json','schema')]:
            with self.subTest(rel=rel,key=key):self.changed_read(A,rel,lambda v:v.__setitem__(key,'tampered'),lambda:c.parent_chain(A))

    def test_case_profile_strict(self):
        case=c.read(A/'CASE.json')
        for key,value in [('origin','explicit_api'),('profile',{}),('background_identity','0'*64),('stop_after','surface')]:
            v=copy.deepcopy(case);v['verification'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):c.selected_case(v,'audit')
        v=copy.deepcopy(case);v['verification']['extra']='bad'
        with self.assertRaises(ValueError):c.selected_case(v,'audit')

    def test_source_mapping(self):
        self.assertEqual(c.package_path(str(c.PKG/'v4/tools/general_sdf_authority.py'),c.PKG),'v4/tools/general_sdf_authority.py')
        for path in (str(c.ORIGINAL/'v4/tools/general_sdf_authority.py'),str(c.PKG/'../escape.py'),'v4/tools/general_sdf_authority.py'):
            with self.subTest(path=path),self.assertRaises(ValueError):c.package_path(path,c.PKG)

    def test_provenance_tampering(self):
        cases=[('first/REQUEST.json',lambda v:v['source_documents'][0].__setitem__('sha256','0'*64)),
            ('first/REQUEST.json',lambda v:v['source_documents'][0].__setitem__('path',str(B/'CASE.json'))),
            ('first/source/ADMISSION.json',lambda v:v['authority'].__setitem__('source_branch','0'*64)),
            ('upstream/surface/A/fresh/SURFACE_AUTHORITY.json',lambda v:v.__setitem__('acceptance_identity','0'*64)),
            ('upstream/surface/A/fresh/IMPLEMENTATION.json',lambda v:v['dependencies'].__setitem__('standalone/extra.py','0'*64))]
        for rel,mutate in cases:
            with self.subTest(rel=rel):self.changed_read(A,rel,mutate,lambda:c.surface_provenance(A,c.PKG,'audit'))

    def test_expected_roots(self):
        for key in ('AUDIT_ROOT','PREFLIGHT','OLD_FAILURE'):
            with self.subTest(key=key),patch.object(c,key,'0'*64),self.assertRaises(ValueError):c.authenticate_pair(A,B,'audit')

    def test_raw_population_and_length(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            for name in c.RAW_NAMES:(folder/name).write_bytes(b'')
            self.assertEqual(len(c.native_raw(folder)),22)
            (folder/'points.f64').write_bytes(b'x')
            with self.assertRaises(ValueError):c.native_raw(folder)
            (folder/'points.f64').write_bytes(b'');(folder/'extra').write_bytes(b'')
            with self.assertRaises(ValueError):c.native_raw(folder)

    def test_array_tampering(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as temp:
            a=Path(temp)/'a.npz';b=Path(temp)/'b.npz'
            np.savez(a,points=np.zeros((1,3)),ids=np.array([1],dtype='<i8'))
            np.savez(b,points=np.zeros((1,3)),ids=np.array([2],dtype='<i8'))
            with self.assertRaises(ValueError):c.exact_arrays(a,b)
            np.savez(b,points=np.zeros((1,3)),ids=np.array([1],dtype='<i4'))
            with self.assertRaises(ValueError):c.exact_arrays(a,b)

    def test_named_measurements_only(self):
        value={'seconds':1.0,'threshold':0.25}
        self.assertEqual(c.measurements(value,('seconds',)),{'seconds':'<measured>','threshold':0.25})
        for bad in (-1,float('nan'),'fake',True):
            with self.subTest(bad=bad),self.assertRaises(ValueError):c.measurements({'seconds':bad},('seconds',))

    def test_duplicate_json(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'x.json';p.write_text('{"x":1,"x":2}',encoding='utf8')
            with self.assertRaises(ValueError):c.read(p)

    def test_completion_closure_tampering(self):
        cases=[('VERIFICATION.json',lambda v:v['completion_files'].pop('CASE.json')),
            ('FINALIZATION_HANDOFF.json',lambda v:v.__setitem__('finalization',{})),
            ('VERIFICATION.json',lambda v:v['closure_records']['source'].__setitem__('path',str(B/'CASE.json'))),
            ('VERIFICATION.json',lambda v:v['closure_records']['supervisor'].__setitem__('sha256','0'*64))]
        for rel,mutate in cases:
            with self.subTest(rel=rel):self.changed_read(A,rel,mutate,lambda:c.completion(A,c.AUDIT_ROOT,'audit'))
        actual=c.sha
        for path in (A/'RESULT.json',A/'background/RESULT.json',A/'background/READBACK.json',Path(c.read(A/'VERIFICATION.json')['closure_records']['source']['path']),Path(c.read(A/'VERIFICATION.json')['closure_records']['supervisor']['path'])):
            with self.subTest(path=str(path)),patch.object(c,'sha',side_effect=lambda p:'0'*64 if Path(p)==path else actual(p)),self.assertRaises(ValueError):c.completion(A,c.AUDIT_ROOT,'audit')

    def test_baseline_closure_tampering(self):
        original=c.sha
        with patch.object(c,'sha',side_effect=lambda p:'0'*64 if Path(p).name=='070bd53d3ebf.SOURCES.json' else original(p)),self.assertRaises(ValueError):c.baseline_closure()

    def test_comparison_lifetime(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);p=root/'input';p.write_bytes(b'original')
            with c.input_scope((root,)) as held:
                with self.assertRaises(OSError):p.write_bytes(b'changed')
                with self.assertRaises(OSError):p.unlink()
                held.verify()
            self.assertEqual(p.read_bytes(),b'original')

    def test_trajectory_tampering(self):
        admission=c.read(A/'first/source/ADMISSION.json');request=c.read(A/'first/REQUEST.json');authority=c.read(A/'upstream/surface/A/fresh/SURFACE_AUTHORITY.json')
        for rel,mutate in [('first/source/TRAJECTORY.json',lambda v:v['branch'].__setitem__('geometry','0'*64)),
            ('first/source/SOURCE_SEGMENTS.checkpoint.json',lambda v:v.__setitem__('chain_sha256','0'*64))]:
            with self.subTest(rel=rel):self.changed_read(A,rel,mutate,lambda:c.trajectory_provenance(A,c.PKG,admission,request,authority))

    def test_coherent_upstream_replacement(self):
        # A coherently rehashed replacement still cannot replace captured pins.
        old=c.sha
        with patch.object(c,'sha',side_effect=lambda p:'0'*64 if Path(p)==A/'first/REQUEST.json' else old(p)),self.assertRaises(ValueError):c.authenticate_pair(A,B,'audit')

    def comparator_functions(self):
        import ast
        import numpy as np
        tree=ast.parse((c.BASE/'work/passage_verify3_compare.py').read_text(encoding='utf8'))
        module=ast.Module(body=[node for node in tree.body if isinstance(node,ast.FunctionDef)],type_ignores=[])
        ns=dict(Path=Path,json=json,np=np,hashlib=c.hashlib,mode='audit',authenticate_pair=lambda *args:{})
        exec(compile(module,'external_comparator_functions','exec'),ns)
        return ns

    def test_nested_quality_identity_preserved(self):
        ns=self.comparator_functions()
        self.assertEqual(ns['without_stamp']({'geometry_identity':'own','nested':{'geometry_identity':'must_match'}}),{'nested':{'geometry_identity':'must_match'}})

    def test_apex_star_record_tampering(self):
        ns=self.comparator_functions();original=ns['read']
        def changed(p):
            value=original(p)
            if Path(p)==A/'background/SHARED_APEX_REPAIR.json':
                value['records'][0]['unexpected_star_change']=True
            return value
        ns['read']=changed
        with self.assertRaises(AssertionError):ns['apex_state'](A,B)

    def test_native_owner_tampering(self):
        original=c.native_raw
        def changed(folder):
            value=original(folder);value['owner.i32']=value['owner.i32'].copy();value['owner.i32'][0]+=1
            return value
        with patch.object(c,'native_raw',side_effect=changed),self.assertRaises(ValueError):c.native_identity(A)

    def test_native_neighbour_tampering(self):
        original=c.native_raw
        def changed(folder):
            value=original(folder);value['neighbour.i32']=value['neighbour.i32'].copy();value['neighbour.i32'][0]+=1
            return value
        with patch.object(c,'native_raw',side_effect=changed),self.assertRaises(ValueError):c.native_identity(A)

    def test_coherent_native_ancestry_permutation(self):
        import numpy as np
        original_raw=c.native_raw;original_load=np.load
        def swapped(value):
            value=value.copy();zero=value==0;one=value==1;value[zero]=1;value[one]=0;return value
        def raw(folder):
            value=original_raw(folder);value['cell_ancestor.i32']=swapped(value['cell_ancestor.i32']);return value
        class ChangedMesh:
            def __enter__(self):
                self.z=original_load(A/'MESH.npz',allow_pickle=False)
                result={k:self.z[k] for k in self.z.files};result['source_columns']=swapped(result['source_columns']);return result
            def __exit__(self,*args):self.z.close()
        def load(p,*args,**kwargs):return ChangedMesh() if Path(p)==A/'MESH.npz' else original_load(p,*args,**kwargs)
        with patch.object(c,'native_raw',side_effect=raw),patch.object(c.np,'load',side_effect=load),self.assertRaises(ValueError):c.native_identity(A)

    def test_native_metadata_tampering(self):
        self.changed_read(A,'background/JOINED_RECONSTRUCTION.json',lambda v:v['cell_metadata'][0].__setitem__('column',1),lambda:c.native_identity(A))

    def test_native_boundary_duplicate(self):
        original=Path.read_text;target=A/'native/constant/polyMesh/boundary'
        def changed(p,*args,**kwargs):
            text=original(p,*args,**kwargs)
            return text+'\nouterFront { type patch; nFaces 59732; startFace 686918; }\n' if p==target else text
        with patch.object(Path,'read_text',changed),self.assertRaises(ValueError):c.native_identity(A)

class NativeInputTests(unittest.TestCase):
    def test_scratch_diff_identical_science(self):
        production=Path(PRE['outputs']['production'])
        self.assertEqual(c.surface_native_input(A,c.PKG),c.surface_native_input(production,c.PKG))
        ga=c.read(A/'upstream/surface/A/fresh/GENERATION.json')
        gp=c.read(production/'upstream/surface/A/fresh/GENERATION.json')
        self.assertNotEqual(ga['timing']['mmgs']['measured_input_bytes'],gp['timing']['mmgs']['measured_input_bytes'])
        for value in (ga,gp):
            normalized=c.measurements(value,c.GEN_MEASUREMENTS)
            self.assertEqual(normalized['timing']['mmgs']['measured_input_bytes'],'<measured>')

    def test_scratch_type_and_scope(self):
        for bad in (-1,True,1.5,float('nan'),'1'):
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                c.measurements({'timing':{'mmgs':{'measured_input_bytes':bad}}},('timing.mmgs.measured_input_bytes',))
        self.assertNotIn('timing.carrier_seconds',c.GEN_MEASUREMENTS)
        self.assertNotIn('projection_seconds_all_queries',c.GEN_MEASUREMENTS)

    def test_independently_pinned_files(self):
        original=c.sha
        folder=A/'upstream/surface/A/fresh'
        for name in ('metric.bin','INITIAL.npz','raw.mesh'):
            with self.subTest(name=name),patch.object(c,'sha',side_effect=lambda p:'0'*64 if Path(p)==folder/name else original(p)),self.assertRaises(ValueError):
                c.surface_native_input(A,c.PKG)
        # Coherent metric plus INITIAL substitution still fails fixed prior pins.
        with patch.object(c,'sha',side_effect=lambda p:'0'*64 if Path(p).name in ('metric.bin','INITIAL.npz') else original(p)),self.assertRaises(ValueError):
            c.surface_native_input(A,c.PKG)

    def test_metric_decoder_tampering(self):
        payload=(A/'upstream/surface/A/fresh/metric.bin').read_bytes()
        for variant in (b'badmagic'+payload[8:],payload[:-1],payload+b'x',payload[:8]+c.struct.pack('<Q',1)+payload[16:]):
            with self.subTest(length=len(variant)),self.assertRaises(ValueError):c.decode_metric(variant)
        for offset in (32+24,32+28,32+29,32+78*37566+12):
            changed=bytearray(payload);changed[offset]=2
            with self.subTest(offset=offset),self.assertRaises(ValueError):c.decode_metric(changed)

    def test_written_metric_policy(self):
        original=c.read
        def changed(p):
            value=original(p)
            if Path(p).name=='WRITTEN_METRIC_POLICY.json':value['coupled_connection']['source_identity']='0'*64
            return value
        with patch.object(c,'read',side_effect=changed),self.assertRaises(ValueError):c.surface_native_input(A,c.PKG)
        # Unknown substantive diagnostic fields remain in full policy equality.
        def extra(p):
            value=original(p)
            if Path(p)==A/'upstream/surface/A/fresh/WRITTEN_METRIC_POLICY.json':value['new_scientific_field']=1
            return value
        with patch.object(c,'read',side_effect=extra):
            self.assertNotEqual(c.surface_native_input(A,c.PKG),c.surface_native_input(Path(PRE['outputs']['production']),c.PKG))

    def test_native_receipt_and_dependency_tampering(self):
        original=c.read
        for target,key in [('mmgs.json','exit_code'),('MMGS_DEPENDENCY_QUALIFICATION.json','library_sha256'),('PASSAGE_VERIFY2_PRODUCTION_STOP_COMPONENT_REVIEW.json','surface_input_output_pins')]:
            def changed(p):
                value=original(p)
                if Path(p).name==target:value[key]='tampered'
                return value
            with self.subTest(target=target),patch.object(c,'read',side_effect=changed),self.assertRaises((ValueError,TypeError)):
                c.surface_native_input(A,c.PKG)
        with patch.object(c,'NATIVE_ANCHOR','0'*64),self.assertRaises(ValueError):c.surface_native_input(A,c.PKG)

if __name__=='__main__':unittest.main(verbosity=2)
