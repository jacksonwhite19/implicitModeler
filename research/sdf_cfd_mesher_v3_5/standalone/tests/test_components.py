import sys, unittest, tempfile, json,uuid,time,subprocess,shlex
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planner import Planner,Request,jsonable
from native import construct,readback
from types import SimpleNamespace
from build_native import linux

def closed_control(planner):
    # Complete cuboid with six face centres. A test asset, not a dispatch path.
    wall=np.array([[a,b,c] for a in (-1.,1.) for b in (-1.,1.) for c in (-1.,1.)])*1e-3
    normals=[];incidents=[[] for _ in wall];faces=[];compatible=np.sign(wall)
    for axis in range(3):
        for sign in (-1,1):
            normal=np.zeros(3);normal[axis]=sign;sheet=len(normals);normals.append(normal)
            ids=np.flatnonzero(wall[:8,axis]==sign*1e-3)
            center=np.mean(wall[ids],axis=0);other=[i for i in range(3) if i!=axis]
            ids=ids[np.argsort(np.arctan2(wall[ids,other[1]],wall[ids,other[0]]))]
            ci=len(wall);wall=np.concatenate((wall,center[None]));compatible=np.concatenate((compatible,normal[None]));incidents.append([sheet])
            for v in ids:incidents[v].append(sheet)
            for i in range(4):
                face=[ci,int(ids[i]),int(ids[(i+1)%4])]
                p=wall[face]
                if np.dot(np.cross(p[1]-p[0],p[2]-p[0]),normal)<0:face[1:]=face[:0:-1]
                faces.append(face)
    h=planner.request.first_height;vectors=compatible
    return planner.state_type(wall,wall+h*compatible,np.array(faces),[str(i) for i in range(len(wall))],np.array(normals),incidents,np.broadcast_to(vectors,(planner.request.layers-1,len(wall),3)),h,'metres_times_declared_dimensionless_vector')

class Components(unittest.TestCase):
    def test_pending_native_launch_cancellation(self):
        root=Path(__file__).resolve().parents[1]
        folder=root/'results'/('cancel_control_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
        scope=folder/'inert.linux_scope.json';launch=folder/'inert.launch.sh'
        launch.write_bytes(('printf \'{"pid":%d,"start_ticks":%s}\\n\' "$$" "$(awk \'{print $22}\' /proc/$$/stat)" > '+shlex.quote(linux(scope))+'\nsleep 30\n').encode())
        command=['wsl','-d','Ubuntu-22.04','--','setsid','--wait','bash',linux(launch)]
        with (folder/'stdout').open('w') as out,(folder/'stderr').open('w') as err:
            proc=subprocess.Popen(command,stdout=out,stderr=err)
            try:
                deadline=time.monotonic()+10
                while not scope.exists() and time.monotonic()<deadline:time.sleep(.05)
                self.assertTrue(scope.exists())
                cancelled=subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3',linux(root/'cancel_native.py'),linux(scope),linux(folder),'not_yet_executed_control'],timeout=10)
                self.assertEqual(cancelled.returncode,0);proc.wait(timeout=10)
            finally:
                if proc.poll() is None:proc.kill();proc.wait(timeout=10)
    def test_twenty_row_numerical_preservation(self):
        p=Planner(Request(.000166527069,1.163359674,20));s=closed_control(p)
        h=np.broadcast_to(p.request.nominal,(14,20))
        actual=p.base.audit_faces(s.wall,s.faces,s.vectors[0],h,accepted_state=s)
        import importlib.util
        source=Path(__file__).resolve().parents[1]/'v3/sdf_guided_advancing_front_qualification/run_naca_staged_planner.py'
        spec=importlib.util.spec_from_file_location('_v35_reference_twenty',source)
        reference=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference)
        expected=reference.audit_faces(s.wall,s.faces,s.vectors[0],h,accepted_state=s)
        self.assertEqual(actual,expected)
    def test_prepopulated_aliases_are_not_mutated(self):
        sentinel=SimpleNamespace(LAYERS=777,H1=888)
        saved={k:sys.modules.get(k) for k in ('run_weight_aware_planner','foundation_geometry')}
        try:
            for k in saved:sys.modules[k]=sentinel
            a=Planner(Request(.000166527069,1.163359674,5))
            self.assertEqual(sentinel.LAYERS,777);self.assertEqual(sentinel.H1,888)
            self.assertIsNot(a.weight,sentinel)
        finally:
            for k,v in saved.items():
                if v is None:sys.modules.pop(k,None)
                else:sys.modules[k]=v
    def test_configured_horizon_and_instance_isolation(self):
        a=Planner(Request(.000166527069,1.163359674,5));b=Planner(Request(2e-5,1.1,20))
        self.assertEqual(a.base.LAYERS,5);self.assertEqual(b.base.LAYERS,20)
        s=closed_control(a);result=a.plan(s)
        self.assertTrue(result['numerical_feasible']);self.assertEqual(result['positions'].shape,(6,14,3))
        self.assertEqual(result['mesh_accepted'],False)
        self.assertEqual(result['source_global_proof'],'pending')
    def test_first_height_and_identity_refuse(self):
        p=Planner(Request(1e-5,1.05,5));s=closed_control(p)
        h=np.broadcast_to(p.request.nominal,(14,5)).copy();h[0,0]*=2
        with self.assertRaisesRegex(ValueError,'ImmutableFirstIncrementChanged'):s.positions(h)
        with self.assertRaisesRegex(ValueError,'WallTopologyMismatch'):s.positions(np.broadcast_to(p.request.nominal,(14,5)),faces=s.faces[:,::-1])
    def test_native_roundtrip(self):
        p=Planner(Request(.000166527069,1.163359674,5));s=closed_control(p);result=p.plan(s)
        self.assertTrue(result['numerical_feasible'])
        directory=Path(__file__).resolve().parents[1]/'results'/('native_control_'+uuid.uuid4().hex[:8])
        a,report=construct(result['positions'],s.faces,directory)
        self.assertTrue(report['passed']);self.assertEqual(report['physical_prisms'],120)
        self.assertEqual(report['max_coordinate_error_m'],0)
        # A malformed native array must refuse before numerical acceptance.
        raw=Path(a['folder'])/'points.f64';original=raw.read_bytes()
        poisoned=np.frombuffer(original,dtype='<f8').copy();poisoned[0]=np.nan;poisoned.tofile(raw)
        try:
            with self.assertRaisesRegex(ValueError,'NonfiniteNativeCoordinates'):
                readback(Path(a['folder']),result['positions'],s.faces,1e-12)
        finally:raw.write_bytes(original)
        raw=Path(a['folder'])/'face_points.i32';original=raw.read_bytes()
        poisoned=np.frombuffer(original,dtype='<i4').copy();poisoned[:3]=poisoned[:3][::-1];poisoned.tofile(raw)
        try:
            with self.assertRaisesRegex(ValueError,'NativeOwnerCycleMismatch|NativeFaceIncidenceMismatch'):
                readback(Path(a['folder']),result['positions'],s.faces,1e-12)
        finally:raw.write_bytes(original)
    def test_thin_coarse_control_refuses(self):
        p=Planner(Request(1e-5,1.05,5));s=closed_control(p);result=p.plan(s,compress=False)
        self.assertFalse(result['numerical_feasible'])
        self.assertGreater(result['selection'].get('audit',{}).get('by_layer',[{'foundation_qualification_failures':1}])[0]['foundation_qualification_failures'],0)

if __name__=='__main__':unittest.main()
