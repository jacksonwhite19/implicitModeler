import unittest,sys,uuid
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(Path(__file__).resolve().parent))
from planner import Planner,Request
from test_components import closed_control
from geometry import owned_runtime
from export_mesh import export

class Export(unittest.TestCase):
    def test_physical_shell_conversion_and_readback(self):
        p=Planner(Request(.000166527069,1.163359674,5));s=closed_control(p);plan=p.plan(s)
        self.assertTrue(plan['numerical_feasible'])
        x=plan['positions'];n=len(s.wall);prisms=np.concatenate((np.arange(5)[:,None,None]*n+s.faces,(np.arange(5)[:,None,None]+1)*n+s.faces),axis=2).reshape(-1,6)
        folder=ROOT/'results'/('export_control_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
        archive=folder/'control.npz'
        np.savez(archive,points=x.reshape(-1,3),prisms=prisms,tetrahedra=np.empty((0,4),dtype='<i8'),wall=s.faces[:,::-1],farfield=s.faces+5*n)
        result=export(archive,folder/'export')
        self.assertTrue(result['passed']);self.assertTrue(result['checkmesh']['literal_mesh_ok'])
        self.assertTrue(result['all_face_incidence_and_winding_exact'])

if __name__=='__main__':unittest.main()
