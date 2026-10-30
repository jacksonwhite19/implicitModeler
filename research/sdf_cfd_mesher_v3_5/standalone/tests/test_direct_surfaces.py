"""Direct surface integration controls. No live authority or global mesh proof.

Parent/manifest authority is an explicit test premise. Real parent/role hashes,
original per-component surface/degree owners and lineage checks are exercised.
"""
from pathlib import Path
import copy,sys,unittest,re,json,hashlib,math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));import geometry
import direct_surface_contract as m
from direct_connection import _construction
from sdf_cfd_mesher_v4.common.authority import array_hash

def tetra(centre,radius):
    p=np.asarray([[1,1,1],[-1,-1,1],[-1,1,-1],[1,-1,-1]],dtype='<f8')*radius+centre
    f=np.array([[0,1,2],[0,3,1],[0,2,3],[1,3,2]],dtype='<i8')
    for row in f:
        if np.dot(np.cross(p[row[1]]-p[row[0]],p[row[2]]-p[row[0]]),p[row].mean(0)-centre)<0:row[:]=row[::-1]
    return p,f

def shell(parts):
    points=[];wall=[];cap=[];prisms=[]
    for w,c,f in parts:
        n=sum(len(p) for p in points);nv=len(w);points.extend([w,c]);wall.extend((f[:,::-1]+n).tolist());cap.extend((f+n+nv).tolist())
        prisms.extend(np.c_[f+n,f+n+nv].tolist())
    pp=np.vstack(points);far,ff=tetra(np.zeros(3),20.)
    points=np.r_[pp,far];wall=np.asarray(wall,dtype='<i8');cap=np.asarray(cap,dtype='<i8');prisms=np.asarray(prisms,dtype='<i8')
    parent=dict(points=pp.copy(),wall=wall.copy(),outer=cap.copy(),cell_vertices=prisms.copy())
    return dict(parent=parent,points=points,wall=wall,cap=cap,farfield=ff+len(pp),prisms=prisms)

def fixture(centres=((-3.,0.,0.),(3.,0.,0.)),radii=None):
    if radii is None:radii=[(.25,.5)]*len(centres)
    parts=[]
    for centre,(wr,cr) in zip(centres,radii):
        w,f=tetra(np.asarray(centre),wr);c,_=tetra(np.asarray(centre),cr);parts.append((w,c,f))
    return shell(parts)

def binding(data):
    p=data['parent'];arrays=dict(points=data['points'],prisms=data['prisms'],prism_ids=np.arange(len(data['prisms']),dtype='<i8'),
        wall=data['wall'],cap=data['cap'],farfield=data['farfield'])
    b=dict(surface_contract=m.profile(),parent_prism_mesh='synthetic-authority-premise')
    parent=dict(binding=copy.deepcopy(b),prefix_points=len(p['points']),array_hashes={
        'points':array_hash(p['points']),'prisms':array_hash(p['cell_vertices']),'prism_ids':array_hash(arrays['prism_ids']),
        'wall':array_hash(p['wall']),'cap':array_hash(p['outer'])})
    return parent,arrays,b

class SurfaceIntegration(unittest.TestCase):
    def setUp(self):self.legacy=_construction();self.check=lambda:None
    def local(self,d):return m.contract(**d,connected_surface=self.legacy.surface,degree=self.legacy.degree,check=self.check)
    def adapted(self,d):
        p,a,b=binding(d)
        # Both paths build this exact interface; reader reconstructs authenticated
        # arrays from the manifest before the unchanged semantic auditor runs.
        producer=m.bind(self.legacy,p,a,self.check,b)
        reader=m.bind(self.legacy,copy.deepcopy(p),{k:v.copy() for k,v in a.items()},self.check,copy.deepcopy(b))
        result={}
        for role in ('farfield','wall','prefix_front'):  # Deliberately not audit order.
            key='cap' if role=='prefix_front' else role
            args=m.BoundSurfaces._compact(d['points'],d[key])
            left=producer.surface(*args,self.check);right=reader.surface(*args,self.check)
            self.assertEqual(left,right);self.assertFalse(left['scientific_acceptance']);result[role]=left
        return result

    def test_original_positive_single_and_two_body_agreement(self):
        for d in (fixture(),fixture(centres=((0.,0.,0.),))):
            with self.subTest(components=len(d['wall'])//4):
                result=self.adapted(d);self.assertTrue(result['wall']['local_contract_passed'])
        # The original global finite-collar predicate is still unchanged.
        d=fixture();p,f=m.BoundSurfaces._compact(d['points'],d['wall'])
        with self.assertRaisesRegex(ValueError,'DisconnectedCollarSurface'):self.legacy.surface(p,f,self.check)

    def test_overlapping_aabbs_disjoint_nonnested(self):
        # Dyadic coordinates give a strict x+y+z separating plane even though
        # all three AABB intervals overlap. This is a provably disjoint pair.
        d=fixture(centres=((-0.375,)*3,(.375,)*3),radii=((.125,.5),(.125,.5)))
        a,b=d['points'][4:8],d['points'][12:16]
        self.assertTrue(np.all(a.min(0)<b.max(0)) and np.all(b.min(0)<a.max(0)))
        self.assertLess(a.sum(1).max(),b.sum(1).min())
        result=self.adapted(d)['wall']
        self.assertEqual(result['nesting'][0]['relation'],'degree_zero_both_directions_conditional_on_global_collision')

    def test_original_negative_contract_controls(self):
        cases=[]
        def parent_faces(d):d['parent']['wall']=d['wall'].copy();d['parent']['outer']=d['cap'].copy()
        d=fixture();d['wall']=d['wall'][1:];d['cap']=d['cap'][1:];parent_faces(d);cases.append(('open',d,'OpenOrReversed'))
        d=fixture();d['wall']=np.r_[d['wall'],d['wall'][:1]];d['cap']=np.r_[d['cap'],d['cap'][:1]];parent_faces(d);cases.append(('duplicate_population',d,'ChangedInheritedFacePopulation'))
        d=fixture();d['wall']=d['wall'][:,::-1].copy();d['cap']=d['cap'][:,::-1].copy();parent_faces(d);cases.append(('inverted_roles',d,'InwardFluidWallRequired'))
        d=fixture();d['cap']=d['cap'].copy();d['cap'][0]=d['cap'][0][::-1];parent_faces(d);cases.append(('cycle',d,'WallCapCycleCorrespondence'))
        d=fixture();d['cap']=d['cap'][4:];cases.append(('missing_cap',d,'ChangedInherited_outer'))
        d=fixture();d['wall']=np.r_[d['wall'],d['farfield']];cases.append(('extra_wall',d,'ChangedInherited_wall'))
        d=fixture();d['points'][0,0]+=1;cases.append(('changed_point',d,'ChangedInheritedPoints'))
        d=fixture(centres=((0.,0.,0.),(0.,0.,0.)),radii=((2.,3.),(.1,.2)));cases.append(('nested',d,'NestedCapComponentsUnsupported'))
        d=fixture();d['points'][4:8],d['points'][12:16]=d['points'][12:16].copy(),d['points'][4:8].copy();d['parent']['points']=d['points'][:16].copy();cases.append(('wrong_body',d,'WallOutsideCorrespondingCap'))
        d=fixture();d['points'][-4:]+=100;cases.append(('outside',d,'ComponentOutsideFarfield'))
        d=fixture();p,f=tetra(np.array([100.,0.,0.]),1.);n=len(d['points']);d['points']=np.r_[d['points'],p];d['farfield']=np.r_[d['farfield'],f+n];cases.append(('disconnected_farfield',d,'ConnectedFarfieldRequired'))
        d=fixture();d['farfield']=d['farfield'][:,::-1].copy();cases.append(('inverted_farfield',d,'OutwardCapFarfieldRequired'))
        for name,d,expected in cases:
            with self.subTest(name=name),self.assertRaisesRegex(ValueError,expected):self.local(d)
        p1,f1=tetra(np.zeros(3),1.);p2,f2=tetra(np.array([2.,2.,0.]),1.)
        pp=np.r_[p1,p2[1:]];ff=np.r_[f1,np.array([0,4,5,6])[f2]]
        for name,p,f,expected in [('pinched',pp,ff,'InvalidCollarVertexLinks'),
            ('duplicate_local',p1,np.r_[f1,f1[:1]],'DuplicateAttachmentCollarFace'),
            ('reversed_face',p1,np.r_[f1[:1,::-1],f1[1:]],'OpenOrReversed'),
            ('three_edge_uses',np.r_[p1,[[0.,0.,10.]]],np.r_[f1,[[0,1,4]]],'OpenOrReversed')]:
            with self.subTest(name=name),self.assertRaisesRegex(ValueError,expected):m.partition(p,f.astype('<i8'),self.legacy.surface,self.check)

    def test_stale_parent_role_and_authority_refuse(self):
        d=fixture();p,a,b=binding(d)
        for key in ('points','wall','cap','prisms','prism_ids'):
            bad=copy.deepcopy(p);bad['array_hashes'][key]='0'*64
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'ChangedDirectSurfaceParent_'):m.bind(self.legacy,bad,a,self.check,b)
        bad=copy.deepcopy(b);bad['surface_contract']['identity']='0'*64
        with self.assertRaisesRegex(ValueError,'ParentBinding'):m.bind(self.legacy,p,a,self.check,bad)
        adapter=m.bind(self.legacy,p,a,self.check,b);args=m.BoundSurfaces._compact(d['points'],d['wall'])
        with self.assertRaisesRegex(ValueError,'UnknownOrAmbiguous'):adapter.surface(args[0]+1,args[1],self.check)
        a['farfield'][0]=a['farfield'][0][::-1]
        with self.assertRaisesRegex(ValueError,'ChangedDirectSurfaceRoleArrays'):adapter.surface(*args,self.check)
        d=fixture();p,a,b=binding(d);adapter=m.bind(self.legacy,p,a,self.check,b);p['binding']['parent_prism_mesh']='stale'
        with self.assertRaisesRegex(ValueError,'ParentBinding'):adapter.surface(*args,self.check)
        def revoked():raise ValueError('SyntheticAuthorityRevoked')
        with self.assertRaisesRegex(ValueError,'AuthorityRevoked'):m.bind(self.legacy,p,a,revoked,b)

    def test_existing_torus_wall_genus_one(self):
        # Read only the qualified compact torus wall. Its original successful
        # scratch was cleaned; the one-layer cap here is a synthetic normal
        # offset of those exact wall coordinates, not a full-run replay.
        solver=ROOT/'results/perf37_generic/02_torus/solver'
        if not solver.exists():self.skipTest('Retained PERF37 torus delivery unavailable')
        inventory=json.loads((solver/'COMPACT.json').read_text())
        from compact_delivery import verify
        verify(solver)
        text=(solver/'polyMesh/points').read_text();start=re.search(r'\n\s*(\d+)\s*\n\(',text)
        count=int(start[1]);body=text[start.end():];lines=body.splitlines()[1:count+1]
        points=np.array([[float(x) for x in line.strip(' ()').split()] for line in lines],dtype='<f8')
        boundary=(solver/'polyMesh/boundary').read_text();wall=re.search(r'\bwall\s*\{(.*?)\}',boundary,re.S)[1]
        nf=int(re.search(r'nFaces\s+(\d+)',wall)[1]);first=int(re.search(r'startFace\s+(\d+)',wall)[1])
        text=(solver/'polyMesh/faces').read_text();start=re.search(r'\n\s*(\d+)\s*\n\(',text);lines=text[start.end():].splitlines()[1:]
        faces=np.array([[int(x) for x in re.fullmatch(r'3\((.*?)\)',line.strip())[1].split()] for line in lines[first:first+nf]],dtype='<i8')
        ids,inv=np.unique(faces,return_inverse=True);w=points[ids];f=inv.reshape(-1,3)[:,::-1].copy()
        centres=w.copy();centres[:,2]=0.;centres[:,:2]*=(.0015/np.linalg.norm(w[:,:2],axis=1))[:,None]
        normal=(w-centres)/np.linalg.norm(w-centres,axis=1)[:,None];cap=w+normal*.00002
        d=shell([(w,cap,f)]);result=self.adapted(d)['wall'];row=result['components']['wall'][0]
        self.assertEqual(row['euler'],0);self.assertEqual(row['faces'],17326);self.assertEqual(len(result['components']['wall']),1)

    def test_cached_result_does_not_outlive_role_or_authority(self):
        for failure in ('role','authority','parent'):
            with self.subTest(failure=failure):
                d=fixture();p,a,b=binding(d);active=[True]
                def check():
                    if not active[0]:raise ValueError('SyntheticAuthorityRevoked')
                adapter=m.bind(self.legacy,p,a,check,b)
                adapter.surface(*m.BoundSurfaces._compact(d['points'],d['wall']),self.check)
                self.assertIsNotNone(adapter._result)
                args=m.BoundSurfaces._compact(d['points'],d['farfield'])
                if failure=='role':a['farfield'][0]=a['farfield'][0][::-1];expected='ChangedDirectSurfaceRoleArrays'
                elif failure=='authority':active[0]=False;expected='AuthorityRevoked'
                else:p['binding']['parent_prism_mesh']='stale';expected='ParentBinding'
                with self.assertRaisesRegex(ValueError,expected):adapter.surface(*args,self.check)

if __name__=='__main__':unittest.main()
