"""Independent exact tensor check before the existing stock MMGS invocation."""
from smooth_authority import *
import struct,os
def verify(path):
 a=SmoothAuthority();provider=load_module('unscaled_metric_verifier',V3/'adaptive_surface_generic_qualification/generic_v3mmg1_provider.py')
 vp=struct.Struct('<ddd i BB dddddd');fp=struct.Struct('<iiii');ep=struct.Struct('<iiiBB')
 def unpack(p):
  data=p.read_bytes();assert data[:8]==b'V3MMG1\x00\x00';nv,nf,ne=struct.unpack_from('<QQQ',data,8);off=32;vertices=[];metrics=[];faces=[];edges=[]
  for i in range(nv):
   r=vp.unpack_from(data,off);off+=vp.size;corner=r[3] if r[4] else 0
   vertices.append(SeedVertex(np.array(r[:3]),a.skin if corner else r[3],0,corner));m=r[6:];metrics.append([[m[0],m[1],m[2]],[m[1],m[3],m[4]],[m[2],m[4],m[5]]])
  for i in range(nf):
   r=fp.unpack_from(data,off);off+=fp.size;faces.append(SeedFace(tuple(v-1 for v in r[:3]),r[3]))
  for i in range(ne):
   r=ep.unpack_from(data,off);off+=ep.size;edges.append(SeedEdge(tuple(v-1 for v in r[:2]),r[2],bool(r[3])))
  assert off==len(data);return vertices,faces,edges,np.array(metrics)
 vertices,faces,edges,actual=unpack(path);basepath=path.with_name('metric_unscaled_verification.bin')
 provider.write_contract(basepath,vertices,faces,edges,a,1.,provider.PrismMetricPolicy(H1,.00121,.00019373087622166372,.7250391657507819),shape_policy=provider.ShapeMetricPolicy())
 scale=float(os.environ['SDF_LENGTH_SCALE']);assert scale in [1.,.8,.64]
 base=unpack(basepath)[3];expected=base/(scale*scale);agree=np.array_equal(actual,expected)
 record=dict(pass_gate=agree,scale=scale,written_before_MMGS=True,carrier_vertices=len(vertices),input_faces=len(faces),input_edges=len(edges),actual_contract_sha256=sha(path),unscaled_contract_sha256=sha(basepath),maximum_absolute_difference=float(np.max(abs(actual-expected))),exact_binary64_tensor_agreement=agree,proof='Reconstruct unchanged fully composed M at every actual carrier point and incident feature; compare written M_scaled=M/scale**2 exactly',canonical_source_changed=False,historical_mesh_comparison=False)
 write('WRITTEN_METRIC_VERIFICATION.json',record);assert agree,record
 seed=np.load(path.parent/'seed/SEED.npz')
 inputs=list((path.parent/'seed').glob('*.input'))
 ref=CODE/'SB/A/wing/surface/A/seed'
 same=ref.resolve()==(path.parent/'seed').resolve()
 comparison=None
 if not same:
  reference=np.load(ref/'SEED.npz')
  comparison=seed.files==reference.files and all(np.array_equal(seed[k],reference[k]) for k in seed.files)
  assert comparison,'Fresh common geometry seed scientific difference'
  assert all(p.read_bytes()==(ref/p.name).read_bytes() for p in inputs),'Fresh common feature-loop input difference'
 write('SEED_TOPOLOGY_VERIFICATION.json',dict(pass_gate=True,common_geometry_sha256=sha(HERE/'WING_AUTHORITY_A.json'),seed_arrays={k:hashlib.sha256(np.ascontiguousarray(seed[k]).tobytes()).hexdigest() for k in seed.files},feature_loop_inputs={p.name:sha(p) for p in inputs},feature_loop_count=len(inputs),fresh_seed_reference=bool(same),fresh_seed_reference_agreement=comparison,historical_seed_used=False))
 print('Actual written metric verified before MMGS',scale,len(vertices),flush=True)
