"""Read-only verification of both final native checkpoints and immutable inputs."""
import json,time
import numpy as np
from production_layer2_run import OUT,L1,REQUEST,SURFACE,ADMISSION,save
from production_surface_authority import load_production_wall
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.common.authority import array_hash,file_hash,identity

def run():
 start=time.perf_counter();wall,tri,surface=load_production_wall(SURFACE);n=len(wall)
 admission=json.loads((ADMISSION/'AUTHORITY.json').read_text());b=admission['binding'];rows=[]
 for label in ('A','B'):
  print('restore final',label,flush=True);s=NativeCheckpointStore(OUT/label/'checkpoints').restart();r=json.loads((OUT/label/'READY.json').read_text())
  assert scientific_hash(s)==r['committed']
  coordinates=[np.array([s['vertices'][f'layer:{k}:source:{i}'] for i in range(n)]) for k in range(3)]
  assert array_hash(coordinates[0])==surface['coordinate_hash'] and array_hash(coordinates[1])==b['coordinates'] and array_hash(coordinates[2])==r['coordinates']
  assert {c['layer'] for c in s['cells'].values()}=={1,2} and {c['last_prism_layer'] for c in s['columns'].values()}=={2}
  assert all(int(k.split(':')[1])<=2 for k in s['vertices'])
  assert len(s['cells'])==67572 and len(s['prism_front'])==33786
  assert len(s['certificate'])==4839083
  assert not any(v.get('receipt_class')=='ProspectiveFirstPrismDeterminant' for v in s['quality'].values())
  incidence=[v for v in s['quality'].values() if v.get('receipt_class')=='FinalIncidenceDeterminant']
  assert len(incidence)==67572 and all(v['value']['determinant']>=.0011 for v in incidence)
  rows.append(dict(committed=scientific_hash(s),coordinates=[array_hash(v) for v in coordinates],topology=array_hash(tri),cells=len(s['cells']),faces=len(s['faces']),vertices=len(s['vertices']),columns=len(s['columns']),certificate_rows=len(s['certificate']),final_incidence_receipts=len(incidence),scoped_first_layer_receipts=0,wall_mutations=0,L1_mutations=0,later_vertices=0))
  del s
 assert rows[0]==rows[1]
 assert file_hash(L1/'A/checkpoints/CURRENT.json')=='c73fecf72406beb57875e9430210dbb0bb4425e1b9822d84d3c785b2a07ea110'
 assert file_hash(REQUEST)==b['request_file']
 assert file_hash(ADMISSION/'AUTHORITY.json')=='9ab7734b79ed2fb2480c92e91eb6fd35336d992d29084684d7a6abe4f78e0ccd'
 result=dict(passed=True,scientific=rows[0],identity=identity(rows[0]),seconds=time.perf_counter()-start)
 save(OUT/'RESTORE_VERIFICATION.json',result);print(json.dumps(result),flush=True)

if __name__=='__main__':run()
