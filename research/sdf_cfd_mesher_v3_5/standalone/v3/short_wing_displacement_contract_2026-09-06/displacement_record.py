from study import *
SCHEMA='V3.LayerDisplacement/1'

def quantities(positions,normals,incidents):
 delta=np.subtract(positions[1:],positions[:-1],dtype=np.float64)
 pairs=np.array([(v,s) for v,ss in enumerate(incidents) for s in ss],dtype='<i8')
 eta=np.einsum('ij,kij->ki',normals[pairs[:,1]],delta[:,pairs[:,0]])
 cumulative=np.einsum('ij,kij->ki',normals[pairs[:,1]],positions[1:,pairs[:,0]]-positions[0,pairs[:,0]])
 length=np.sqrt(np.sum(delta*delta,axis=2));eps=np.finfo(float).eps;ops=2*len(delta)+8;gamma=ops*eps/(1-ops*eps)
 error=gamma*(abs(positions[0])+np.sum(abs(delta),axis=0))
 return dict(delta=delta,incident_pairs=pairs,normal_increment=eta,source_normal_cumulative=cumulative,path_length=length,arithmetic_bound=error)

def save_record(directory,positions,faces,vertex_keys,face_keys,sheet_keys,normals,incidents,policy):
 directory=safe(directory);directory.mkdir(exist_ok=False)
 x=np.asarray(positions,dtype='<f8',order='C');f=np.asarray(faces,dtype='<i8');nn=np.asarray(normals,dtype='<f8')
 if x.ndim!=3 or x.shape[2]!=3 or len(x)<2 or not np.isfinite(x).all():raise ValueError('InvalidPositions')
 if f.ndim!=2 or f.shape[1]!=3 or f.min()<0 or f.max()>=x.shape[1]:raise ValueError('InvalidTopology')
 if len(set(vertex_keys))!=len(vertex_keys) or len(vertex_keys)!=x.shape[1]:raise ValueError('AmbiguousPersistentOrder')
 if not np.allclose(np.linalg.norm(nn,axis=1),1,rtol=0,atol=1e-13):raise ValueError('NonunitSourceNormals')
 if len(incidents)!=x.shape[1] or not policy.get('scalar_quantity'):raise ValueError('AmbiguousScalarPolicy')
 q=quantities(x,nn,incidents);arrays=dict(positions=x,faces=f,source_sheet_normals=nn,**q)
 for key,a in arrays.items():np.save(directory/(key+'.npy'),a,allow_pickle=False)
 meta=dict(schema=SCHEMA,vertex_order=vertex_keys,face_order=face_keys,sheet_order=sheet_keys,incidents=incidents,normal_support='Exact original persistent physical source sheets; never advancing triangle normals',wall_array_sha256=ah(x[0]),first_front_array_sha256=ah(x[1]),scalar_policy=policy,validation_states=['A_current_physical_prefix','B_final_isolated_shell','C_prospective_final_fluid'],arithmetic='Expected positions authoritative; delta=fl64(Xk-Xprev); native actual=fl64(actual_prev+delta), one addition per stage.',arrays={k:dict(file=k+'.npy',sha256=sha(directory/(k+'.npy')),shape=list(a.shape),dtype=a.dtype.str) for k,a in arrays.items()},native_coordinate_tolerance_m=float(np.max(np.linalg.norm(q['arithmetic_bound'],axis=1))),serialization_loss=0)
 write(directory/'record.json',meta);load_record(directory)
 return meta

def load_record(directory):
 directory=Path(directory);m=read(directory/'record.json')
 if m.get('schema')!=SCHEMA or m.get('validation_states')!=['A_current_physical_prefix','B_final_isolated_shell','C_prospective_final_fluid']:raise ValueError('UnsupportedOrAmbiguousStateContract')
 a={}
 for k,v in m['arrays'].items():
  p=directory/v['file']
  if sha(p)!=v['sha256']:raise ValueError('ArrayHashMismatch')
  a[k]=np.load(p,allow_pickle=False)
  if list(a[k].shape)!=v['shape'] or a[k].dtype.str!=v['dtype']:raise ValueError('ArrayLayoutMismatch')
 if not np.isfinite(a['positions']).all() or ah(a['positions'][0])!=m['wall_array_sha256'] or ah(a['positions'][1])!=m['first_front_array_sha256']:raise ValueError('AuthorityMismatch')
 q=quantities(a['positions'],a['source_sheet_normals'],m['incidents'])
 if any(not np.array_equal(q[k],a[k]) for k in q):raise ValueError('DerivedQuantityMismatch')
 # This is an encoding check, not the alternate-arithmetic materializer check.
 for key in ['positions','delta']:
  tokens=[format(float(v),'.17g') for v in a[key].ravel()];restored=np.array([float(v) for v in tokens]).reshape(a[key].shape)
  if not np.array_equal(restored.view(np.uint64),a[key].view(np.uint64)):raise ValueError('DecimalBridgeLoss')
 return m,a

def legacy_positions(wall,vector,increments,semantics,expected=None):
 if semantics!='metres_times_declared_dimensionless_vector':raise ValueError('AmbiguousLegacyScalar')
 if increments.ndim!=2 or increments.shape[0]!=len(wall) or vector.shape!=wall.shape:raise ValueError('LegacyShapeMismatch')
 x=[np.array(wall,dtype=float,copy=True)]
 for k in range(increments.shape[1]):x.append(x[-1]+increments[:,k,None]*vector)
 x=np.array(x)
 if expected is not None and not np.array_equal(x,expected):raise ValueError('MixedPlanCannotUseOneVector')
 return x
