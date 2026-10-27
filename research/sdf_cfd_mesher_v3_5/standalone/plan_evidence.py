"""Persist exact numerical inputs and proposals before any admission refusal.

These records confer no source/global proof or mesh acceptance. Cell IDs are
layer-major planner IDs, not native mesh IDs. No predicate or selector changes.
"""
from pathlib import Path
import hashlib,json
import numpy as np
from planner import jsonable


def _json(path,value):
    path.write_text(json.dumps(value,indent=2,default=jsonable,allow_nan=False)+'\n')


def _manifest(folder):
    return {str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.rglob('*')) if p.is_file() and p.name!='MANIFEST.json'}


def plan_and_capture(planner,state,out,*,face_refs=None,provenance=None,compress=True):
    """Capture even a rejected plan; raise only after its evidence is saved."""
    out=Path(out);folder=out/'plan_evidence';folder.mkdir(exist_ok=False)
    nominal=np.broadcast_to(planner.request.nominal,(len(state.wall),planner.request.layers)).copy()
    state.save(folder/'input',nominal)
    root=Path(__file__).resolve().parent
    sources=[root/name for name in ('planner.py','plan_evidence.py','mesh.py')]+sorted((root/'v3').rglob('*.py'))
    _json(folder/'PROVENANCE.json',dict(kind='contemporaneous_planner_input',
        provenance=provenance or {},implementation_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        mesh_accepted=False,source_global_proof='pending'))
    try:
        plan=planner.plan(state,compress=compress)
    except Exception as exc:
        _json(folder/'EXCEPTION.json',dict(type=type(exc).__name__,message=str(exc),proposal_available=False))
        _json(folder/'MANIFEST.json',_manifest(folder));raise
    public={k:v for k,v in plan.items() if k not in ('positions','delta','schedule')}
    if 'selection_repair' in public:
        repair=dict(public['selection_repair']);baseline=repair.pop('baseline_schedule')
        state.save(folder/'baseline_proposal',baseline)
        public['selection_repair']=repair
    _json(out/'PRISM_PLAN.json',public)
    schedule=plan.get('schedule',plan.get('selection',{}).get('proposal'))
    summary=dict(numerical_feasible=bool(plan['numerical_feasible']),mesh_accepted=False,
                 proposal_available=schedule is not None,cell_identity='zero-based layer-major planner ID; source face is input triangle ordinal')
    if schedule is not None:
        schedule=np.asarray(schedule,dtype='<f8');state.save(folder/'proposal',schedule)
        x=state.positions(schedule)
        if 'positions' in plan and x.tobytes()!=plan['positions'].tobytes():raise ValueError('CapturedProposalArithmeticMismatch')
        f=state.faces;K=len(x)-1;F=len(f)
        refs=np.full(F,-1,dtype='<i8') if face_refs is None else np.asarray(face_refs)
        if refs.shape!=(F,):raise ValueError('CaptureFaceReferenceShapeMismatch')
        rows=[];failed=[];det=planner.stack_audit.__globals__['determinant']
        for k in range(K):
            p=np.concatenate([x[k][f],x[k+1][f]],axis=1)
            vectors=planner.math._face_vectors(p)
            a=dict(cell_id=k*F+np.arange(F),source_face_id=np.arange(F),source_face_ref=refs,
                   triangle_vertex_ids=f,centres=p.mean(axis=1),face_vectors=vectors)
            for role,upper in [('B',k<K-1),('C',True)]:
                mask=np.ones(5);mask[0]=k>0;mask[1]=upper
                avg=np.sum(np.linalg.norm(vectors,axis=2)*mask,axis=1)/sum(mask)
                scaled=vectors/avg[:,None,None];tensor=np.einsum('nfi,nfj,f->nij',scaled,scaled,mask)
                value=det(p,k>0,upper)
                if value.tobytes()!=planner.math._det_symm(tensor).tobytes():raise ValueError('CapturePredicateMismatch')
                bad=value<planner.base.FOUNDATION_QUAL
                a.update({role+'_mask':mask,role+'_normalization':avg,role+'_tensor':tensor,
                          role+'_determinant':value,role+'_margin':value-planner.base.FOUNDATION_QUAL,role+'_failed':bad})
                rows.append(dict(layer=k+1,role=role,failed=int(bad.sum()),minimum=float(value.min())))
                failed.extend(dict(layer=k+1,role=role,cell_id=int(k*F+i),source_face_id=int(i),source_face_ref=int(refs[i]),determinant=float(value[i])) for i in np.flatnonzero(bad))
            np.savez_compressed(folder/('row_%02d.npz'%(k+1)),**a)
        summary.update(threshold=planner.base.FOUNDATION_QUAL,rows=rows,failed_cells=failed)
    _json(folder/'SUMMARY.json',summary)
    _json(folder/'MANIFEST.json',_manifest(folder))
    if not plan['numerical_feasible']:raise ValueError('V3CompleteStackNumericalRefusal')
    return plan
