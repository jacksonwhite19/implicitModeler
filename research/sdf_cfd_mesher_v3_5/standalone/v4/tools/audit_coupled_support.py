"""Separate immutable support defects from failure of the current local solve."""
import coupled_tangent_study as c
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import rational_vector,cross,sub
q=c.q;np=c.np

def audit(case):
    zero=[]
    for col in case.core:
        v=[rational_vector(p) for p in case.low[case.f[col]]];a=cross(sub(v[1],v[0]),sub(v[2],v[0]))
        if not any(a):zero.append(dict(column=int(col),vertices=case.f[col].tolist(),low_coordinates=case.low[case.f[col]].tolist(),exact_area=[str(x) for x in a],in_original_core=bool(col in case.p.core_columns),proof='For any upper vertices, bottom-corner Jacobian equals zero because the fixed lower triangle cross product is exactly zero.'))
    metrics=case.metrics(case.high);bad=[]
    for e in metrics['events']:
        if e['entity'] in (case.verify.side_edges if e['kind']=='side' else case.core):bad.append(e)
    return c.w.clean(dict(name=case.name,support=case.support,fixed_bottom_zero_area=zero,baseline_verification_events=bad,scope='A zero lower face rules out this enlarged-support candidate with fixed previous layer. It is not a core capacity proof or proof for every support/restart choice.',capacity_authority=False))

if __name__=='__main__':
    for spec in q.definitions(q.load_schedule().targets):
        if spec['kind'] not in ('spheres','wedge','frustum') or spec.get('cavity'):continue
        case=c.Case(spec);r=audit(case);c.save(case.name+'/support_audit.json',r);print(case.name,len(case.active),len(r['fixed_bottom_zero_area']),len(r['baseline_verification_events']),flush=True)
