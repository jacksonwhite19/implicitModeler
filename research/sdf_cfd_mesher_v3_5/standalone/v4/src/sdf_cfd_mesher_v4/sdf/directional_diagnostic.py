"""Read-only conservative directional diagnosis; never an admission predicate."""
from fractions import Fraction as Q
import hashlib
from .regional import Expression,Interval,UnresolvedRegion,exact,region,exact_fallback
from .operations import point
from .source_topology import c1_region
from . import parametric_segment as line

class DiagnosticError(ValueError):pass
class DiagnosticLimitError(DiagnosticError):pass
MAX_NODES=128
MAX_DEPTH=32
SCALAR_BYTES=16384

def rational(value):
    value=exact(value)
    if any((abs(z).bit_length()+3)//4+3>SCALAR_BYTES for z in (value.numerator,value.denominator)):
        raise DiagnosticLimitError('DiagnosticRationalSizeLimit')
    return dict(numerator_hex=hex(value.numerator),denominator_hex=hex(value.denominator))

def interval(value):return dict(lo=rational(value.lo),hi=rational(value.hi))
def vector(values):return [rational(v) for v in values]
def identity(expression):
    try:representation=repr(expression)
    except ValueError as failure:raise DiagnosticLimitError('DiagnosticExpressionRepresentationLimit') from failure
    if len(representation)>SCALAR_BYTES:raise DiagnosticLimitError('DiagnosticExpressionTextLimit')
    return hashlib.sha256(representation.encode()).hexdigest()
def sign(value):return 'strict_negative' if value.hi<0 else 'strict_positive' if value.lo>0 else 'contains_zero'
def box(a,d,lo,hi):return tuple(Interval(min(x+w*lo,x+w*hi),max(x+w*lo,x+w*hi)) for x,w in zip(a,d))

def _snapshot(expression,start_m,end_m,lo,hi,scale,bits):
    if type(expression) is not Expression or type(lo) not in (Q,int) or type(hi) not in (Q,int):raise DiagnosticError('ExactExpressionAndParametersRequired')
    lo,hi,scale=exact(lo),exact(hi),exact(scale)
    if not 0<=lo<=hi<=1 or scale<=0 or bits!=80 or type(bits) is not int:raise DiagnosticError('DiagnosticDomainOrPrecision')
    a,b=point(start_m),point(end_m);d=tuple(y-x for x,y in zip(a,b));nodes=[]
    # Bound topology BEFORE any enclosure evaluation or output construction.
    stack=[(expression,0)];count=0
    while stack:
        node,depth=stack.pop();count+=1
        if count>MAX_NODES or depth>MAX_DEPTH or len(node.parameters)>8:raise DiagnosticLimitError('DiagnosticExpressionBudget')
        stack.extend((child,depth+1) for _,child in node.children)
    def visit(node,origin,direction,path,depth):
        k,p,c=node.kind,node.parameters,dict(node.children)
        item=dict(path=path,expression_sha256=identity(node),operation=k,parameters_exact=vector(p),origin_frame_exact=vector(origin),direction_frame_exact=vector(direction),children=[dict(role=role,path=path+'/'+role,expression_sha256=identity(child)) for role,child in node.children],value_role='Node value before ancestor operators; frame incorporates ancestor coordinate transforms.',value=None,directional_derivative=None,c1_sufficient=False)
        nodes.append(item)
        bounds=box(origin,direction,lo,hi)
        try:item['c1_sufficient']=bool(c1_region(node,tuple(v/scale for v in bounds),bits))
        except (UnresolvedRegion,ValueError) as failure:item['c1_unresolved']=dict(type=type(failure).__name__,args=[str(x)[:512] for x in failure.args])
        value=derivative=None
        try:value=line._value(node,origin,direction,lo,hi,scale,bits);item['value']=interval(value)
        except UnresolvedRegion as failure:item['value_unresolved']=list(failure.args)
        try:derivative=line.bound(node,origin,direction,lo,hi,scale,bits);item['directional_derivative']=interval(derivative);item['derivative_sign']=sign(derivative)
        except UnresolvedRegion as failure:item['derivative_unresolved']=list(failure.args)
        if k=='translate':
            visit(c['child'],tuple(x-v*scale for x,v in zip(origin,p)),direction,path+'/child',depth+1)
        elif k=='scale':
            if any(v<=0 for v in p):raise DiagnosticError('PositiveSourceScaleRequired')
            item['child_to_parent_value_multiplier']=rational(min(p));visit(c['child'],tuple(x/v for x,v in zip(origin,p)),tuple(x/v for x,v in zip(direction,p)),path+'/child',depth+1)
        elif k in ('smooth_union','smooth_intersect','smooth_subtract'):
            if p[0]<=0:raise DiagnosticError('PositiveSmoothWidthRequired')
            role_a='base' if k=='smooth_subtract' else 'a';role_b='tool' if k=='smooth_subtract' else 'b'
            av,ap=visit(c[role_a],origin,direction,path+'/'+role_a,depth+1);bv,bp=visit(c[role_b],origin,direction,path+'/'+role_b,depth+1)
            if k=='smooth_subtract':
                if bv is not None:bv=-bv
                if bp is not None:bp=-bp
            blend=dict(second_operand_negated=k=='smooth_subtract',width_exact=rational(p[0]*scale))
            if av is not None and bv is not None:
                weight=line._clamp(Q(1,2)+(bv-av if k=='smooth_union' else av-bv)/(2*p[0]*scale))
                blend.update(first_value=interval(av),second_value=interval(bv),first_weight=interval(weight),second_weight=interval(1-weight))
                if ap is not None:blend['weighted_first_derivative']=interval(weight*ap)
                if bp is not None:blend['weighted_second_derivative']=interval((1-weight)*bp)
                if ap is not None and bp is not None:
                    blend['independent_contribution_sum']=interval(weight*ap+(1-weight)*bp)
                    blend['shared_weight_derivative']=interval(Interval(min(w*ap.lo+(1-w)*bp.lo for w in (weight.lo,weight.hi)),max(w*ap.hi+(1-w)*bp.hi for w in (weight.lo,weight.hi))))
            item['blend']=blend
        else:
            for role,child in node.children:visit(child,origin,direction,path+'/'+role,depth+1)
        return value,derivative
    value,progress=visit(expression,a,d,'root',0);root=nodes[0];gradient_progress=None;gradient=None;norm=None
    try:
        jet=region(expression,box(a,d,lo,hi),scale,precision_bits=bits);gradient=jet.gradient
        gradient_progress=sum((g*w for g,w in zip(gradient,d)),Interval.point(0));norm=sum((g.square() for g in gradient),Interval.point(0)).sqrt(bits)
    except UnresolvedRegion as failure:root['gradient_unresolved']=list(failure.args)
    selected=progress
    if gradient_progress is not None:
        if selected is None:selected=gradient_progress
        else:
            lower,upper=max(selected.lo,gradient_progress.lo),min(selected.hi,gradient_progress.hi)
            if lower>upper:raise DiagnosticError('DisjointConservativeDerivativeBounds')
            selected=Interval(lower,upper)
    justified=root['c1_sufficient'] and selected is not None
    classification='unresolved_differentiability_or_derivative' if not justified else sign(selected)
    report=dict(schema='V4DirectionalDiagnostic/1',diagnostic_only=True,parameter_interval_exact=[str(lo),str(hi)],point_site=lo==hi,source_expression_sha256=identity(expression),precision_bits=bits,metres_per_model_unit_exact=rational(scale),start_m_exact=vector(a),end_m_exact=vector(b),displacement_m_exact=vector(d),coordinate_interval_m_exact=[[rational(v.lo),rational(v.hi)] for v in box(a,d,lo,hi)],node_visits=len(nodes),maximum_node_visits=MAX_NODES,nodes=nodes,c1_sufficient=root['c1_sufficient'],classification=classification,selected_derivative_interval=interval(selected) if selected is not None else None,parametric_derivative_interval=interval(progress) if progress is not None else None,root_gradient_progress_interval=interval(gradient_progress) if gradient_progress is not None else None,root_gradient_intervals=[interval(g) for g in gradient] if gradient is not None else None,root_gradient_norm_interval=interval(norm) if norm is not None else None,negative_point_witness=lo==hi and justified and selected.hi<0,positive_over_reported_interval=lo<hi and justified and selected.lo>0,meaning='dF(a+t*d)/dt in fieldmetres/unit-t. No unitgradient assumption, wallclearance or acceptance certificate. Positivepoints do not certify intervalpositivity.')
    if norm is not None and norm.lo>0 and gradient_progress is not None:
        report['source_unit_normal_projection_m']=interval(gradient_progress/norm)
    return report

def protected_snapshot(scope,start_m,end_m,lo,hi):
    from .regional_owner import CanonicalRegionScope
    if type(scope) is not CanonicalRegionScope:raise DiagnosticError('CanonicalDiagnosticScopeRequired')
    scope.verify()
    try:
        expression,scale,budgets,_=scope._batch_inputs()
        if budgets!=(6,1000,80):raise DiagnosticError('OriginalProofBudgetsRequired')
        # This explicitly requested exact diagnostic is not a failed-filter retry.
        return exact_fallback(lambda:_snapshot(expression,start_m,end_m,lo,hi,scale,budgets[2]))
    finally:scope.verify()
