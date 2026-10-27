"""Reversible audit timers. No mathematical predicate or result is replaced."""
import time,collections,hashlib,functools
import sympy as sp
from sdf_cfd_mesher_v4.certificate.incremental import kernel

class Instruments:
    def __init__(self,extra=False):self.extra=extra;self.restore=[];self.seen=collections.defaultdict(set);self.reset()
    def reset(self):
        self.rows=collections.defaultdict(lambda:dict(calls=0,total=0.,self=0.,repeated_input=0));self.stack=[];self.comparisons=hashlib.sha256();self.roots=hashlib.sha256();self.text={}
    def text_of(self,x):
        try:
            if x not in self.text:self.text[x]=str(x)
            return self.text[x]
        except TypeError:return str(x)
    def wrap(self,label,fn,trace=None,key=False):
        @functools.wraps(fn)
        def call(*args,**kwargs):
            row=self.rows[label];row['calls']+=1
            if key:
                try:
                    k=(args,tuple(sorted(kwargs.items())))
                    if k in self.seen[label]:row['repeated_input']+=1
                    else:self.seen[label].add(k)
                except TypeError:pass
            start=time.perf_counter();frame=[0.];self.stack.append(frame)
            try:result=fn(*args,**kwargs)
            finally:
                dt=time.perf_counter()-start;self.stack.pop();row['total']+=dt;row['self']+=dt-frame[0]
                if self.stack:self.stack[-1][0]+=dt
            if trace=='compare':self.comparisons.update((self.text_of(args[0])+'|'+self.text_of(args[1])+'|'+str(result)+'\n').encode())
            if trace=='root':self.roots.update(repr(result).encode())
            return result
        return call
    def patch(self,obj,name,label,trace=None,key=False,descriptor=None):
        original=getattr(obj,name);raw=vars(obj).get(name,original);self.restore.append((obj,name,raw))
        wrapped=self.wrap(label,original,trace,key)
        setattr(obj,name,staticmethod(wrapped) if descriptor=='static' else wrapped)
    def __enter__(self):
        m=kernel.module('exact_algebraic');n=kernel.module('noncoincident_rank3_certificate')
        for name,label in [('sqf_part','square_free'),('factor_list','factorization_poly'),('real_roots','root_isolation'),('intervals','root_intervals'),('clear_denoms','clear_denominators'),('primitive','primitive_coefficients'),('lift','algebraic_lift')]:self.patch(sp.Poly,name,label,key=True)
        if self.extra:
            for name in ('gcd','sturm'):self.patch(sp.Poly,name,name,key=True)
        self.patch(sp.Poly,'__new__','polynomial_construction',descriptor='static')
        self.patch(sp,'factor_list','factorization_expression',key=True)
        for name,label in [('compare','exact_compare'),('polynomial_zero_at','algebraic_zero'),('specialize','specialization'),('exact_field','algebraic_field'),('root_record','root_record'),('primitive','canonicalization')]+([('field_sign','field_sign'),('eval_field','eval_field'),('sturm_variations','sturm_variations')] if self.extra else []):
            original=getattr(m,name);wrapped=self.wrap(label,original,'compare' if name=='compare' else 'root' if name=='root_record' else None)
            for module in (m,n):
                if getattr(module,name,None) is original:self.restore.append((module,name,original));setattr(module,name,wrapped)
        return self
    def __exit__(self,*args):
        for obj,name,original in reversed(self.restore):setattr(obj,name,original)
    def result(self):return dict(functions=dict(self.rows),comparison_trace=self.comparisons.hexdigest(),root_trace=self.roots.hexdigest())
