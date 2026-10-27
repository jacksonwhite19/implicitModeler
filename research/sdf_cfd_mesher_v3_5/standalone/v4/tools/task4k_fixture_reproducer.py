"""Separate a pre-existing SymPy fixture cache interaction from V4 reuse."""
from task4k_common import *
import subprocess

def run():
    rows=[]
    for primed in (False,True):
        code="import sympy as sp\nx,t=sp.symbols('x t')\n"
        if primed:code+="sp.Poly(x**3-x-1,x).real_roots(radicals=False)\n"
        code+="alpha=sp.CRootOf(t**3-t-1,0)\nprint(alpha)\nsp.Poly(x-alpha/2,x,extension=alpha)\n"
        p=subprocess.run([sys.executable,'-B','-c',code],capture_output=True,text=True)
        rows.append(dict(primed=primed,code=code,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
    assert rows[0]['returncode']==0 and rows[1]['returncode']!=0
    save('SYMPY_FIXTURE_REPRODUCER.json',dict(rows=rows,imports_V4_optimization=False,production_change=False,mitigation='Run test modules in separate processes; preserve all test assertions.'))
if __name__=='__main__':run()
