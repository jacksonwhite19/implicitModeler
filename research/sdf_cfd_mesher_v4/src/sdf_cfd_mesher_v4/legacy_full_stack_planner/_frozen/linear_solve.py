"""Same LP equations; bounded deterministic recovery from presolve failure."""
from scipy.optimize import linprog

def solve_lp(cost,**kwargs):
 result=linprog(cost,**kwargs)
 result.execution_recovery=None
 if result.status==2:
  retry=dict(kwargs);retry['options']=dict(kwargs.get('options',{}),presolve=False)
  second=linprog(cost,**retry)
  second.execution_recovery=dict(initial_status=int(result.status),initial_message=result.message,only_change='presolve=False',final_status=int(second.status))
  return second
 return result
