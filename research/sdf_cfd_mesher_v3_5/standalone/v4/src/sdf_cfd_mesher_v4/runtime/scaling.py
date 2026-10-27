"""Python integer arithmetic only; never allocates the projected matrix."""
from dataclasses import dataclass
import json
from ..common.authority import ROOT,REPO,file_hash

@dataclass(frozen=True)
class AllocationDecision:
    rows: int
    dofs: int
    bytes_per_scalar: int
    projected_bytes: int
    budget_bytes: int
    allowed: bool
    route: str

def dense_bytes(rows,dofs,bytes_per_scalar=8):
    if any(type(x)!=int or x<0 for x in (rows,dofs)) or type(bytes_per_scalar)!=int or bytes_per_scalar<=0:raise ValueError('InvalidMatrixDimensions')
    return rows*dofs*bytes_per_scalar

def guard_dense(rows,dofs,budget_bytes,bytes_per_scalar=8):
    if type(budget_bytes)!=int or budget_bytes<0:raise ValueError('InvalidBudget')
    amount=dense_bytes(rows,dofs,bytes_per_scalar)
    return AllocationDecision(rows,dofs,bytes_per_scalar,amount,budget_bytes,amount<=budget_bytes,'allowed' if amount<=budget_bytes else 'bounded_local_work_required')

def v3_scaling_case():
    rows=json.loads((ROOT/'authorities/TASK1_SOURCE_BINDING.json').read_text())['files']
    r=next(r for r in rows if r['path'].endswith('/FULL_DOMAIN_PARTITION.json'));p=REPO/r['path']
    assert file_hash(p)==r['sha256'];data=json.loads(p.read_text())
    # V3 sigma has one row per column per requested layer, not one per column.
    from ..forecast.schedule import load_schedule
    nrows=int(data['columns'])*len(load_schedule().increments);dofs=sum(data['direction_dof_counts'])
    result=guard_dense(nrows,dofs,12*1024**3)
    assert result.projected_bytes==data['dense_sigma_jacobian_bytes']
    return result
