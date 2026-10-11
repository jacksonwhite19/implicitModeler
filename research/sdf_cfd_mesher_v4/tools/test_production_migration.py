"""Run retained tests with derived writes isolated from sealed results."""
from pathlib import Path
import pytest
import generalize_local_remesh as g
from audit_production_migration import OUT

def run():
    original=g.save
    def save(path,value):
        path=Path(path)
        try:path=OUT/'test_derived'/path.relative_to(g.OUT)
        except ValueError:pass
        return original(path,value)
    g.save=save
    tests=['test_production_migration.py','test_mixed_front.py','test_remesh_transaction.py','test_termination_policy.py',
        'test_remesh_generalization.py','test_local_cavity_remesh.py','test_rolling.py',
        'test_complete_capacity_fixtures.py','test_capacity_witness.py','test_capacity_valid_prefix.py',
        'test_capacity_forecast.py','test_branch_aware_capacity.py']
    return pytest.main([str(g.s.ROOT/'tests'/name) for name in tests]+['-q'])

if __name__=='__main__':raise SystemExit(run())
