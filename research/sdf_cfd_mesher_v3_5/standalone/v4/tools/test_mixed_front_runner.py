"""File-based Windows spawn entry point; preserve sealed derived evidence."""
from pathlib import Path
import generalize_local_remesh as g
import pytest

def main():
    original=g.save
    def save(path,value):
        path=Path(path)
        try:path=g.s.ROOT/'results/purpose_built_mixed_front_runner/test_derived'/path.relative_to(g.OUT)
        except ValueError:pass
        return original(path,value)
    g.save=save
    tests=['test_mixed_front.py','test_remesh_transaction.py','test_termination_policy.py',
           'test_remesh_generalization.py','test_local_cavity_remesh.py','test_rolling.py']
    return pytest.main([str(g.s.ROOT/'tests'/p) for p in tests]+['-q'])

if __name__=='__main__':raise SystemExit(main())
