"""Retain regression assertions while redirecting their derived output writes."""
import sys
from pathlib import Path
import generalize_local_remesh as g
import pytest
OUT=g.s.ROOT/'results/production_automatic_termination_remesh'
original=g.save


def private_save(path,value):
    path=Path(path)
    try:relative=path.relative_to(g.OUT)
    except ValueError:return original(path,value)
    return original(OUT/'test_derived_results'/relative,value)


g.save=private_save
if __name__=='__main__':
    tests=['test_remesh_transaction.py','test_termination_policy.py','test_remesh_generalization.py',
           'test_local_cavity_remesh.py','test_rolling.py']
    raise SystemExit(pytest.main([str(g.s.ROOT/'tests'/p) for p in tests]+['-q']))
