"""Bounded production replay on unchanged production-scale authorities."""
import sys
sys.dont_write_bytecode=True
import argparse
import run_production_scale as production
from first_layer_determinant_trace import OUT

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--label',default='A')
    a=p.parse_args();production.OUT=OUT
    production.run(a.case,a.label,steps=1)
