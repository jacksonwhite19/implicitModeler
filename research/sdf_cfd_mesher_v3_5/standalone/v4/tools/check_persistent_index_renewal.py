"""Reuse the bounded preparation control with new index-renewal evidence."""
import argparse
from pathlib import Path
import sys
V4=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
import check_recovered_authorization as prior
from benchmark_retained_l6 import read
from path_preserving_deduplication import durable_json,file_hash

OUT=V4/'results/persistent_index_renewal'
OWN=['research/sdf_cfd_mesher_v4/'+p for p in (
    'CURRENT_TASK.md','README.md','PERSISTENT_INDEX_RENEWAL_REPORT.md',
    'tools/persistent_index_renewal.py','tools/check_persistent_index_renewal.py',
    'tools/benchmark_actual_prepare.py','tests/test_persistent_index_renewal.py',
    'results/persistent_index_renewal/')]


def configure():
    prior.OUT=OUT;prior.OWN=OWN


def baseline():
    configure();prior.baseline()
    b=read(OUT/'BASELINE.json')
    b['authorization']='User: Keep going. Figure it out. Allow authorization. Renew existing authenticated persistent-index access and resume retained preparation. Original scientific authorization remains unchanged; no new layer.'
    b['scope']='Capsule refresh_records and existing readers; new metadata-only manifests with shared original payloads; actual retained preparation after independent review.'
    for n in ('persistent_numeric_incidence.py','prism_source_lookup.py','incremental_pair_storage.py'):
        p=str(V4/'tools'/n);b['protected'][p]=file_hash(p)
    durable_json(OUT/'BASELINE.json',b)


def access_check():
    configure();prior.access_check()


def finish():
    configure();prior.finish()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','access_check','finish'])
    globals()[p.parse_args().mode]()
