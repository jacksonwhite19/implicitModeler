"""Resume the existing L6 publication boundary with its authenticated replay.

Use the shared preparation control and original optimized recursive publisher.
No generation, scientific-proof recomputation or advance to L7.
"""
import argparse
import sys
from pathlib import Path
import benchmark_actual_prepare as shared

OUT=shared.V4/'results/retained_l6_workflow'


def configure():
    shared.OUT=OUT;shared.RUN=OUT/'actual_prepare'


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['request','worker','monitor'])
    a=p.parse_args();configure()
    if a.mode=='request':shared.request(publish=True)
    elif a.mode=='worker':shared.worker(publish=True)
    else:sys.exit(shared.monitor(command=[sys.executable,'-B',__file__,'worker']))
