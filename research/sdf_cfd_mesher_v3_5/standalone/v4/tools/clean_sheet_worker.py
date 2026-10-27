"""Install candidate allowlist before importing any meshing/storage modules."""
from pathlib import Path
import json,sys,runpy
from clean_sheet_isolation import ReadBoundary
V4=Path(__file__).resolve().parents[1];repo=V4.parents[1]
out=V4/'results/clean_sheet_incremental_l4_to_l5'
base=json.loads((out/'BASELINE.json').read_text())
base['allowed_inputs'].update(json.loads((out/'REQUEST_INPUT_CLOSURE.json').read_text())['allowed_inputs'])
runtime=[sys.prefix,sys.base_prefix]+[p for p in sys.path if p and not Path(p).resolve().is_relative_to(repo)]
boundary=ReadBoundary(repo,base['allowed_inputs'],out,runtime);boundary.install()
mode=sys.argv[1] if len(sys.argv)>1 else 'enroll'
if mode not in ('enroll','reopen'):raise ValueError('UnknownIsolatedPhase')
sys.argv=[str(V4/'tools/clean_sheet_l4_bootstrap.py'),mode]
try:runpy.run_path(sys.argv[0],run_name='__main__')
finally:
 record=boundary.record()
 with (out/('WORKER_READ_LOG.json' if mode=='enroll' else 'REOPEN_READ_LOG.json')).open('w',encoding='utf-8',newline='\n') as f:json.dump(record,f,sort_keys=True,indent=2)
