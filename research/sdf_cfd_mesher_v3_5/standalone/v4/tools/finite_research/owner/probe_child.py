from pathlib import Path
import json,subprocess,sys

source=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(source/'tools'),str(source/'src'),str(source/'tools/finite_support')]
import native_scope

binding=native_scope.require_transport()
for stage in ('first','second'):
    result=subprocess.run(['wsl.exe','-e','/usr/bin/python3.10','-B','-c','pass'],check=True,timeout=10)
    if result.returncode!=0:
        raise ValueError('HarmlessNativeProbeFailed',stage)
print(json.dumps(dict(passed=True,synthetic_workload=True,scientific_acceptance=False,binding=binding)),flush=True)
