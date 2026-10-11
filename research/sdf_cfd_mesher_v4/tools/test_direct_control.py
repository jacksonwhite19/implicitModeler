"""Existing transaction/core/state regressions plus bounded encoding tests."""
from pathlib import Path
import json,subprocess,sys,os
if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    selected=json.loads((root/'results/persistent_publication_optimization/TEST_SELECTION.json').read_text())
    selected += [str(root/'tests'/n) for n in ('test_critical_prism.py','test_critical_index_pipeline.py','test_direct_changed_encoding.py')]
    os.environ['V4_PERSISTENT']='1'
    raise SystemExit(subprocess.call([sys.executable,str(Path(__file__).with_name('profile_direct_prism.py')),'tests',*selected,'-q','--junitxml='+str(root/'results/direct_changed_persistence/FINAL_TESTS.xml')]))
