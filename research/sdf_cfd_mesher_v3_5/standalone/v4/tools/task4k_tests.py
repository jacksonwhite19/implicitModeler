from task4k_common import *
import subprocess,re

def run():
    names=['test_task4k_cad_cache','test_task4j_certification','test_efficient_certificate','test_production_certificate_adapter','test_certificate_replay_equivalence','test_incremental_certificate']
    rows=[];logs=[];start=time.perf_counter();env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(ROOT/'tests'),str(ROOT/'src'),str(ROOT/'tools')]))
    for name in names:
        s=time.perf_counter();r=subprocess.run([sys.executable,'-B','-m','unittest','-v',name],env=env,capture_output=True,text=True)
        text=r.stdout+r.stderr;match=re.search(r'Ran (\d+) tests?',text)
        rows.append(dict(module=name,run=int(match.group(1)) if match else 0,returncode=r.returncode,seconds=time.perf_counter()-s));logs.append(text)
    (OUT/'TESTS.txt').write_bytes(('\n'.join(logs).rstrip()+'\n').encode())
    save('TESTS.json',dict(modules=rows,run=sum(r['run'] for r in rows),successful=all(r['returncode']==0 for r in rows),seconds=time.perf_counter()-start,module_process_isolation=True,initial_combined_process_fixture_error_preserved=True,full_development_suite_run=False,historical_repository_state_assertions_run=False))
    print(rows);assert all(r['returncode']==0 for r in rows)
if __name__=='__main__':run()
