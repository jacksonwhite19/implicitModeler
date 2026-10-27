from pathlib import Path
import hashlib,json,os,sys,time

source=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(source/'tools'),str(source/'src'),str(source/'tools/finite_support')]
from finite_research import direct_publication as publication,process_entry

root=Path(sys.argv[1]).resolve()
prepared_sha256=sys.argv[2]
output=Path(sys.argv[3]).resolve()
context_path=Path(sys.argv[4]).resolve()
if publication.file_sha(context_path)!=sys.argv[5]:
    raise ValueError('ChangedColdReaderContext')
context=json.loads(context_path.read_bytes())
if os.getppid()!=context['parent_pid']:
    raise ValueError('WrongColdReaderParent')
manifest_path=Path(context['manifest'])
if publication.file_sha(manifest_path)!=context['manifest_sha256']:
    raise ValueError('ChangedColdReaderManifest')
manifest=json.loads(manifest_path.read_bytes())

def check():
    if time.monotonic()>=context['parent_deadline_monotonic']:
        raise TimeoutError('ColdReaderInheritedDeadline')

def closure():
    files={str(Path(module.__file__).resolve()) for module in list(sys.modules.values())
        if getattr(module,'__file__',None) and Path(module.__file__).is_file()}
    changed=[name for name in files if manifest.get(name)!=publication.file_sha(name)]
    if changed:
        raise ValueError('UnboundColdReaderSource',changed)
    return {name:manifest[name] for name in sorted(files)}

check()
closure()
path=root/'DIRECT_PUBLICATION_PREPARED.json'
if publication.file_sha(path)!=prepared_sha256:
    raise ValueError('ChangedColdReaderJournal')
prepared=json.loads(path.read_bytes())
commit=publication.committed(root,prepared)
result=dict(passed=False,publication_state='COMMITTED' if commit else 'NOT_COMMITTED',
    synthetic_workload=sys.argv[6]=='synthetic',scientific_acceptance=False,normal_run_qualified=False)
if commit:
    if sys.argv[6]=='synthetic':
        current=prepared['current']
        if current.get('schema')!='Task162SyntheticCurrent/1' or current.get('scientific_authority') is not False:
            raise ValueError('SyntheticReaderCannotReadScientificAuthority')
        if publication.snapshot(root/'CURRENT.json')!=publication.encoded(current):
            raise ValueError('ColdSyntheticPointerMismatch')
        result['passed']=True
    elif sys.argv[6]=='scientific':
        expected_path=root/'DIRECT_PUBLICATION_EXPECTATIONS.json'
        if publication.file_sha(expected_path)!=prepared['expectations_sha256']:
            raise ValueError('ChangedColdIndependentExpectations')
        expected=json.loads(expected_path.read_bytes())
        reference=expected.pop('source_pins_reference')
        if publication.file_sha(reference['path'])!=reference['sha256']:
            raise ValueError('ChangedColdSourcePins')
        expected['source_pins']=json.loads(Path(reference['path']).read_bytes())
        expected['source_pins'][reference['path']]=reference['sha256']
        consumed=publication.readback(root,expected,check)
        if hashlib.sha256(publication.encoded(consumed)).hexdigest()!=commit['consumed_sha256']:
            raise ValueError('ColdSemanticReadbackChanged')
        result.update(passed=True,consumer=consumed)
    else:
        raise ValueError('ExplicitColdReadbackModeRequired')
result['source_closure']=closure()
check()
process_entry.write(output,result)
check()
