"""Bounded retained-input writer/reader control; never runs a front stage."""
import argparse
from collections import defaultdict
from contextlib import ExitStack
import hashlib
import json
import mmap
from pathlib import Path
import re
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
import current_front_dependencies as codec
import immutable_state_access as access
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from record_receipt_resume import disk, status


def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))


def allocation(paths):
    rows = [metadata(p) for p in paths]
    unique = {physical_key(r):r for r in rows}
    return dict(files=len(rows), logical_bytes=sum(r['bytes'] for r in rows),
                unique_allocation_bytes=sum(r['cluster_bytes'] for r in unique.values()),
                path_allocation_bytes=sum(r['cluster_bytes'] for r in rows),
                duplicate_path_allocation=sum(r['cluster_bytes'] for r in rows)-sum(r['cluster_bytes'] for r in unique.values()),
                externally_shared_allocation=sum(r['cluster_bytes'] for r in unique.values() if r['links']>1), records=rows)


def baseline(out):
    out.mkdir(parents=True,exist_ok=True)
    if (out/'BASELINE.json').exists(): raise ValueError('PreserveExistingBaseline')
    paths = [V4/'results/retained_l5_finish'/n for n in ('LAST_QUALIFIED.json','EVIDENCE.json')]
    paths += [V4/'results/wing_continuation_layers2to5/AUTHORIZATION.json']
    paths += [ROOT/'src/sdf/aerospace'/n for n in ('wing.rs','airfoil.rs','naca_spline.rs')]
    paths += [V4/'tools'/n for n in ('composable_state.py','immutable_state_access.py','critical_transaction.py',
              'incremental_transaction.py','accepted_trajectory_contract.py','accepted_production_contract.py',
              'recursive_replay_retention.py','prism_receipt_witness.py','production_forecast_admission.py',
              'prism_persistent_hotpath.py','critical_index_pipeline.py')]
    paths += [V4/'src/sdf_cfd_mesher_v4/advancing_front/mixed_front.py']
    protected = [dict(path=str(p.relative_to(ROOT)),sha256=file_hash(p),bytes=p.stat().st_size) for p in paths]
    # Census only the retained wing campaign. No proof payloads are decoded or
    # hashed by this allocation walk. Active references are listed separately.
    campaign = V4/'results/wing_receipt_resume/A'
    files = [p for p in campaign.rglob('*') if p.is_file()]
    census = allocation(files); groups = defaultdict(list)
    from audit_wing_campaign_storage import stage
    for r in census['records']:
        p=Path(r['path']); rel=p.relative_to(campaign); layer=rel.parts[0]
        realization='replay' if 'replay' in rel.parts else 'primary_or_shared'
        groups[(layer,realization,stage(p,campaign))].append(r)
    grouped=[]
    for (layer,realization,phase),rows in sorted(groups.items()):
        unique={physical_key(r):r for r in rows}
        grouped.append(dict(layer=layer,realization=realization,stage=phase,files=len(rows),
            logical_bytes=sum(r['bytes'] for r in rows),unique_allocation_bytes=sum(r['cluster_bytes'] for r in unique.values())))
    dependencies=[]
    for layer in range(2,6):
        folder=campaign/f'L{layer}'; plan=read(folder/'CANDIDATE_PLAN.json')
        p=folder/codec.NAME; record=plan['references'][str(p.resolve())]
        if file_hash(p)!=record['sha256'] or access.token(p)!=record['token']:
            raise ValueError('RetainedDependencyAuthenticationMismatch',layer)
        dependencies.append(dict(layer=layer,path=str(p),record=record,
            semantic=plan['components']['forecast']['replace'],metadata=metadata(p),
            plan_sha256=file_hash(folder/'CANDIDATE_PLAN.json')))
    nodes=[]; reference=read(V4/'results/retained_l5_finish/LAST_QUALIFIED.json')['reference']
    while reference['kind']=='recursive':
        folder=Path(reference['directory']); pointer=read(folder/'CURRENT.json'); root=read(folder/pointer['file'])
        from sdf_cfd_mesher_v4.common.authority import identity
        if identity(root)!=reference['root']: raise ValueError('WrongRetainedRecursiveRoot')
        nodes.append(dict(reference=reference,root_file_sha256=file_hash(folder/pointer['file'])))
        reference=root['plan']['parent']
    nodes.append(dict(reference=reference))
    result=dict(starting_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        status=status(),protected=protected,dependencies=dependencies,campaign=census,groups=grouped,
        active_ancestry=nodes,last_qualified=read(V4/'results/retained_l5_finish/LAST_QUALIFIED.json'),disk=disk())
    durable_json(out/'BASELINE.json',result)
    print(json.dumps(dict(campaign={k:v for k,v in census.items() if k!='records'}, groups=len(grouped),protected=len(protected),disk=result['disk'])))


def retained_rows(path):
    """Read the existing canonical top-level row framing with bounded memory.

    This diagnostic reader is not a checkpoint reader. The recomputed expanded
    hash must equal every original byte; omitted/extra/framing data fail closed.
    """
    decoder=json.JSONDecoder()
    with Path(path).open('rb') as stream, mmap.mmap(stream.fileno(),0,access=mmap.ACCESS_READ) as data:
        matches=re.finditer(rb'(?m)^  ("(?:\\.|[^"\\])*"): \{',data)
        current=next(matches,None)
        while current is not None:
            following=next(matches,None); end=following.start() if following else len(data)-2
            text=data[current.end()-1:end].decode('utf-8').strip()
            row,finish=decoder.raw_decode(text)
            if text[finish:].strip() not in ('',','): raise ValueError('UnsupportedLegacyRowFraming')
            yield json.loads(current.group(1)),row
            current=following


class DiagnosticOwner:
    """Same guarded verification method as the production committed reader."""
    def __init__(self,path,semantic):
        from collections import Counter
        self.closed=False; self.verified=set(); self.stats=Counter(); self.stack=ExitStack()
        self.out=path.parent; self.kind='recursive';self.production=True
        refs={str(path.resolve()):access.attest(path,file_hash(path))}
        refs.update(codec.reference_records(path))
        for p in refs:self.stack.enter_context(access.guard(p))
        self.body=dict(plan=dict(references=refs),produced={})
        self.components=dict(forecast=dict(replace=semantic))
    def check_live(self):
        if self.closed:raise ValueError('ClosedCommittedState')
    def verify(self,path):
        from composable_state import CommittedState
        return CommittedState.verify(self,path)
    def close(self):self.closed=True;self.stack.close()


def verify(out):
    start=time.perf_counter(); saved=read(out/'BASELINE.json')
    from composable_state import CommittedState
    from sdf_cfd_mesher_v4.common.authority import identity
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
    codec.install_reader(); results=[]
    for record in saved['dependencies']:
        layer=record['layer'];source=Path(record['path']);folder=out/f'L{layer}';folder.mkdir()
        path=folder/codec.NAME; tick=time.perf_counter()
        with access.guard(source):
            if access.token(source)!=record['record']['token'] or file_hash(source)!=record['record']['sha256']:
                raise ValueError('ChangedRetainedDependency')
            written=codec.write(path,retained_rows(source),shared_directory=out/'shared_dependencies')
            if written['expanded_sha256']!=record['semantic'] or written['expanded_sha256']!=record['record']['sha256']:
                raise ValueError('RetainedExpandedHashMismatch')
            writer_seconds=time.perf_counter()-tick;owner=DiagnosticOwner(path,record['semantic']);tick=time.perf_counter()
            try:
                # The actual installed CommittedState forecast branch calls the
                # same authenticated reader; no scientific state is fabricated.
                view=CommittedState.__getitem__(owner,'forecast')
                compact_hash=scientific_digest(view); expected=hashlib.sha256();expected.update(b'{');count=0
                # Legacy rows are already in sorted canonical order. Compare
                # every row and separately hash native scientific serialization.
                previous=None
                for key,row in retained_rows(source):
                    if previous is not None and key<=previous:raise ValueError('UnsortedRetainedRows')
                    previous=key
                    if view[key]!=row:raise ValueError('RetainedRowMismatch',key)
                    if count:expected.update(b',')
                    expected.update(json.dumps(key).encode());expected.update(b':')
                    expected.update(json.dumps(row,sort_keys=True,separators=(',',':'),allow_nan=False).encode())
                    count+=1
                expected.update(b'}')
                if count!=len(view) or compact_hash!=expected.hexdigest():raise ValueError('ScientificDependencyMismatch')
                reader_seconds=time.perf_counter()-tick
            finally:owner.close()
            if access.token(source)!=record['record']['token'] or file_hash(source)!=record['record']['sha256']:
                raise ValueError('RetainedDependencyChangedAfterVerification')
        metrics=allocation([path,Path(read(path)['shared']['path'])])
        old=record['metadata']
        results.append(dict(layer=layer,source=str(source),original_sha256=record['record']['sha256'],
            encoded_sha256=file_hash(path),native_scientific_digest=compact_hash,compared_rows=count,
            exact_row_equality=True,exact_expanded_hash=True,exact_native_scientific_digest=True,
            writer=written,writer_seconds=writer_seconds,reader_seconds=reader_seconds,
            allocation=metrics,avoided_logical_bytes=old['bytes']-metrics['logical_bytes'],
            avoided_allocation_bytes=old['cluster_bytes']-metrics['unique_allocation_bytes']))
        durable_json(out/f'L{layer}_RESULT.json',results[-1])
        print(json.dumps({k:v for k,v in results[-1].items() if k not in ('allocation','writer')}),flush=True)
    # Guarded reopen of existing sealed L5; no forecast aggregate/proof traversal.
    from sealed_front_connection import qualified_dispatch
    with qualified_dispatch():
        with CommittedState(saved['last_qualified']['reference']) as state:
            state.check()
            if state.scientific!=saved['last_qualified']['scientific'] or state.layer!=5:raise ValueError('ChangedSealedL5')
            reopen=dict(layer=state.layer,scientific=state.scientific,root=state.root,passed=True)
    for r in saved['protected']:
        if file_hash(ROOT/r['path'])!=r['sha256']:raise ValueError('ProtectedFileChanged',r['path'])
    import psutil
    process=psutil.Process().memory_info()
    result=dict(classification='V4CurrentFrontSharedScheduleEncodingExactRetainedControl',
        results=results,sealed_L5_reopen=reopen,elapsed_seconds=time.perf_counter()-start,
        peak_process_working_set=getattr(process,'peak_wset',None),disk=disk(),
        no_new_layer=True,no_sealed_state_modified=True,bytes_reclaimed=0,
        saved_old_files_preserved=True,scope='Encoding control; no production transition/publication executed')
    durable_json(out/'VERIFIED.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='results'}),flush=True)


def controls(out):
    start=time.perf_counter(); folder=out/'controls';folder.mkdir(parents=True)
    from check_sf2_continuation import access_controls
    access_controls(folder)
    import psutil
    durable_json(folder/'MEASURED.json',dict(seconds=time.perf_counter()-start,
        peak_process_working_set=psutil.Process().memory_info().peak_wset,
        reused_existing_access_controls=True,no_recovery=True,no_layer=True))


def finish(out):
    """Finish only readbacks after source-lock repair; keep completed encodings."""
    start=time.perf_counter(); saved=read(out/'BASELINE.json')
    from composable_state import CommittedState
    from sealed_front_connection import qualified_dispatch
    from sdf_cfd_mesher_v4.common.authority import identity
    results=[read(out/f'L{k}_RESULT.json') for k in range(2,6)]
    for r in results:
        path=out/f"L{r['layer']}"/codec.NAME
        if file_hash(path)!=r['encoded_sha256']:raise ValueError('ChangedVerifiedEncoding')
        original=next(x for x in saved['dependencies'] if x['layer']==r['layer'])
        if file_hash(original['path'])!=r['original_sha256'] or access.token(original['path'])!=original['record']['token']:
            raise ValueError('ChangedVerifiedOriginal')
        if not all(r[k] for k in ('exact_row_equality','exact_expanded_hash','exact_native_scientific_digest')):
            raise ValueError('IncompletePriorVerification')
        content=read(Path(original['path']).parent/'CANDIDATE_PLAN.json')['content']
        # The complete original scientific content is unchanged when only the
        # forecast storage representation is substituted with its semantic hash.
        altered=json.loads(json.dumps(content));altered['components']['forecast']={'replace':r['writer']['expanded_sha256']}
        if altered!=content or identity(altered)!=identity(content):raise ValueError('ChangedScientificContent')
        r['complete_content_identity_preserved']=identity(content)
    with qualified_dispatch():
        with CommittedState(saved['last_qualified']['reference']) as state:
            state.check()
            if state.scientific!=saved['last_qualified']['scientific'] or state.layer!=5:raise ValueError('ChangedSealedL5')
            reopen=dict(layer=state.layer,scientific=state.scientific,root=state.root,passed=True)
    for r in saved['protected']:
        if file_hash(ROOT/r['path'])!=r['sha256']:raise ValueError('ProtectedFileChanged',r['path'])
    for node in saved['active_ancestry']:
        reference=node['reference']
        if reference['kind']!='recursive':continue
        directory=Path(reference['directory']);pointer=read(directory/'CURRENT.json')
        if pointer['root']!=reference['root'] or file_hash(directory/pointer['file'])!=node['root_file_sha256']:
            raise ValueError('ChangedActiveAncestry')
    retained=allocation([p for p in out.rglob('*') if p.is_file()])
    import psutil
    result=dict(classification='V4CurrentFrontSharedScheduleEncodingExactRetainedControl',
        results=results,sealed_L5_reopen=reopen,finish_seconds=time.perf_counter()-start,
        finish_peak_process_working_set=psutil.Process().memory_info().peak_wset,
        full_comparison_memory=read(out/'MEMORY_OBSERVATION.json'),
        capsule_controls=read(out/'controls_fixed/controls/CAPSULE.json'),
        controls_measured=read(out/'controls_fixed/controls/MEASURED.json'),
        disk=disk(),retained=retained,no_new_layer=True,no_sealed_state_modified=True,
        bytes_reclaimed=0,saved_old_files_preserved=True,
        avoided_logical_bytes=sum(r['avoided_logical_bytes'] for r in results),
        avoided_allocation_bytes=sum(r['avoided_allocation_bytes'] for r in results),
        scope='Encoding and guarded access control; no production transition or state publication')
    durable_json(out/'VERIFIED.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('results','retained','capsule_controls')}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline','verify','controls','finish'])
    parser.add_argument('--output',type=Path,default=V4/'results/current_front_dependency_encoding')
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(V4/'results'):raise ValueError('OutputOutsideV4Results')
    globals()[args.mode](out)
