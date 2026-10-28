"""Owned local source execution; no changed numerical predicates or disk issuer."""
from contextlib import ExitStack
from pathlib import Path
import hashlib,json,math,multiprocessing as mp,os,threading,time,uuid
import numpy as np
from multiprocessing.connection import wait
from sdf_cfd_mesher_v4.common.authority import array_hash,identity,frozen

VERSION='V35OwnedLocalSourceExecution/1'
MIN_SEGMENTS=1024

def _semantic(source):
    from source_handoff import _context
    value=_context(source)
    if value is None or value[3] is not None:raise ValueError('UnsupportedParallelSourceContext')
    return value[2]

def eligible(source,positions,mode,workers):
    if workers==1:return 'serial_default'
    if mode!='production':return 'audit_serial_recompute'
    from source_handoff import _context
    context=_context(source)
    if context is None or context[3] is not None:return 'unsupported_source_context'
    if source.queries._scope._batch_inputs()[2][0]>6:return 'proof_validator_depth_limit'
    if (len(positions)-1)*len(positions[0])<MIN_SEGMENTS or len(positions[0])<workers:return 'small_population'
    binding=getattr(source,'_connection_binding',None)
    if binding is None or getattr(binding,'metric_connection',None) is None:return 'unsupported_connection'
    binding.metric_connection.verify(binding)
    return None

def _refusal(error,layer):
    from sdf_cfd_mesher_v4.sdf.regional import UnresolvedRegion
    diagnostic=getattr(error,'_outward_segment_diagnostic',{})
    height=type(error) is ValueError and error.args in (('UniqueSourceWallNormalUnresolved',),('TrueSourceFullNormalHeightUnresolved',))
    progress=(type(error) is UnresolvedRegion and error.args==('SourceOutwardSegmentUnresolved',) and diagnostic.get('attempt',{}).get('reason') in ('endpoint_positive_unresolved','depth_exhausted_progress_unresolved'))
    if not (height or progress):raise error
    segment=diagnostic.get('segment',{})
    if segment and segment.get('index_domain')!='row of qualify_segments input arrays':raise ValueError('UnknownShardDiagnosticDomain')
    local=segment.get('index',diagnostic.get('completed_outward_proof',{}).get('segment_index'))
    if type(local) is not int or local<0:raise ValueError('MissingShardRefusalIndex')
    return dict(layer=layer,type=type(error).__name__,args=list(error.args),diagnostic=diagnostic,local_index=local)

def _worker(connection,case,case_sha,record,session,worker,expected,assurance,numpy_errors):
    """A fresh stock owner, created only after the owning Job assignment."""
    try:
        if connection.recv()!=('start',session,worker):raise ValueError('SourceWorkerStartBinding')
        import geometry,standalone_tool as cli
        from immutable_state_access import guard
        from source_custody import held,sha
        np.seterr(**numpy_errors)
        with ExitStack() as life:
            life.enter_context(guard(Path(case)))
            if sha(Path(case))!=case_sha:raise ValueError('SourceWorkerCaseChanged')
            custody=life.enter_context(held(Path(record).parent,Path(record)))
            token=cli._surface_assurance.set(assurance);life.callback(cli._surface_assurance.reset,token)
            host,binding=cli.geometry(Path(case));life.callback(host.close)
            source=binding.authority;source.verify()
            if _semantic(source)!=expected:raise ValueError('SourceWorkerContextMismatch')
            connection.send(dict(state='ready',session=session,worker=worker,context=expected,runtime=custody['identity'],files=custody['files']))
            sequence=0
            while True:
                packet=connection.recv()
                if packet==('close',session,worker):break
                if packet.get('session')!=session or packet.get('worker')!=worker or packet.get('sequence')!=sequence+1:raise ValueError('SourceWorkerRequestBinding')
                sequence+=1
                x=frozen(packet['positions'],'<f8');h1=packet['height'];lo,hi=packet['range']
                if x.ndim!=3 or x.shape[1]!=hi-lo or x.shape[2]!=3 or array_hash(x)!=packet['operands']:raise ValueError('SourceWorkerOperandsBinding')
                if _semantic(source)!=expected:raise ValueError('SourceWorkerContextChanged')
                for layer in range(1,len(x)):
                    command=connection.recv()
                    if command!=('layer',session,sequence,worker,layer):raise ValueError('SourceWorkerLayerBinding')
                    proof=None;error=None;started=time.perf_counter()
                    try:proof=source.qualify_segments(x[layer-1],x[layer],first_height_m=h1 if layer==1 else None)
                    except ValueError as failure:
                        error=_refusal(failure,layer);source.verify()
                    if _semantic(source)!=expected:raise ValueError('SourceWorkerContextChanged')
                    connection.send(dict(state='layer',session=session,sequence=sequence,worker=worker,layer=layer,range=[lo,hi],operands=packet['operands'],proof=proof,error=error,seconds=time.perf_counter()-started))
                    next_command=connection.recv()
                    if next_command==('discard',session,sequence):break
                    if next_command!=('continue',session,sequence):raise ValueError('SourceWorkerPopulationDisposition')
                    if error is not None:raise ValueError('SourceWorkerContinuedRefusal')
        if custody.get('passed') is not True or custody.get('state')!='closed':raise ValueError('SourceWorkerCustodyClosure')
        connection.send(dict(state='closed',session=session,worker=worker,runtime=custody['identity'],files=len(custody['files']),case_sha256=case_sha,passed=True))
    except BaseException as error:
        try:connection.send(dict(state='fatal',session=session,worker=worker,error=repr(error)))
        except BaseException:pass
        raise
    finally:connection.close()

def validate_layer(parts,x,layer,h1,geometry,expression,ranges):
    """Validate all exact envelopes and stock raw proof encodings, in order."""
    from sdf_cfd_mesher_v4.sdf.segment_receipts import validate_proof
    if len(parts)!=len(ranges):raise ValueError('IncompleteSourceShardPopulation')
    raw=[];cursor=0
    for part,(lo,hi) in zip(parts,ranges):
        if lo!=cursor or hi<=lo or part.get('range')!=[lo,hi]:raise ValueError('MissingOrDuplicateSourceShard')
        cursor=hi;q=part['proof'];height=h1 if layer==1 else None
        if (q.get('passed') is not True or q.get('segments')!=hi-lo or q.get('start')!=array_hash(x[layer-1,lo:hi]) or q.get('end')!=array_hash(x[layer,lo:hi]) or q.get('geometry_identity')!=geometry or q.get('first_source_normal_height_m')!=height or q.get('exact_field_progress') is not True or q.get('positive_endpoints') is not True or len(q.get('segment_proofs',[]))!=hi-lo or q.get('height_reserve_m')!=(1e-12 if layer==1 else None)):
            raise ValueError('InvalidSourceShardProofBinding')
        error=q.get('maximum_first_height_error_m')
        if (layer==1 and (type(error) not in (int,float) or not math.isfinite(error) or not 0<=error<=1e-12)) or (layer!=1 and error is not None):raise ValueError('InvalidSourceShardHeightBinding')
        for proof in q['segment_proofs']:validate_proof(proof,expression)
        for key,value in q.items():
            if key not in ('segments','start','end','segment_proofs','maximum_first_height_error_m') and raw and value!=raw[0].get(key):raise ValueError('ChangedSourceShardSchema')
        if raw and set(q)!=set(raw[0]):raise ValueError('ChangedSourceShardSchema')
        raw.append(q)
    if cursor!=len(x[0]):raise ValueError('IncompleteSourceShardPopulation')
    return dict(raw[0],segments=cursor,start=array_hash(x[layer-1]),end=array_hash(x[layer]),segment_proofs=[p for q in raw for p in q['segment_proofs']],maximum_first_height_error_m=max(q['maximum_first_height_error_m'] for q in raw) if layer==1 else None)

class _Population:
    __slots__=('owner','source','context','height','positions','payload','sequence','used')
    def __init__(self,owner,x,payload):
        self.owner=owner;self.source=owner.source;self.context=owner.context;self.height=owner.height;self.positions=x;self.payload=payload;self.sequence=owner.sequence;self.used=False

class Pool:
    def __init__(self,source,height,workers,deadline):
        from source_handoff import _context
        from source_process_owner import Job
        from immutable_state_access import guard
        from source_custody import held,ROOT,sha
        self.source=source;self.height=height;self.workers=workers;self.deadline=deadline
        self.context=_context(source);self.semantic=_semantic(source);self.thread=threading.get_ident()
        self.session=uuid.uuid4().hex;self.sequence=0;self.pending=None;self.poisoned=False;self.closed=False;self.execution=None
        self.pipes=[];self.processes=[];self.life=ExitStack();self.job=None;self.transcript=[]
        binding=source._connection_binding;binding.metric_connection.verify(binding)
        case=Path(binding.metric_connection.path).resolve();self.case_sha=binding.metric_connection.pin
        self.geometry=source.geometry_identity;self.expression=hashlib.sha256(repr(source.queries._scope._batch_inputs()[0]).encode()).hexdigest()
        logs=ROOT/'run_logs';logs.mkdir(exist_ok=True)
        try:
            self.job=Job()
            self.life.enter_context(guard(case))
            binding.metric_connection.verify(binding)
            if sha(case)!=self.case_sha:raise ValueError('SourcePoolCaseChangedBeforeCustody')
            self.custody=self.life.enter_context(held(logs,logs/(self.session+'_parent.SOURCES.json')))
            spawn=mp.get_context('spawn')
            for i in range(workers):
                parent,child=spawn.Pipe();p=spawn.Process(target=_worker,args=(child,str(case),self.case_sha,str(logs/(self.session+f'_{i}.SOURCES.json')),self.session,i,self.semantic,binding.qualification._assurance,np.geterr()))
                self.pipes.append(parent);self.processes.append(p);p.start();child.close()
                self.job.assign(p.sentinel);parent.send(('start',self.session,i))
            ready=self._receive('ready')
            for row in ready:
                if row['context']!=self.semantic or row['runtime']!=self.custody['identity'] or row['files']!=self.custody['files']:raise ValueError('SourceWorkerRuntimeMismatch')
        except BaseException:
            self.abort();raise
    def _live(self):
        if self.poisoned or self.closed or threading.get_ident()!=self.thread:raise ValueError('ExpiredOrDetachedSourcePool')
        if time.perf_counter()>self.deadline:raise TimeoutError('SourcePoolWholeWorkerDeadline')
    def _receive(self,state):
        pending=set(self.pipes);rows={}
        while pending:
            self._live()
            for pipe in wait(list(pending),timeout=min(.1,max(0,self.deadline-time.perf_counter()))):
                try:row=pipe.recv()
                except (EOFError,BrokenPipeError) as error:raise ChildProcessError('SourceWorkerTransportClosed') from error
                i=self.pipes.index(pipe)
                if row.get('state')!=state or row.get('session')!=self.session or row.get('worker')!=i:raise ValueError('SourceWorkerReplyBinding',row)
                rows[i]=row;pending.remove(pipe)
            for p,pipe in zip(self.processes,self.pipes):
                if pipe in pending and not p.is_alive():raise ChildProcessError('SourceWorkerExitedBeforeReply')
        return [rows[i] for i in range(self.workers)]
    def qualify(self,positions):
        from source_handoff import _context,_same_context
        try:
            self._live();self.pending=None;self.sequence+=1;x=frozen(positions,'<f8')
            if not _same_context(_context(self.source),self.context):raise ValueError('SourcePoolParentContextChanged')
            n=len(x[0]);ranges=[(i*n//self.workers,(i+1)*n//self.workers) for i in range(self.workers)];operands=[]
            for i,(pipe,(lo,hi)) in enumerate(zip(self.pipes,ranges)):
                shard=frozen(x[:,lo:hi],'<f8');operands.append(array_hash(shard))
                pipe.send(dict(session=self.session,sequence=self.sequence,worker=i,range=[lo,hi],positions=shard,operands=operands[-1],height=self.height))
            proofs=[]
            for layer in range(1,len(x)):
                for i,pipe in enumerate(self.pipes):pipe.send(('layer',self.session,self.sequence,i,layer))
                parts=self._receive('layer')
                for i,part in enumerate(parts):
                    if part.get('sequence')!=self.sequence or part.get('layer')!=layer or part.get('range')!=list(ranges[i]) or part.get('operands')!=operands[i]:raise ValueError('SourceShardEnvelopeMismatch')
                failures=[(i,p['error']) for i,p in enumerate(parts) if p['error'] is not None]
                self.transcript.append(dict(sequence=self.sequence,layer=layer,operands=array_hash(x),replies=identity(parts),scientific_refusals=len(failures),raw_refusals=[dict(worker=i,range=list(ranges[i]),error=e) for i,e in failures]))
                for pipe in self.pipes:pipe.send(('discard' if failures else 'continue',self.session,self.sequence))
                if failures:
                    for i,error in failures:
                        if not 0<=error['local_index']<ranges[i][1]-ranges[i][0]:raise ValueError('InvalidShardRefusalIndex')
                    i,error=min(failures,key=lambda item:ranges[item[0]][0]+item[1]['local_index'])
                    detail=json.loads(json.dumps(error));local=detail.pop('local_index');diag=detail['diagnostic']
                    if 'segment' in diag:
                        diag['segment'].update(index=ranges[i][0]+local,start_array_hash=array_hash(x[layer-1]),end_array_hash=array_hash(x[layer]))
                    if 'completed_outward_proof' in diag:diag['completed_outward_proof']['segment_index']=ranges[i][0]+local
                    if not _same_context(_context(self.source),self.context):raise ValueError('SourcePoolParentContextChanged')
                    from collision_policy import SourceSegmentRefusal
                    raise SourceSegmentRefusal(detail)
                proofs.append(validate_layer(parts,x,layer,self.height,self.geometry,self.expression,ranges))
            if not _same_context(_context(self.source),self.context):raise ValueError('SourcePoolParentContextChanged')
            payload=json.dumps(proofs,allow_nan=False,separators=(',',':')).encode()
            self.pending=_Population(self,x,payload)
            return self.pending
        except BaseException as error:
            from collision_policy import SourceSegmentRefusal
            if type(error) is not SourceSegmentRefusal:self.abort()
            raise
    def take(self,capability,source,height,positions):
        self._live()
        from source_handoff import _context,_same_context
        if (type(capability) is not _Population or capability is not self.pending or capability.owner is not self or capability.used or capability.source is not source or type(height) is not type(self.height) or repr(height)!=repr(self.height) or capability.sequence!=self.sequence or positions.shape!=capability.positions.shape or positions.tobytes()!=capability.positions.tobytes() or not _same_context(_context(source),capability.context)):
            self.abort();raise ValueError('InvalidSourcePopulationCapability')
        capability.used=True;self.pending=None
        return json.loads(capability.payload)
    def finish(self,positions,proofs):
        self._live()
        try:
            for i,pipe in enumerate(self.pipes):pipe.send(('close',self.session,i))
            closed=self._receive('closed')
            for row in closed:
                if row['passed'] is not True or row['case_sha256']!=self.case_sha or row['runtime']!=self.custody['identity'] or row['files']!=len(self.custody['files']):raise ValueError('SourceWorkerClosureMismatch')
            for p in self.processes:
                p.join(max(0,self.deadline-time.perf_counter()))
                if p.exitcode!=0:raise ValueError('SourceWorkerExitFailure')
            if self.job.active()!=0:raise ValueError('SourceWorkerDescendantsRemain')
            self.life.close()
            if self.custody.get('passed') is not True:raise ValueError('SourceParentCustodyClosure')
            if positions is None:
                self.closed=True;self.job.close()
                for pipe in self.pipes:pipe.close()
                return None
            self.execution=dict(schema=VERSION,requested_workers=self.workers,effective_workers=self.workers,reason='eligible_production_population',state='closed',runtime_identity=self.custody['identity'],runtime_files=len(self.custody['files']),case_sha256=self.case_sha,geometry=self.geometry,height_hex=float(self.height).hex(),positions_shape=list(positions.shape),positions_sha256=array_hash(positions),proof_identity=identity(proofs),semantic_context_sha256=hashlib.sha256(self.semantic.encode()).hexdigest(),session=self.session,sequence=self.sequence,transcript_sha256=identity(self.transcript),refusal_diagnostics=[r for entry in self.transcript for r in entry['raw_refusals']],workers=closed,contained_active_processes=0,source_assurance='owned_current_local_execution; not an independent H1 certificate',speculation='all shards resolve current layer before earliest original index refusal')
            self.closed=True;self.job.close()
            for pipe in self.pipes:pipe.close()
            return self.execution
        except BaseException:self.abort();raise
    def abort(self):
        self.pending=None;self.poisoned=True
        try:
            if self.job is not None:self.job.retire()
        finally:
            if self.job is not None:self.job.close()
            for p in self.processes:
                if p.pid is not None:
                    if p.is_alive():p.kill()
                    p.join(5)
            for pipe in self.pipes:pipe.close()
            self.life.close();self.closed=True
