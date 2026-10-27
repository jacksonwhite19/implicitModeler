"""Operational compute limits. These never classify mesh feasibility."""
import json,time
from pathlib import Path


class BranchStopped(BaseException):
    pass


class ProgressLedger:
    def __init__(self,path,clock=time.monotonic):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True)
        self.clock=clock;self.start=clock();self.last_progress=self.start
        self.best=None;self.rejected=0;self.unchanged=0;self.branch='none';self.layer=0

    def record(self,event,**row):
        row=dict(seconds=self.clock()-self.start,event=event,**row)
        with self.path.open('a',encoding='utf8') as stream:stream.write(json.dumps(row,sort_keys=True)+'\n')
        return row

    def progress(self,event,**row):
        self.last_progress=self.clock();return self.record(event,progress=True,**row)

    def start_branch(self,name):
        self.branch=name;self.best=None;self.rejected=0;self.unchanged=0
        self.record('branch',name=name)

    def trial(self,checked,layer):
        failures=checked.get('quality',{}).get('failures',[])
        keys=frozenset((r['kind'],str(r.get('id','')),str(sorted(r.get('owners',[]))),str(r.get('vertex',''))) for r in failures)
        classes={k[0] for k in keys}
        improved=self.best is None or keys<self.best or bool(classes-{k[0] for k in self.best})
        if checked.get('passed') or improved:
            self.best=keys;self.rejected=0;self.unchanged=0
            self.progress('trial_progress',layer=layer,branch=self.branch,failures=len(keys),passed=checked.get('passed',False))
        else:
            self.rejected+=1
            self.unchanged=self.unchanged+1 if keys==self.best else 0
            self.record('trial_rejected',layer=layer,branch=self.branch,failures=len(keys),consecutive=self.rejected)
        if self.unchanged>=3:
            self.record('branch_stop',reason='SameBlockerThreeTimesWithoutNewEvidence',branch=self.branch)
            raise BranchStopped('SameBlockerThreeTimesWithoutNewEvidence')
        if self.rejected>=10:
            self.record('branch_stop',reason='TenRejectedWithoutNewEvidence',branch=self.branch)
            raise BranchStopped('TenRejectedWithoutNewEvidence')


def stop_reason(elapsed,since_progress,has_scientific_progress,case_used=0,task_used=0):
    if task_used+elapsed>=14400:return 'TaskHardCap'
    if case_used+elapsed>=4500:return 'CaseHardCap'
    if since_progress>=900:return 'StageStagnation15Minutes'
    if case_used+elapsed>=2700 and not has_scientific_progress:return 'DefaultCaseBudget'
    return None


def termination(reason,progress):
    if reason=='StageStagnation15Minutes':return 'BudgetStoppedStagnant'
    return 'BudgetStoppedAfterProgress' if progress else 'BudgetStoppedStagnant'


def checkpoint_stop_record(folder,reason,progress):
    from sdf_cfd_mesher_v4.common.authority import file_hash
    store=Path(folder)/'checkpoints';pointer=store/'CURRENT.json';current=None
    if pointer.exists():
        current=json.loads(pointer.read_text())
        name=current['checkpoint']
        if Path(name).name!=name or file_hash(store/name)!=current['sha256']:
            raise ValueError('BudgetStopCheckpointCorrupt')
    return dict(reason=termination(reason,progress),control=reason,current=current,
                scientific_feasibility='NotDeterminedByComputeStop')


def prior_case_progress(root,name):
    """Operational progress belongs to the case, not the worker lifetime."""
    for path in (Path(root)/name).glob('*/PROGRESS.jsonl'):
        with path.open(encoding='utf8') as stream:
            for line in stream:
                try:row=json.loads(line)
                except json.JSONDecodeError:continue
                if row.get('event')=='checkpoint_published' and row.get('layer',0)>2:return True
                if row.get('event')=='certificate_complete' and row.get('passed') is True and row.get('new',0)>0:return True
    return False


def record_validated_stage(ledger,name,receipt):
    """Only completed production validation receipts count as this milestone."""
    if name not in ('numerical_validation','certify_validation'):return
    if not isinstance(receipt,dict) or receipt.get('passed') is not True:return
    if not receipt.get('state') or not receipt.get('proof'):return
    ledger.progress('production_validation_passed',stage=name,state=receipt['state'],proof=receipt['proof'])
