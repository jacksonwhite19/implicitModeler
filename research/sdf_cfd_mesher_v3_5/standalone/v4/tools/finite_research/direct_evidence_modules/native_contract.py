from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,sys,math

def native_credit(request,invocation_row):
    reference=request.get('native_budget_reservation',{})
    path=local(reference['path'])
    if sha(path)!=reference.get('sha256'):
        raise ValueError('ChangedNativeBudgetReservation')
    reservation=json.loads(path.read_text())
    budget_path=local(reservation['budget_path'])
    if sha(budget_path)!=reservation['budget_sha256']:
        raise ValueError('ChangedNativeBudgetIdentity')
    budget=json.loads(budget_path.read_text())
    expected_folder=local(invocation_row['path']).parent/'NATIVE_BUDGET'
    stages=['farfield','candidate','export']
    stage=request.get('mode','export')
    if (stage not in stages or path!=expected_folder/(stage+'_RESERVATION.json')
        or budget_path!=expected_folder/'BUDGET.json' or budget.get('schema')!='DirectNativeBudget/1'
        or budget.get('limit_seconds')!=7200 or budget.get('stage_limit_seconds')!=3600 or budget.get('stages')!=stages
        or budget.get('binding')!=request['binding']
        or budget.get('invocation_sha256')!=invocation_row['sha256']
        or local(budget['invocation_path'])!=local(invocation_row['path'])
        or reservation.get('schema')!='DirectNativeReservation/1'
        or reservation.get('binding')!=request['binding'] or reservation.get('stage')!=stage
        or reservation.get('invocation_sha256')!=invocation_row['sha256']
        or reservation.get('publication_enabled') is not False):
        raise ValueError('MisboundNativeBudgetReservation')
    rows=reservation['prior_debits']
    if len(rows)!=stages.index(stage):
        raise ValueError('MissingNativeBudgetPriorDebits')
    used=0.
    for index,row in enumerate(rows):
        debit_path=local(row['path'])
        if debit_path!=expected_folder/(stages[index]+'_DEBIT.json') or sha(debit_path)!=row['sha256']:
            raise ValueError('ChangedNativeBudgetDebit')
        debit=json.loads(debit_path.read_text())
        elapsed=debit['seconds']
        if (type(elapsed) not in (int,float) or not math.isfinite(elapsed) or elapsed<0
            or debit.get('passed') is not True or debit.get('stage')!=stages[index]):
            raise ValueError('FailedNativeBudgetDebit')
        used+=elapsed
        if debit.get('used_seconds')!=used:
            raise ValueError('ChangedNativeBudgetDebitTotal')
        cleanup=row.get('cleanup_seconds')
        if type(cleanup) not in (int,float) or not math.isfinite(cleanup) or cleanup<0:
            raise ValueError('NativeBudgetCleanupChargeRequired')
        allocated=debit.get('allocated_seconds')
        if (type(allocated) not in (int,float) or not math.isfinite(allocated)
            or not 30<allocated<=3600 or elapsed+cleanup>allocated):
            raise ValueError('NativeStageDebitAllowanceExceeded')
        used+=cleanup
    allocated=reservation['seconds']
    requested=request['seconds']
    if (any(type(value) not in (int,float) or not math.isfinite(value) for value in (allocated,requested))
        or reservation.get('used_seconds')!=used or allocated<=30 or not 0<requested<=allocated<=min(3600,7200-used)
        or datetime.now(timezone.utc)>=datetime.fromisoformat(reservation['expires_utc'])):
        raise ValueError('NativeAggregateCreditExceededOrExpired')
    return reservation

def local(name):
    value=str(name)
    if sys.platform=="linux" and value[1:2]==":":
        return Path("/mnt/"+value[0].lower()+value[2:].replace("\\","/"))
    return Path(value)

def sha(path):
    with local(path).open("rb") as stream:
        digest=hashlib.sha256()
        for block in iter(lambda:stream.read(1048576),b""):
            digest.update(block)
        return digest.hexdigest()

def owners(request):
    rows=request["owners"]
    required={"task","authorization","launch_review","invocation","clock","input_pins","stage"}
    if set(rows)!=required:
        raise ValueError("CompleteDirectNativeOwnersRequired")
    values={}
    for name,row in rows.items():
        if sha(row["path"])!=row["sha256"]:
            raise ValueError("ChangedDirectNativeOwner",name)
        values[name]=json.loads(local(row["path"]).read_text())
    task=values["task"]
    grant=values["authorization"]
    review=values["launch_review"]
    invocation=values["invocation"]
    selected=values["input_pins"]
    for name in ("task","authorization","launch_review","clock","stage"):
        if selected.get(rows[name]["path"])!=rows[name]["sha256"]:
            raise ValueError("UnownedDirectNativeInput",name)
    if (task.get("schema")!="PreparedFiniteLiveTask/1" or task.get("live_execution_authorized") is not True
        or task.get("maximum_invocations")!=1 or task.get("payload_seconds")!=14400 or task.get("owner_seconds")!=14700
        or grant.get("approved") is not True or grant.get("mode")!="fresh_live_once"
        or grant.get("task_sha256")!=rows["task"]["sha256"]
        or grant.get("publication_enabled") is not True
        or grant.get("native_seconds")!=7200
        or grant.get("native_scope")!="DOWNSTREAM_NATIVE_ONLY"
        or review.get("approved") is not True or review.get("mode")!="fresh_live_once"
        or review.get("authorization_sha256")!=rows["authorization"]["sha256"]
        or review.get("task_sha256")!=rows["task"]["sha256"]
        or review.get("manifest_sha256")!=grant.get("manifest_sha256")
        or review.get("source_release_sha256")!=request["binding"]["source_release_sha256"]
        or invocation.get("schema")!="DirectRuntimeInvocation/1" or invocation.get("binding")!=request["binding"]
        or invocation.get("publication_enabled") is not False
        or request.get("publication_enabled") is not False
        or request["binding"]["request_sha256"]!=rows["stage"]["sha256"]
        or values["stage"].get("execution_target")!="direct_complete"
        or values["stage"].get("publication_enabled") is not False
        or values["stage"].get("downstream_policy",{}).get("native_seconds")!=7200
        or datetime.now(timezone.utc)>=datetime.fromisoformat(task["deadline_utc"])):
        raise ValueError("DirectNativeLiveOwnerRequired")
    if (values["clock"].get("payload_seconds")!=14400 or type(values["clock"].get("launch_pid")) is not int
        or invocation.get("clock_sha256")!=rows["clock"]["sha256"]):
        raise ValueError("ConsumedDirectOwnerClockRequired")
    native_credit(request,rows['invocation'])
    if "inputs" in request:
        if (invocation["cap_sha256"]!=request["inputs"]["ORIGINAL_CAP_INPUT"]["sha256"]
            or invocation["policy_sha256"]!=request["policy_sha256"]):
            raise ValueError("ChangedDirectInvocationGeometry")
        if request["mode"]=="candidate":
            frozen=request["frozen_boundary"]
            if sha(frozen["path"])!=frozen["sha256"]:
                raise ValueError("ChangedFrozenDirectBoundary")
            value=json.loads(local(frozen["path"]).read_text())
            if (value["binding"]!=request["binding"] or value["cap_sha256"]!=invocation["cap_sha256"]
                or value["farfield_sha256"]!=request["inputs"]["OUTER"]["sha256"]):
                raise ValueError("MisboundDirectFrozenBoundary")
    return values
