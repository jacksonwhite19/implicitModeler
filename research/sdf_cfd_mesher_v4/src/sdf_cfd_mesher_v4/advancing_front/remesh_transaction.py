"""Content-addressed, private mixed-cell transactions.

The only publication point is CURRENT.json. Pending rollback never edits the
published mesh. A crash before publication resumes from the old mesh; a crash
after publication recognizes the event in the new mesh. This storage primitive
does not itself authorize geometry: the caller must supply the qualified exact
validator and current evidence. No scientific tolerances are defined here.
"""
from copy import deepcopy
from pathlib import Path
import json
from ..common.authority import identity, file_hash
from .rolling_state import atomic_json
from .termination_policy import LocalTerminationPolicy, REMESH_CHECKS

VERSION = 'V4MixedTransaction/1'


def bind(payload):
    return dict(payload=payload, sha256=identity(payload))


def unpack(path):
    value = json.loads(Path(path).read_text())
    if identity(value['payload']) != value['sha256']:
        raise ValueError('CorruptMixedCheckpoint')
    return value['payload']


def mesh_identity(mesh):
    return identity(mesh)


def entity_inputs(mesh):
    """Scientific geometry identity, including ordered faces and ownership.

    Canonical sets identify faces only. Ordered perimeter geometry is retained
    in the inputs. No diagonal is invented for a quadrilateral.
    """
    points = mesh['points']; cells = {}; faces = {}
    for cell in mesh['cells']:
        key = str(cell['id'])
        if key in cells:
            raise ValueError('DuplicateMixedCell')
        cells[key] = identity(dict(cell=cell, coordinates=[points[v] for v in cell['vertices']]))
        for face in cell['faces']:
            fk = ','.join(map(str, sorted(face)))
            faces.setdefault(fk, []).append(dict(owner=cell['id'], perimeter=face,
                                                coordinates=[points[v] for v in face]))
    return dict(cells=cells, faces={k: identity(sorted(v, key=lambda x: x['owner'])) for k, v in faces.items()})


def preservation(before, after):
    """Changes are explicit; wall and first-layer coordinates stay immutable."""
    if before['wall'] != after['wall'] or before['first_layer'] != after['first_layer']:
        raise ValueError('WallFirstLayerMutation')
    n = len(before['wall']['coordinates'])
    for mesh in (before, after):
        if mesh['points'][:n] != mesh['wall']['coordinates'] or mesh['points'][n:2*n] != mesh['first_layer']:
            raise ValueError('WallFirstLayerCoordinateMismatch')
    old = entity_inputs(before); new = entity_inputs(after)
    changed = {k: sorted(i for i in set(old[k]) | set(new[k]) if old[k].get(i) != new[k].get(i)) for k in old}
    first = {str(c['id']) for c in before['cells'] if c['layer'] == 1}
    if first.intersection(changed['cells']):
        raise ValueError('FirstPrismLayerMutation')
    return dict(changed=changed, retained={k: sorted(i for i in old[k] if old[k][i] == new[k].get(i)) for k in old})


def retain_receipts(receipts, before, after):
    """Keep a receipt only when *all* named scientific inputs are identical.

    Receipt dependencies use cell/face keys and may include non-geometric
    inputs (implementation, schedule, SDF). Missing dependencies fail closed.
    """
    retained = {}; invalidated = []
    for key, receipt in sorted(receipts.items()):
        deps = receipt.get('inputs', {})
        if deps and all(before.get(d) == h == after.get(d) for d, h in deps.items()):
            retained[key] = deepcopy(receipt)
        else:
            invalidated.append(key)
    return retained, invalidated


def flattened_inputs(mesh, authority):
    data = entity_inputs(mesh)
    return {**{f'{kind}:{key}': val for kind, items in data.items() for key, val in items.items()},
            **{f'authority:{k}': v for k, v in authority.items()}}


class MixedTransactionStore:
    """Single-writer atomic publication; writers must serialize at runner level."""
    def __init__(self, path):
        self.path = Path(path); self.path.mkdir(parents=True, exist_ok=True)

    def initialize(self, mesh, state, authority):
        if (self.path / 'CURRENT.json').exists():
            current = self.restart()
            if current['authority'] != authority:
                raise ValueError('MixedAuthorityMismatch')
            return current
        payload = dict(version=VERSION, transaction=0, mesh=deepcopy(mesh), local=deepcopy(state),
                       authority=deepcopy(authority), events={}, previous=None)
        entity_inputs(mesh)
        self._publish(payload)
        return self.restart()

    def _publish(self, payload):
        h = identity(payload); name = f'STATE_{h}.json'
        atomic_json(self.path / name, bind(payload))
        atomic_json(self.path / 'CURRENT.json', dict(format=VERSION, checkpoint=name, sha256=file_hash(self.path / name)))

    def restart(self):
        pointer = json.loads((self.path / 'CURRENT.json').read_text())
        if pointer.get('format') != VERSION:
            raise ValueError('MixedPointerVersion')
        if Path(pointer['checkpoint']).name != pointer['checkpoint']:
            raise ValueError('InvalidMixedCheckpointPath')
        p = self.path / pointer['checkpoint']
        if file_hash(p) != pointer['sha256']:
            raise ValueError('CorruptMixedPointer')
        state = unpack(p)
        if state['version'] != VERSION:
            raise ValueError('MixedCheckpointVersion')
        return state

    def dispatch(self, evidence, backend, rebuild, validate, interrupt=None):
        """backend(request, private_mesh) -> replacement, details.

        rebuild(before, replacement, details) -> local state, invalidations.
        validate returns a context-bound qualified receipt. Exceptions leave
        CURRENT byte-identical. BaseException is used by crash tests only.
        """
        before = self.restart(); base = identity(before)
        event = identity(dict(base=evidence['context']['base'], evidence=identity(evidence)))
        if event in before['events']:
            return deepcopy(before['events'][event])
        if evidence['context']['base'] != base:
            raise ValueError('StaleMixedTransaction')
        policy = LocalTerminationPolicy.start(evidence)
        if policy.decision['next_action'] != 'RollbackRemesh':
            return dict(event=event, committed=False, decision=policy.decision)
        # Full stable pre-transaction snapshot is already content-addressed.
        atomic_json(self.path / 'PENDING.json', bind(dict(event=event, base=base, evidence=evidence)))
        if interrupt: interrupt('before_dispatch')
        staged = {}
        def work(request):
            replacement, details = backend(request, deepcopy(before['mesh']))
            if interrupt: interrupt('after_private_rollback')
            audit = preservation(before['mesh'], replacement)
            removed = set(audit['changed']['cells']) & {str(c['id']) for c in before['mesh']['cells']}
            allowed = set(map(str, details['removed_cells']))
            if removed != allowed:
                raise ValueError('RemovedCellManifestMismatch')
            local, invalidations = rebuild(deepcopy(before), replacement, details)
            receipt = validate(request, replacement, local, details)
            staged.update(mesh=deepcopy(replacement), local=deepcopy(local), details=deepcopy(details),
                          preservation=audit, invalidations=invalidations)
            return receipt
        decision = policy.invoke(evidence['context'], work)
        record = dict(event=event, committed=False, decision=decision, invocation=policy.invocation)
        if decision['disposition'] != 'RemeshResolved':
            atomic_json(self.path / f'FAILED_{event}.json', bind(record))
            if identity(self.restart()) != base:
                raise ValueError('ConcurrentMixedWriter')
            return record
        if identity(self.restart()) != base:
            raise ValueError('ConcurrentMixedWriter')
        record.update(committed=True, **{k: staged[k] for k in ('details', 'preservation', 'invalidations')})
        payload = dict(version=VERSION, transaction=before['transaction'] + 1, mesh=staged['mesh'],
                       local=staged['local'], authority=before['authority'], previous=base,
                       events={**before['events'], event: record})
        record['checkpoint'] = dict(transaction=payload['transaction'], event=event,
            trigger=identity(evidence), cavity=identity(staged['details']),
            removed=identity(staged['details']['removed_cells']),
            replacement=identity(staged['mesh']['cells']), layer_map=identity(staged['mesh']['last_prism']),
            front=identity(staged['mesh']['outer']), local_state=identity(staged['local']))
        if interrupt: interrupt('before_publication')
        self._publish(payload)
        if interrupt: interrupt('after_publication')
        return deepcopy(record)
