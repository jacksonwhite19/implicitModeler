"""Operational exception-path supervision; never a scientific acceptance gate."""
import json
import shutil
import time
from pathlib import Path
import psutil


class AttemptStopped(BaseException):
    """Escapes search kernels which correctly catch invalid geometric trials."""


class Events:
    def __init__(self, path, seconds=600):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.start = time.monotonic()
        self.seconds = seconds

    def __call__(self, event, **data):
        mem = psutil.Process().memory_info()
        row = dict(event=event, elapsed=time.monotonic()-self.start,
                   rss=mem.rss, peak_rss=getattr(mem, 'peak_wset', mem.rss),
                   disk_free=shutil.disk_usage(self.path.parent).free, **data)
        with self.path.open('a', encoding='utf8') as stream:
            stream.write(json.dumps(row, sort_keys=True)+'\n')
        print(json.dumps(row, sort_keys=True), flush=True)
        if row['elapsed'] > self.seconds:
            raise AttemptStopped('StageTimeBudget:scientific_feasibility_undetermined')
        return row


def require_space(path, measured_input_bytes, measured_trial_bytes):
    """Reserve one input-sized work copy and one measured trial; no GB constant."""
    required = measured_input_bytes + measured_trial_bytes
    free = shutil.disk_usage(path).free
    if free < required:
        raise AttemptStopped('InsufficientMeasuredStageScratch')
    return dict(free=free, required=required)


def failure_keys(checked):
    return frozenset((r['kind'], str(r.get('id', '')),
                      tuple(sorted(r.get('owners', []))), r.get('vertex'))
                     for r in checked['quality']['failures'])


class BestEvidence:
    """A strict failure-set improvement is provisional, never mesh acceptance."""
    def __init__(self, checked):
        self.keys = failure_keys(checked)
        self.trial = None
        self.no_new_diagnosis = 0
        self.seen = {self.keys}

    def observe(self, checked, trial=None):
        keys = failure_keys(checked)
        improved = keys < self.keys or checked.get('passed', False)
        novel = keys not in self.seen
        self.seen.add(keys)
        self.no_new_diagnosis = 0 if improved or novel else self.no_new_diagnosis+1
        if improved:
            self.keys = keys
            self.trial = trial.copy() if trial is not None else None
        return dict(improved=improved, new_diagnosis=novel, failures=len(keys),
                    best_failures=len(self.keys), repeated=self.no_new_diagnosis)


def failure_components(cells, failures):
    """Face adjacency among failing cells, including both owners of bad faces."""
    by_id = {c['id']:c for c in cells}
    bad = {r['id'] for r in failures if r.get('id') in by_id}
    bad.update(i for r in failures for i in r.get('owners', []) if i in by_id)
    faces = {}
    adjacent = {i:set() for i in bad}
    for i in sorted(bad):
        for face in by_id[i]['faces']:
            key = tuple(sorted(face))
            for other in faces.get(key, []):
                adjacent[i].add(other); adjacent[other].add(i)
            faces.setdefault(key, []).append(i)
    remaining = set(bad); groups = []
    while remaining:
        todo = [min(remaining)]; component = set(todo); remaining.difference_update(todo)
        for i in todo:
            for j in sorted(adjacent[i] & remaining):
                remaining.remove(j); component.add(j); todo.append(j)
        groups.append(sorted(component))
    return groups


def validate_disposable(path, workspace, referenced, replacements):
    """Read-only deletion eligibility; callers still record a manifest first."""
    import hashlib
    path=Path(path).resolve();workspace=Path(workspace).resolve()
    if not path.is_relative_to(workspace):raise ValueError('OutsideWorkspace')
    if path in {Path(p).resolve() for p in referenced}:raise ValueError('ReferencedAuthority')
    if not replacements:raise ValueError('MissingReplacementEvidence')
    def digest(p):
        h=hashlib.sha256()
        with p.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
        return h.hexdigest()
    old=digest(path)
    # This intentionally narrow cleanup rule allows exact duplicate files only.
    # A merely summarized scientific result needs a separate human-reviewed rule.
    if not any(Path(p).resolve()!=path and Path(p).is_file() and digest(Path(p))==old for p in replacements):
        raise ValueError('NoExactPreservedReplacement')
    return dict(path=str(path),bytes=path.stat().st_size,sha256=old,
                replacements=[str(Path(p).resolve()) for p in replacements],reason='Exact duplicate; original authority remains preserved')
