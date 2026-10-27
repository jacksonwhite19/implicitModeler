"""Non-binding local capacity diagnostics. No builder or mesh mutation.

Only analytic source domains supply capacity bounds. A forecast failure alone
never establishes exhaustion. Exact rationals preserve binary64 schedule inputs.
An open wedge station is a nominal-path warning, never a domain capacity proof.
"""
from fractions import Fraction
from collections import OrderedDict
import copy
import hashlib
import json
import numpy as np

VERSION = "V4ProspectiveCapacity/1"


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def rational(value):
    return Fraction.from_float(float(value))


def fraction_record(value):
    return dict(numerator=value.numerator, denominator=value.denominator)


def schedule_demand(schedule, current):
    targets = np.asarray(schedule.targets, dtype=np.float64)
    if targets.ndim != 1 or len(targets) == 0:
        raise ValueError("InvalidLayerSchedule")
    if not isinstance(current, (int, np.integer)) or not 0 <= current <= len(targets):
        raise ValueError("InvalidCommittedLayer")
    if not np.isfinite(targets).all() or np.any(np.diff(np.r_[0., targets]) <= 0):
        raise ValueError("InvalidLayerSchedule")
    base = rational(targets[current-1]) if current else Fraction(0)
    return dict(current=current, requested=len(targets), remaining_layers=len(targets)-current,
                remaining_thickness_exact=fraction_record(rational(targets[-1])-base),
                remaining_thickness_binary64=float(targets[-1])-float(base),
                target_hex=[float(x).hex() for x in targets])


def first_limit(targets, depth):
    # Equality collapses distinct opposing fronts or a positive-area cavity front.
    return next((i+1 for i, t in enumerate(targets) if rational(t) >= depth), None)


def analytic_bound(source):
    """Source SDF parameters, not estimated curvature/medial distance.

    Slab fluid domain: |x| < h; phi=h-|x| <= h.
    Ball/cylinder cavity: phi=R-|x| or R-sqrt(x*x+y*y) <= R.
    At the bound the level set loses dimensions required by a prism front.
    """
    kind = source["kind"]
    if kind == "parallel_gap":
        d = rational(source["spacing"])
        if d <= 0: raise ValueError("InitiallyIntersectingDomain")
        return d/2, "OpposingOffsetFrontContact", "global_slab"
    if kind in ("spherical_cavity", "cylindrical_cavity"):
        d = rational(source["radius"])
        if d <= 0: raise ValueError("InvalidRadius")
        return d, "OffsetLevelSetCollapse", "closed_radial_domain"
    if kind == "open_wedge":
        # z=+-a*x, phi=(a*x-|z|)/sqrt(1+a*a). Fixture uses 3-4-5 normal.
        x = rational(source["station"])
        if x <= 0: raise ValueError("InvalidStation")
        return 3*x/5, "FixedStationOffsetPinch", "fixed_station_only"
    raise ValueError("NoAnalyticCapacityWitness")


def screen(schedule, current, source=None, events=()):
    schedule_demand(schedule, current)
    warnings = sorted((dict(e) for e in events if int(e["layer"]) > current), key=lambda e: (e["layer"], identity(e)))
    first = None
    if source is not None:
        depth, mechanism, scope = analytic_bound(source)
        first = first_limit(schedule.targets, depth)
    return dict(active=bool(first is not None or warnings), first_candidate_layer=first,
                events=warnings, source=source)


class CapacityForecast:
    """Bounded content-addressed cache. Scientific inputs are copied into keys.

    atlas_candidates is a read-only adapter to existing production warning
    columns and vertex ownership. It neither calls activate nor changes state.
    Unsupported mechanisms remain unresolved even when the screen is empty.
    """
    def __init__(self, cache_capacity=128):
        if cache_capacity < 0: raise ValueError("InvalidCacheCapacity")
        self.capacity=cache_capacity; self.cache=OrderedDict(); self.hits=0; self.misses=0

    @staticmethod
    def atlas_candidates(atlas):
        columns = sorted({int(c) for e in atlas.events if e["layer"] > atlas.layer for c in e["columns"]})
        vertices = np.unique(atlas.top.triangles[columns]).tolist() if columns else []
        return dict(columns=columns, vertices=vertices, current=atlas.layer,
                    evidence_identity=identity(atlas.events), binding=atlas.authority)

    def analyze(self, region, schedule, current, *, source=None, events=(), geometry_identity, topology_identity):
        demand=schedule_demand(schedule,current)
        inputs=dict(version=VERSION,region=region,demand=demand,source=source,events=list(events),
                    geometry=geometry_identity,topology=topology_identity)
        key=identity(inputs)
        if key in self.cache:
            self.hits+=1; self.cache.move_to_end(key); return copy.deepcopy(self.cache[key])
        self.misses+=1; signal=screen(schedule,current,source,events)
        status="CapacityWarning" if signal["active"] else "CapacityUnresolved"
        first=None;last=None;mechanism="NoSourceCapacityWitness";evidence={}
        if source is not None:
            depth, mechanism, scope=analytic_bound(source)
            candidate=first_limit(schedule.targets,depth)
            evidence=dict(bound=fraction_record(depth),scope=scope,nominal_first_infeasible=candidate,
                          inequality="target >= bound",source_identity=identity(source))
            if scope != "fixed_station_only":
                first=candidate;last=candidate-1 if candidate else len(schedule.targets)
                status="CapacityLimitPredicted" if candidate else "CapacityClearThroughRequestedLayers"
                evidence["qualification"]="Geometric capacity only; not a hard-quality certificate"
            else:
                status="CapacityWarning" if candidate else "CapacityUnresolved"
                evidence["limitation"]="Open channel permits tangential escape; station is not invariant"
        record=dict(region_id=region,current_committed_layer=current,requested_final_layer=len(schedule.targets),
                    demand=demand,predicted_last_feasible_layer=last,predicted_first_infeasible_layer=first,
                    mechanism=mechanism,evidence=evidence,status=status,screen=signal,scientific_input_identity=key,
                    termination_authority=False)
        if self.capacity:
            self.cache[key]=copy.deepcopy(record)
            while len(self.cache)>self.capacity:self.cache.popitem(last=False)
        return record


def classify_steering(record, *, validated_trajectory=False, proof_identity=None):
    """Only qualified accepted steering evidence can discharge a warning.

    A capacity bound cannot be overridden by an unbound success flag. This
    helper produces study labels only; no caller can publish/commit from it.
    """
    result=copy.deepcopy(record)
    if record["status"] == "CapacityLimitPredicted": return result
    if validated_trajectory and proof_identity:
        result["status"]="SteerableHazardNotCapacityLimited"
        result["evidence"]["accepted_trajectory_proof"]=proof_identity
        result["evidence"]["scope"]="Observed qualified trajectory only"
    return result
