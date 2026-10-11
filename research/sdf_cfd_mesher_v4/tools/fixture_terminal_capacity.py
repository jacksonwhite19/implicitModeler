"""Exact range receipts for preserved complete capacity fixture definitions."""
from fractions import Fraction
from pathlib import Path
import complete_capacity_fixtures as fixtures
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
VERSION='V4CompleteFixtureRangeBound/1'

def witness(spec,target,columns):
 k=spec['kind']
 if k=='box':bound=min(map(float,spec['half']));proof='inward box distance <= minimum half extent'
 elif k=='cylinder':bound=min(float(spec['radius']),float(spec['half_length']));proof='inward capped cylinder distance <= min(radius,half_length)'
 elif k=='spheres' and spec.get('cavity'):bound=max(map(float,spec['radii']));proof='max_i(radius_i-distance_i) <= max_i(radius_i)'
 else:return None
 if Fraction(float(target))<=Fraction(bound):return None
 p=dict(version=VERSION,source=fixtures.FixtureSDF(spec).source_identity,implementation=file_hash(Path(fixtures.__file__)),
        target=str(Fraction(float(target))),upper_bound=str(Fraction(bound)),columns=sorted(set(columns)),proof=proof,status='TargetLevelSetAbsent',scope='GlobalTerminalCapacity')
 return dict(payload=p,sha256=identity(p))
