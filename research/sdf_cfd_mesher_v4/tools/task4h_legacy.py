"""Pinned read-only baseline for repeatable before/after profiling."""
from task4h_common import *
import types
REF='cf27a74f537b66b6849f240687bb5d279b674dfc'
NAME='src/sdf_cfd_mesher_v4/advancing_front/persistent_planning.py'
source=subprocess.check_output(['git','show',REF+':research/sdf_cfd_mesher_v4/'+NAME],cwd=REPO)
assert hashlib.sha256(source).hexdigest()==read(OUT/'PREFLIGHT.json')['source_before'][NAME]
module=types.ModuleType('sdf_cfd_mesher_v4.advancing_front._task4h_frozen_planner')
module.__package__='sdf_cfd_mesher_v4.advancing_front';module.__file__=str(ROOT/NAME)
exec(compile(source,'Task4H-pinned-planner-'+REF,'exec'),module.__dict__)
SpotPlanner=module.SpotPlanner
SpatialQueries=module.SpatialQueries
