"""Bounded private development using the existing guarded evidence owners."""
from pathlib import Path
import sys,json,hashlib,subprocess,importlib.util
from contextlib import ExitStack
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;V4=HERE.parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
from immutable_state_access import guard,token
from path_preserving_deduplication import durable_json
from sdf_cfd_mesher_v4.common.authority import identity
from owned_runtime import verify_release
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def git(*args):return subprocess.check_output(['git','--no-optional-locks',*args],cwd=ROOT,text=True).rstrip('\r\n')
def publish(p,value):
 assert not p.exists();authored=identity(value);durable_json(p,value)
 with guard(p):assert identity(read(p))==authored;return sha(p)
def bounded(label,command,seconds,pins):
 support=Path(__file__).resolve().parent/'repair_support.py'
 pins={Path(p):h for p,h in pins.items()};pins[support]=sha(support)
 with ExitStack() as held:
  for p,h in pins.items():held.enter_context(guard(p));assert sha(p)==h
  before={p:token(p) for p in pins};spec=importlib.util.spec_from_file_location('existing_bound',support);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
  with (HERE/(label+'.stdout')).open('x',encoding='utf-8') as out,(HERE/(label+'.stderr')).open('x',encoding='utf-8') as err:
   r=m.bounded(command,seconds,out,err)
  assert all(sha(p)==h and token(p)==before[p] for p,h in pins.items())
  h=publish(HERE/(label+'_EXECUTION.json'),dict(**r,input_pins={str(p):h for p,h in pins.items()},before_after_equal=True,all_guards_released=False))
 publish(HERE/(label+'_EXECUTION_CLOSURE.json'),dict(receipt_sha256=h,all_guards_released=True));print(r,flush=True);return r
def baseline():
 release=verify_release()
 publish(HERE/'BASELINE.json',dict(head=git('rev-parse','HEAD'),branch=git('branch','--show-current'),index_sha256=sha(ROOT/'.git/index'),status=git('status','--porcelain=v1').splitlines(),sources=release['sources'],source_inventory_identity=release['source_inventory_identity'],release_sha256=sha(V4/'runtime/RELEASE.json'),authorization='Human explicitly authorizes sustained generic shape-transition tool implementation and CLI connection, bounded tests on existing prisms, any needed agents/tools, until complete or majorblocker. Shape improvement independent of sizegrowth. All existing gates/geometry/ownership/custody remain. No shape dispatch, V3/kernel edit, cleanup or unrelated staging. Private development proposals may fail; only reviewed selected runtime can run dependent production.'))
def wsl(p):return '/mnt/c/'+str(Path(p).resolve()).replace('\\','/')[3:]
if __name__=='__main__':baseline()
