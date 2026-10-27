"""Source-denial harness for standalone WSL Python backend validation."""
import os,sys
roots=('c:/users/jackson/desktop/02_projects/09b_implicit_cad_claude/','/mnt/c/users/jackson/desktop/02_projects/09b_implicit_cad_claude/')
blocked=tuple(r+p for r in roots for p in ('research/sdf_cfd_mesher_v3/','research/sdf_cfd_mesher_v4/','research/finite_cli_runs/','src/','shaders/'))
def check(value):
    if isinstance(value,(str,bytes,os.PathLike)):
        path=os.fsdecode(value).replace('\\','/').lower()
        if any(path.startswith(p) for p in blocked):raise PermissionError('OriginalSourceAccessForbidden:'+path)
def audit(event,args):
    if event=='open':check(args[0])
    if event=='subprocess.Popen':
        check(args[0]);command=args[1] if isinstance(args[1],(list,tuple)) else (args[1],)
        if any(p in os.fsdecode(x).replace('\\','/').lower() for x in command for p in blocked):raise PermissionError('OriginalSourceSubprocessForbidden')
sys.addaudithook(audit)
