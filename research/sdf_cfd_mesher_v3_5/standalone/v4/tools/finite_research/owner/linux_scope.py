"""Task-local Linux native-command broker and exact systemd-scope retirement.

This is operational custody only. It changes no meshing algorithms or gates.
The broker inherits the caller's Linux environment through systemd-run --scope.
Ordinary fork/exec/setsid descendants stay in its non-delegated cgroup.
"""
import hashlib
from decimal import Decimal
import hmac
import json
import os
from pathlib import Path
import re
import signal
import socketserver
import subprocess
import sys
import threading
import time
from wire import receive, send, MAX_CHUNK


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, sort_keys=True, allow_nan=False)
        stream.flush(); os.fsync(stream.fileno())


def properties(unit):
    if not re.fullmatch(r'v4-fixture-[0-9a-f]{32}\.scope', unit):
        raise ValueError('ExactOwnedUnitRequired')
    result = subprocess.run(['systemctl', '--user', 'show', unit,
        '-p', 'Id', '-p', 'ActiveState', '-p', 'LoadState', '-p', 'InvocationID',
        '-p', 'ControlGroup', '-p', 'KillMode', '-p', 'SendSIGKILL',
        '-p', 'RuntimeMaxUSec', '-p', 'TimeoutStopUSec', '-p', 'Delegate'],
        capture_output=True, text=True, timeout=4)
    if result.returncode: raise RuntimeError('ScopeInspectionFailed', result.stderr)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def process(pid):
    try:
        stat = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=int(pid), starttime=int(stat[19]), state=stat[0])
    except (FileNotFoundError, ProcessLookupError): return None


def cgroup_path(binding):
    name = binding['cgroup']
    if not name.startswith('/') or '..' in Path(name).parts or not name.endswith('/'+binding['unit']):
        raise ValueError('InvalidOwnedCgroup')
    return Path('/sys/fs/cgroup')/name.lstrip('/')


def inspect(binding):
    if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != binding['boot_id']:
        raise ValueError('LinuxBootChanged')
    props = properties(binding['unit']); path = cgroup_path(binding)
    if props.get('InvocationID') not in ('', binding['invocation_id']):
        raise ValueError('ScopeInvocationChanged')
    if props.get('ControlGroup') not in ('', binding['cgroup']):
        raise ValueError('ScopeCgroupChanged')
    if path.exists():
        if path.stat().st_ino != binding['cgroup_inode']: raise ValueError('ScopeCgroupReused')
        events = dict(line.split() for line in (path/'cgroup.events').read_text().splitlines())
        ids = []
        for file in [path/'cgroup.procs', *path.rglob('*/cgroup.procs')]:
            try: ids.extend(int(p) for p in file.read_text().split())
            except FileNotFoundError: pass
        members = [p for pid in sorted(set(ids)) if (p := process(pid)) is not None and p['state'] != 'Z']
        populated = events.get('populated') != '0'
    else: members = []; populated = False
    original = process(binding['broker']['pid'])
    original_live = (original is not None and original['starttime'] == binding['broker']['starttime'] and original['state'] != 'Z')
    return dict(properties=props, cgroup_exists=path.exists(), populated=populated,
        live_members=members, original_broker_live=original_live,
        retired=not populated and not members and not original_live)


def require_active(binding):
    result = inspect(binding); props = result['properties']
    if (props.get('ActiveState') != 'active' or props.get('InvocationID') != binding['invocation_id']
            or props.get('ControlGroup') != binding['cgroup'] or props.get('KillMode') != 'control-group'
            or props.get('SendSIGKILL') != 'yes' or props.get('Delegate') != 'no'
            or not result['original_broker_live']):
        raise ValueError('OwnedActiveScopeRequired', result)
    return result


def retire(binding, resources=None):
    started = time.monotonic(); before = inspect(binding); attempts = []
    memory_before = None; memory_error = None
    if resources is not None and before['cgroup_exists']:
        try: memory_before = memory(binding, *resources)
        except Exception as error: memory_error = repr(error)
    if not before['retired']:
        # A matching cgroup inode and systemd invocation must still own the work.
        props = before['properties']
        if props.get('InvocationID') != binding['invocation_id'] or props.get('ControlGroup') != binding['cgroup']:
            raise ValueError('IncompleteRetirementOwnership', before)
        try:
            result = subprocess.run(['systemctl', '--user', 'stop', binding['unit']],
                capture_output=True, text=True, timeout=6)
            attempts.append(dict(action='stop', returncode=result.returncode, stderr=result.stderr))
        except subprocess.TimeoutExpired: attempts.append(dict(action='stop', timed_out=True))
        after = inspect(binding)
        if not after['retired']:
            props = after['properties']
            if props.get('InvocationID') != binding['invocation_id'] or props.get('ControlGroup') != binding['cgroup']:
                raise ValueError('OwnershipChangedBeforeEscalation', after)
            result = subprocess.run(['systemctl', '--user', 'kill', '--kill-who=all', '--signal=SIGKILL', binding['unit']],
                capture_output=True, text=True, timeout=4)
            attempts.append(dict(action='kill_all_SIGKILL', returncode=result.returncode, stderr=result.stderr))
    until = time.monotonic()+4
    while True:
        after = inspect(binding)
        if after['retired']: break
        if time.monotonic() >= until: raise RuntimeError('OwnedLinuxProcessesRemain', after)
        time.sleep(.05)
    return dict(passed=True, before=before, after=after, attempts=attempts,
        memory_before_retirement=memory_before,memory_error=memory_error,
        seconds=time.monotonic()-started, verification='Exact invocation/cgroup inode; no live cgroup members or original broker.')


def lifetime(binding, seconds):
    if not 2 < seconds <= 14700: raise ValueError('ScopeLifetimeOutOfBounds')
    check = require_active(binding)
    value = check['properties'].get('RuntimeMaxUSec', '')
    multipliers = dict(us=1, ms=1000, s=1000000, min=60000000, h=3600000000)
    total = Decimal(0)
    for field in value.split():
        match = re.fullmatch(r'([0-9]+(?:\.[0-9]+)?)(us|ms|s|min|h)', field)
        if match is None: raise ValueError('UnrecognizedScopeLifetime', value)
        total += Decimal(match.group(1))*multipliers[match.group(2)]
    if total != Decimal(str(seconds))*1000000: raise ValueError('ScopeLifetimeReadbackMismatch', value, seconds)
    return dict(passed=True, requested_seconds=seconds, verified_microseconds=int(total),
        systemd_display=value, inspection=check)


def memory(binding, maximum, swap):
    """Read the authenticated cgroup's effective kernel limits and durable events."""
    check = inspect(binding)
    if not check['cgroup_exists']: raise ValueError('MemoryScopeMissing')
    path = cgroup_path(binding)
    actual = int((path/'memory.max').read_text())
    actual_swap = int((path/'memory.swap.max').read_text())
    if actual != maximum or actual_swap != swap or maximum <= 0 or swap != 0:
        raise ValueError('LinuxMemoryLimitReadbackMismatch', actual, actual_swap)
    events = {key:int(value) for key,value in (line.split() for line in (path/'memory.events').read_text().splitlines())}
    return dict(passed=True, memory_max_bytes=actual, swap_max_bytes=actual_swap,
        current_bytes=int((path/'memory.current').read_text()), events=events)


def serve(config_path):
    # No payload is admitted until an authenticated parent handshake. A stalled
    # startup must not leave an idle, unbound broker for a full fixture budget.
    startup = threading.Timer(12, lambda: os._exit(124)); startup.daemon = True; startup.start()
    config = read(config_path); root = Path(config_path).resolve().parent
    unit = config['unit']; props = properties(unit)
    control_secret=sys.stdin.buffer.readline(128).decode().strip()
    if hashlib.sha256(control_secret.encode()).hexdigest()!=config.get('delegation_control_sha256'):
        raise ValueError('OwnerControlSecretRequired')
    admission_lock=threading.RLock()
    admission=dict(token=config['token'],active=0,delegated=False,closed=False)
    own = process(os.getpid()); own_group = Path('/proc/self/cgroup').read_text().strip().split('::', 1)[1]
    if own_group != props.get('ControlGroup'): raise ValueError('BrokerOutsideOwnedScope')
    binding = dict(unit=unit, invocation_id=props['InvocationID'], cgroup=own_group,
        cgroup_inode=(Path('/sys/fs/cgroup')/own_group.lstrip('/')).stat().st_ino,
        broker=own, boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        uid=os.getuid(), distribution=os.environ.get('WSL_DISTRO_NAME'),
        token_sha256=hashlib.sha256(config['token'].encode()).hexdigest())
    require_active(binding)
    write(root/'BINDING.json', binding)
    limits = config.get('resources')
    if limits is not None:
        write(root/'MEMORY_START.json', memory(binding, limits['linux_memory_max_bytes'], limits['linux_swap_max_bytes']))

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(15)
            line = self.rfile.readline(65537)
            if len(line) > 65536: raise ValueError('NativeRequestHeaderTooLarge')
            header = json.loads(line)
            if header.get('op') in ('ping','control') and not hmac.compare_digest(str(header.get('token')), config['token']):
                self.wfile.write(b'{"error":"NativeScopeAuthenticationFailed"}\n'); return
            if header.get('op') == 'ping':
                startup.cancel()
                self.wfile.write((json.dumps(dict(binding=binding,monotonic_seconds=time.monotonic()))+'\n').encode()); return
            if header.get('op') == 'control':
                # Live inspections stay in the same capped scope. Only the
                # external retirement verifier must survive stopping this scope.
                action=header.get('action'); extra=header.get('extra',[])
                try:
                    if action in ('delegate','close_admission'):
                        message=dict(action=action,extra=extra,unit=config['unit'])
                        expected=hmac.new(control_secret.encode(),json.dumps(message,sort_keys=True).encode(),'sha256').hexdigest()
                        if not hmac.compare_digest(str(header.get('owner_signature')),expected):
                            raise ValueError('AuthenticatedOwnerAdmissionControlRequired')
                        with admission_lock:
                            if action=='delegate':
                                if admission['delegated'] or admission['closed'] or admission['active'] or len(extra)!=1:
                                    raise ValueError('ExclusiveNativeDelegationUnavailable')
                                membership=inspect(binding)
                                if (membership['retired'] or not membership['original_broker_live']
                                    or any(member['pid']!=binding['broker']['pid'] for member in membership['live_members'])):
                                    raise ValueError('BrokerOnlyMembershipRequiredForDelegation')
                                if not isinstance(extra[0],str) or not re.fullmatch('[0-9a-f]{64}',extra[0]):
                                    raise ValueError('FreshDelegatedAdmissionTokenRequired')
                                admission.update(token=extra[0],delegated=True)
                            else:
                                if extra or not admission['delegated']:
                                    raise ValueError('DelegatedAdmissionRequiredBeforeClose')
                                admission['closed']=True
                            result=dict(passed=True,active=admission['active'],closed=admission['closed'],delegated=admission['delegated'])
                    elif action=='inspect' and not extra: result=inspect(binding)
                    elif action=='lifetime' and len(extra)==1: result=lifetime(binding,float(extra[0]))
                    elif action=='memory' and limits is not None and not extra:
                        result=memory(binding,limits['linux_memory_max_bytes'],limits['linux_swap_max_bytes'])
                    else: raise ValueError('UnsupportedNativeInspection')
                except BaseException as error: result=dict(error=repr(error))
                self.wfile.write((json.dumps(result)+'\n').encode()); return
            with admission_lock:
                if admission['closed'] or not hmac.compare_digest(str(header.get('token')),admission['token']):
                    self.wfile.write(b'{"error":"NativeWorkloadAdmissionClosedOrDelegated"}\n'); return
                admission['active']+=1
            try:
                job = header.get('job')
                if not isinstance(job, str) or not re.fullmatch('[0-9a-f]{32}', job): raise ValueError('ExactJobRequired')
                folder = root/'jobs'/job
                if folder.resolve() != folder.absolute(): raise ValueError('IndirectNativeJob')
                request_bytes = (folder/'REQUEST.json').read_bytes()
                if hashlib.sha256(request_bytes).hexdigest() != header.get('request_sha256'):
                    raise ValueError('NativeRequestChangedAfterSubmission')
                request = json.loads(request_bytes); argv = request['argv']; cwd = request['cwd']
                if not isinstance(argv, list) or not argv or any(not isinstance(a, str) or '\0' in a for a in argv):
                    raise ValueError('NativeArgvRequired')
                if limits is not None:
                    checked = memory(binding, limits['linux_memory_max_bytes'], limits['linux_swap_max_bytes'])
                    if any(checked['events'].get(key,0) for key in ('max','oom','oom_kill','oom_group_kill')):
                        raise ValueError('NativeResourceFailureBeforeAdmission', checked)
                write(folder/'ADMITTED.json', dict(invocation_id=binding['invocation_id'], cgroup=own_group,
                    request_sha256=header['request_sha256']))
                self.connection.settimeout(None)
                with (folder/'STDOUT.bin').open('xb') as out, (folder/'STDERR.bin').open('xb') as err:
                    child = subprocess.Popen(argv, cwd=cwd, stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
                    write(folder/'STARTED.json', dict(pid=child.pid, identity=process(child.pid), argv=argv,
                        cgroup=own_group, original_child_of_broker=True))
                    lock = threading.Lock(); failures = []; stdin_bytes = [0]
                    def output(source, target, kind):
                        try:
                            while data := os.read(source.fileno(), MAX_CHUNK):
                                target.write(data)
                                with lock: send(self.connection, kind, data)
                        except (OSError, EOFError, ValueError) as error: failures.append('output:'+repr(error))
                        finally: source.close()
                    def input_stream():
                        try:
                            while True:
                                kind, data = receive(self.rfile)
                                if kind == b'F' and not data: break
                                if kind != b'I': raise ValueError('NativeStdinFrameRequired')
                                view = memoryview(data)
                                while view:
                                    count = child.stdin.write(view); view = view[count:]
                                stdin_bytes[0] += len(data)
                        except BrokenPipeError: pass  # Child may intentionally exit without consuming all stdin.
                        except (OSError, EOFError, ValueError) as error: failures.append('input:'+repr(error))
                        finally: child.stdin.close()
                    readers = [threading.Thread(target=output, args=(child.stdout, out, b'O'), daemon=True),
                               threading.Thread(target=output, args=(child.stderr, err, b'E'), daemon=True)]
                    for reader in readers: reader.start()
                    threading.Thread(target=input_stream, daemon=True).start()
                    code = child.wait()
                    for reader in readers: reader.join()
                result = dict(returncode=code, stdout_bytes=(folder/'STDOUT.bin').stat().st_size,
                    stderr_bytes=(folder/'STDERR.bin').stat().st_size, stdin_bytes=stdin_bytes[0],
                    transport_errors=failures)
                write(folder/'RESULT.json', result)
                with lock: send(self.connection, b'X', json.dumps(result).encode())

            finally:
                with admission_lock:
                    admission['active']-=1

    class Server(socketserver.ThreadingTCPServer):
        daemon_threads = True
        allow_reuse_address = False
    with Server(('127.0.0.1', config['port']), Handler) as server:
        write(root/'READY.json', dict(binding=binding, host='127.0.0.1', port=server.server_address[1]))
        server.serve_forever(poll_interval=.05)


if __name__ == '__main__':
    if sys.argv[1] == 'serve': serve(sys.argv[2])
    else:
        binding = read(sys.argv[2])
        if sys.argv[1] == 'memory': result = memory(binding, int(sys.argv[3]), int(sys.argv[4]))
        elif sys.argv[1] == 'lifetime': result = lifetime(binding, float(sys.argv[3]))
        elif sys.argv[1] == 'retire': result = retire(binding,tuple(map(int,sys.argv[3:5])) if len(sys.argv)>3 else None)
        else: result = {'inspect': inspect, 'retire': retire}[sys.argv[1]](binding)
        print(json.dumps(result), flush=True)
