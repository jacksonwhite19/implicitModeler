"""Read-only running-case checks; no solver control changes."""
import json
import sys
import capsule_steady_models as s
if '--fine' in sys.argv:
    import capsule_steady_models_fine as fine
    fine.activate();names=fine.NAMES
else:
    s.activate();names=s.NAMES
import capsule_gw_status as status
import capsule_gw_health as health
health.f.ROOT=s.ROOT
if __name__=='__main__':
    ready=tuple(n for n in names if (s.ROOT/n/'pilot.log').exists() and (s.ROOT/n/'pilot.log').read_text().count('Time =')>=2)
    if ready:
        for name in ready:
            assert status.p.l.u.e.warning_audit((s.ROOT/name/'pilot.log').read_text()),name
        s.gw.NAMES=ready
        print(json.dumps(status.status()));health.inspect()
    else:
        print({n:[p.name for p in (s.ROOT/n).glob('*.json')] for n in names})
