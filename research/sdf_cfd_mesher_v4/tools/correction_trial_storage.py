"""Keep full rejected-search evidence on disk, not in a live history graph."""
import json
import os
import gzip
import io
from pathlib import Path
from tempfile import NamedTemporaryFile
from sdf_cfd_mesher_v4.common.authority import file_hash
import generalize_local_remesh as g


class GateArchive:
    def __init__(self, folder):
        self.folder = Path(folder)

    def record(self, gate):
        self.folder.mkdir(parents=True, exist_ok=True)
        # Encoder streaming avoids a second complete serialization string.
        with NamedTemporaryFile(mode='wb',dir=self.folder,suffix='.tmp',delete=False) as raw:
            temporary = Path(raw.name)
            with gzip.GzipFile(filename='',fileobj=raw,mode='wb',mtime=0) as compressed:
                with io.TextIOWrapper(compressed,encoding='utf8',newline='\n') as stream:
                    json.dump(g.s.r.w.clean(gate),stream,sort_keys=True,separators=(',',':'))
                    stream.write('\n')
        digest = file_hash(temporary)
        destination = self.folder/(digest+'.json.gz')
        os.replace(temporary, destination)
        return dict(passed=gate['passed'], sha256=digest, record='CORRECTION_TRIALS/'+destination.name,
                    scope=gate.get('scope'), production_acceptance=gate.get('production_acceptance'),
                    failure_count=len(gate.get('quality', {}).get('failures', [])))


class CompactGateArchive:
    """Rejected-trial diagnosis when a coordinate reproducer is stored separately.

    This record is not a full validation receipt and cannot admit a candidate.
    Fully validated candidates retain the original complete archive semantics.
    """
    def __init__(self,folder):self.folder=Path(folder)

    def record(self,gate):
        if gate.get('passed') and gate.get('production_acceptance') is not False:return GateArchive(self.folder).record(gate)
        self.folder.mkdir(parents=True,exist_ok=True)
        quality=gate.get('quality',{})
        record=dict(schema='V4RejectedTrialDiagnosis/1',passed=False,production_acceptance=False,
                    failures=quality.get('failures',[]),ownership=gate.get('ownership'),
                    vertex_links=gate.get('vertex_links'),
                    populations={k:len(quality.get(k,[])) for k in ('cells','faces','exact')},
                    scope='Compact diagnosis; coordinate reproducer required; not a complete hard-quality receipt')
        payload=(json.dumps(g.s.r.w.clean(record),sort_keys=True,separators=(',',':'))+'\n').encode()
        import hashlib
        digest=hashlib.sha256(payload).hexdigest();path=self.folder/(digest+'.json')
        path.write_bytes(payload)
        return dict(passed=False,sha256=digest,record='CORRECTION_TRIALS/'+path.name,
                    production_acceptance=False,failure_count=len(record['failures']),scope=record['scope'])
