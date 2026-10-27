"""Recheck published first-layer geometry without expanding proof populations."""
import sys
sys.dont_write_bytecode=True
import json,sqlite3,time
from contextlib import closing
from first_layer_determinant_trace import OUT,g,production
from run_geometry_family import AnalyticProvider
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import file_hash


def run():
    records=[]
    for spec in production.cohort():
        label='B' if spec['name']=='separated_regions_production' else 'A'
        directory=OUT/spec['name']/label/'checkpoints';store=NativeCheckpointStore(directory)
        found=[]
        for path in sorted(directory.glob('NATIVE_*.json')):
            envelope=json.loads(path.read_text());components=envelope['components']
            table=store._object(components['quality'])
            with closing(sqlite3.connect(table.resolve().as_uri()+'?mode=ro',uri=True)) as db:
                row=db.execute('SELECT value FROM receipts WHERE key=?',('first_prism_incidence',)).fetchone()
            if row:found.append((path,components,json.loads(row[0])))
        if len(found)!=1:raise ValueError('FirstPrismCheckpointPopulation:'+spec['name'])
        path,components,receipt=found[0];tick=time.perf_counter()
        state={key:json.loads(store._object(components[key]).read_text())
               for key in ('vertices','cells','faces','outer','versions')}
        state['quality']={'first_prism_incidence':receipt}
        provider=AnalyticProvider(spec,OUT/spec['name']/'post_commit_audit')
        assert provider.validate_initial_commit(state)
        records.append(dict(case=spec['name'],checkpoint=path.name,checkpoint_sha256=file_hash(path),
            post_commit_passed=True,metric_minimum=receipt['minimum'],seconds=time.perf_counter()-tick,
            scope='Hash-verified published geometry and receipt; certificate population remains in its native immutable table'))
        g.save(OUT/'POST_COMMIT.json',records)

if __name__=='__main__':run()
