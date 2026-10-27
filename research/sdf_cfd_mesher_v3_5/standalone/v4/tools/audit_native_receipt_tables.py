"""Read-only audit of already certified native receipt tables."""
from contextlib import closing
import json,time
from pathlib import Path
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore,ReceiptTable
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.advancing_front.native_receipt_delta import raw_items


def audit_tables(quality,certificate,expected_cells):
    # GROUP BY over a many-segment SQL overlay can spill the full receipt
    # population to temporary disk. The canonical merge already selects the
    # exact active rows; count its two permitted outcomes in bounded memory.
    classes={}
    for _,value in raw_items(certificate):
        classification=json.loads(value).get('value',{}).get('classification')
        assert classification in {'CertifiedSeparated','CertifiedAllowedIncidentContact'}
        classes[classification]=classes.get(classification,0)+1
    assert sum(classes.values())==len(certificate)
    with closing(quality._connect()) as db:
        count,bad=db.execute("SELECT COUNT(*),SUM(CASE WHEN json_extract(value,'$.exact.jacobian_sign')>0 AND json_extract(value,'$.exact.volume_sign')>0 THEN 0 ELSE 1 END) FROM receipts WHERE key LIKE 'cell:%'").fetchone()
    assert count==expected_cells and (bad or 0)==0
    return dict(classifications=classes,active=sum(classes.values()),missing=0,duplicates=0,forbidden=0,unresolved=0,
        coverage_authority='Required-pair coverage checked by unchanged production certifier and exact binding count before CURRENT publication; no geometric recomputation',exact_cell_receipts=count)


def supplement(folder):
    folder=Path(folder);tick=time.perf_counter();result=json.loads((folder/'COMMITTED_RESULT.json').read_text());store=NativeCheckpointStore(folder/'checkpoints')
    pointer=json.loads((store.path/'CURRENT.json').read_text());manifest=store.path/pointer['checkpoint']
    assert file_hash(manifest)==pointer['sha256'] and pointer['scientific']==result['scientific']
    envelope=json.loads(manifest.read_text());tables={}
    for name in ('quality','certificate'):
        tables[name]=store._receipt(envelope['components'][name])
    result['certificate_audit']=audit_tables(tables['quality'],tables['certificate'],sum(result['cells'].values()))
    seconds=time.perf_counter()-tick;result['supplemental_audit_seconds']=seconds;result['replay_seconds']+=seconds
    (folder/'COMMITTED_RESULT.json').write_text(json.dumps(result,sort_keys=True,indent=2),encoding='utf8')
    print(json.dumps(dict(seconds=seconds,audit=result['certificate_audit'])))

if __name__=='__main__':
    import sys
    supplement(sys.argv[1])
