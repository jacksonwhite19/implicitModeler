"""Consolidate explicit retention removals and qualified replay replacements."""
from production_bounded_layers import *
from path_preserving_deduplication import probe_file_id

def collect():
    removals=[];replacements=[]
    for layer in (4,5):
        folder=OUT/f'L{layer}';result=read(folder/'RETENTION_RESULT.json')
        assert result['passed']
        for row in read(folder/'CLEANUP_PLAN.json')['actions']:
            assert not Path(row['path']).exists()
            removals.append(dict(layer=layer,**row))
        for p in (folder/'REPLAY_DEDUP').glob('*.json'):
            r=read(p);assert r['stage']=='verified'
            assert file_hash(r['after']['path'])==r['original']['sha256']==r['canonical']['sha256']
            assert physical_key(metadata(r['after']['path']))==physical_key(metadata(r['canonical']['path']))
            assert probe_file_id(r['original']['file_id'])==result['deleted_file_control_error']
            replacements.append(dict(layer=layer,journal=str(p),journal_sha256=file_hash(p),**r))
    record=dict(removals=removals,replacements=replacements,removed_paths=len(removals),replaced_paths=len(replacements),
        removal_logical_bytes=sum(r['bytes'] for r in removals),removal_physical_reclaimed=sum(read(OUT/f'L{k}/RETENTION_RESULT.json')['physical_reclaimed'] for k in (4,5)),
        replacement_logical_bytes_shared=sum(r['original']['bytes'] for r in replacements),
        replacement_duplicate_physical_eliminated=sum(r['original']['cluster_bytes'] for r in replacements),
        deleted_measurement_controls=[dict(path=str(OUT/f'L{k}/recovery/retired_id_control'),bytes=len(b'qualified deleted-file identity control'),physical_reclamation_claim=0) for k in (4,5)],
        note='Explicit successful-retention and path-preserving replacement actions. Ordinary native writer scratch lifetimes are captured by sampled storage, not presented as retained-object reclamation.')
    save(OUT/'CLEANUP_DEDUP_MANIFEST.json',record)

if __name__=='__main__':collect()
