"""Independent global audit plus explicit migration legacy-identity equality."""
import argparse,json
from pathlib import Path
from identity_qualification_audit import final

def run(checkpoints,output,migration):
    temporary=output.with_name(output.stem+'_population.json')
    final(checkpoints,temporary)
    result=json.loads(temporary.read_text());prior=json.loads(migration.read_text())
    exact=result['comparison']['legacy_scientific']==prior['legacy_scientific']
    if not exact:raise ValueError('BlockMigrationLegacyScientificMismatch')
    result.update(migration_legacy_exact=True,patricia_root=prior['legacy_root'],new_authority=prior['new_authority'])
    output.write_text(json.dumps(result,indent=2)+'\n');print('BLOCK_GLOBAL_AUDIT '+json.dumps(result),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('checkpoints',type=Path);p.add_argument('output',type=Path);p.add_argument('migration',type=Path)
    a=p.parse_args();run(a.checkpoints,a.output,a.migration)
