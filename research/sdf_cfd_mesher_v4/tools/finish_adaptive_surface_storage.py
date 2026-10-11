"""Remove only byte-identical task scratch with retained scientific replacement."""
import json, shutil
from production_adaptive_surface import OUT,sha,save

def main():
    path=OUT/'CLEANUP.json';record=json.loads(path.read_text())
    if record.get('post_run_deleted'):raise RuntimeError('CleanupAlreadyRecorded')
    protected=set(json.loads((OUT/'fresh/SURFACE_AUTHORITY.json').read_text())['files'])
    rows=[]
    for name in ('raw.mesh','metric.bin','initial_raw.off'):
        candidate=(OUT/'replay'/name).resolve();replacement=(OUT/'fresh'/name).resolve()
        if not candidate.is_relative_to(OUT.resolve()) or not replacement.is_relative_to(OUT.resolve()):raise ValueError('CleanupPathEscape')
        if name in protected:raise ValueError('ProtectedSurfaceArtifact')
        if not candidate.is_file() or sha(candidate)!=sha(replacement):continue
        rows.append(dict(path=str(candidate),bytes=candidate.stat().st_size,sha256=sha(candidate),
            replacement=str(replacement),replacement_sha256=sha(replacement),
            scientific_value='Raw pre-projection mesh, metric contract or extraction export; byte-identical first-run evidence retained.',
            reason='Duplicate task-created scratch. Both final surfaces, all quality/certificate arrays and both run histories remain.',
            active_authority_reference=False))
    record['post_run_deleted']=rows;record['bytes_deleted']=sum(r['bytes'] for r in rows)
    save(path,record) # preserve disposition and hashes before deletion
    for row in rows:
        from pathlib import Path
        Path(row['path']).unlink()
    record['ending_free_bytes']=shutil.disk_usage(OUT).free
    record['retained_task_bytes_at_cleanup']=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file())
    record['task_created_bytes_at_cleanup']=record['retained_task_bytes_at_cleanup']+record['bytes_deleted']
    save(path,record);print(json.dumps(record,indent=2))

if __name__=='__main__':main()
