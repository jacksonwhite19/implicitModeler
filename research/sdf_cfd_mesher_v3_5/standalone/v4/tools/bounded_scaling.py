"""Measured historical and current scaling; missing fields remain missing."""
from production_bounded_layers import *

def collect():
    result={}
    for layer in (2,3,4,5):
        p=V4/'results'/f'production_adaptive_capsule_layer{layer}' if layer<4 else OUT/f'L{layer}'
        f=read(p/'A/FINAL.json');d=read(p/'A/DIAGNOSTICS.json');c=read(p/'A/CERTIFICATE.json')
        r=dict(source=str(p),times=f['times'],peak_rss=f['peak_rss'],retained_rss=f.get('retained_rss'),
            required_pairs=c['required_pairs'],reused_pairs=c['reused'],fresh_pairs=c['new'],
            restore_seconds=read(p/'RESOURCES.json')['A_restore'] if layer==2 else read(p/'A/RESTORE.json')['seconds'],
            quality=d['quality'],fronts=d.get('fronts',dict(edges=d.get('edges'),triangles=d.get('triangles'))),conditioning=d['conditioning'])
        if layer==2:
            r.update(new_unique_physical=None,new_unique_physical_note='Not measured in original L2 task; logical retained size is not a substitute.',
                receipt_delta_bytes=4858589184,receipt_delta_note='Historical intermediate table; final publication was flat.',retained_active_physical=None)
        elif layer==3:
            storage=read(p/'STORAGE_FINAL.json')
            r.update(new_unique_physical=storage['new_unique_physical_bytes'],receipt_delta_bytes=6048980992,
                retained_active_physical=12919132160,retained_active_note='Sealed L3 rolling-retention study measurement; scope includes diagnostic quality cache.')
        else:
            storage=read(p/'STORAGE_FINAL.json');receipts=read(p/'RECEIPT_AUDIT.json')
            r.update(new_unique_physical=storage['new_unique_physical_before_retention'],
                retained_new_unique_physical=storage['new_unique_physical_retained'],
                receipt_delta_bytes=receipts['delta_bytes'],retained_active_physical=storage['active_current_physical'],
                receipt_publication_seconds=f['receipt_publication_seconds'],retention_reclaimed=storage['physical_reclaimed'])
        result[f'L{layer}']=r
    save(OUT/'SCALING.json',dict(layers=result,note='Historical schedules and process concurrency differ. These are observed costs, not normalized benchmarks or extrapolations. Missing historical allocation stays null.'))

if __name__=='__main__':collect()
