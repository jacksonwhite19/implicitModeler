"""Describe the already measured cohort; no geometry or SDF evaluation."""
from pathlib import Path
import sys,json
import numpy as np
from scipy.stats import spearmanr,rankdata
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
from sdf_cfd_mesher_v4.forecast.detailed.scientific import plain,scientific_hash
OUT=ROOT/'results/task2'
DESCRIPTORS=['edge_ratio_max','triangle_area_ratio','neighbor_normal_angle_max_degrees','section_curvature_max','successive_direction_change_max_degrees','sdf_residual_abs_max','normal_convergence_max','step_length_max']
DIRECT=['sigma','G_bound','determinant','weight','skewness','symmetric_volume_ratio','aspect','nonorthogonality','side_normal_separation_degrees','jacobian','volume']
def decode(v):
    if isinstance(v,dict) and set(v)=={'nonfinite'}:return float(v['nonfinite'])
    if isinstance(v,dict):return {k:decode(x) for k,x in v.items()}
    if isinstance(v,list):return [decode(x) for x in v]
    return v
def read(name):return decode(json.loads((OUT/name).read_text()))
def write(name,value):(OUT/name).write_text(json.dumps(plain(value),indent=2,sort_keys=True,allow_nan=False)+'\n')
def failures(r):
    gates={'volume':r['volume']<=0 or r['signed_volume']<=0,'jacobian':r['jacobian']<=0,'determinant':r['determinant']<.0011,'weight':r['weight']<.055,'skewness':r['skewness']>3.63636,'symmetric_volume_ratio':r['symmetric_volume_ratio']<.01,'aspect':r['aspect']>1000}
    gates['undefined_metric']=not all(np.isfinite(r[k]) for k in ('volume','signed_volume','jacobian','determinant','weight','skewness','symmetric_volume_ratio','aspect'))
    return [k for k,v in gates.items() if v]
def correlation(x,y):
    valid=np.isfinite(x)&np.isfinite(y);x=x[valid];y=y[valid]
    return dict(count=int(valid.sum()),spearman=float(spearmanr(x,y).statistic) if len(np.unique(x))>1 and len(np.unique(y))>1 else None)
def main():
    rows=read('DIRECT_PRISM_ROWS.json');n=len(rows);bad=np.array([bool(failures(r)) for r in rows]);layer=np.array([r['layer'] for r in rows])
    data={k:np.asarray([r[k] for r in rows]) for k in DESCRIPTORS+DIRECT};examples=[]
    for desc in DESCRIPTORS:
        x=data[desc];rank=rankdata(x);badids=np.flatnonzero(bad);goodids=np.flatnonzero(~bad)
        lo=badids[np.lexsort((badids,x[badids]))[0]];hi=goodids[np.lexsort((goodids,-x[goodids]))[0]]
        def entry(i):return dict(row=rows[i],descriptor_rank=float(rank[i]),cohort_count=n,measured_gate_failures=failures(rows[i]))
        examples.append(dict(descriptor=desc,lowest_descriptor_among_direct_failures=entry(lo),highest_descriptor_among_measured_gate_passes=entry(hi)))
    redundancy=[dict(a=a,b=b,**correlation(data[a],data[b])) for i,a in enumerate(DESCRIPTORS) for b in DESCRIPTORS[i+1:]]
    stratified=[]
    for k in range(1,21):
        use=layer==k
        for desc in DESCRIPTORS:
            for metric in DIRECT:stratified.append(dict(layer=k,descriptor=desc,metric=metric,**correlation(data[desc][use],data[metric][use])))
    cells=read('DIRECT_PRISM_ROWS.json');faces=read('DIRECT_FACE_ROWS.json')
    nonfinite=dict(cells={k:sum(not np.isfinite(r[k]) for r in cells) for k in DIRECT},faces={k:sum(not np.isfinite(r[k]) for r in faces) for k in ('weight','skewness','symmetric_volume_ratio','nonorthogonality')})
    write('COUNTEREXAMPLES.json',examples);write('DESCRIPTOR_REDUNDANCY.json',redundancy);write('LAYER_STRATIFIED_ASSOCIATIONS.json',stratified);write('NONFINITE_METRICS.json',nonfinite)
    record=dict(source_rows_sha256=__import__('hashlib').sha256((OUT/'DIRECT_PRISM_ROWS.json').read_bytes()).hexdigest(),output_scientific_hash=scientific_hash(dict(examples=examples,redundancy=redundancy,stratified=stratified,nonfinite=nonfinite)),measured_internal_scope_passing_cells=int((~bad).sum()),cells_with_measured_failures_or_undefined_metrics=int(bad.sum()))
    write('DESCRIPTIVE_ANALYSIS.json',record);print(json.dumps(record))
if __name__=='__main__':main()
