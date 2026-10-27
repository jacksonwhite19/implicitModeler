"""Preflight estimate only; never generates a mesh or authorizes Stage 2."""
import json,shutil,time
import numpy as np
import capsule_steady_models as s

def run():
    started=time.perf_counter();g,c,_,_=s.recovered();target=c['regional_refinement']['target'];fine=target/2
    x,t,seeds=s.f.prior.control.field_inputs();field=s.gw.ExactCachedSizingField(x[t],seeds,g=g['g'],h_max=g['h_max'])
    z=np.load(s.ROOT/'GC2-S1/W1/NETGEN.npz');n=len(z['tets']);ids=np.arange(0,n,64)
    centers=z['points'][z['tets'][ids]].mean(1);generic=np.array([field.evaluate(p) for p in centers])
    coarse=np.minimum(generic,[s.f.broad_cap(p,target) for p in centers]);refined=np.minimum(generic,[s.f.broad_cap(p,fine) for p in centers])
    factors=(coarse/refined)**3
    estimate=float(n*np.mean(factors));cells=estimate+371646
    oldcells=n+371646
    # Volume scaling is an estimate, not a guarantee for a constrained tetra generator.
    samples=[json.loads(line) for line in (s.ROOT/'RESOURCE_SAMPLES.jsonl').read_text().splitlines()]
    peak=max(row['cases']['GC2-S1']['rss_bytes'] for row in samples)
    out=s.h.seal(dict(schema='GC2FResourceEstimate/1',status='NOT_AUTHORIZED_PENDING_COMPLETED_STAGE1_EVIDENCE',
        target=fine,target_hex=fine.hex(),target_derivation='Exact binary64 sealed C2 target / 2',
        sample_definition='Every64th canonical GC2-S1 tetra centroid.64 is sampling bookkeeping only and does not enter sizing.',
        sample_count=len(ids),sample_ids_sha256=s.h.array_hash(ids),factors=s.f.statistics(factors),
        estimated_tetrahedra=estimate,estimated_total_cells=cells,cell_factor=cells/oldcells,
        sampled_current_solver_RSS=peak,estimated_pair_solver_RSS=2*peak*cells/oldcells,
        free_bytes=shutil.disk_usage(s.ROOT).free,wall_seconds=time.perf_counter()-started,
        method='Cell-count estimate weights each current tetra by cubed requested-size ratio. G1 remains exact. Baseline generic C2 field is everywhere >=G1 because its growth and cap are larger and seeds/distances identical. This permits min(G1,C2regional) for this analytical estimate.',
        limitations='Not a hard memory/runtime guarantee. Re-estimate from completed Stage1 timings and physical allocations before any optional generation. No mesh generated.'))
    s.h.durable_json(s.ROOT/'OPTIONAL_REFINEMENT_ESTIMATE.json',out);print(json.dumps(out))

if __name__=='__main__':run()
