"""Render full measured continuation evidence without inventing acceptance gates."""
import json
import capsule_sa_stabilization as c

def performance():
    samples=[json.loads(line) for line in (c.ROOT/'RESOURCE_SAMPLES.jsonl').read_text().splitlines()]
    result={}
    for name in c.NAMES:
        text=(c.ROOT/name/'continuation.resources').read_text()
        values=dict(line.strip().split(': ',1) for line in text.splitlines() if ': ' in line)
        result[name]=dict(gnu_time=values,sampled_peak_aggregate_RSS=max(r['cases'][name]['rss_bytes'] for r in samples),sampled_rank_counts=sorted(set(r['cases'][name]['ranks'] for r in samples)),executable=(c.ROOT/name/'EXECUTABLE.txt').read_text().strip(),exit=int((c.ROOT/name/'continuation.exit').read_text()),timing_limitation='Concurrent shared-machine runs; not a precise speed comparison')
    c.h.durable_json(c.ROOT/'PERFORMANCE.json',c.h.seal(result))

def render():
    out=['# Frozen GW1 steady SA continuation','',
         'NON_QUALIFYING continuation from the exact retained 500 state. No new model, mesh, or numerical treatment. Iteration means are diagnostic statistics, not accepted steady results.','']
    pre=c.h.read(c.ROOT/'PREFLIGHT.json')
    out+=['Starting HEAD: '+pre['starting_head'],'Prior result: '+pre['prior'],'', 'Frozen prism evidence:', '```json',json.dumps(pre['frozen'],indent=2),'```']
    for name in c.NAMES:
        folder=c.ROOT/name;m=c.h.read(folder/'MEASUREMENTS.json');con=c.h.read(folder/'CONTRACT.json')
        out+=['', '## '+name,'', 'Contract: '+con['identity'],'Candidate: '+con['candidate'],'Measurements: '+m['identity'],'','Administrative diff:','```diff',*con['diff'],'```','',
              'Command: `mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case pilot`','',
              'Resources:','```',(folder/'continuation.resources').read_text(),'```','',
              'Complete checkpoint extrema and canonical binary64 identities:','',
              '| iteration | field | min | max | negative | nonfinite | SHA-256 |','|---|---|---|---|---|---|---|']
        for step in sorted(m['checkpoints'],key=int):
            cp=m['checkpoints'][step]
            for field,v in cp.items():out.append(f"|{step}|{field}|{v['min']:.17g}|{v['max']:.17g}|{v['negative']}|{v['nonfinite']}|{v['sha256']}|")
        out+=['','Initial residual checkpoints:','','|iteration|p|Ux|Uy|Uz|nuTilda|','|---|---|---|---|---|---|']
        for step in sorted(m['residual_checkpoints'],key=int):
            res=m['residual_checkpoints'][step]
            out.append('|'+step+'|'+'|'.join(f"{res[eq]['initial']:.17g}" for eq in ('p','Ux','Uy','Uz','nuTilda'))+'|')
        for window,signals in m['windows'].items():
            out+=['','### '+window,'','All residuals are outer initial residuals. Force units N, moment units N m, angles unwrapped degrees. Slopes are per outer iteration. Population standard deviation.','',
                  '|signal|mean|median|std|min|max|range|slope|detrended range|','|---|---|---|---|---|---|---|---|---|']
            for key,v in signals.items():out.append('|'+key+'|'+'|'.join(f'{v[k]:.12g}' for k in ('mean','median','std','min','max','range','slope','detrended_range'))+'|')
        out+=['','Conservation and solver health:','```json',json.dumps({k:m[k] for k in ('conservation','linear_totals','bounds','maxiter','last')},indent=2),'```']
    decision=c.ROOT/'DECISION.json'
    if decision.exists():out+=['','## Decision','','```json',json.dumps(c.h.read(decision),indent=2),'```']
    out+=['','No production authority replaced. Run A = NO. Run A2 = NO. New SST/SST-LM/URANS/transient/SAS/DES/LES runs = 0. Prism changes = 0. Mesh changes = 0.','']
    (c.ROOT/'REPORT.md').write_text('\n'.join(out),newline='\n')

if __name__=='__main__':
    performance();render()
