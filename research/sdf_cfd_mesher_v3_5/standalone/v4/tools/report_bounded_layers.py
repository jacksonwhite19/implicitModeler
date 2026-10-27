"""Render sealed bounded-layer evidence without recomputing geometry."""
import argparse
import xml.etree.ElementTree as ET
from production_bounded_layers import *

def render():
    lines=['# Bounded L4–L5 qualification evidence','',
        'Quantile rows use min / p1 / p5 / median / p95 / p99 / max. Values are rounded to 12 significant digits for display. JSON authorities retain binary64 values. No diagnostic threshold is added.','']
    def emit(text=''):lines.append(text)
    def stats(value,prefix):
        if not isinstance(value,dict):return
        keys=('min','p1','p5','median','p95','p99','max')
        if all(k in value for k in keys):
            emit(f"| {prefix} | "+' / '.join(format(value[k],'.12g') for k in keys)+f" | {value.get('nonfinite','not recorded')} |")
        else:
            for k,v in value.items():stats(v,f'{prefix}.{k}')
    for layer in (4,5):
        p=OUT/f'L{layer}';q=read(p/'QUALIFIED.json');d=read(p/'A/DIAGNOSTICS.json')
        emit(f'## L{layer}');emit();emit(f"Classification: {q['scientific']['classification']}. Qualification identity: `{q['identity']}`.");emit()
        for name in ('ADMISSION','FINAL_GATE_SUMMARY','RECEIPT_AUDIT','RETENTION_RESULT','STORAGE_FINAL'):
            emit(f'### {name}');emit();emit('```json');emit(json.dumps(read(p/(name+'.json')),indent=2));emit('```');emit()
        for name in ('READY','FINAL','RESTORE','POST_RETENTION','CERTIFICATE'):
            emit(f'### A/{name}');emit();emit('```json');emit(json.dumps(read(p/'A'/(name+'.json')),indent=2));emit('```');emit()
        emit('### Distributions');emit();emit('| Measure | Quantiles | Nonfinite |');emit('|---|---|---|')
        stats(d,f'L{layer}');emit()
        emit('### Exact diagnostic evidence');emit();emit('```json');emit(json.dumps(d,indent=2));emit('```');emit()
    emit('## Rolling active-state analysis');emit();emit('```json');emit(json.dumps(read(OUT/'ACTIVE_RETIREMENT_ANALYSIS.json'),indent=2));emit('```')
    emit('## Measured scaling');emit();emit('```json');emit(json.dumps(read(OUT/'SCALING.json'),indent=2));emit('```')
    emit('## Final task storage census');emit();emit('```json');emit(json.dumps(read(OUT/'TASK_STORAGE_FINAL.json'),indent=2));emit('```')
    emit('## Tests');emit()
    for name in ('PIPELINE_TESTS.xml','RETENTION_PERSISTENCE_TESTS.xml','RESULT_TESTS.xml'):
        suite=ET.parse(OUT/name).getroot().find('testsuite')
        emit(name+': '+json.dumps(suite.attrib,sort_keys=True))
    emit('Two initial development test failures were corrected: restoration of shared driver configuration between tests, and creation of the replay-dedup journal directory. Their original XML evidence remains retained. The first scaling-report adapter attempted the later-layer restore filename for L2; it was corrected to read L2 RESOURCES.json. This reporting failure did not affect geometry, certification or checkpoints.')
    manifest=read(OUT/'CLEANUP_DEDUP_MANIFEST.json')
    emit('## Cleanup manifest');emit();emit('```json');emit(json.dumps({k:v for k,v in manifest.items() if k not in ('removals','replacements')},indent=2));emit('```')
    emit();emit('Complete path records: CLEANUP_DEDUP_MANIFEST.json. SHA-256: '+file_hash(OUT/'CLEANUP_DEDUP_MANIFEST.json'))
    emit();emit('## Remaining limits and next task');emit()
    emit('Complete AdvancingFrontQualified remains NO. L6–L11 and the core remain unqualified. L6 generation and post-L5 forecasting did not run. Read-only L6 initialization passed. Historical-only checkpoint objects and old diagnostic quality caches remain protected; zero checkpoint retirement was performed.')
    emit('Retained receipt chains still grow. Export retention and exact-content sharing control avoidable duplication, but do not bound active-prefix growth. The A/B process overlap differs from earlier studies; runtimes are measured observations, not normalized speedups. Temporary peaks are sampled lower bounds. L2 historical unique physical growth is unavailable.')
    emit('Recommended next task: QualifyAndCommitV4CapsuleLayer6Bounded. Limit the next chunk to one layer because transaction runtime, restore runtime, peak RSS and retained active allocation increased through L5. Repeat independent admission, certification, transactional commit, durable restore and qualified retention; stop on any failure. Do not start it in this task.')
    path=OUT/'REPORT.md';path.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return path

if __name__=='__main__':print(render())
