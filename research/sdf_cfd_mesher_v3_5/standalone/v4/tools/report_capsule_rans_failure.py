"""Seal the stopped startup attempt without changing controls or retrying it."""
import json
import shutil
import subprocess
from pathlib import Path
import capsule_rans_first_aa as c
import capsule_rans_checks as k

def main():
    h=c.m.h;o=c.OUT
    contract=h.read(o/'CONTRACT.json');h.verify_record(contract)
    authority=h.read(c.m.OUT/'AUTHORITY.json')
    for name,digest in authority['openfoam']['members'].items():
        assert h.file_hash(c.m.CORE/'foundation/openfoam_case/constant/polyMesh'/name)==digest
        if name!='boundary':assert h.file_hash(o/'A/constant/polyMesh'/name)==digest
    for name,digest in contract['controls'].items():assert h.file_hash(o/'A'/name)==digest
    adapter=h.read(o/'MESH_ADAPTER.json');h.verify_record(adapter)
    assert h.file_hash(o/'A/constant/polyMesh/boundary')==adapter['adapted_boundary']
    result=k.startup_result((o/'RunA.log').read_text(),(o/'RunA.resources').read_text())
    assert result['fatal'] and result['iterations']==0 and not (o/'A2').exists()
    pre=h.read(o/'PREFLIGHT.json')
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    unrelated=[s for s in status if not any(x in s for x in ('capsule_rans_first_aa','capsule_rans_checks','report_capsule_rans_failure'))]
    assert unrelated==pre['status']
    size=lambda p:sum(f.stat().st_size for f in p.rglob('*') if f.is_file())
    report=h.seal(dict(schema='V4CapsuleRANSStartupFailure/1',**result,
        starting_head='d842be0c91310501dfb846fb477a88b6cfe932b8',mesh=authority['identity'],
        contract=contract['identity'],adapter=adapter['identity'],canonical_mesh_unchanged=True,
        case_control_hashes=contract['controls'],physical=contract['physical'],turbulence=contract['turbulence'],
        audit_correction='CASE_AUDIT.json passed=true was a false positive: function registration was not validated. Runtime audit fails. Preserve original evidence.',
        stopping_status='Prospectively frozen engineering proposal, not numerically qualified. No trajectory judged.',
        command='mpirun -np 6 foamRun -solver incompressibleFluid -parallel -case A',
        checkMesh='Mesh OK.',foundation='13 / 13-cde978a97c93',
        executable_sha256='f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e',
        run_a=dict(exit_status=1,wall_seconds=.78,launcher_user_cpu=.02,launcher_system_cpu=.01,
            launcher_peak_rss_KiB=26444,aggregate_rank_cpu_rss='not measured',outer_iterations=0,linear_iterations=0,
            filesystem_input_blocks=272,filesystem_output_blocks=16528,
            wrapper_exit_warning='RunA.exit contains 0 due shell expansion; time footer and MPI report exit 1.',
            flux=None,forces=None,moments=None,terminal_fields=None,
            warnings=['Removing patchGroup wall which clashes with patch 0 of the same name'],
            failure='Unknown function type fieldMinMax'),
        run_a2='Not run: Run A failed',aa='Not evaluated',numerical_authority=None,
        retained_file_bytes=size(o),case_a_file_bytes=size(o/'A'),starting_free_bytes=pre['free_bytes'],
        ending_free_bytes=shutil.disk_usage(o).free,deleted_intermediate_bytes=0,
        allocation_note='Logical file lengths, not physical allocated blocks. No terminal or intermediate solution times exist.',
        epoch_renewal=False,unrelated_dirty_entries=len(unrelated),v3_unchanged=True,
        evidence={p.name:h.file_hash(p) for p in o.iterdir() if p.is_file() and p.suffix in ('.log','.resources','.txt','.xml','.json') and p.name not in ('REPORT.json','FAILURE.json')},
        unqualified=['RANS convergence','conservation','force stabilization','A/A reproducibility','spatial convergence','numerical discretization error','domain independence','experimental validation'],
        next_task='Qualify Foundation 13 monitor registration and stopping integration, then run a fresh capsule A/A from original initial fields under a newly audited contract.'))
    h.durable_json(o/'FAILURE.json',report)
    lines=['# Capsule RANS startup failure',report['classification'],'',
        'Run A stopped before iteration 1. No controls were changed after launch. No retry or A2 occurred.',
        'The original case audit was incomplete and falsely passed unsupported fieldMinMax. The corrected runtime audit fails.',
        'Canonical mesh members and frozen case controls rehashed unchanged. Adapted checkMesh: Mesh OK.',
        'Wall type and physicalType changed only on the ordinary case-local copy. No hardlinks.',
        'No numerical-case qualification authority is issued. CONTRACT.json is a frozen proposal, not acceptance evidence.',
        '', '## Exact evidence', '```json',json.dumps(report,indent=2),'```',
        '', '## Provenance and limits',
        'A: strict PCG/DIC controls are retained as a compatible symmetric pressure solver mechanism; the potential-flow qualification does not qualify RANS.',
        'B: installed motorBikeSteady supplies steady SST algorithm, relaxation, transport solvers and rank decomposition. airFoil2D and boundary source supply freestream and spatial schemes. Source snapshots are retained.',
        'C: physical inputs are user-specified. NASA Wind-US SST convention omega=10 U/Lref, nut=0.001 nu, k=omega nut is explicitly adopted with capsule length 0.16 m. It is not measured ambient turbulence.',
        'C: force and moment budgets were chosen prospectively. A 50-step write cadence alone is not evidence of a sufficient stabilization window. Conservation thresholds and their RANS applicability require further validation before qualification.',
        'D: no converged capsule measurements exist.',
        'E: fieldMinMax was unsupported and is rejected. No successful complete case audit can be claimed.',
        'The registry exposes volFieldValue as a possible future replacement. It has not been substituted or qualified here.',
        'The full stopping monitor integration and max-linear-iteration detection have not been demonstrated. Tests exercise helper logic only.',
        '', '## Frozen controls (including failed monitor)',]
    for name in contract['controls']:
        lines.extend(['### '+name,'SHA-256: '+contract['controls'][name],'```', (o/'controls'/name).read_text(),'```'])
    (o/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(identity=report['identity'],failure_sha256=h.file_hash(o/'FAILURE.json'),report_sha256=h.file_hash(o/'REPORT.md'),storage=report['retained_file_bytes'],case=report['case_a_file_bytes'],free=report['ending_free_bytes'],turbulence=contract['turbulence']),indent=2))

if __name__=='__main__':main()
