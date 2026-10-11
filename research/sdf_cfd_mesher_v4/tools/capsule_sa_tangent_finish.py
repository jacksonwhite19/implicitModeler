"""Seal partial tangent qualification without claiming a physical Jacobian test."""
import hashlib
import json
import re
import shutil
import subprocess
import numpy as np
import capsule_sa_tangent as t
from capsule_sa_operator_finish import allocation as prior_allocation

CLASSIFICATION='V4CapsuleSteadySAPhysicalRotationalTangentStillNotQualified'
NEXT='Build and validate one coupled cell-velocity/face-flux/pressure linear tangent projector, including derivatives of HbyA, rAtU and the freestream mixed coefficients, with a boundary-consistent whole-mesh lift of the resolved rotational seed; then repeat the gated operator test.'


def seal():
    verification=t.verify()
    admission=t.h.read(t.ROOT/'ADMISSION.json')
    previous_extra=t.h.read(t.old.ROOT/'EXTRA_SOURCE_PROVENANCE.json')
    for path,item in admission['source_files'].items():
        digest=subprocess.check_output(['wsl','sha256sum',path],text=True).split()[0]
        assert digest==item['sha256']
        if path in previous_extra:assert digest==previous_extra[path]['sha256']
    verification.update(new_CFD_iterations=0,mesh_changes=0,prism_changes=0,L11_changes=0,
                        retained_field_mutations=0,source_reverification=True)
    t.h.durable_json(t.ROOT/'FINAL_VERIFICATION.json',t.h.seal(verification))
    contract=dict(
        classification=CLASSIFICATION,decision='F',
        seed='Active scalar y*dq/dx-x*dq/dy; vector ez cross U minus (-y,x,0) dot grad(U).',
        metric='rAU=1/A(relaxed UEqn); rAtU=1/max(1/rAU-H1,0.1/rAU), installed consistent SIMPLE algebra.',
        matrix='fvm::laplacian(rAtU,psi)==fvc::div(seedPhi), zero initial psi, retained corrected laplacian, one linear solve.',
        seed_flux='fvc::flux(seedU) with implicit linearized freestream U boundary and zero wall U.',
        correction='deltaPhi=seedPhi-projectionMatrix.flux(); candidate deltaU=seedU-rAtU*grad(psi), then reapply velocity boundary derivative.',
        pressure_boundary='Homogeneous mixed multiplier with retained pressure valueFraction; NOT the full inhomogeneous physical pressure derivative.',
        wall='Zero velocity and flux tangent; fixed nuTilda tangent zero; nut derivative evaluated by native Spalding boundary update.',
        solver=dict(solver='PCG',preconditioner='DIC',tolerance=1e-13,relTol=0,maxIter=5000),
        optimization='Frozen metric face-flux constraint projection only. Closest complete augmented-state tangent NOT established; no minimization of Jg.',
        constitutive='chi=N/nu, b=Cv1^3; nut=N*chi^3/(chi^3+b); dnut/dN=chi^3*(chi^3+4*b)/(chi^3+b)^2; Cv1=7.1.',
        wall_epsilon=[1e-3,1e-4,1e-5,1e-6],
        epsilon_basis='Dimensionless angular amplitudes, four logarithmic decades to separate centered truncation from roundoff; all interior perturbed N stay positive. No Jg epsilon study admitted.',
        statuses=dict(flux_projection='FrozenMetricConservativeFaceFluxProjectionValidated',
            generator='PhysicalRotationalGeneratorNotQualified',
            boundary='CompleteAugmentedBoundaryConsistencyNotQualified',
            physical_pressure='PhysicalPressureTangentNotQualified',
            reduced_SIMPLE='ReducedSIMPLETangentNotQualified',
            Jg='NotEvaluatedTangentAdmissionFailed'),
        no_new_convergence_rule=True,no_stopping_contract=True,no_CFD_qualification=True,
        next_task=NEXT)
    t.h.durable_json(t.ROOT/'CONTRACT.json',t.h.seal(contract))
    p=t.h.read(t.ROOT/'PROJECTION.json');m=t.h.read(t.ROOT/'LOW_MODES.json')
    files={str(x.relative_to(t.ROOT)):dict(sha256=t.h.file_hash(x),bytes=x.stat().st_size)
           for folder in ('outputs','build') for x in (t.ROOT/folder).rglob('*') if x.is_file()}
    for x in t.ROOT.glob('*.npz'):files[x.name]=dict(sha256=t.h.file_hash(x),bytes=x.stat().st_size)
    t.h.durable_json(t.ROOT/'ARRAY_MANIFEST.json',t.h.seal(dict(files=files)))
    # The proven Windows allocation helper is reused with its module root changed only in memory.
    import capsule_sa_operator_finish as af
    oldroot=af.o.ROOT
    af.o.ROOT=t.ROOT
    try:storage=af.allocation()
    finally:af.o.ROOT=oldroot
    storage.update(starting_free_bytes=admission['free_bytes'],ending_free_bytes=shutil.disk_usage(t.ROOT).free,reclaimed_bytes=0)
    t.h.durable_json(t.ROOT/'STORAGE.json',t.h.seal(storage))
    logs={name:(t.ROOT/(name+'.log')).read_text() for name in t.c.NAMES}
    performance={name:dict(linear_solves=re.findall(r'LINEAR_CONSTRAINT_ONLY ([^\n]+)',log),
        wall=re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): ([^\n]+)',log).group(1),
        user_seconds=float(re.search(r'User time \(seconds\): ([\d.]+)',log).group(1)),
        system_seconds=float(re.search(r'System time \(seconds\): ([\d.]+)',log).group(1)),
        time_maxRSS_kB=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',log).group(1)),
        MPI_ranks=6,exit_status=0) for name,log in logs.items()}
    for log in logs.values():assert 'Exit status: 0' in log and 'TANGENT_CONSTRAINT_AUDIT_COMPLETE' in log
    t.h.durable_json(t.ROOT/'PERFORMANCE.json',t.h.seal(performance))
    report=[f'# {CLASSIFICATION}',
        'Outcome F. The conservative face-flux projection works. The closest complete physical augmented tangent is not qualified. No new Jg interpretation is admitted.',
        f'Starting HEAD: {t.HEAD}. Ending HEAD: the task-owned commit containing this report (printed after commit).',
        'No retained CFD state was advanced, replaced or written. Only linear constraint matrices were solved. No production convergence rule, stopping contract or CFD qualification was created.',
        '## Authority and immutability',
        f"Prior result {admission['prior']}. All previous evidence, implementation and 5918 retained array files rehashed. Retained state files: {verification['retained_files']}; ordinary prior scratch input files: {verification['scratch_files']}. Unrelated dirty entries preserved: {verification['unrelated_count']}.",
        'The native utility reads the existing six-rank scratch snapshots. It calls setTime only to select 1000. It never increments time. Its sole solve call is constraint.solve(solver). It does not call SA::correct, momentumPredictor, correctPressure or field/runtime write. Binary diagnostic arrays go only to the new task directory.',
        'Explicit: new CFD iterations = 0; mesh changes = 0; prism changes = 0; L11 changes = 0; retained field mutations = 0. V3 unchanged.',
        'Iteration 900 was verified as reference input; projection evaluations use iteration 1000 only.']
    for name in t.c.NAMES:
        ca=admission['cases'][name]
        report += [f"{name} candidate: {ca['candidate']}.",json.dumps(ca['frozen'],sort_keys=True),
                   'Iteration-1000 canonical hashes: '+json.dumps({k:v['sha256'] for k,v in ca['checkpoints']['1000'].items()},sort_keys=True)]
    report+=['GW1 sizing: dd239298d7e671b70e1248120e6ebd51c25a52dd6c7013c10fbf3059d1eb0c90.',
             'Prism scientific: 48f39ddfae1f782f724321e0d8882f9454928c5c0c2898652220dcfa6db3f1a1. Qualification: 75d827a837748dfca5014b873a18f4076843752ef6789622565f6de6d2ba8e7b. Prism/L11 authority: 356e5a357bbbdf923f7dd3a2df0642632ba3497518b0305c44f372d6fec7f9b3.',
             '## Installed source and unchanged operators',
             'Foundation 13-cde978a97c93. foamRun SHA-256: '+admission['foamRun_sha256'],
             'R_U=(A_U U-b_U)/V using the installed unrelaxed steady operator; R_c=div(phi_retained); R_SA=(A_N N-b_N)/V; R_nut=nut_retained-nut(N). These definitions and the prior operator evidence remain unchanged. Pressure-correction, continuity and linear solver residuals remain distinct. No new residual or Jacobian arrays from an unqualified tangent are presented.']
    for path,item in admission['source_files'].items():report.append(f"- `{path}`: `{item['sha256']}` ({item['lines']} lines).")
    report+=['## Tangent and projection algebra',contract['seed'],contract['metric'],contract['matrix'],contract['correction'],
        'The fixed coefficient projection enforces one face-flux divergence constraint. It does not differentiate A, H, H1, the consistent rAtU coefficient or the complete HbyA/pressure coupling. Its multiplier psi is not physical delta p.',
        'Wall multiplier BC: zeroGradient. Farfield multiplier BC: mixed, refValue=0, refGrad=0, retained pressure valueFraction. Coupled processor conditions and addressing are unchanged. No adjustPhi was called: the retained pressure mixed boundary removes the pressure reference requirement. constrainPressure changes fixedFluxPressure patches only; neither physical patch has that class.',
        'PCG/DIC; tolerance 1e-13; relTol 0; maxIter 5000. These are constraint-solve settings, not edits to the scientific case. One solve per direction and deterministic replay. Frozen corrected-laplacian explicit term is evaluated from zero initial psi. Raw diagonal, upper/lower, source and boundary matrix coefficients are retained.',
        'For velocity, w=(Ui+Ub)/2; f=.5-.5*(w dot n)/|w|; b=Uinf-Ui; a=df/dw. The local homogeneous linearized implicit relation is (I-.5*b*a^T) deltaUb=(1-f)deltaUi+.5*b*(a dot deltaUi). It is solved analytically. This is the derivative of the implicit boundary relation, not one lagged updateCoeffs call.',
        'For pressure, fp=.5+.5*(Ub dot n)/|Ub|. Physical deltaPb=(1-fp)deltaPi+(pinf-Pi)deltafp. The second term is absent from the homogeneous projection multiplier BC. The reported defect tests this explicit missing term.',
        'Scalar freestream nuTilda uses the sign of phi (inletOutlet). Its derivative is branchwise fixed-value/zero-gradient away from zero flux; branch crossing was not qualified for a final augmented tangent. Wall delta nuTilda is exactly zero. Farfield calculated nut follows the algebraic constitutive assignment when SA correctNut is evaluated.',
        contract['constitutive'],
        'The Spalding wall nut derivative is evaluated with the actual installed boundary implementation, including its bounded Newton procedure, through centered U perturbations. It is not an independent rotation of retained nut. The retained constitutive lag is not removed from the scientific state.',
        '## Generator evidence',
        '| Seed | native U norm | unlimited U norm | limiter change norm | quadratic/native cosine | quadratic/unlimited cosine |',
        '|---|---:|---:|---:|---:|---:|']
    for name in t.c.NAMES:
        g=p[name]['generator'];report.append(f"| {name} | {g['native_norm']:.12g} | {g['unlimited_norm']:.12g} | {g['limiter_change_norm']:.12g} | {g['reconstruction_native_cosine']:.9g} | {g['reconstruction_unlimited_cosine']:.9g} |")
    report+=['Native and unlimited norms use volume-weighted RMS over all cells. Reconstruction cosines use the identical retained deterministic sample set and its volume weights. Removing the limiter does not remove reconstruction disagreement. The evidence does not uniquely separate interpolation, cell-scale variation and boundary effects; none is declared the sole cause.',
        'Quadratic finite-angle source evidence is reused after hash verification: eps=0.01,0.003,0.001 rad; polynomial axisymmetric artifacts were removed, but retained-field disagreement remained. No full-field reconstruction was promoted.',
        'Low modes: 492 rings, 256 angles, 125952 points per seed. Native FV centers, 48-cell weighted quadratic reconstruction. m=0,1,2 retained in evidence; m=1 tangent is inverseFFT(-i*m*qhat_m) in cylindrical components. Body volume weights and wake disk area weights are kept separate. No unsupported lift to unsampled box cells was invented.',
        '| Seed/region | m1 U tangent RMS | m1 p tangent RMS | m1 nuTilda tangent RMS |',
        '|---|---:|---:|---:|']
    for name in t.c.NAMES:
        for region in ('body','wake_0.5','wake_1','wake_2','wake_4','wake_6','wake_8'):
            report.append('| '+name+'/'+region+' | '+' | '.join(f"{m[name][k][region]['m1_tangent_norm']:.10g}" for k in ('U','p','nuTilda'))+' |')
        report.append('Sampling quality '+name+': '+json.dumps(m[name]['sampling'],sort_keys=True))
    report+=['## Conservative projection evidence',
        '| Seed/direction | before div RMS | after div RMS | after div Linf | correction/seed U norm | corrected/seed cosine |',
        '|---|---:|---:|---:|---:|---:|']
    for name in t.c.NAMES:
        for seed in ('native','unlimited'):
            q=p[name][seed];report.append(f"| {name}/{seed} | {q['before']['L2']:.12g} | {q['after']['L2']:.12g} | {q['after']['Linf']:.12g} | {q['correction_over_seed']:.10g} | {q['corrected_seed_cosine']:.10g} |")
    report+=['These are derivatives per unit angular seed amplitude; no arbitrary near-null threshold is applied. Divergence RMS uses cell volumes. The small post-projection divergence is measured, not zero by construction through frozen phi.',
        '| Seed/direction | farfield net deltaPhi | wall max deltaPhi | farfield flux/U mismatch max | pressure BC defect max | div(flux(corrected U)) RMS |',
        '|---|---:|---:|---:|---:|---:|']
    for name in t.c.NAMES:
        for seed in ('native','unlimited'):
            q=p[name][seed];report.append(f"| {name}/{seed} | {q['boundary']['farfield']['net']:.12g} | {q['boundary']['wall']['Linf']:.12g} | {q['boundary']['farfield']['cell_flux_mismatch_Linf']:.12g} | {q['pressure_BC_defect_Linf']:.12g} | {q['cell_flux_divergence']:.12g} |")
    report+=['Interior pressure-corrected phi need not equal flux(U) in a collocated SIMPLE discretization. Therefore the last column alone is not a failure criterion. The actual problem is that the complete derivative of the installed cell/face/pressure relation has not been established; the physical-boundary mismatch and missing pressure-BC term are directly measured gaps.',
        'Both seeds reproduce cell arrays exactly in the retained six-rank decomposition. Face arrays are canonicalized with signed global addressing. Coupled duplicates differ by at most 4.336808689942018e-19. This is not a claim of bitwise equivalence after repartitioning; no repartition was performed.',
        '## Constitutive and synthetic validation',
        '| Seed/epsilon | interior derivative relative error | wall derivative RMS | wall adjacent-epsilon array change |',
        '|---|---:|---:|---:|']
    for name in t.c.NAMES:
        for e in ('0.001','0.0001','1e-05','1e-06'):
            q=p[name]['wall_nut'][e];report.append(f"| {name}/{e} | {p[name]['SA_interior'][e]['relative_error']:.10g} | {q['RMS']:.10g} | {q.get('relative_array_change','n/a')} |")
        report.append('Actual-mesh validation '+name+': '+json.dumps(p[name]['validation'],sort_keys=True))
    report+=['The conservative actual-mesh validation uses retained phi/Uref, whose small existing divergence is measured; it is not falsely described as an exact analytic zero field. The deliberate linear-strain direction is non-conservative after discretization. Separate analytic tests cover axisymmetric zero derivative, scalar m1 phase, coordinate-plus-basis vector rotation, a discrete cycle-space divergence-free flux, and removal of imposed divergence.',
        '## Admission decision and unavailable evidence',
        json.dumps(contract['statuses'],sort_keys=True),
        'Outcome F. The face-flux constraint projection and constitutive sub-blocks are useful and reproducible. They do not establish the nearest boundary/SIMPLE-compatible augmented rotational tangent. No projection multiplier was relabelled physical pressure. No Jg-minimizing optimization was used.',
        'Not evaluated: qualified momentum/SA/continuity Jg; Jg epsilon convergence; admissible control-direction ratios; residual projection onto qualified Jg; conservative 900-to-1000 augmented change; regional/prism/near-body/wake/outer/farfield Jg shares; individual box-patch Jg; azimuthal operator-energy scan. Each is gated on tangent qualification. This prevents repeating the previous conditional-probe interpretation error.',
        'No result here establishes or rejects a physical near-null direction. No mathematically neutral eigenmode, eigenvalue, physical-time rotation, physical instability, square-box dominance or mesh-pinning dominance is proved. The prior nonzero steady residual evidence remains unchanged; its rotational/non-rotational physical partition is still unresolved.',
        '## Verification, cost and storage',
        'Focused plus prior SA/operator/rotation/mesh-authority/model regressions: 122 passed. Native projection replay: exact cell arrays and signed face arrays. No maxIter event in accepted linear solves. Two rejected utility development attempts are retained: mixed-field construction and a mutable lower-matrix accessor that changed matrix storage. Both were fixed; no retained field was changed.',
        'Focused rerun: 19 passed in 0.50 s. CRLF-aware whitespace check passed for authored source, reports and JSON. Exact raw OpenFOAM logs retain native trailing spaces and are explicitly excluded from formatting changes. The final whitespace-only native-source cleanup rebuilt to the identical executable hash.',
        'Native utility SHA-256: '+t.h.file_hash(t.ROOT/'build/bin/saTangentProjection')+'. Source SHA-256: '+t.h.file_hash(t.c.s.f.prior.control.V4/'tools/sa_tangent/saTangentProjection.C')+'.',
        json.dumps(performance,sort_keys=True),
        'The time-tool RSS is not sampled aggregate MPI RSS. Timings include shared-machine effects and are not solver-speed comparisons.',
        json.dumps(storage,sort_keys=True),
        'No full CFD case was duplicated. No prior evidence or task arrays were deleted. Raw projection matrices, directions, phi, source snapshots, low-mode samples, replay arrays, logs and reports are retained.',
        'Explicit: no convergence rule; no stopping contract; no CFD qualification; new SA iterations=0; new SST iterations=0; SST-LM runs=0; URANS runs=0; SAS runs=0; DES runs=0; LES runs=0; Run A=NO; Run A2=NO; production authority replaced=NO.',
        'Remaining uncertainty: the physical rotational generator and its coupled conservative boundary-compatible lift; pressure mapping; reduced SIMPLE derivative; admissible Jg; the cause of the remaining operator imbalance.',
        'Exactly one recommended next task: '+NEXT]
    (t.ROOT/'REPORT.md').write_text('\n\n'.join(report)+'\n')
    implementation={str(x.relative_to(t.c.s.f.prior.control.V4)):t.h.file_hash(x)
                    for x in (t.c.s.f.prior.control.V4/'tools').glob('capsule_sa_tangent*.py')}
    for x in (t.c.s.f.prior.control.V4/'tools/sa_tangent').rglob('*'):
        if x.is_file():implementation[str(x.relative_to(t.c.s.f.prior.control.V4))]=t.h.file_hash(x)
    test=t.c.s.f.prior.control.V4/'tests/test_capsule_sa_tangent.py';implementation[str(test.relative_to(t.c.s.f.prior.control.V4))]=t.h.file_hash(test)
    evidence={str(x.relative_to(t.ROOT)):t.h.file_hash(x) for x in t.ROOT.rglob('*') if x.is_file()
              and 'build' not in x.relative_to(t.ROOT).parts and 'outputs' not in x.relative_to(t.ROOT).parts
              and x.suffix!='.npz' and x.name not in ('RESULT.json',)}
    result=t.h.seal(dict(classification=CLASSIFICATION,starting_head=t.HEAD,evidence=evidence,implementation=implementation,
        contract=t.h.read(t.ROOT/'CONTRACT.json')['identity'],projection=p['identity'],low_modes=m['identity'],
        next_task=NEXT))
    t.h.durable_json(t.ROOT/'RESULT.json',result)
    print(result['identity'],t.h.file_hash(t.ROOT/'REPORT.md'),flush=True)


if __name__=='__main__':seal()
