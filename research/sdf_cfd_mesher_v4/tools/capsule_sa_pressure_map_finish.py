"""Seal the pressure-map evidence and the failed physical-tangent admission."""
import json
import re
import shutil
import subprocess
from pathlib import Path
import capsule_sa_pressure_map as m

CLASSIFICATION='V4CapsuleSteadySAExactFoundationPressureCorrectionTangentStillNotQualified'
NEXT='Construct a direct linearized retained-state boundary/pressure closure, including mixed-boundary update state and pressure-matrix derivatives, and qualify its physical tangent before any rotational operator test.'


def seal():
    verify=m.verify();admission=m.h.read(m.ROOT/'ADMISSION.json')
    for name in m.c.NAMES:
        m.c.audit(name)
        assert m.c.canonical(m.c.ROOT/name/'pilot',1000)==admission['cases'][name]['checkpoints']['1000']
    sources=m.h.read(m.ROOT/'SOURCES.json')
    for path,digest in sources.items():
        if path=='identity':continue
        assert subprocess.check_output(['wsl','sha256sum',path],text=True).split()[0]==digest
        assert m.h.file_hash(m.ROOT/'sources'/Path(path).name)==digest
    verify.update(L11_changes=0,V3_unchanged=True,source_hashes_reverified=True)
    m.h.durable_json(m.ROOT/'FINAL_VERIFICATION.json',m.h.seal(verify))
    contract=dict(classification=CLASSIFICATION,outcome='F',
        map='Retained-state relaxed UEqn assembly followed by installed consistent-SIMPLE pressure correction, without momentum predictor solve or SA solve; temporary fields only.',
        equations=['rAU=1/A','HbyA=constrainHbyA(rAU*H,U,p)','rAtU=1/max(1/rAU-H1,0.1/rAU)',
            'phiHbyA=flux(HbyA)+interpolate(rAtU-rAU)*snGrad(p)*magSf',
            'HbyA=HbyA-(rAU-rAtU)*grad(p)','laplacian(rAtU,p)=div(phiHbyA)-sourceProxy(p)',
            'phiCorrected=phiHbyA-pEqn.flux()','UCorrected=HbyA-rAtU*grad(p)'],
        pressure='p is the solved temporary pressure field; pChange=pOutput-pInput. Neither is a projection multiplier. Derivatives belong to this sequential map, not a complete SIMPLE fixed-point map.',
        input_flux='Retained phi plus flux(perturbed boundary-evaluated U) minus flux(base boundary-evaluated U). Output phi uses exact pressure correction; input flux path remains explicitly classified.',
        constitutive='Preserve retained interior nut offset: nutInput=nutRetained+nutSA(Nperturbed)-nutSA(Nretained); evaluate native wall boundary. dnut/dN=t*(t+4*b)/(t+b)^2, t=(N/nu)^3, b=7.1^3.',
        epsilons=list(m.EPS),epsilon_basis='Four dimensionless angular decades selected before execution; centered differences separate truncation and fixed pressure-solve noise. No solver tolerance change.',
        controls_changed=0,new_CFD_iterations=0,no_momentum_solve=True,no_SA_solve=True,
        pressure_solver=dict(solver='PCG',preconditioner='DIC',tolerance=1e-13,relTol=0,maxIter=5000),
        statuses=dict(map='SequentialPressureCorrectionMapDifferentiatedAndReplayed',
            pressure='TemporaryPressureMapDerivativeDefined;PhysicalPressureTangentNotQualified',
            reduced_SIMPLE='ReducedSIMPLETangentNotQualified',
            boundary='SequentialMixedBoundaryDerivativeValidated;SimultaneousAugmentedBoundaryTangentNotQualified',
            generator='NativeAndUnlimitedClassified;FullFieldReconstructionDisagreementUnresolved;LowModeWholeMeshLiftNotQualified',
            full_tangent='PhysicalRotationalTangentNotQualified',Jg='NotEvaluatedAdmissionFailed'),
        stop_reason='Persistent simultaneous farfield U/p derivative defects after the exact sequential map; no common certified epsilon regime for the complete augmented tangent. Do not silently iterate the map or tune pressure tolerance.',
        next_task=NEXT,no_convergence_rule=True,no_stopping_contract=True,no_CFD_qualification=True)
    m.h.durable_json(m.ROOT/'CONTRACT.json',m.h.seal(contract))
    analysis=m.h.read(m.ROOT/'MAP_ANALYSIS.json');face=m.h.read(m.ROOT/'FACE_MATRIX_ANALYSIS.json')
    performance={}
    for name in m.c.NAMES:
        log=(m.ROOT/(name+'_derivatives.log')).read_text()
        assert 'Exit status: 0' in log and 'READ_ONLY_PRESSURE_MAP_COMPLETE' in log
        solves=[(int(a),float(b),float(c)) for a,b,c in re.findall(r'PRESSURE_MAP_SOLVE (\d+) ([\deE.+-]+) ([\deE.+-]+)',log)]
        assert len(solves)==22 and max(x[0] for x in solves)<5000
        performance[name]=dict(MPI_ranks=6,solves=solves,linear_iterations_total=sum(x[0] for x in solves),
            wall=re.search(r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): ([^\n]+)',log)[1],
            user_seconds=float(re.search(r'User time \(seconds\): ([\d.]+)',log)[1]),
            system_seconds=float(re.search(r'System time \(seconds\): ([\d.]+)',log)[1]),
            time_maxRSS_kB=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',log)[1]),exit_status=0)
    m.h.durable_json(m.ROOT/'PERFORMANCE.json',m.h.seal(performance))
    arrays={str(x.relative_to(m.ROOT)):dict(bytes=x.stat().st_size,sha256=m.h.file_hash(x))
        for folder in ('outputs','build') for x in (m.ROOT/folder).rglob('*') if x.is_file()}
    m.h.durable_json(m.ROOT/'ARRAY_MANIFEST.json',m.h.seal(dict(files=arrays)))
    import capsule_sa_operator_finish as af
    old=af.o.ROOT;af.o.ROOT=m.ROOT
    try:storage=af.allocation()
    finally:af.o.ROOT=old
    storage.update(starting_free_bytes=admission['free_bytes'],ending_free_bytes=shutil.disk_usage(m.ROOT).free,reclaimed_bytes=0,cleanup='None; prior evidence untouched; no complete CFD case copied.')
    m.h.durable_json(m.ROOT/'STORAGE.json',m.h.seal(storage))
    report=[CLASSIFICATION,
        'Outcome F. The sequential pressure-correction map is differentiated and exactly replayed. A complete physical azimuthal tangent is not qualified. No new Jg test was admitted.',
        f'Starting HEAD: {m.HEAD}. Ending HEAD is the task-owned commit containing this report, printed after commit.',
        'new CFD iterations = 0; mesh changes = 0; prism changes = 0; L11 changes = 0; retained state mutations = 0. Retained files 444 and scratch files 292 rehashed; unrelated dirty entries 155 preserved; V3 unchanged.',
        'Iteration 1000 is the only map input. Iteration 900 was verified as reference evidence; no late-change operator test was admitted.',
        'Prior result 54a64b588292884770a3f680ad53bc7fbfb6b34b94c118cf56a64e52e12f62bf; prior report 3bc297a8f32442349d046b4d64dba4bb466d0be31c3e27ce2d55005b0061e276; prior contract ee7b0fb4e2347b9b6ef61e5cf45391032a484c853ee45fc641bdbecbbbc70495; projection e70bebc49bad3d71778d1556bb5ad8038e934dad1400f436f57866bccf942850; low-mode c0bdcc3b64c6c5e3656342b195216436b4f020ac2bfd94f19a1efc52ecb29382; final verification fe32e839fca3b80ff23e6c3cb74518ea7ccdfbbc0fe9d83603f2d8db88308826. All reverified.',
        'GW1 sizing dd239298d7e671b70e1248120e6ebd51c25a52dd6c7013c10fbf3059d1eb0c90. Prism scientific 48f39ddfae1f782f724321e0d8882f9454928c5c0c2898652220dcfa6db3f1a1; qualification 75d827a837748dfca5014b873a18f4076843752ef6789622565f6de6d2ba8e7b; authority 356e5a357bbbdf923f7dd3a2df0642632ba3497518b0305c44f372d6fec7f9b3.']
    for name in m.c.NAMES:
        ca=admission['cases'][name]
        report += [name+' candidate '+ca['candidate'],json.dumps(ca['frozen'],sort_keys=True),
            'Iteration-1000 canonical fields: '+json.dumps({k:v['sha256'] for k,v in ca['checkpoints']['1000'].items()},sort_keys=True)]
    report+=['Foundation 13-cde978a97c93; foamRun SHA256 f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e.',
        'Installed source snapshots and hashes:']
    report += [path+' '+digest for path,digest in sources.items() if path!='identity']
    report += [contract['map'],*contract['equations'],contract['pressure'],contract['input_flux'],contract['constitutive'],
        'The exact source sequence relaxes and constrains UEqn, then constructs A/H/H1 and the consistent coefficients. Static steadyState ddtCorr is identically zero. No MRF, moving mesh, volume source or special constraint is present. constrainPressure only acts on fixedFluxPressure patches, absent here. Mixed pressure removes the reference requirement. nNonOrthogonalCorrectors=0 means one pressure solve. p relaxation is absent. No post-U pressure boundary reevaluation was added.',
        'Only pEqn.solve() is called. No momentum or SA solve, outer SIMPLE loop, time increment or scientific field write occurs. setTime selects retained 1000; output consists only of task-owned binary arrays. Existing six-rank decomposition is unchanged.',
        'Unchanged residual definitions: RU=(AU*U-bU)/V, Rc=div(phi), RSA=(AN*N-bN)/V, Rnut=nutRetained-nutSA(N). No new residual/Jacobian evaluation was admitted. Linear pressure-solver residuals below are not nonlinear PDE residuals.',
        'Base map values and corrections. RMS is volume-weighted vector magnitude or scalar RMS. Linf is maximum absolute component.']
    report+=['| Seed | quantity | RMS | Linf | canonical SHA256 |','|---|---|---:|---:|---|']
    for name in m.c.NAMES:
        for key,q in analysis[name]['base'].items():
            if key=='regions':continue
            report.append(f"| {name} | {key} | {q['RMS']:.12g} | {q['Linf']:.12g} | {q['sha256']} |")
        report.append('Base regional corrections '+name+': '+json.dumps(analysis[name]['base']['regions'],sort_keys=True))
        report.append('Face/matrix statistics '+name+': '+json.dumps(face[name],sort_keys=True))
    report+=['All base cell, face and matrix export files replay byte-for-byte in the retained six-rank decomposition. Cell arrays use global cell addressing; face flux uses signed global face addressing. Matrix off-diagonal/patch identities are explicitly partition-ordered; no claim of bitwise repartition independence is made.',
        'Seeds: scalar y*dq/dx-x*dq/dy; vector ez cross U minus (-y,x,0) dot grad(U). Native uses retained limited grad(U); unlimited uses default Gauss linear without editing dictionaries. p and N use retained gradients. No seed was adjusted to reduce Jg.',
        'Retained reconstruction evidence remains unresolved: quadratic/native cosine S1 0.764544853, S2 0.798918721; quadratic/unlimited 0.779142393/0.813413009. Removing the limiter does not remove disagreement. The previous verified quadratic finite-angle and 492-ring/256-angle m=0,1,2 representations are carried as diagnostic evidence only. No unsupported whole-mesh lift was invented. No reconstruction or low-mode Jg was run after the boundary gate failed.',
        'Epsilon values selected prospectively: 1e-3, 1e-4, 1e-5, 1e-6 angular amplitudes. Smallest epsilon independently reset and replayed. All perturbed internal nuTilda values remained positive. PCG/DIC tolerance 1e-13, relTol 0, maxIter 5000 unchanged.',
        '| Seed/direction | epsilon | correction/seed U | cosine | div derivative RMS | div derivative Linf | adjacent U array change |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for name in m.c.NAMES:
        for seed,s in analysis[name]['seeds'].items():
            for e in s['epsilon_study']:
                q=e['fields']['div'];u=e['fields']['U']
                report.append(f"| {name}/{seed} | {e['epsilon']} | {e['velocity_correction_ratio']:.10g} | {e['velocity_seed_cosine']:.10g} | {q['RMS']:.10g} | {q['Linf']:.10g} | {u.get('adjacent_relative_array_difference','n/a')} |")
    report+=['Continuity noise grows as finite differences amplify the unchanged linear pressure-solve floor. No smaller-epsilon-is-better assumption is made. Direct divergence of canonical deltaPhi is also retained in FACE_MATRIX_ANALYSIS. Wall deltaU, deltaPhi and deltaN are exactly zero for every evaluated direction/epsilon.',
        'Derivative intermediates at epsilon 1e-6; these are map derivatives, NOT Jg:',
        '| Seed/direction | dA | dH | dH1 | drAU | drAtU | dHbyA | dp output | d(p output-input) | dU output |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for name in m.c.NAMES:
        for seed,s in analysis[name]['seeds'].items():
            e=s['epsilon_study'][-1]
            report.append('| '+name+'/'+seed+' | '+' | '.join(f"{e['fields'][k]['RMS']:.11g}" for k in ('A','H','H1','rAU','rAtU','HbyA','p','pChange','U'))+' |')
            report.append('Boundary evidence '+name+'/'+seed+': '+json.dumps(e['boundary'],sort_keys=True)+'; mixed derivative audit '+json.dumps(e['mixed_boundary_audit'],sort_keys=True))
            report.append('Derivative canonical hashes '+name+'/'+seed+': '+json.dumps({k:v['sha256'] for k,v in e['fields'].items()},sort_keys=True))
    report+=['For pressure fp=.5+.5*(Ub dot n)/|Ub|; deltaPb=(1-fp)deltaPi-deltafp*Pi for reference pressure zero. The map uses Ub from the pressure stage, not final corrected Ub. The stage formula is reproduced; replacing that Ub by final Ub leaves an epsilon-persistent defect. This is measured sequencing behavior, not an invented failure of OpenFOAM.',
        'For velocity w=(Ui+Ub)/2; fU=.5-.5*(w dot n)/|w|. A native update uses the boundary value available at that stage. It is not an implicit solution of the simultaneous mixed relation. The final mapped derivative retains a nonzero simultaneous relation defect.',
        'For scalar farfield N, installed freestream/inletOutlet uses the phi-sign branch. Native input boundary evaluation is differentiated; N and nut are not reevaluated after the pressure map changes phi/U. Complete post-map scalar/wall constitutive closure is therefore not asserted. Prior native Spalding and interior constitutive tests remain verified, but they do not establish a new complete augmented closure.',
        'The prior homogeneous-multiplier pressure defect was about 2.64e-4/2.78e-4. The exact map now includes derivative pressure fractions and coefficient couplings, yet native final simultaneous defects remain 2.816252e-4/2.640478e-4. Similar magnitude is not proof of identical cause: this task explicitly identifies the stage-versus-final velocity distinction.',
        'Corrected phi equals phiHbyA-pEqn.flux by construction and native extraction. It need not equal flux(UCorrected): HbyA constraint, consistent pressure-gradient correction, pressure face flux, corrected cell gradient and final mixed-boundary update are distinct discrete operations. No equality of these two fluxes was imposed.',
        json.dumps(contract['statuses'],sort_keys=True),
        'Admission failed before Jg. Not evaluated: momentum/SA/continuity Jg, reduced SIMPLE response, control-direction Jg, near-null ratios, residual projection, regional Jg shares, patch Jg shares, low-mode Jg, 900-to-1000 augmented Jg, box/mesh pinning attribution. No neutral eigenmode or physical-time interpretation is established. Prior nonzero residuals remain evidence of imbalance; their physical rotational/non-rotational partition remains unresolved.',
        'Both seeds show similar large velocity corrections and the same sequential boundary limitation. This does not establish either box dominance or mesh-realization dominance.',
        'Synthetic validation: analytic axisymmetric scalar, m1 scalar, vector basis cancellation, SA derivative, mixed fraction/pressure product rule and a pressure linear-system derivative including matrix change passed. Prior actual-mesh conservative/nonconservative projection tests passed and remain sealed. A complete actual-mesh synthetic validation of a coupled physical pressure-map tangent was NOT completed after the admission failure; no such qualification is claimed.',
        'Tests: '+(m.ROOT/'tests.log').read_text().strip()+'; additional prior model regressions: '+(m.ROOT/'model_tests.log').read_text().strip(),
        'Test command: python -m pytest test_capsule_sa_pressure_map.py test_capsule_sa_tangent.py test_capsule_sa_operator.py test_capsule_sa_rotation.py test_capsule_sa_stabilization.py test_capsule_complete_mesh_authority.py test_capsule_monitoring_admission.py -q (all under research/sdf_cfd_mesher_v4/tests).',
        'Native map replay is exact for both seeds and both smallest-epsilon derivatives. Independent Python analysis replay is byte-identical: '+json.dumps(m.h.read(m.ROOT/'ANALYSIS_REPLAY.json'),sort_keys=True)+'. Source guards verify only the pressure solve, no runtime advance or field write. CRLF-aware authored-file whitespace check is run before commit; raw native logs are excluded from formatting changes.',
        'Performance for final instrumented evaluations: '+json.dumps(performance,sort_keys=True),
        'Each final seed evaluation has 22 linear pressure solves. An earlier base-only pass used two solves per seed and the initial derivative pass used 22 per seed before additional boundary diagnostics were added. Total pressure-only linear solves: 92. No momentum/SA solve or CFD outer iteration. Early development timings are not used for comparisons. Time-tool maximum RSS is not sampled aggregate MPI RSS.',
        'Storage: '+json.dumps(storage,sort_keys=True),
        'Utility SHA256 '+m.h.file_hash(m.ROOT/'build/bin/saPressureMap')+'; native source SHA256 '+m.h.file_hash(m.c.s.f.prior.control.V4/'tools/sa_pressure_map/saPressureMap.C'),
        'Explicit: no convergence rule created; no stopping contract created; no CFD qualification created; new SA iterations=0; new SST iterations=0; SST-LM runs=0; URANS runs=0; Transient runs=0; SAS runs=0; DES runs=0; LES runs=0; Run A=NO; Run A2=NO; production authority replaced=NO.',
        'Remaining uncertainty: a tangent of simultaneous retained-state constraints versus the derivative of a sequential submap; robust whole-field rotational generator; derivative noise at unchanged pressure tolerance; admissible operator stiffness and residual partition.',
        'Exactly one recommended next task: '+NEXT]
    modes=m.h.read(m.previous.ROOT/'LOW_MODES.json')
    low_table=['Reverified prior low-mode diagnostic values; body and wake use separate original measures.',
        '| Seed/region | m1 U tangent RMS | m1 p tangent RMS | m1 N tangent RMS |','|---|---:|---:|---:|']
    for name in m.c.NAMES:
        for region in ('body','wake_0.5','wake_1','wake_2','wake_4','wake_6','wake_8'):
            low_table.append('| '+name+'/'+region+' | '+' | '.join(f"{modes[name][k][region]['m1_tangent_norm']:.10g}" for k in ('U','p','nuTilda'))+' |')
    report[-1:-1]=low_table
    (m.ROOT/'REPORT.md').write_text('\n\n'.join(report)+'\n')
    v4=m.c.s.f.prior.control.V4
    files=list((v4/'tools').glob('capsule_sa_pressure_map*.py'))+list((v4/'tools/sa_pressure_map').rglob('*'))+[v4/'tests/test_capsule_sa_pressure_map.py']
    implementation={str(x.relative_to(v4)):m.h.file_hash(x) for x in files if x.is_file()}
    evidence={str(x.relative_to(m.ROOT)):m.h.file_hash(x) for x in m.ROOT.rglob('*') if x.is_file() and not {'build','outputs'}.intersection(x.relative_to(m.ROOT).parts) and x.name!='RESULT.json'}
    result=m.h.seal(dict(classification=CLASSIFICATION,starting_head=m.HEAD,evidence=evidence,implementation=implementation,next_task=NEXT,
        contract=m.h.read(m.ROOT/'CONTRACT.json')['identity'],map_analysis=analysis['identity'],face_analysis=face['identity']))
    m.h.durable_json(m.ROOT/'RESULT.json',result)
    print('RESULT',result['identity'],'REPORT',m.h.file_hash(m.ROOT/'REPORT.md'),flush=True)


if __name__=='__main__':seal()
