"""Render all decision evidence for the complete-map study."""
from pathlib import Path
import json
import capsule_sa_outer_map as m
h=m.h;R=m.ROOT
CLASS='V4CapsuleSteadySACompleteIterationMapPreservesLateDriftButAzimuthalModeRemainsUnqualified'
NEXT='Qualify a whole-domain coherent azimuthal map-input tangent, then use the verified complete map to separate its response from the slow SA-amplitude and retained late-change directions.'
MODES=('native','unlimited','projected','streamwise','strain','SA','late')


def create():
    base=h.read(R/'BASE.json');ad=h.read(R/'ADMISSION.json');con=h.read(R/'CONTRACT.json');sources=h.read(R/'SOURCES.json')
    data={k:h.read(R/(k.upper()+'_FINE.json')) for k in MODES}
    bounds={k:h.read(R/(k.upper()+'_FINE_BOUNDARY.json')) for k in MODES}
    health=h.read(R/'HEALTH.json');inter=h.read(R/'INTERMEDIATES.json');zero=h.read(R/'ZERO.json')
    lines=[]
    def p(s=''):lines.append(str(s))
    def fmt(x):
        if isinstance(x,float):return f'{x:.10g}'
        return str(x)
    def table(head,rows):
        block=['| '+' | '.join(head)+' |','| '+' | '.join(['---']*len(head))+' |']
        for row in rows:block.append('| '+' | '.join(fmt(v) for v in row)+' |')
        p('\n'.join(block))
    p('# '+CLASS)
    p('The complete temporary map is reproduced and validated. It preserves the retained late-change direction much more closely than the native/unlimited gradient rotation probes. The SA-amplitude control also has nearly unit retention. A unique physical azimuthal iteration mode is not established. No PDE-neutral eigenmode is proved.')
    p('Starting HEAD: `'+m.HEAD+'`. Branch: `feature/sdf-cfd-mesher-v4-advancing-front`. The ending commit is reported in the terminal response; this report is a member of that commit.')
    p('All 444 retained files and 292 scratch inputs were rehashed before and after. The 155 unrelated dirty entries and their tracked diff were preserved. No V3 file changed. Scientific continuation iterations=0; new retained times=0; mesh/prism/L11 changes=0; retained field mutations=0.')
    p('**Verified authorities.**')
    previous=h.read(m.prior.ROOT/'RESULT.json')
    p('Previous result: `'+previous['identity']+'`. Previous report SHA-256: `9dc799ed38363433c55a4efaae1987a156a19e2676a90817614d50ba82fbaaaf`.')
    p('Previous contract `b44f9921bb6c410e2769d9f5a7382bf4a2345388fb2f48528f72ac45cef4ec90`; map analysis `affad7836e72ad7239de14a0a0e5befdc032e01ccaba34ed90bbe45d247e2f75`; face/matrix analysis `d49b50d53a2e1052855b3e2d8687aaf859323dc8294be4bbe5e80df2f8c0966c`; final verification `86e63718977a031382a458409ec94fa2180258c435337e8705360c8510a0e2e0`.')
    for n in m.c.NAMES:
        p(n+' candidate: `'+ad['cases'][n]['candidate']+'`.')
    p('GW1 sizing: `dd239298d7e671b70e1248120e6ebd51c25a52dd6c7013c10fbf3059d1eb0c90`.')
    frozen=m.c.s.f.prior.frozen()
    p('Prism scientific `48f39ddfae1f782f724321e0d8882f9454928c5c0c2898652220dcfa6db3f1a1`; qualification `75d827a837748dfca5014b873a18f4076843752ef6789622565f6de6d2ba8e7b`; prism/L11 authority `356e5a357bbbdf923f7dd3a2df0642632ba3497518b0305c44f372d6fec7f9b3`.');p('Frozen prism/L11 contract:');p('```json');p(json.dumps(frozen,sort_keys=True,indent=2));p('```')
    table(['Seed','Field','Canonical iteration-1000 SHA-256'],[(n,k,v['sha256']) for n in m.c.NAMES for k,v in ad['cases'][n]['checkpoints']['1000'].items()])
    p('**Installed map and state.**')
    p('Foundation `13-cde978a97c93`; foamRun SHA-256 `f8f49ab14fc2ede5b5cdfe61f45f9f935a291791882929f4f6de02afe2747d4e`. All source bindings were reverified against the installation.')
    for i,s in enumerate(con['order'],1):p(f'{i}. {s}.')
    p('State: '+con['state']+'. Constructor validation is initialization, not an outer iteration; the independent native-module reference restores the exact retained scalar fields after construction. Patch classes, coefficient storage and update flags are restored before every map call. A value-only reset failed replay and was rejected. A seed-preparation auxiliary-state offset was also detected and corrected before interpreting derivatives. Rejected arrays, logs and source are retained explicitly.')
    p('The utility is `tools/sa_outer_map/saOuterMap.C`. Each map call has a count=1 guard and unchanged timeIndex guard. It performs one vector momentum solve, one scalar pressure solve and one scalar SA solve: five scalar solver log entries. It does not increment time, call a second outer loop, or write scientific fields. Temporary results are binary diagnostic arrays, not iteration1001. The independent native module path calls its control.loop exactly once.')
    p('Pressure sequencing is preserved: pressure sees momentum-stage U; pressure solves; phi updates; U updates; U boundary conditions update; SA corrects. No simultaneous final U/p boundary relation is imposed. The retained pressure/momentum/SA controls and all dictionaries are unchanged.')
    p('Executed with six existing MPI ranks: `mpirun -np 6 saOuterMap -parallel -case <existing operator scratch>/<seed> -output <task outputs>/<seed>/<direction> -direction <direction> [-fine]`. The projected variant additionally supplies the previous frozen-metric face tangent. Exact commands are in each log and the retained shell scripts.')
    p('**Base map and fixed-point defect.**')
    p('Both seeds: every five-block base output is bitwise identical to its replay and to the independently executed installed incompressibleFluid module. All saved base/intermediate replay hashes are in ARRAY_MANIFEST.json. Canonical cell and oriented-face addressing is used. Other MPI decompositions were not tested and no repartition occurred.')
    table(['Seed','Block','Output RMS','Defect RMS','Defect Linf','Relative defect','Cosine with 900â†’1000'],[(n,k,v['output']['RMS'],v['defect']['RMS'],v['defect']['Linf'],v['relative_defect'],v.get('late_change_relation',{}).get('cosine','separate late table')) for n in m.c.NAMES for k,v in base[n].items()])
    p('U units m/s; p mÂ²/sÂ²; N=nuTilda and NT=nut mÂ²/s; phi mÂ³/s. Cell norms are volume-weighted RMS. Face norms use |Sf| weights. Linf is maximum absolute component. Blocks are never combined with arbitrary dimensional weights.')
    table(['Seed','Block','Region','Defect RMS','Defect Linf'],[(n,k,r,a['RMS'],a['Linf']) for n in m.c.NAMES for k,v in base[n].items() for r,a in v.get('regions',{}).items()])
    table(['Seed','Native intermediate','RMS','Linf'],[(n,k,v['RMS'],v['Linf']) for n in m.c.NAMES for k,v in inter[n].items()])
    p('A/H/H1 are the relaxed momentum matrix accessors used after the predictor. Raw pressure diagonal/source/upper/patch coefficients and corrected states are retained. The unrelaxed momentum matrix and internal SA matrix were not separately exported; the exact installed SA correct() performs the actual assembly/relax/solve. Final N and nut, native solve logs and source bindings provide the SA-stage evidence.')
    p('**Input directions and derivative construction.**')
    p(con['seed_convention']+'.')
    for k,v in con['directions'].items():p('- '+k+': '+v+'.')
    p(con['input_path'])
    p(con['constitutive'])
    p('Native U RMS seeds are 0.200482447/0.200063775; unlimited 0.205180183/0.204696505. Previous sampled quadratic/native cosines are 0.764544853/0.798918721 and quadratic/unlimited 0.779142393/0.813413009. Removing the limiter does not resolve reconstruction disagreement. The low-mode evidence covers 492 rings Ã—256 angles, not the entire box. No unsampled whole-domain perturbation is invented. Neither sampled reconstruction nor m=1 is promoted to an authoritative full-map input.')
    p('Centered DSg=(S(q+epsilon*g)-S(q-epsilon*g))/(2epsilon). Initial epsilon=1e-3,1e-4,1e-5,1e-6. Coarse native/unlimited samples cross farfield or solver-stopping branches, so they are excluded from local linear interpretation. The prospective fine study uses 1e-6,1e-7,1e-8. Strain at +1e-6 crosses 2/1 farfield faces and is excluded; strain at 1e-7 and1e-8 has no crossings. Other fine directions have no detected farfield crossings. No bounding, nonfinite or maxIter event occurred.')
    p('Representative tables use epsilon=1e-7. This is a measured compromise between stopping-branch effects and cancellation noise, not an acceptance threshold. Full scalar/vector arrays and all three fine steps are retained. Directional continuity is noise-limited and is not treated as a nonzero physical mass source.')
    table(['Seed','Direction','epsilon','U gain','U retention','Adjacent U array change','Adjacent p change','Adjacent N change','Adjacent phi change'],[(n,k,row['epsilon'],row['fields']['U'].get('gain','zero U input'),row['fields']['U'].get('retention','NA'),row['fields']['U'].get('adjacent_array_difference','first'),row['fields']['p'].get('adjacent_array_difference','first'),row['fields']['N'].get('adjacent_array_difference','first'),row['fields']['phi'].get('adjacent_array_difference','first')) for k in MODES for n in m.c.NAMES for row in data[k][n]['epsilon_study']])
    table(['Seed','Direction','Block','Gain','Cosine','Retention mu','Orthogonal output fraction'],[(n,k,b,v.get('gain','zero input'),v.get('cosine','NA'),v.get('retention','NA'),v.get('orthogonal_fraction','NA')) for k in MODES for n in m.c.NAMES for b,v in data[k][n]['epsilon_study'][1]['fields'].items()])
    p('mu is a directional retention coefficient, not an eigenvalue. Zero-input blocks can have cross-coupled output; no ratio is fabricated for them. The late direction is the full retained 100-step secant, not one iteration and not assumed to be pure rotation.')
    table(['Seed','Direction','Stage','U gain','U cosine','U retention','Orthogonal fraction'],[(n,k,stage,v['gain'],v['cosine'],v['retention'],v['orthogonal_fraction']) for k in ('native','projected','late') for n in m.c.NAMES for stage,v in data[k][n]['epsilon_study'][1]['stages'].items()])
    p('Boundary correction does not change internal U at that stage, so preBoundaryU and final U internal metrics coincide. Physical patch arrays are retained separately. SA and nut updates do not alter U afterward. Their own block metrics above show SA-solve and nut-correction response. HbyA is an intermediate, not an accepted velocity field.')
    p('**Defect alignment and localization.**')
    table(['Seed','Direction','Block','Dâ€“g cosine','Best coefficient','Orthogonal defect fraction'],[(n,k,b,v['fixed_point_defect_projection'].get('cosine','NA'),v['fixed_point_defect_projection'].get('retention','NA'),v['fixed_point_defect_projection'].get('orthogonal_fraction','NA')) for k in ('native','projected','late') for n in m.c.NAMES for b,v in data[k][n]['epsilon_study'][1]['fields'].items()])
    table(['Seed','Direction','Region','Squared share of DSg_Uâˆ’g_U','Regional RMS'],[(n,k,r,v['squared_identity_error_share'],v['identity_error_RMS']) for k in ('native','projected','late') for n in m.c.NAMES for r,v in data[k][n]['epsilon_study'][1]['fields']['U']['regions'].items()])
    table(['Seed','Direction','Box-adjacent patch','Squared identity-error share'],[(n,k,p,v['squared_share']) for k in ('native','projected','late') for n in m.c.NAMES for p,v in bounds[k][n]['patch_adjacent_identity_error'].items()])
    p('Box-adjacent regions overlap at edges and are not a partition. The disjoint near-farfield share is about1.84â€“1.86% for gradient rotation and0.19â€“0.23% for late change. Near-body core dominates gradient-probe non-identity response; near-body and protected wake dominate late-change non-identity response. These results do not support farfield dominance for the tested paths. They do not rule out subtle box pinning or prove mesh causation. Both seeds give similar conditional responses; no preferred azimuth scan was performed.')
    p('**Conservation, health and numerical floor.**')
    table(['Seed','Direction','Output','div RMS','div Linf','Farfield net flux','Wall flux Linf'],[(n,k,tag,v['divergence']['RMS'],v['divergence']['Linf'],v['farfield_net'],v['wall_flux_Linf']) for k in ('native','projected','late') for n in m.c.NAMES for tag,v in bounds[k][n].items() if tag in ('base0','d0','d1','d2')])
    p('The base outputs have global normalized flux imbalance around5e-15. Wall U, wall nuTilda and wall phi are exactly zero for the base and directional outputs. The installed flux formula is evaluated directly; flux(deltaU)=deltaPhi is not imposed. Directional divergence increases roughly as pressure-solve/roundoff noise divided by epsilon; this limits continuity-derivative interpretation without changing the original solver tolerance.')
    table(['Seed','Block','Zero-direction DS RMS','Even cancellation Linf'],[(n,k,v['derivative']['RMS'],v['even']['Linf']) for n in m.c.NAMES for k,v in zero[n].items()])
    p('DS(0) is exactly zero. Nonzero base-map defect cancels in the centered subtraction. The largest zero-path even error is about1.4e-11 in U. At epsilon=1e-7, actual late-change derivative arrays differ from the1e-6 result by about1e-6 for U/p, a few1e-9 for SA, and a few1e-6 for phi. Gradient U array differences are about0.0016â€“0.0020, despite much tighter scalar gain stability; this is retained as uncertainty. Smaller epsilon is not declared more accurate.')
    table(['Log','Temporary map calls','Pressure iterations','SA iterations','Timing/RSS'],[(k,v['map_calls'],v['iterations']['p'],v['iterations']['nuTilda'],'; '.join(v['time_statistics'])) for k,v in health.items() if isinstance(v,dict)])
    p('Accepted logs contain202 temporary map evaluations and1010 scalar linear solves. Totals: Ux1293, Uy1204, Uz1394, pressure60080, SA606 linear iterations. Retained development logs account for 76 additional temporary map evaluations rejected for auxiliary-state preparation; they are retained separately and are not interpreted. These retained logs account for 278 diagnostic map calls and 1390 scalar solve records, not scientific continuation. Timing/RSS are /usr/bin/time process-tree reports, not sampled aggregate MPI RSS or precise shared-machine speed comparisons.')
    p('**Decision.**')
    p('CompleteOneOuterIterationMapQualifiedForDeclaredCheckpointInitialization. Declared-direction derivatives have reproducible finite-step evidence with the stated solver/noise limits. PhysicalAzimuthalIterationModeNotQualified. Native/unlimited rotational velocity probes contract and change direction substantially; they are not near-identity and contract more than streamwise/lateral controls. Conservative input flux changes those results only slightly. The actual late-change U direction is weakly contracting, while some late p/SA block gains exceed1. These block gains do not prove an unstable full-state eigenmode. SA-amplitude retention is also near1, so slow behavior is not unique to azimuthal rotation.')
    p('No mathematically neutral eigenmode, PDE eigenvalue, physical-time rotation, convergence, mesh independence or production accuracy is established. No Krylov/Arnoldi solve was performed: a uniquely physical whole-domain rotational direction is still missing, and the requested direct comparisons already identify non-unique slow behavior.')
    p('No convergence rule created. No stopping contract created. No CFD qualification created. New retained SA/SST iterations=0; SST-LM/URANS/transient/SAS/DES/LES runs=0. Run A=NO; Run A2=NO; production authority replaced=NO.')
    p('Tests:176 passed, including25 focused complete-map tests and all relevant prior SA/operator/rotation/tangent/pressure-map/model/mesh-authority/monitoring regressions. Synthetic tests cover zero perturbation, known linear maps, m=1/axisymmetric rotation, discrete divergence-free perturbations, constitutive lag, centered cancellation, block metrics and native ordering. Prior mixed-BC/constitutive tests were rerun. Synthetic fields were not substituted for the scientific capsule state. Task-owned CRLF-aware whitespace is checked before commit; raw installed-source snapshots and exact logs are excluded from cosmetic whitespace edits.')
    if (R/'STORAGE.json').exists():
        storage=h.read(R/'STORAGE.json');p('Storage: starting free271333240832B. No prior evidence deleted and no complete case copied. No cleanup yet; reclaimed0B. Current retained allocation: '+json.dumps(storage,sort_keys=True)+'.')
    p('Remaining uncertainty: sampled reconstruction does not yet define a robust full-domain physical tangent; input conservative flux is not uniquely fixed by rotation on the square mesh; finite-tolerance native solves limit some derivative arrays; the retained state is not a fixed point; one local map derivative is not a long-term spectral or physical stability proof.')
    p('Exactly one recommended next task: '+NEXT)
    p('**Source hashes.**')
    table(['Installed source','SHA-256'],sources.items())
    p('**Evidence identities.**')
    for file in ('ADMISSION.json','CONTRACT.json','BASE.json','ZERO.json','INTERMEDIATES.json','NATIVE_FINE.json','UNLIMITED_FINE.json','PROJECTED_FINE.json','STREAMWISE_FINE.json','STRAIN_FINE.json','SA_FINE.json','LATE_FINE.json','HEALTH.json','ARRAY_MANIFEST.json','FINAL_VERIFICATION.json'):
        if (R/file).exists():p(file+': `'+h.read(R/file).get('identity',h.file_hash(R/file))+'`.')
    (R/'REPORT.md').write_text('\n\n'.join(lines)+'\n',encoding='utf-8')
    derived=dict(classification=CLASS,map_status='CompleteOneOuterIterationMapQualifiedForDeclaredCheckpointInitialization',physical_azimuthal_mode='NotQualified',near_neutral_PDE_mode_proved=False,next_task=NEXT,no_convergence_rule=True,no_stopping_contract=True,no_CFD_qualification=True,scientific_continuation_iterations=0,new_retained_times=0)
    h.durable_json(R/'DECISION.json',h.seal(derived))
    evidence={p.name:h.file_hash(p) for p in sorted(R.iterdir()) if p.is_file() and p.suffix in ('.json','.md','.log','.sh') and p.name!='RESULT.json'}
    v4=m.c.s.f.prior.control.V4
    paths=list((v4/'tools/sa_outer_map').rglob('*'))+list((v4/'tools').glob('capsule_sa_outer_map*.py'))+[v4/'tests/test_capsule_sa_outer_map.py']
    implementation={str(p.relative_to(v4)):h.file_hash(p) for p in paths if p.is_file()}
    h.durable_json(R/'RESULT.json',h.seal(dict(classification=CLASS,decision=h.read(R/'DECISION.json')['identity'],starting_HEAD=m.HEAD,evidence=evidence,implementation=implementation)))
    print(CLASS);print('result',h.read(R/'RESULT.json')['identity']);print('report',h.file_hash(R/'REPORT.md'))

if __name__=='__main__':create()
