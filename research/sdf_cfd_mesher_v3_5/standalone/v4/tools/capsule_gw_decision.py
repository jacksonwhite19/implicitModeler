"""Seal the measured diagnostic decision; no acceptance tolerance is introduced."""
import capsule_gw_pair as g
h=g.h
c=h.read(g.ROOT/'COMPARISON.json');d=h.read(g.ROOT/'DENSE_WAKE.json')
h.verify_record(c);h.verify_record(d)
pairs={}
for a,b in [('W1','R1'),('C2','C2-R'),('GW1-S1','GW1-S2')]:
    x,y=[c['meshes'][n]['windows']['451-500'] for n in (a,b)]
    pairs[a+'/'+b]=dict(drag_mean_difference=abs(x['signals']['Fz']['mean']-y['signals']['Fz']['mean']),lateral_force_mean_difference=abs(x['lateral']['F']['magnitude']['mean']-y['lateral']['F']['magnitude']['mean']),lateral_moment_mean_difference=abs(x['lateral']['M']['magnitude']['mean']-y['lateral']['M']['magnitude']['mean']))
findings=[
    'The exact min(G1,W1) pair reduces realization sensitivity in late drag, lateral-force magnitude, lateral-moment magnitude and protected-wake integral/width. This is a relative diagnostic conclusion, not a numerical pass threshold or mesh-independence claim.',
    'Final-50 pair differences are 0.0022750888450744355 N drag, 0.004553612166533938 N lateral magnitude and 0.00012766439530325496 N m bending magnitude. W1/R1 differences are 0.025007179625835207 N, 0.03758336420054309 N and 0.001165290725245613 N m. C2/C2-R differences are 0.014503456837562712 N, 0.03384230089698326 N and 0.0009558335434521285 N m.',
    'At 8D the pair deficit integrals are 0.01564666693851517 and 0.015663074725434697 m3/s; RMS widths are 0.036943290656567485 and 0.037095880060368125 m. At 10D the integrals differ by 0.000800058766776028 m3/s. Nearest-centroid sampling and terminal-state drift limit this comparison.',
    'Both realizations preserve G1 farfield restraint and W1 protected-wake resolution. Relative to G1 alone, dedicated wake refinement changes the downstream deficit and width while late drag remains near the G1 result. G1 has only one retained realization, so its realization variance is unknown.',
    'Relative to W1/R1, global restraint materially changes loads and reduces pair spread. Relative to C2/C2-R, GW1 has smaller pair spread but different mean loads and wake width. C2 broad-body refinement is not shown necessary for the next diagnostic, and its eventual necessity is not ruled out.',
    'The final force azimuths are -90.55768131838549 and -177.55547350169414 degrees. Finite off-axis load and late drift remain. Their differing direction alone is not a mesh-failure criterion. No steady fixed point, physical shedding or CFD numerical qualification is claimed.',
    'Both meshes pass complete hard gates, full continuous certification and Foundation Mesh OK. Both fresh six-rank steady cases complete exactly 500 iterations with no turbulence bounding, nonfinite fields or maxIter events. Conservative flux remains accurate and wall flux is zero.',
    'Path A is supported for a bounded paired URANS diagnostic, with explicit checks for physical-time stationarity and realization sensitivity. A stronger G1+C2 pair is not built or recommended before that diagnostic. No production authority is replaced.'
]
r=h.seal(dict(schema='CapsuleG1PlusW1Decision/1',classification='V4CapsuleG1PlusW1PairReducesCoreRealizationSensitivity',path='A',comparison=c['identity'],dense_wake=d['identity'],pair_differences=pairs,findings=findings,mesh_independence_established=False,steady_convergence_established=False,C2_body_refinement_necessity='Not established; not ruled out',stronger_G1_C2_next=False,production_promotion=False,URANS_runs=0,Run_A=False,Run_A2=False,stopping_contract=False,recommended_next_task=dict(name='BoundedPairedV4CapsuleURANSDiagnosticOnGW1S1AndGW1S2',scope='Use these two separately qualified cores with the same frozen prisms, physical case and SST model. Derive physical-time and Courant controls prospectively. Compare rotationally invariant loads and wake statistics across realizations; do not infer physical unsteadiness from steady-iteration spectra.',reason='The combined field substantially reduces observed realization spread while persistent off-axis loads and iteration drift still prevent a steady-flow conclusion.')))
h.durable_json(g.ROOT/'DECISION.json',r)
print(r['identity'])
