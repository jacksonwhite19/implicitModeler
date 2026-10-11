"""Evidence-based closure of this bounded diagnostic, without new pass thresholds."""
import capsule_broad_global_family as f
h=f.h


def run():
    comparison=h.read(f.ROOT/'COMPARISON.json');dense=h.read(f.ROOT/'DENSE_WAKE.json')
    h.verify_record(comparison);h.verify_record(dense)
    w={n:v['windows']['451-500'] for n,v in comparison['meshes'].items()}
    drag=lambda n:w[n]['signals']['Fz']['mean']
    lateral=lambda n:w[n]['lateral']['F']['magnitude']['mean']
    evidence=dict(C2_R_minus_C2_drag=drag('C2-R')-drag('C2'),C2_R_minus_C2_lateral=lateral('C2-R')-lateral('C2'),G1_minus_C2_drag=drag('G1')-drag('C2'),G1_minus_W1_drag=drag('G1')-drag('W1'),G1_minus_W1_lateral=lateral('G1')-lateral('W1'),wake_8D={n:dense['variants'][n]['stations']['8'] for n in ('C2','C2-R','G1')})
    decision=h.seal(dict(classification='V4CapsuleBroadAndGlobalCoreRefinementShowsResidualMeshSensitivity',comparison=comparison['identity'],dense_wake=dense['identity'],evidence=evidence,findings=[
        'All three new cores passed exact frozen-prism/L11 preservation, hard mesh gates, complete changed-face continuous certification and Foundation Mesh OK. No geometric invalidity was found.',
        'G1 achieved global restraint in the actual mesh: hV p95=0.057374450911961m and max=0.07567776497866711m; baseline values are0.12375554549606509m and0.17079586691474863m. It uses no body or wake cap.',
        'C2 and C2-R refine the complete body neighborhood and the wake. They reduce geometry-only wake-sector imbalance, but do not eliminate CFD realization sensitivity.',
        'C2 is much quieter than W1 late in the run. C2-R retains substantial azimuth and bending-moment drift. Their final50 drag and lateral-magnitude means differ by more than either corresponding final50 range. This is not a difference in azimuth alone.',
        'G1 changes rotationally invariant loads as well as direction. It gives lower drag and lateral magnitude than W1/R1/C2/C2-R. It does not provide a universal improvement: its downstream wake remains coarser and broader than the targeted refined wakes.',
        'C2/C2-R have similar terminal deficit integrals and widths compared with their directional changes, but this does not erase the load difference or establish convergence. Nearest-cell sampling and unconverged iterative phase limit attribution.',
        'The experiments are non-nested. G1 changes global growth, cap and resulting realization; C2 changes both body coverage and wake target. Outer-core-only causation and body-only causation are not isolated.',
        'No mesh-independent CFD core, steady CFD solution, physical unsteadiness, domain independence or experimental validity is established. No production authority is replaced.'
    ],general_refinement_matters='It materially changes the measured solution in this controlled experiment; the contribution from outer cells alone is not isolated.',broad_body_refinement_matters='The broad/finer C2 treatment changes the measured solution and improves one realization markedly. Body coverage cannot be separated from its simultaneous wake-target reduction.',realization_sensitivity='Still present in invariant loads and drift, not only final azimuth.',production_promotion=False,URANS_ready=False,provisional_URANS_reference_pair=['C2','C2-R'],URANS_selection='C2/C2-R are the better-resolved wake references, but do not start URANS yet. Resolve the missing combination of global restraint and broad/wake protection first.',recommended_next_task=dict(name='BuildAndCompareV4CapsuleCombinedC2G1CorePairBeforeURANS',scope='One bounded pair using h(x)=min(G1 generic L11 field,C2 regional cap), with existing seed1/seed2 mechanisms, exact frozen prisms, full mesh gates and the same prospective steady observation horizon. Compare against this retained family. No new sizing numbers or arbitrary acceptance percentages are needed.',reason='G1 removed coarse outer cells but left a weaker downstream wake. C2 protected the wake but retained the coarse outer tail. The intersection of the two existing fields is the one missing controlled comparison before spending on URANS.'),URANS_runs=0,Run_A=False,Run_A2=False,stopping_contract=False,compatibility_note='Internal reused helper names W1 and WAKE_CONTRACT are filenames/schema labels only. Actual variant identity, generic growth and regional-refinement selection are sealed separately. G1 uses ExactCachedSizingField directly.'))
    h.durable_json(f.ROOT/'DECISION.json',decision)
    print(decision['identity'])


if __name__=='__main__':run()
