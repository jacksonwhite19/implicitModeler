"""Evidence-based terminal interpretation; no fitted convergence gates."""
import capsule_steady_models as s
h=s.h


def run():
    records={}
    for name in ('FAMILY_CONTRACT','FINE_FAMILY_CONTRACT','STAGE1_DECISION','FINE_COMPARISON','FINE_DENSE_WAKE','MESH_SELECTION','SSTLM_ADMISSION','SA_ADMISSION','MODEL_COMPARISON','WALL_STRESS_COMPARISON','PERFORMANCE'):
        r=h.read(s.ROOT/(name+'.json'));h.verify_record(r);records[name]=r['identity']
    m=h.read(s.ROOT/'MODEL_COMPARISON.json')
    assert all(not r['summary']['bounds'] and not r['summary']['maxiter'] and r['summary']['wall_max']==0 for r in m['runs'].values())
    decision=h.seal(dict(schema='CapsuleSteadyMeshAndModelSensitivityDecision/1',terminal=True,
        classification='V4CapsuleSteadyRANSSpatialSensitivityPersistsAndSAChangesLoads',
        evidence=records,mesh_classification='V4CapsuleSteadyRANSGC2FStillShowsResidualSpatialSensitivity',
        mesh_findings=[
            'GC2 and GC2F preserve exact prisms and L11 and pass every hard complete-mesh gate.',
            'GC2F halves the regional target exactly and approximately halves protected-wake realized hV. No further level was created.',
            'GC2F has larger late load excursions and terminal wake disagreement despite improved sector-density isotropy.',
            'Spatial and iterative sensitivities remain coupled. No Richardson estimate, discretization error, or mesh independence is established.',
            'GW1 is retained only as the more repeatable controlled model-sensitivity reference; it is not promoted as mesh independent.'],
        model_findings=[
            'SST-LM inputs can be calculated, but solved yPlus conflicts with documented transition wall-resolution guidance. No LM run or exploratory accuracy claim.',
            'SA viscosity-equivalent freestream nuTilda is derived from the installed model with zero binary64 nut inversion residual.',
            'SA changes drag mainly through pressure drag and changes terminal wake deficit and rear-wall pressure/reverse-shear diagnostics.',
            'SA has smoother final load magnitudes than SST on this pair. Drag and azimuth continue to drift; SA seed 2 lateral momentum residuals increase in the final window.',
            'SA drag differences from SST exceed the selected-pair drag spread, but that does not remove the separate unresolved spatial and iterative errors.',
            'No model is ranked for physical accuracy. No model has earned a repeatable steady-solution qualification.'],
        run_behavior={
            'SST-GW1-S1':'Persistent slow drift with bounded iterative oscillation',
            'SST-GW1-S2':'Persistent slow drift with bounded iterative oscillation',
            'SST-GC2-S1':'Persistent slow drift',
            'SST-GC2-S2':'Persistent slow drift',
            'SST-GC2F-S1':'Persistent bounded load excursions and late reversal',
            'SST-GC2F-S2':'Persistent bounded load excursions and late reversal',
            'SA-GW1-S1':'Approaching flatter invariant loads, but persistent slow drag and azimuth drift',
            'SA-GW1-S2':'Flatter invariant loads, but persistent slow drag and azimuth drift; growing late lateral residuals'},
        prism_vertices_changed=0,prism_connectivity_changed=0,L11_vertices_changed=0,L11_topology_changed=0,
        new_steady_runs=6,outer_iterations_each=500,reused_SST_runs=2,SSTLM_runs=0,
        URANS_runs=0,SAS_runs=0,DES_runs=0,LES_runs=0,transient_runs=0,
        Run_A=False,Run_A2=False,stopping_contract=False,CFD_numerical_authority=False,production_authority_replaced=False,
        remaining_uncertainty=['Steady fixed point','Spatial convergence and discretization error','Wall/transition accuracy with frozen prisms','Model accuracy and experimental validation','Domain independence'],
        recommended_next_task='BoundedSteadySAStabilizationAndFixedPointAssessmentOnFrozenGW1Pair',
        next_task_scope='Use the retained pair and unchanged SA controls. Prospectively bound the observation horizon and assess full-vector drift before any renewed spatial-convergence or model-accuracy claim. No automatic extension or production promotion.'))
    h.durable_json(s.ROOT/'DECISION.json',decision);print(decision['identity'])


if __name__=='__main__':run()
