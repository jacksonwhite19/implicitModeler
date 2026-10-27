"""Reproduce policy arithmetic and input bindings, without meshing."""
import json
from pathlib import Path
import shutil
import time
import psutil
from initial_boundary_layer_policy import estimate, explicit_progression, ASSUMPTIONS, policy_identity
from production_prism_sizing import inspect_first_layer_inputs, contract_identity
from sdf_cfd_mesher_v4.common.authority import file_hash, identity

V4=Path(__file__).resolve().parents[1]
REPO=V4.parents[1]
OUT=V4/'results/geometry_independent_initial_boundary_layer_sizing'
OLD=V4/'results/capsule_physical_prism_request'


def save(name,data):
    (OUT/name).write_text(json.dumps(data,sort_keys=True,indent=2,allow_nan=False)+'\n',encoding='utf8')


def main():
    begin=time.perf_counter()
    expected={'report.md':'aa65ca11ee5e83dad451f7d24a995006cee2b93948be67cc1b850292e8abb7d8',
              'AUTHORITY.json':'be63ff31125e57912fb686a09d3e4632dfe21ba45fdd2636f3a4fac4b94e4729'}
    for name,digest in expected.items():
        assert file_hash(OLD/name)==digest
    wall=inspect_first_layer_inputs(V4/'results/production_adaptive_capsule_surface_foundation/fresh')
    assert wall['surface_identity']=='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
    assert contract_identity()=='7de6b4527da1cd55659c1796927394cbd9af2f581cffbd85d2f891705abbf7dd'
    save('SURFACE_VERIFICATION.json',wall)
    reference=json.loads((OLD/'V3_REFERENCE_CONDITION.json').read_text())
    controls=[]
    for model in ('blasius_laminar','smooth_turbulent_power_law'):
        arguments=dict(model=model,speed=reference['U'],density=reference['rho'],viscosity=reference['mu'],
            yplus=10.,shear_start_m=reference['length_m'],coverage_end_m=reference['length_m'],
            assumptions=list(ASSUMPTIONS),height_scope='global')
        cold=estimate(**arguments);replay=estimate(**json.loads(json.dumps(arguments)))
        assert cold==replay
        controls.append(dict(inputs=arguments,output=cold,replay_exact=True,
            role='Historical 1 m flat-plate arithmetic control only; NOT a capsule policy binding'))
    schedule=json.loads((OLD/'SCHEDULE_PROVENANCE.json').read_text())
    progression=explicit_progression(h1=schedule['sf2_h1'],growth=schedule['sf2_growth'],
        count=schedule['sf2_count'],physical_total_intent_m=schedule['physical_total_intent'])
    assert progression['increments_m_hex']==schedule['increments_hex']
    save('ARITHMETIC_CONTROLS.json',dict(controls=controls,progression=progression))
    files=[]
    prior=json.loads((OLD/'EVIDENCE_MANIFEST.json').read_text())['files']
    for row in prior:
        p=REPO/row['path']
        # All prior direct authority bindings remain checked, including V3 sources.
        assert file_hash(p)==row['sha256']
        files.append(row)
    for p in [OLD/'report.md',OLD/'AUTHORITY.json',Path(__file__),
              V4/'tools/initial_boundary_layer_policy.py']:
        files.append(dict(path=p.relative_to(REPO).as_posix(),sha256=file_hash(p),bytes=p.stat().st_size))
    save('EVIDENCE_MANIFEST.json',dict(files=files,identity=identity(files)))
    unresolved=['case flow direction and reference-condition selection',
        'wall-treatment profile and explicit target y+',
        'surrogate model/regime and roughness/transition assumptions',
        'physically interpreted finite shear-start distance and coverage-end distance',
        'case-owner adoption of engineering-estimate limitations',
        'progression selection policy: count/growth and separate physical coverage intent']
    save('CAPSULE_POLICY_STATUS.json',dict(status='InitialPhysicalRequestUnbound',unresolved=unresolved,
        policy_identity=policy_identity(),contract_identity=contract_identity(),
        global_h1_m=None,physical_total_m=None,growth=None,count=None,
        automatic_geometry_physical_scale=None,generation_authorized=False,
        prism_vertices_generated=0,prisms_generated=0,layer1=False,layer2=False,shell=False,core=False))
    memory=psutil.Process().memory_info()
    save('AUDIT_RUNTIME.json',dict(audit_seconds=time.perf_counter()-begin,
        audit_peak_rss_bytes=getattr(memory,'peak_wset',None),audit_retained_rss_bytes=memory.rss,
        whole_task_peak_rss='NotMeasured',free_bytes_after_audit=shutil.disk_usage(REPO).free))
    print(json.dumps(dict(controls=controls,policy_identity=policy_identity()),indent=2))


if __name__=='__main__':main()
