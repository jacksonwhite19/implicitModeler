"""Bounded policy cohort adapters. All evidence is bound to sealed artifacts."""
import gzip,hashlib,json,time
from pathlib import Path
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front import termination_policy as policy
s=g.s;q=g.q;np=g.np;ROOT=s.ROOT
OUT=ROOT/'results/production_termination_trigger_policy'
SEALED=ROOT/'results/local_termination_remesh_generalization'
BINDINGS={};AUTHORITIES={}

def read(rel):
    path=ROOT/rel;data=path.read_bytes();digest=hashlib.sha256(data).hexdigest()
    case=Path(rel).parts[1];a=AUTHORITIES.get(case)
    if a is None:
        ap=ROOT/'results'/case/'AUTHORITY.json';a=json.loads(ap.read_text());AUTHORITIES[case]=a
        BINDINGS[str(ap.relative_to(ROOT)).replace('\\','/')]=hashlib.sha256(ap.read_bytes()).hexdigest()
    if rel not in a['evidence_sha256'] or a['evidence_sha256'][rel]!=digest:
        raise ValueError('UnsealedSource: '+rel)
    BINDINGS[rel]=digest
    if path.suffix=='.gz':return json.loads(gzip.decompress(data))
    z=json.loads(data)
    if isinstance(z,dict) and z.get('format')=='Task3CExactJsonGzip/2':
        return read(str(path.with_name(z['payload']).relative_to(ROOT)).replace('\\','/'))
    return z

def ctx(region,layer,base,topology,schedule):
    return dict(region=region,layer=layer,base=base,topology=topology,schedule=schedule)

def ref(c,p,**fields):return dict(context=c,proof=q.identity(p),**fields)

def initial(c,source,status='unknown',seed=None):
    return dict(context=c,support=ref(c,source,status='clean'),immediate=ref(c,source,status=status,seed=seed or {}),warnings=[],witnesses=[],attempts=[],accepted=None)

def accepted(c,source,stage='NominalAdvance'):
    return ref(c,source,stage=stage,checks={k:True for k in policy.IMMEDIATE_CHECKS})


def positive_cases():
    cases=[]
    for spec in g.fixtures.definitions():
        name=spec['name'];z=read(f'results/local_termination_remesh_generalization/{name}/result.json')
        x,tri=g.fixtures.build(spec);k=z['event']['layer'];source=z['event']
        c=ctx(name,k,q.identity(dict(spec=spec,prior_layer=k-1)),q.array_hash(tri),q.array_hash(q.load_schedule().targets))
        assert z['support']['passed'] and z['classification']=='TerminationRemeshQualified'
        vertices,columns=g.failure_vertices(tri,source['quality'],source['outer'])
        seed=dict(vertices=sorted(vertices),columns=columns,cells=[1000000*k+i for i in columns],faces=sorted({i for row in source['outer']['failures'] for i in row['pair']}))
        e=initial(c,source,'invalid',seed);e['warnings']=[q.identity(source)]
        if spec.get('capacity'):
            assert k==spec['capacity']['first_geometric_limit']
            kind='QualifiedCapacityLimit'
        else:
            assert source['outer']['failures'];kind='ExactCollision'
        e['witnesses']=[ref(c,source,kind=kind,scope='immediate',qualified=True,seed=seed)]
        cases.append(dict(name=name,positive=True,evidence=e,first_warning=k,first_hard_failure=k,backend=dict(kind='generic',spec=spec,record=z),invalid_next_layer_committed=False))
    # Historical current-stack overlap is related invalid geometry to REMOVE,
    # not unrelated support contamination to freeze outside the cavity.
    conflicts=read('results/terminal_cap_conformity/FORENSICS.json')
    proof=read('results/terminal_cap_conformity/RESULT.json')
    old=read('results/local_termination_cavity_remesh/SELECTED.json')
    assert AUTHORITIES['terminal_cap_conformity']['exact_frozen_volume_overlap_pairs']==10
    vertices=sorted({v for row in conflicts for f in row['source_vertices'] for v in f})
    cols=sorted({col for row in conflicts for col in row['columns']})
    seed=dict(vertices=vertices,columns=cols,cells=sorted({1000000*row['layer']+col for row in conflicts for col in row['columns']}),faces=cols)
    c=ctx('general_sdf_baseline',5,q.identity(dict(prior_source=AUTHORITIES['local_termination_cavity_remesh']['report_sha256'])),q.identity(old['outer']),q.array_hash(q.load_schedule().targets))
    e=initial(c,conflicts,'invalid',seed);e['warnings']=[q.identity(conflicts)]
    e['witnesses']=[ref(c,dict(forensics=conflicts,volume_proof=proof),kind='PositiveVolumeOverlap',scope='current_stack',qualified=True,seed=seed)]
    residual=read('results/residual_layer5_failure/RESULT.json')
    e['diagnostics']=dict(recorded_solver_iterates=residual['recorded_solver_iterates'],verified_endpoints=residual['verified_endpoint_count'],thickness_debt='Descriptive only; no trigger threshold')
    cases.insert(0,dict(name='general_sdf_baseline',positive=True,evidence=e,first_warning=next(z['warning_onset'] for z in read('results/layer_capacity_valid_prefix/RESULTS.json') if z['fixture']=='general_asymmetric_passage'),first_hard_failure=min(row['layer'] for row in conflicts),backend=dict(kind='baseline',record=old),invalid_next_layer_committed=False,retrospective_overlap_layer=4))
    return cases


def negative_cases():
    cases=[];sf=read('results/task4l/SCIENTIFIC_COMPARISON.json');assert sf['passed']
    histories=sf['histories'] if 'histories' in sf else next(v for v in sf.values() if isinstance(v,list) and v and isinstance(v[0],dict) and 'accepted' in v[0])
    for k in range(1,21):
        base=f'results/task4l/fresh/LAYER_{k:02}'
        r=read(base+'_VALIDATION_RECEIPT.json');cert=read(base+'_CERTIFICATE.json')
        assert r['gate']['passed'] and r['exact']['passed'] and not r['exact']['failures']
        assert cert['complete'] and cert['candidate_pairs']==cert['completed_pairs'] and not any(cert[v] for v in ('forbidden','unresolved','invalid_surfaces'))
        c=ctx('SF2',k,r['inputs']['low'],r['inputs']['topology'],r['inputs']['schedule'])
        h=next(h for h in histories if h['layer']==k);stage='TangentialCorrection' if h['accepted'] else 'NominalAdvance'
        e=initial(c,r,'valid')  # Correction activity alone does not prove a nominal hard failure.
        if k==2:
            w=read('results/task4l/fresh/LAYER2_WARNINGS.json');assert w
            e['warnings']=[q.identity(w)]
            # Preserve an actual forecast-only snapshot as a separate negative.
            warning=initial(c,w);warning['warnings']=[q.identity(w)]
            cases.append(dict(name='SF2_steerable_forecast_before_correction',positive=False,evidence=warning,expected='WarningOnly'))
        e['accepted']=accepted(c,dict(receipt=r,certificate=cert,history=h),stage)
        cases.append(dict(name=f'SF2_layer_{k:02}',positive=False,evidence=e,expected='NoRemesh'))
    results=read('results/layer_capacity_valid_prefix/RESULTS.json')
    for z in results:
        if z['fixture'] not in ('parallel_wide','curved_wide'):continue
        assert z['last_valid_layer']==20
        for r in z['history']:
            assert r['passed'] and r['committed'] and r['certificate']['passed']
            c=ctx(z['fixture'],r['layer'],r['coordinate_hash'],z['foundation']['topology_hash'],q.array_hash(q.load_schedule().targets))
            e=initial(c,r,'valid');e['accepted']=accepted(c,r)
            cases.append(dict(name=f"{z['fixture']}_{r['layer']:02}",positive=False,evidence=e,expected='NoRemesh'))
    comp=read('results/general_local_front_correction/general_asymmetric_passage/result.json')
    assert comp['accepted_gate']['passed'] and comp['accepted_gate']['certificate']['passed'] and comp['target_admission_passed']
    c=ctx('historical_compression_acceptance',4,comp['coordinate_hash'],q.identity(comp['support']),q.array_hash(q.load_schedule().targets))
    e=initial(c,comp['baseline'],'invalid');e['warnings']=[q.identity(comp['baseline'])]
    e['attempts']=[ref(c,comp['tangent_reference'],stage='TangentialCorrection',status='failed')]
    e['accepted']=accepted(c,comp['accepted_gate'],'CompressionCorrection')
    cases.append(dict(name='historical_immediate_compression_resolved',positive=False,evidence=e,expected='NoRemesh',scope='Historical immediate acceptance only. Later full current-stack overlap invalidates it and is a separate positive event.'))
    warning=next(z for z in results if z['fixture']=='general_asymmetric_passage')
    c=ctx('capacity_warning_without_obstruction',3,warning['history'][2]['coordinate_hash'],warning['foundation']['topology_hash'],q.array_hash(q.load_schedule().targets))
    e=initial(c,warning['witness_history']);e['warnings']=[q.identity(warning['witness_history'])]
    cases.append(dict(name='general_sdf_warning_only',positive=False,evidence=e,expected='WarningOnly'))
    return cases


class QualifiedBackend:
    """Private discovery/reconstruction. Reuse exact receipts only after equality.

    A fresh selected-case certificate may be requested. The policy receives the
    same acceptance facts either way; caching cannot change the decision.
    """
    def __init__(self,case,fresh=False):self.case=case;self.fresh=fresh;self.runtime=None;self.details=None
    def __call__(self,request):
        st=time.perf_counter();c=self.case['evidence']['context'];seed=request['seed'];description=self.case['backend'];old=description['record']
        assert request['context']==c and seed==policy.cavity_seed(self.case['evidence'])
        if description['kind']=='baseline':
            case,prefix,history,previous,seedfront,targets,high,record,timing=s.r.reproduce()
            x,tri,fronts=case.x,case.f,prefix+[high]
            oldmap=read('results/local_layer_termination_topology/candidate_1_map.json');base=np.full(len(x),5);base[oldmap['stopped_vertices']]=4
        else:
            spec=description['spec'];x,tri=g.fixtures.build(spec);fronts=[g.fixtures.front(spec,d) for d in q.load_schedule().targets[:c['layer']]];base=None
        first=min(i//1000000 for i in seed['cells'])
        levels,history=g.discover(x,tri,fronts,set(seed['vertices']),first,base)
        assert levels is not None and levels.tolist()==old['vertex_depth']
        p,cells,outer,check,times=g.validation(x,tri,fronts,levels,self.fresh)
        assert q.identity(cells)==q.identity(old['cells']) and q.identity(outer)==q.identity(old['outer'])
        if description['kind']=='baseline':
            assert np.array_equal(p,np.load(s.OUT/'mesh_points.npy'))
            quality=read('results/local_termination_cavity_remesh/ALL_CELL_QUALITY.json');assert q.identity(check['quality'])==q.identity(quality)
            cert=read('results/local_termination_cavity_remesh/FULL_FACE_CERTIFICATE.json');volume=read('results/local_termination_cavity_remesh/VOLUME_AUDIT.json');continuation=read('results/local_termination_cavity_remesh/CONTINUATION.json')
            assert cert['passed'] and volume['positive_cells'] and volume['interior_degree']['passed'] and volume['exact_volume_balance'] and continuation['passed']
            # Exact coordinate comparison above authorizes unchanged receipt reuse.
        else:
            assert q.identity(check['quality'])==q.identity(old['result']['quality'])
            assert q.identity(p[sorted({v for f in outer for v in f})].tolist())==q.identity(old['ingest_input']['coordinates'])
            assert old['result']['passed'] and old['continuation']['passed']
            if self.fresh:
                assert q.identity(check)==q.identity(old['result'])
                co=g.continue_local(spec,x,tri,fronts,levels,set(seed['vertices']));co.pop('runtime');assert q.identity(co)==q.identity(old['continuation'])
        name=self.case['name'];core=read(f'results/local_termination_remesh_generalization/{name}/CORE_INGESTION.json');assert core['passed']
        if name!='general_sdf_baseline':
            account=read(f'results/local_termination_remesh_generalization/{name}/accounting.json');assert account['balance'] and account['wall_and_first_layer_unchanged'] and account['retained_interface_exact'] and account['continuation_assigned_offset']['admitted']
        receipt=ref(request['context'],dict(cells=q.identity(cells),outer=q.identity(outer),source=q.identity(old)),request_id=request['request_id'],checks={k:True for k in policy.REMESH_CHECKS})
        self.details=dict(seed=seed,discovery=history,layer_map_hash=q.identity(levels.tolist()),cell_hash=q.identity(cells),outer_hash=q.identity(outer),fresh_full_certification=self.fresh,qualification_source=str(SEALED/ name/'result.json'),passed=True)
        self.runtime=time.perf_counter()-st
        return receipt
