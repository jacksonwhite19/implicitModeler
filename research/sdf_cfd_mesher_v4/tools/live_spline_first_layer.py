"""Source-sheet binding to the existing exactly-one-layer V4 executors.

No geometry, compatibility solver, cell predicate or collision algorithm here.
"""
import ast, copy, json, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from production_first_layer_run import ProductionFirstLayerProvider, save
from production_surface_authority import load_production_wall,verify_surface_sidecars
from production_prism_sizing import inspect_first_layer_inputs,read_request,require_generation_authorization
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash
import generalize_local_remesh as g

V4=Path(__file__).resolve().parents[1];V3=V4.parent/'sdf_cfd_mesher_v3'
SF2=V3/'fresh_supported_refinement_family_qualification_2026-09-09'
from owned_runtime import SOURCE
CORNER=SOURCE/'corner_authority.py'
BRANCH=SOURCE/'verify_first_branch.py'

def existing_compatibility():
    node=next(n for n in ast.parse(CORNER.read_text(encoding='utf-8')).body if isinstance(n,ast.FunctionDef) and n.name=='height_compatibility')
    ns={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),str(CORNER),'exec'),ns)
    return ns['height_compatibility']

def branch_evidence(authority):
    """Execute the existing exact convexity prefix on live stored controls.

    Do not import study.py or its writers. Stop before the historical run/gate
    I/O; all mathematical assertions in that prefix remain unchanged.
    """
    nodes=[]
    for n in ast.parse(BRANCH.read_text(encoding='utf-8')).body:
        if isinstance(n,ast.ImportFrom) and n.module=='study':continue
        if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='q' for t in n.targets):break
        nodes.append(n)
    ns={'read':lambda _:copy.deepcopy(authority.source),'HERE':Path('.')}
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(BRANCH),'exec'),ns)
    return dict(passed=True,source=file_hash(BRANCH),spline_identity=authority.spline_packet['spline_identity'],
        exact_curvature_numerator_minimum=str(ns['minimum']),span_count=len(ns['c']),
        scope='Existing exact source convexity/branch assertions; positive incident supporting-normal displacement gives outward first vertex segments. Mesh-interior and contact checks remain separate.',
        no_offset_sdf_reprojection=True)


def write_certificate_rows(path, rows, *, compact=False):
    """Preserve decoded UTF-8/LF bytes and proof order in both storage forms.

    The qualified native encoder avoids one Python dictionary/JSON conversion
    per proof. Explicit FastProd storage uses the existing bounded gzip writer;
    archived production/qualification callers retain the plain stream.
    """
    from optimized_mixed_certificate import PackedRows
    if type(compact) is not bool:raise ValueError('InvalidCertificateStorageSelection')
    if compact:
        if Path(path).suffix!='.gz':raise ValueError('ExplicitCompressedCertificatePathRequired')
        if isinstance(rows, PackedRows):
            rows.write_jsonl(path,newline=b'\n')
        else:
            import gzip
            with Path(path).open('wb') as output,gzip.GzipFile(fileobj=output,mode='wb',mtime=0) as stream:
                for row in rows:
                    stream.write((json.dumps(row,sort_keys=True,separators=(',',':'))+'\n').encode('utf-8'))
        return
    if isinstance(rows, PackedRows):
        with Path(path).open('wb') as stream:
            for block in rows.encoded_chunks(b'\n'):
                stream.write(block)
                stream.write(b'\n')
    else:
        with Path(path).open('w', encoding='utf-8', newline='\n') as stream:
            for row in rows:
                stream.write(json.dumps(row, sort_keys=True, separators=(',', ':'))+'\n')

class SourceSheetFirstLayerProvider(ProductionFirstLayerProvider):
    def certify_candidate(self,fronts,levels,checked):
        # Same existing stage calls as AnalyticProvider; stop before the next
        # stage on a failed continuous certificate, retaining every proof row.
        from reuse_mixed_certificate import certify
        candidate=self.pack(fronts,levels)
        p,c,o=g.s.construct(self.x,self.tri,fronts,levels)
        owners,_=g.t.ownership(c,o,[tuple(reversed(f)) for f in self.tri])
        faces=[owners[k][0][1] for k in sorted(owners)]
        cert=self.phase('continuous_faces',lambda:certify(p,faces,candidate,self.previous_certificate_state))
        checked['certificate']=cert
        save(self.folder/'CERTIFICATE_SUMMARY.json',{k:v for k,v in cert.items() if k!='proofs'})
        from production_surface_authority import fastprod_surface_scope_active
        compact=fastprod_surface_scope_active(getattr(self,'surface',None))
        self.phase('certificate_evidence_write', lambda:write_certificate_rows(
            self.folder/('CERTIFICATE.jsonl.gz' if compact else 'CERTIFICATE.jsonl'),
            cert['proofs'],compact=compact))
        if not cert['passed']:
            checked['passed']=False
            save(self.folder/'FIRST_FAILURE.json',dict(stage='continuous_faces',certificate={k:v for k,v in cert.items() if k!='proofs'}))
            return checked
        volume=self.phase('volume_degree',lambda:g.volume.audit(p,c,o,[tuple(map(int,f)) for f in self.tri]))
        checked.update(volume=volume,passed=checked['quality']['passed'] and checked['ownership']['passed'] and checked['vertex_links']['passed'] and volume['exact_volume_balance'] and volume['positive_cells'] and volume['interior_degree']['passed'])
        save(self.folder/'VOLUME.json',volume)
        if not checked['passed']:save(self.folder/'FIRST_FAILURE.json',dict(stage='volume_degree',volume=volume))
        return checked

    def __init__(self,folder,surface,request,binding,*,_rolling_schedule=None,_rolling_completion=None,_rolling_completed=False,_rolling_transition=None):
        # Only replace the capsule-specific input binding. Inherited pack,
        # first-layer incidence, certificate, volume and commit remain intact.
        from canonical_spline_authority import LiveSplineAuthority
        from general_sdf_authority import GeneralSdfAuthority
        if type(binding.authority) not in (LiveSplineAuthority,GeneralSdfAuthority):raise ValueError('SupportedSourceAuthorityRequired')
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=False);self.times={}
        self.surface=Path(surface);self.request=Path(request);self.binding=binding;self.source=binding.authority
        self.source.verify();binding.qualification.verify(self.source)
        self.x,self.tri,self.wall_authority=load_production_wall(surface)
        if self.source.geometry_identity!=self.wall_authority['geometry_identity']:raise ValueError('GeometryIdentityMismatch')
        self.admission=inspect_first_layer_inputs(surface,request);require_generation_authorization(self.admission)
        req=read_request(request,self.wall_authority)
        if len(req['increments'])!=1 or req['declaration']['fluid_side']!='positive_sdf':raise ValueError('OneOutwardLayerRequired')
        self.h1=float(req['increments'][0]);self.schedule=SimpleNamespace(targets=req['targets'])
        self._rolling_schedule=None;self._trajectory_plan=None
        if type(_rolling_completed) is not bool:raise ValueError('InvalidCompletedFrontDeclaration')
        self._rolling_completed=_rolling_completed
        self._rolling_completion=copy.deepcopy(_rolling_completion)
        self._rolling_transition_json=None
        if _rolling_transition is not None:
            from standalone_background import validate_policy
            if (type(_rolling_transition) is not dict or _rolling_transition.get('schema')!='V4GradedBackground/6' or
                    _rolling_schedule is None or not _rolling_completed or type(self.source) is not GeneralSdfAuthority):
                raise ValueError('RollingGeneralSourceTetTransitionRequired')
            self._rolling_transition_json=json.dumps(_rolling_transition,sort_keys=True,allow_nan=False)
            validate_policy(json.loads(self._rolling_transition_json))
        if _rolling_completion is not None:
            from accepted_trajectory_contract import validate_direction_controls
            validate_direction_controls(_rolling_completion)
            if _rolling_schedule is None or not _rolling_completed:raise ValueError('RollingCompletedSourceScheduleRequired')
        if _rolling_schedule is not None:
            from sdf_cfd_mesher_v4.common.authority import frozen
            values=np.asarray(_rolling_schedule,dtype='<f8')
            if values.ndim!=1 or not len(values) or not np.isfinite(values).all() or np.any(values<=0) or values[0]!=self.h1:
                raise ValueError('InvalidRollingPrismPlanSchedule')
            self._rolling_schedule=frozen(values,'<f8')
        _,reference_hash=verify_surface_sidecars(self.surface,self.wall_authority,verify_implementation=False)
        if file_hash(self.surface/'REFERENCES.npz')!=reference_hash:raise ValueError('ReferenceIdentityMismatch')
        with np.load(self.surface/'REFERENCES.npz') as z:self.refs={k:z[k] for k in z.files}
        if type(self.source) is GeneralSdfAuthority:
            from production_surface_authority import reuse_source_coverage
            self.coverage=reuse_source_coverage(self.surface,self.wall_authority,binding.qualification,self.source,self.x,self.tri,self.refs)
        else:self.coverage=binding.qualification.check_source_coverage(self.source,self.x,self.tri,self.refs['face_refs'],self.refs['curve_edges'],self.refs['corner_refs'])
        from standalone_geometry import branch_evidence as bound_branch
        self.branch=bound_branch(binding)
        import first_layer_metric_state
        from native_mixed_face_certificate import SOURCE
        self.authority=dict(schedule=req['schedule_identity'],quality=file_hash(Path(g.t.__file__)),certificate=file_hash(Path(g.s.cert.__file__)),
            certificate_filter=file_hash(SOURCE),first_prism_metric=file_hash(Path(first_layer_metric_state.__file__)),sdf=self.wall_authority['geometry_identity'],
            production_surface=self.wall_authority['surface_identity'],request=self.admission['request_identity'],
            source_references=file_hash(self.surface/'REFERENCES.npz'),compatibility=file_hash(CORNER),
            source_branch=identity(self.branch) if type(self.source) is GeneralSdfAuthority else file_hash(BRANCH))
        if type(self.source) is GeneralSdfAuthority:self.authority['surface_assurance']=binding.qualification.assurance
        self.records=[];self.checked={};self.meshes={};self.numerical={};self.reuse_certificates=True;self.previous_certificate_state=None
        self.x.setflags(write=False);self.tri.setflags(write=False)
        save(self.folder/'ADMISSION.json',dict(admission=self.admission,coverage=self.coverage,branch=self.branch,authority=self.authority,
            top_role='ProspectiveFirstPrismDeterminant: every top remains internal in final shell/core; no future centroid or complete-neighbour metric claimed',
            no_terminal_cap=True))

    def _generate_trajectory(self):
        a=self.source;compatible=existing_compatibility()
        from general_sdf_authority import GeneralSdfAuthority
        incident=[set() for _ in self.x]
        for f,r in zip(self.tri,self.refs['face_refs']):
            for i in f:incident[i].add(int(r))
        direction=np.empty_like(self.x);normals=[];errors=[];ranks=[]
        for i,refs in enumerate(incident):
            n=np.array([a.exact_normal(r,self.x[i]) for r in sorted(refs)])
            if not np.isfinite(n).all() or not np.allclose(np.linalg.norm(n,axis=1),1.,rtol=0,atol=1e-13):raise ValueError('SourceUnitNormalFailure:'+str(i))
            row=compatible(n)
            if row['classification']!='SharedFiniteCornerTrajectoryFeasible':
                save(self.folder/'FIRST_FAILURE.json',dict(stage='normal_compatibility',vertex=i,refs=sorted(refs),result=row));raise ValueError('IncidentNormalIncompatible')
            direction[i]=row['displacement_per_height'];normals.append(n);errors.append(row['max_residual']);ranks.append(row['rank'])
        self.direction=direction
        # Exactly the recovered SF2 first_row construction. Compatible vectors
        # are deliberately NOT normalized. Qualification never snaps the front.
        self.high=self.x+self.h1*direction
        displacement=self.high-self.x
        heights=[n@d for n,d in zip(normals,displacement)]
        residual=np.array([max(abs(v-self.h1)) for v in heights])
        minimum=np.array([min(v) for v in heights])
        self.assigned=dict(admitted=bool(np.isfinite(self.high).all() and np.all(minimum>0) and max(residual)<1e-12),
            residual_max=float(max(residual)),height_reserve_m=1e-12,unit_normal_reserve=1e-13,
            provenance='V3 first_row.py and accepted_front_state.py; no scalar-offset target at sharp features',
            compatibility_residual_max=float(max(errors)),normal_height_min=float(min(minimum)),projection_operations=0,projection_failures=0,normal_fallbacks=0)
        np.savez(self.folder/'CANDIDATE.npz',wall=self.x,triangles=self.tri,front=self.high,direction=direction,residual=residual,
            normal_height_min=minimum,incident_rank=np.array(ranks),**self.refs)
        if type(a) is not GeneralSdfAuthority:
            save(self.folder/'TRAJECTORY.json',dict(self.assigned,incident_counts={str(k):sum(len(v)==k for v in incident) for k in range(1,4)},
                direction_norm_min=float(np.linalg.norm(direction,axis=1).min()),direction_norm_max=float(np.linalg.norm(direction,axis=1).max()),
                branch=self.branch,coverage=self.coverage,full_height_m=self.h1,full_height_hex=self.h1.hex()))
        if type(a) is GeneralSdfAuthority:
            if self._rolling_schedule is not None:
                from accepted_trajectory_contract import source_front_plan
                self._trajectory_plan=source_front_plan(a,self.x,self.high,self.tri,self.refs['face_refs'],self._rolling_schedule,_completion=self._rolling_completion,_completed_front=self._rolling_completed,
                    _transition=None if self._rolling_transition_json is None else json.loads(self._rolling_transition_json))
                save(self.folder/'SCHEDULE_FEASIBILITY.json',self._trajectory_plan.summary())
            numerical_admitted=self.assigned['admitted']
            self.assigned['admitted']=False
            save(self.folder/'TRAJECTORY.json',dict(self.assigned,source_proof='pending',full_height_m=self.h1))
            proof=a.qualify_segments(self.x,self.high,first_height_m=self.h1)
            save(self.folder/'SOURCE_SEGMENTS.json',proof)
            self.assigned['source_normal_and_segments']=proof
            self.assigned['admitted']=numerical_admitted and proof['passed']
            save(self.folder/'TRAJECTORY.json',dict(self.assigned,branch=self.branch,coverage=self.coverage,
                full_height_m=self.h1,full_height_hex=self.h1.hex(),projection_operations=0))

    def generate_candidate(self):
        start=time.perf_counter()
        from general_sdf_authority import GeneralSdfAuthority
        if type(self.source) is GeneralSdfAuthority:
            # Existing source/control/arithmetic custody and conservative filter.
            # This is neither construction nor surface acceptance reuse.
            if self.source.stock_prism_queries():
                with self.source.prism_query_scope() as statistics:
                    with self.source.verified_operation('trajectory'):
                        self._generate_trajectory()
                    arithmetic=statistics()
            else:
                # Preserve custom callback invocation semantics and old scope.
                with self.source.query_scope() as statistics:
                    self._generate_trajectory()
                    arithmetic=statistics()
            save(self.folder/'SOURCE_QUERY_ARITHMETIC.json',arithmetic)
        else:
            self._generate_trajectory()
        self.times['source_trajectory_binding']=time.perf_counter()-start
        if not self.assigned['admitted']:raise ValueError('FullNormalHeightAdmissionFailed')
        p,c,o,check,times=self.phase('raw_quality',lambda:g.validation(self.x,self.tri,[self.high],np.ones(len(self.x),int),False))
        self.times.update(times)
        check=self.phase('first_prism_incidence',lambda:self.initial_quality(check,self.high))
        self.points,self.cells,self.outer,self.check=p,c,o,check
        save(self.folder/'RAW_QUALITY.json',check)
        save(self.folder/'QUALITY.json',check['quality'])
        save(self.folder/'TOPOLOGY.json',dict(ownership=check['ownership'],vertex_links=check['vertex_links']))
        if not (check['quality']['passed'] and check['ownership']['passed'] and check['vertex_links']['passed']):
            save(self.folder/'FIRST_FAILURE.json',dict(stage='hard_quality_or_topology',failures=check['quality']['failures'],ownership=check['ownership'],vertex_links=check['vertex_links']))
            return False
        check=self.phase('continuous_and_volume',lambda:self.certify_candidate([self.high],np.ones(len(self.x),int),check));self.check=check
        return check['passed'] and self.assigned['admitted']

    def verify_wall(self):
        x,f,r=load_production_wall(self.surface)
        if x.tobytes()!=self.x.tobytes() or f.tobytes()!=self.tri.tobytes() or r!=self.wall_authority:raise ValueError('WallMutation')
        self.source.verify()
        return True
