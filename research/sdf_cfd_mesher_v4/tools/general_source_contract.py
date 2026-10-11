"""Unselected binding to the existing surface QualificationContract owner.

The declared continuous target is the authenticated real expression. Native
extraction is a proposal, not an acceptance report or a native-normal proof.
"""
from dataclasses import asdict
from fractions import Fraction as exact_fraction
import json
import numpy as np
from surface_binding import QualificationContract
from procedural_authorities import POSITION_LIMIT,NORMAL_LIMIT_RAD
from sdf_cfd_mesher_v4.sdf.canonical import GeometryFailure
from sdf_cfd_mesher_v4.sdf.regional import exact
from sdf_cfd_mesher_v4.sdf.operations import ProtectedQueries
from sdf_cfd_mesher_v4.sdf.source_topology import protected_star_shaped_source
from sdf_cfd_mesher_v4.common.authority import array_hash


def snapshot(value):return json.dumps(value,sort_keys=True,allow_nan=False,separators=(',',':'))

def array_binding(value):
    # Existing array_hash authenticates bytes only. Shape and dtype remain
    # explicit independent members of this materialized surface binding.
    return dict(dtype=value.dtype.str,shape=list(value.shape),sha256=array_hash(value))


class GeneralSourceQualification(QualificationContract):
    """Derive source premises here; never accept a caller's passed receipt."""
    def __setattr__(self,name,value):
        if getattr(self,'_sealed',False):raise ValueError('ProtectedGeneralSourceQualification')
        object.__setattr__(self,name,value)

    def __init__(self,authority,interior_m,*,topology_depth,topology_boxes,coverage_depth,coverage_boxes,assurance='qualification'):
        from general_sdf_authority import GeneralSdfAuthority
        from sdf_cfd_mesher_v4.sdf.operations import budgets
        if type(authority) is not GeneralSdfAuthority:raise ValueError('ProtectedGeneralAuthorityRequired')
        authority.verify();scope=authority.queries._scope
        budgets(coverage_depth,coverage_boxes,scope._budgets[2])
        if assurance not in ('numerical','qualification'):raise ValueError('ExplicitSurfaceAssuranceRequired')
        proof=(protected_star_shaped_source(scope,interior_m,maximum_depth=topology_depth,maximum_boxes=topology_boxes)
            if assurance=='qualification' else dict(status='NotRun',passed=None,
                reason='Routine numerical acceptance does not prove global C1, radial topology or zero-set completeness.'))
        authority.verify()
        if assurance=='qualification' and proof['status']!='Certified':
            raise GeometryFailure('SourceTopologyUnresolved','source_qualification',proof,scope._identity)
        target=dict(schema='V4CanonicalRealExpressionTarget/1',geometry_identity=scope._identity,
            kernel_source_sha256=scope._source,units='m',unit_scale_exact=str(exact(scope._scale)),
            expression=repr(scope._expression),
            semantics='Exact real operations on authenticated binary32 source coefficients; SI unit scale is the exact real value of the captured binary64 coefficient. No binary32 coordinate quantization in this continuous target.',
            native_role='Proposal extraction and separately labelled scalar diagnostics only. Native zero set, normals and topology are not claimed equivalent.',
            residual_role=('Upper real-target field residual, not a native residual or asserted Euclidean distance.'
                if assurance=='qualification' else 'Numerical real-target field residual estimate; no upper bound, native equivalence or Euclidean distance proof.'),
            features=('No nonsmooth geometric features under proved C1 source profile; semantic region IDs are not inferred.'
                if assurance=='qualification' else 'One root-field reference; sharp feature identities are not supplied or certified.'))
        super().__init__(1 if assurance=='qualification' else None,2 if assurance=='qualification' else None,
            0,authority.sheet_names,{}, {},POSITION_LIMIT,NORMAL_LIMIT_RAD,2e-12,
            ('Authenticated continuous real expression with exhaustive C1 positive-radial zero cover; source-to-mesh coverage and face accuracy must pass on the final surface.'
             if assurance=='qualification' else 'Authenticated source expression; sampled numerical geometry acceptance of the closed oriented mesh. No whole-source completeness or continuous error proof.'),__file__)
        object.__setattr__(self,'_authority',authority)
        object.__setattr__(self,'_scope',scope)
        object.__setattr__(self,'_target',snapshot(target))
        object.__setattr__(self,'_topology',snapshot(proof))
        object.__setattr__(self,'_coverage_budget',(coverage_depth,coverage_boxes))
        object.__setattr__(self,'_assurance',assurance)
        object.__setattr__(self,'_contract',snapshot(asdict(self)))
        object.__setattr__(self,'_issuer_pin',self.custody_identity())
        object.__setattr__(self,'_sealed',True)
        self.verify(authority)

    @property
    def target(self):
        held=self._authority.acceptance_source(self)
        return held[0] if held is not None else json.loads(self._target)
    @property
    def source_topology(self):
        held=self._authority.acceptance_source(self)
        return held[1] if held is not None else json.loads(self._topology)
    @property
    def assurance(self):
        held=self._authority.acceptance_source(self)
        return held[3] if held is not None else self._assurance

    def custody_identity(self):
        return (self._authority,self._scope,self._assurance,self._target,self._topology,self._coverage_budget,self._contract)

    def batch_contract(self):
        policy=self._authority.acceptance_contract(self)
        self.validate()
        values=json.loads(self._contract) if policy is None else policy
        for name in ('sheet_names','curve_names','corner_names'):
            values[name]={int(k):v for k,v in values[name].items()}
        return QualificationContract(**values)

    def validate(self):
        if self._authority.acceptance_active(self):return
        if self.custody_identity()!=self._issuer_pin:raise ValueError('ChangedGeneralSourceCustody')
        if self.assurance=='qualification':super().validate()
        elif (self.assurance!='numerical' or self.expected_components is not None or self.expected_euler is not None
                or self.expected_boundary_loops!=0 or self.position_limit_m!=POSITION_LIMIT
                or self.normal_limit_rad!=NORMAL_LIMIT_RAD or self.projection_tolerance_m!=2e-12):
            raise ValueError('InvalidNumericalSurfaceContract')
        if snapshot(asdict(self))!=self._contract:raise ValueError('ChangedGeneralSourceQualification')
        self._scope.verify()

    def verify(self,authority):
        if authority is self._authority and authority.acceptance_active(self):return True
        self.validate();authority.verify();self._scope.verify(authority)
        if authority is not self._authority or authority.queries._scope is not self._scope:
            raise ValueError('DetachedGeneralSourceQualification')
        if authority.sheet_names!=self.sheet_names or authority.curve_names or authority.corner_names:
            raise ValueError('ChangedGeneralSourceReferences')
        return True

    def check_source_coverage(self,authority,points,faces,refs,curves,corners):
        try:return self._check_source_coverage(authority,points,faces,refs,curves,corners)
        except BaseException:
            self._authority.reject_acceptance();raise

    def _check_source_coverage(self,authority,points,faces,refs,curves,corners):
        self.verify(authority)
        if self.assurance!='qualification':raise ValueError('FormalQualificationRequired')
        limits=self.batch_contract()
        def inputs():
            return (np.asarray(points,dtype=float),np.asarray(faces),np.asarray(refs),
                    np.asarray(curves),np.asarray(corners))
        # Copy the entire surface/reference tuple BEFORE validation, triangle
        # derivation or hashing. Every proof and identity refers to these same
        # private bytes, not different moments of mutable caller containers.
        x,f,r,ce,cr=(np.array(v,copy=True,order='C') for v in inputs())
        if (x.ndim!=2 or x.shape[1]!=3 or not np.isfinite(x).all() or
                f.ndim!=2 or f.shape[1]!=3 or f.dtype.kind not in 'iu' or not len(f) or
                np.any(f<0) or np.any(f>=len(x)) or
                r.shape!=(len(f),) or r.dtype.kind not in 'iu' or set(r)!=set(limits.sheet_names) or
                ce.shape!=(0,3) or ce.dtype.kind not in 'iu' or
                cr.shape!=(len(x),) or cr.dtype.kind not in 'iu' or np.any(cr)):
            raise ValueError('GeneralRootReferenceCoverageRequired')
        # Detach the actual binary64 triangles before the proof. No mutable
        # caller array can alter a triangle after its distance witness is used.
        triangles=x[f].copy();triangles.flags.writeable=False
        # Bind this result to EXACT materialized input bytes, including labels.
        # The final audit owner must retain/compare these identities when it
        # consumes the receipt; it must not import a bare passed boolean.
        before=tuple(array_binding(v) for v in (x,f,r,ce,cr))
        materialized=dict(zip(('coordinates','faces','face_refs','curve_edges','corner_refs'),before))
        materialized['triangles']=array_binding(triangles)
        held=self._authority.acceptance_source(self)
        depth,boxes=held[2] if held is not None else self._coverage_budget
        from sdf_cfd_mesher_v4.sdf.regional import radial_mesh_coverage
        # This owner derived and sealed the exhaustive one-root-per-ray source
        # proof in __init__. The mathematical helper cannot issue that premise
        # or consume a caller's claimed source topology.
        expression,scale,budgets,_=self._scope._batch_inputs()
        result=radial_mesh_coverage(expression,
            tuple(tuple(float(v) for v in row) for row in x),
            tuple(tuple(int(v) for v in row) for row in f),
            tuple(exact_fraction(v) for v in self.source_topology['interior_m']),
            metres_per_model_unit=scale,position_limit_m=limits.position_limit_m,
            maximum_depth=depth,maximum_patches=boxes,precision_bits=budgets[2])
        self.verify(authority)
        current=tuple(array_binding(v) for v in inputs())
        if current!=before:raise ValueError('ChangedMaterializedCoverageInputs')
        if result['status']!='Certified' or result.get('source_empty'):
            raise GeometryFailure('SourceCoverageUnresolved','source_qualification',result,self._scope._identity)
        return dict(passed=True,method='protected_radial_source_and_final_triangle_cycle',proof=result,
                    source_topology=self.source_topology,continuous_target=self.target,materialized_surface=materialized,
                    does_not_prove='Final mesh topology/ownership, face accuracy, native equivalence or prism/cell acceptance.')
