"""Explicit direct-complete inputs and evidence. No meshing or authority issuer."""
import hashlib
import json
import math
from fractions import Fraction

METHOD="DelaunayDirectSizing095SharedApex231/1"
POLICY="V4DirectFillEngineeringPolicy/1"
ACCEPTANCE="V4DirectFillEngineeringAcceptance/1"
REQUIRED=frozenset(("preservation","domain","incidence","winding","interface_qa","joined_qa",
    "physical_qa","surfaces","boundary_collision","enclosure","collision","filling"))
ARRAYS={"points":("<f8",3),"prisms":("<i8",6),"tetrahedra":("<i8",4),
    "wall":("<i8",3),"farfield":("<i8",3),"cap":("<i8",3)}
BINDING_FIELDS=frozenset(("parent_prism_mesh","case_sha256","canonical_source_identity",
    "source_release_sha256","policy_identity","method_identity","cap_identity","farfield_identity",
    "seed_identity","domain_identity","quality_targets_identity","geometry_identity","arrays_identity"))

def identity(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()

def digest(value):
    return type(value) is str and len(value)==64 and all(letter in "0123456789abcdef" for letter in value)

def method():
    from finite_research.direct_recipe155 import expected_options
    return dict(name=METHOD,options=expected_options(),generation_factor_hex=(.95).hex(),
        generation_dimensions=[3],callback="owned_direct_c_query",
        optimizers=[dict(name=name,threshold=.3,force=True,iterations="API default") for name in ("Gmsh","Netgen")],
        exact_binary_reload=True,interface_completion=False,transition=False,
        acceptance_contract="DirectFillEngineering/1",post_native_repair="GenericInteriorSharedApexRepair/231",
        repair_bounds=dict(vertices=32,passes=1,stages=2,maxiter=120,evaluations_per_vertex=8192,seconds_per_vertex=30,construction_seconds=3600),
        farfield_preparation="separate_authenticated_upstream",actual_parallelism="NOT_MEASURED")

def validate_policy(value):
    fields={"schema","backend_method","bounds_m","growth_per_m","maximum_size_m","interface_seed_policy",
        "quality_targets","downstream_seconds","native_seconds","geometry_assurance","continuous_sdf_fluid",
        "acceptance_contract_version","acceptance_contract_identity"}
    if type(value) is not dict or set(value)!=fields or value["schema"]!=POLICY or value["backend_method"]!=METHOD:
        raise ValueError("ExplicitDirectCompletePolicyRequired")
    if value["interface_seed_policy"]!="balanced_centroid_volume/1":
        raise ValueError("ExactDirectSeedMethodRequired")
    bounds=value["bounds_m"]
    if (type(bounds) is not list or len(bounds)!=3 or any(type(row) is not list or len(row)!=2
        or any(type(number) not in (int,float) or not math.isfinite(number) for number in row)
        or row[0]>=row[1] for row in bounds)):
        raise ValueError("ExplicitDirectDomainRequired")
    for name in ("growth_per_m","maximum_size_m"):
        if type(value[name]) not in (int,float) or not math.isfinite(value[name]) or value[name]<=0:
            raise ValueError("ExplicitDirectSizingRequired",name)
    if (value["downstream_seconds"]!=10800 or type(value["downstream_seconds"]) not in (int,float)
        or value["native_seconds"]!=7200 or type(value["native_seconds"]) not in (int,float)):
        raise ValueError("DirectBudgetRequired")
    if value["geometry_assurance"]!="all_twelve_fresh" or value["continuous_sdf_fluid"]!="NOT_PERFORMED":
        raise ValueError("SeparateDirectAssuranceRequired")
    from finite_support.transition_front_targets import validate
    validate(value["quality_targets"])
    from finite_research.direct_fill_contract import VERSION,TARGETS,contract_identity
    if (value["acceptance_contract_version"]!=VERSION or value["acceptance_contract_identity"]!=contract_identity()
        or value["quality_targets"]!=TARGETS):raise ValueError("ExplicitImmutableDirectFillContractRequired")
    return value

def validate_binding(value):
    if type(value) is not dict or set(value)!=BINDING_FIELDS or not all(digest(part) for part in value.values()):
        raise ValueError("CompleteIndependentDirectBindingRequired")
    if value["method_identity"]!=identity(method()):
        raise ValueError("ChangedDirectMethodIdentity")
    return value

def validate_acceptance(value,expected=None):
    if expected is None:
        raise ValueError("IndependentDirectAcceptanceRequired")
    if (type(value) is not dict or value!=expected.get("descriptor")
        or value.get("schema")!=ACCEPTANCE or value.get("execution_target")!="direct_complete"
        or value.get("method_identity")!=identity(method())
        or value.get("source_release_sha256")!=expected.get("source_release_sha256")
        or value.get("mesh")!=expected.get("binding",{}).get("parent_prism_mesh")
        or value.get("provisional") is not False or value.get("scope_exit_verified") is not True
        or value.get("publication_enabled") is not True or value.get("normal_run_qualified") is not False
        or value.get("solver_ready") is not False or value.get("continuous_sdf_fluid")!="NOT_PERFORMED"
        or value.get("stored_boundary_geometry")!="ALL_TWELVE_VALIDATED"
        or "transition_accepted" in value or "lease" in value):
        raise ValueError("ChangedDirectAcceptanceDescriptor")
    from finite_research.direct_fill_contract import VERSION,contract_identity
    applicability=value.get("acceptance_contract")
    if applicability!=dict(version=VERSION,contract_identity=contract_identity(),policy_identity=expected.get("binding",{}).get("policy_identity")):
        raise ValueError("ChangedDirectAcceptanceApplicability")
    for name in ("validation_sha256","parent_expectations_sha256","handoff_sha256","evidence_index_sha256",
        "scope_exit_sha256","worker_completion_sha256","owner_retirement_sha256","source_release_sha256"):
        if not digest(value.get(name)):
            raise ValueError("IncompleteDirectAcceptanceDescriptor",name)
    return value
