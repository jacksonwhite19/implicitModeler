"""Prospective engineering contract; percentage diagnostics never waive hard QA."""
import math
from fractions import Fraction
from finite_research.direct_contract import identity

VERSION='DirectFillEngineering/1'
TARGETS={'outer_tetrahedra':{'minimum':.25,'population_targets':[{'fraction':.99,'minimum':.4},{'fraction':.9,'minimum':.6}]},
         'outer_triangles':{'minimum':.3,'population_targets':[{'fraction':.99,'minimum':.6},{'fraction':.9,'minimum':.8}]}}
CONTRACT=dict(version=VERSION,minimum_shape_role='MANDATORY',percentage_population_role='NONBLOCKING_ENGINEERING_DIAGNOSTIC',
    populations=['all_bulk_tetrahedra','interface_tetrahedra','preserved_cap_triangles'],targets=TARGETS,
    hard_geometry_obligations='ALL_TWELVE_MANDATORY_UNCHANGED',hard_quality_gates='INHERITED_UNCHANGED',
    approval=dict(human_answer='Yes.',advisor_thread='01a166fe-ac1a-7d81-9489-3c296a2310a4',task=231,
        proposal='Versioned direct-fill contract preserving hard QA and minimum shape floors; uncalibrated population percentages remain engineering diagnostics.'),
    rationale='Trial construction percentages were not validated as solver-use thresholds. Engineering direct-fill acceptance requires all hard QA and minimum floors.',
    limitations=['No CFD accuracy or yplus validation','No independent transition band','No continuous SDF fluid proof','No retrospective acceptance or default promotion'])

def contract_identity():return identity(CONTRACT)

def binding(policy):
    from finite_research.direct_contract import validate_policy
    validate_policy(policy)
    return dict(version=VERSION,contract_identity=contract_identity(),policy_identity=identity(policy))

def require_binding(value,policy):
    if value!=binding(policy):raise ValueError('ChangedDirectFillContractBinding')
    return value

def classify(report,target):
    """Keep source target verdicts intact and add separate mandatory acceptance."""
    validate_report(report,target,classified=False)
    value=dict(report,contract_version=VERSION,contract_identity=contract_identity(),
        minimum_role='MANDATORY',percentage_role='NONBLOCKING_ENGINEERING_DIAGNOSTIC',
        population_targets_blocking=False,
        mandatory_passed=report['minimum_passed'],
        percentage_targets_met=all(row['passed'] for row in report['population_targets']))
    value['percentage_status']='TARGETS_MET' if value['percentage_targets_met'] else 'TARGET_NOT_MET'
    return value

def validate_report(report,target,*,classified=True):
    if type(report) is not dict or type(report.get('count')) is not int or report['count']<=0:
        raise ValueError('MalformedDirectFillPopulationCount')
    n=report['count'];minimum=report.get('minimum')
    if type(minimum) not in (float,int) or not math.isfinite(minimum) or minimum<=0 or report.get('required_minimum')!=target['minimum']:
        raise ValueError('MalformedDirectFillMinimum')
    mp=minimum>=target['minimum']
    if type(report.get('minimum_passed')) is not bool or report['minimum_passed']!=mp:
        raise ValueError('ChangedDirectFillMinimumVerdict')
    rules=report.get('population_targets')
    if type(rules) is not list or len(rules)!=len(target['population_targets']):raise ValueError('MissingDirectFillTargets')
    for row,expected in zip(rules,target['population_targets']):
        if type(row) is not dict or row.get('minimum')!=expected['minimum'] or row.get('fraction')!=expected['fraction']:
            raise ValueError('ChangedDirectFillTarget')
        got=row.get('actual_count');need=math.ceil(Fraction(str(expected['fraction']))*n)
        if (type(got) is not int or not 0<=got<=n or type(row.get('required_count')) is not int or row['required_count']!=need
            or type(row.get('actual_fraction')) not in (float,int) or not math.isfinite(row['actual_fraction'])
            or row['actual_fraction']!=got/n or type(row.get('passed')) is not bool or row['passed']!=(got>=need)):
            raise ValueError('MalformedDirectFillTargetCounts')
    all_targets=all(row['passed'] for row in rules)
    if type(report.get('passed')) is not bool or report['passed']!=(mp and all_targets):raise ValueError('ChangedDirectFillDiagnosticVerdict')
    if 'minimum_actual_count' in report:
        got=report['minimum_actual_count']
        if type(got) is not int or not 0<=got<=n or (got==n)!=mp:raise ValueError('MalformedDirectFillMinimumCount')
    if classified and (report.get('contract_version')!=VERSION or report.get('contract_identity')!=contract_identity()
        or report.get('minimum_role')!='MANDATORY' or report.get('percentage_role')!='NONBLOCKING_ENGINEERING_DIAGNOSTIC'
        or report.get('population_targets_blocking') is not False or 'minimum_actual_count' not in report
        or type(report.get('mandatory_passed')) is not bool or report['mandatory_passed']!=mp
        or type(report.get('percentage_targets_met')) is not bool or report['percentage_targets_met']!=all_targets
        or report.get('percentage_status')!=('TARGETS_MET' if all_targets else 'TARGET_NOT_MET')):
        raise ValueError('ChangedDirectFillApplicability')
    return report

def acceptance(reports,policy):
    from finite_research.direct_contract import validate_policy
    validate_policy(policy)
    expected={'all_bulk_tetrahedra':'outer_tetrahedra','interface_tetrahedra':'outer_tetrahedra','preserved_cap_triangles':'outer_triangles'}
    if type(reports) is not dict or set(reports)!=set(expected):raise ValueError('CompleteDirectFillPopulationsRequired')
    for name,kind in expected.items():validate_report(reports[name],policy['quality_targets'][kind])
    return dict(contract=binding(policy),mandatory_minima_passed=all(r['mandatory_passed'] for r in reports.values()),
        percentage_targets_met=all(r['percentage_targets_met'] for r in reports.values()),
        percentage_status={n:r['percentage_status'] for n,r in reports.items()},
        scope='Shape applicability only; all twelve hard proofs, native fidelity, publication, cold readback and retirement remain separately mandatory.')

def require_minima(reports,policy):
    value=acceptance(reports,policy)
    if not value['mandatory_minima_passed']:raise ValueError('DirectFillMandatoryMinimumFailed')
    return value
