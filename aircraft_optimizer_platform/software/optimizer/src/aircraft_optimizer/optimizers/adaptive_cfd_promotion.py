from __future__ import annotations

from dataclasses import dataclass
from statistics import median
from typing import Any


@dataclass(frozen=True)
class AdaptiveCfdPromotionPolicy:
    """Adaptive first-pass CFD promotion policy.

    The first few candidates should run the expanded alpha sweep to establish a
    campaign-local baseline. After that, first-pass results are compared against
    that baseline and only clearly weak/broken candidates are rejected.
    """

    min_expanded_baseline_candidates: int = 5
    first_pass_compare_alpha_deg: float = 4.0
    max_lift_loss_fraction_vs_baseline_best: float = 0.65
    max_abs_coefficient: float = 10.0
    require_positive_lift_at_any_sampled_alpha: bool = True


@dataclass(frozen=True)
class AdaptiveCfdPromotionDecision:
    action: str
    reason: str
    promote_to_expanded_sweep: bool
    reject_candidate: bool
    baseline: dict[str, Any]
    comparisons: dict[str, Any]


def decide_adaptive_cfd_promotion(
    *,
    candidate_first_pass: dict[str, Any],
    expanded_baseline_sweeps: list[dict[str, Any]],
    policy: AdaptiveCfdPromotionPolicy | None = None,
) -> AdaptiveCfdPromotionDecision:
    active_policy = policy or AdaptiveCfdPromotionPolicy()
    baseline = _baseline_envelope(
        expanded_baseline_sweeps,
        compare_alpha_deg=active_policy.first_pass_compare_alpha_deg,
    )
    comparisons = _candidate_comparisons(
        candidate_first_pass,
        baseline,
        compare_alpha_deg=active_policy.first_pass_compare_alpha_deg,
    )

    if baseline["expanded_baseline_count"] < active_policy.min_expanded_baseline_candidates:
        return AdaptiveCfdPromotionDecision(
            action="run_expanded_baseline_sweep",
            reason="expanded_baseline_not_established",
            promote_to_expanded_sweep=True,
            reject_candidate=False,
            baseline=baseline,
            comparisons=comparisons,
        )

    hard_failure = _hard_failure_reason(
        candidate_first_pass,
        max_abs_coefficient=active_policy.max_abs_coefficient,
        require_positive_lift=active_policy.require_positive_lift_at_any_sampled_alpha,
    )
    if hard_failure is not None:
        return AdaptiveCfdPromotionDecision(
            action="reject_after_first_pass",
            reason=hard_failure,
            promote_to_expanded_sweep=False,
            reject_candidate=True,
            baseline=baseline,
            comparisons=comparisons,
        )

    candidate_cl = comparisons.get("candidate_cl_at_compare_alpha")
    best_baseline_cl = baseline.get("best_cl_at_compare_alpha")
    if candidate_cl is None:
        return AdaptiveCfdPromotionDecision(
            action="promote_to_expanded_sweep",
            reason="missing_compare_alpha_so_result_is_uncertain",
            promote_to_expanded_sweep=True,
            reject_candidate=False,
            baseline=baseline,
            comparisons=comparisons,
        )
    if best_baseline_cl and best_baseline_cl > 0.0:
        loss_fraction = 1.0 - (candidate_cl / best_baseline_cl)
        comparisons["lift_loss_fraction_vs_baseline_best"] = loss_fraction
        if loss_fraction >= active_policy.max_lift_loss_fraction_vs_baseline_best:
            return AdaptiveCfdPromotionDecision(
                action="reject_after_first_pass",
                reason="compare_alpha_lift_far_below_baseline",
                promote_to_expanded_sweep=False,
                reject_candidate=True,
                baseline=baseline,
                comparisons=comparisons,
            )

    return AdaptiveCfdPromotionDecision(
        action="promote_to_expanded_sweep",
        reason="first_pass_not_clearly_bad",
        promote_to_expanded_sweep=True,
        reject_candidate=False,
        baseline=baseline,
        comparisons=comparisons,
    )


def adaptive_cfd_promotion_report(
    *,
    expanded_baseline_sweeps: list[dict[str, Any]],
    candidate_first_pass: dict[str, Any] | None = None,
    policy: AdaptiveCfdPromotionPolicy | None = None,
) -> dict[str, Any]:
    active_policy = policy or AdaptiveCfdPromotionPolicy()
    baseline = _baseline_envelope(
        expanded_baseline_sweeps,
        compare_alpha_deg=active_policy.first_pass_compare_alpha_deg,
    )
    best_cl = baseline.get("best_cl_at_compare_alpha")
    threshold = (
        float(best_cl) * (1.0 - active_policy.max_lift_loss_fraction_vs_baseline_best)
        if best_cl is not None
        else None
    )
    report: dict[str, Any] = {
        "policy": {
            "min_expanded_baseline_candidates": active_policy.min_expanded_baseline_candidates,
            "first_pass_compare_alpha_deg": active_policy.first_pass_compare_alpha_deg,
            "max_lift_loss_fraction_vs_baseline_best": (
                active_policy.max_lift_loss_fraction_vs_baseline_best
            ),
            "max_abs_coefficient": active_policy.max_abs_coefficient,
            "require_positive_lift_at_any_sampled_alpha": (
                active_policy.require_positive_lift_at_any_sampled_alpha
            ),
        },
        "baseline": baseline,
        "promotion_bar": {
            "baseline_ready": (
                baseline["expanded_baseline_count"]
                >= active_policy.min_expanded_baseline_candidates
            ),
            "compare_alpha_deg": active_policy.first_pass_compare_alpha_deg,
            "best_baseline_cl_at_compare_alpha": best_cl,
            "reject_if_cl_at_compare_alpha_lte": threshold,
            "reject_if_lift_loss_fraction_gte": (
                active_policy.max_lift_loss_fraction_vs_baseline_best
            ),
            "hard_reject_reasons": [
                "missing_force_coefficients",
                "invalid_non_positive_drag",
                "absurd_force_coefficient",
                "no_positive_lift_at_sampled_alphas",
                "compare_alpha_lift_far_below_baseline",
            ],
        },
    }
    if candidate_first_pass is not None:
        decision = decide_adaptive_cfd_promotion(
            candidate_first_pass=candidate_first_pass,
            expanded_baseline_sweeps=expanded_baseline_sweeps,
            policy=active_policy,
        )
        report["candidate"] = {
            "decision": {
                "action": decision.action,
                "reason": decision.reason,
                "promote_to_expanded_sweep": decision.promote_to_expanded_sweep,
                "reject_candidate": decision.reject_candidate,
            },
            "comparisons": decision.comparisons,
        }
    return report


def _baseline_envelope(
    sweeps: list[dict[str, Any]],
    *,
    compare_alpha_deg: float,
) -> dict[str, Any]:
    expanded = [
        sweep
        for sweep in sweeps
        if _is_expanded_sweep(sweep)
    ]
    cl_values = [
        float(point["cl"])
        for sweep in expanded
        for point in [_point_at_alpha(sweep, compare_alpha_deg)]
        if point is not None and point.get("cl") is not None
    ]
    ld_values = [
        float(point["ld"])
        for sweep in expanded
        for point in [_point_at_alpha(sweep, compare_alpha_deg)]
        if point is not None and point.get("ld") is not None
    ]
    score_values = [
        float(sweep["score"])
        for sweep in expanded
        if sweep.get("score") is not None
    ]
    return {
        "expanded_baseline_count": len(expanded),
        "compare_alpha_deg": compare_alpha_deg,
        "best_cl_at_compare_alpha": max(cl_values, default=None),
        "median_cl_at_compare_alpha": median(cl_values) if cl_values else None,
        "best_ld_at_compare_alpha": max(ld_values, default=None),
        "median_score": median(score_values) if score_values else None,
    }


def _candidate_comparisons(
    sweep: dict[str, Any],
    baseline: dict[str, Any],
    *,
    compare_alpha_deg: float,
) -> dict[str, Any]:
    point = _point_at_alpha(sweep, compare_alpha_deg)
    candidate_cl = float(point["cl"]) if point and point.get("cl") is not None else None
    best_baseline_cl = baseline.get("best_cl_at_compare_alpha")
    ratio = (
        candidate_cl / float(best_baseline_cl)
        if candidate_cl is not None and best_baseline_cl
        else None
    )
    return {
        "compare_alpha_deg": compare_alpha_deg,
        "candidate_cl_at_compare_alpha": candidate_cl,
        "candidate_ld_at_compare_alpha": (
            float(point["ld"]) if point and point.get("ld") is not None else None
        ),
        "cl_fraction_of_baseline_best": ratio,
    }


def _hard_failure_reason(
    sweep: dict[str, Any],
    *,
    max_abs_coefficient: float,
    require_positive_lift: bool,
) -> str | None:
    points = [
        point
        for point in sweep.get("points", [])
        if point.get("cd") is not None
        and point.get("cl") is not None
        and point.get("cm") is not None
    ]
    if not points:
        return "missing_force_coefficients"
    for point in points:
        cd = float(point["cd"])
        cl = float(point["cl"])
        cm = float(point["cm"])
        if cd <= 0.0:
            return "invalid_non_positive_drag"
        if any(abs(value) > max_abs_coefficient for value in (cd, cl, cm)):
            return "absurd_force_coefficient"
    if require_positive_lift and all(float(point["cl"]) <= 0.0 for point in points):
        return "no_positive_lift_at_sampled_alphas"
    return None


def _is_expanded_sweep(sweep: dict[str, Any]) -> bool:
    if sweep.get("stage_role") == "survivor_expanded_rough_scoring":
        return True
    completed = sweep.get("completed_alpha_count")
    if completed is not None and int(completed) >= 5:
        return True
    return len(sweep.get("points", [])) >= 5


def _point_at_alpha(
    sweep: dict[str, Any],
    alpha_deg: float,
    *,
    tolerance: float = 1e-6,
) -> dict[str, Any] | None:
    for point in sweep.get("points", []):
        if abs(float(point.get("alpha_deg", 1e9)) - alpha_deg) <= tolerance:
            return point
    return None
