"""Connection of existing provider metric terms; no new metric mathematics.

The coupled route never supplies metric_override. Its writer is the original
provider function with a private SF2-style post-shape scaling global. Other
provider users and the historical capsule path are not mutated.
"""
import dataclasses
import types
import numpy as np
from historical_capsule_surface import provider, SeedVertex
from prism_host_metric import intersect_tangent_metric
from shape_aware_metric import (generic_geometry_metric,
    generic_geometry_metric_from_differential, apply_shape_metric)


class CoupledSurfaceMetric:
    def __init__(self, authority, sizing, prism, shape, length_scale, source_identity):
        self.authority, self.sizing = authority, sizing
        self.prism, self.shape, self.length_scale = prism, shape, length_scale
        self.source_identity = source_identity

    def bounds(self, matrix):
        if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
            raise ValueError('InvalidComposedMetric')
        # write_contract serializes these six entries and reconstructs symmetry.
        serialized = np.triu(matrix) + np.triu(matrix, 1).T
        values = np.linalg.eigvalsh(serialized)
        if not np.all(values > 0):
            raise ValueError('NonSPDComposedMetric')
        # These are compatibility checks, NEVER eigenvalue clipping. A stronger
        # physical/feature requirement cannot be weakened to fit a size floor.
        if values.max() > 1 / self.sizing.minimum_size_m**2:
            raise ValueError('MetricMinimumSizeConflict')
        if values.min() < 1 / self.sizing.maximum_size_m**2:
            raise ValueError('MetricMaximumSizeConflict')
        return matrix

    def finish_shape(self, matrix, principal, policy):
        if policy != self.shape:
            raise ValueError('CoupledShapePolicyMismatch')
        result, stats = apply_shape_metric(matrix, principal, policy)
        result = self.bounds(result / (self.length_scale * self.length_scale))
        return result, dict(stats, resolution_length_scale=self.length_scale,
                            statistics_before_uniform_scale=True)

    def evaluate(self, point, sheet_ref, *, curves=(), differential=False, trace=False):
        vertex = SeedVertex(np.asarray(point, dtype=float), int(sheet_ref), 0, 0)
        fn = generic_geometry_metric_from_differential if differential else generic_geometry_metric
        geometry, principal = fn(self.authority, vertex, self.sizing.metric_multiplier)
        combined = geometry
        feature = None
        if curves:
            feature = self.authority.metric(vertex, set(curves), self.sizing.metric_multiplier)
            combined = provider.spd_intersection(combined, feature)
        host, host_stats = intersect_tangent_metric(combined,
            self.authority.exact_normal(vertex.sheet_ref, vertex.point), self.prism)
        result, shape_stats = self.finish_shape(host, principal, self.shape)
        if trace:
            return result, dict(geometry=geometry, feature=feature,
                geometry_feature=combined, prism=host, principal=principal,
                prism_statistics=host_stats, shape_statistics=shape_stats)
        return result

    def carrier(self, point, sheet_ref):
        # Exact historical SF2 predictor interface: no curve ledger is supplied
        # by assess_carrier_face; feature constraints enter the nodal writer.
        # The same host/shape/scale terms apply, with the existing differential
        # predictor (2.50 reserve), not a fabricated ownership classification.
        return self.evaluate(point, sheet_ref, differential=True)

    def write_contract(self, path, vertices, faces, edges, *, metric_override=None):
        if metric_override is not None:
            raise ValueError('CoupledMetricOverrideForbidden')
        # Reuse the entire provider composition, incidence and serialization.
        # Private globals reproduce resolution.install_surface without mutation
        # of the shared imported provider or duplication of write_contract.
        ns = dict(provider.write_contract.__globals__)
        ns['apply_shape_metric'] = self.finish_shape
        writer = types.FunctionType(provider.write_contract.__code__, ns,
            provider.write_contract.__name__, provider.write_contract.__defaults__)
        stats = writer(path, vertices, faces, edges, self.authority,
            self.sizing.metric_multiplier, prism_policy=self.prism,
            shape_policy=self.shape)
        stats['coupled_connection'] = dict(source_identity=self.source_identity,
            policy=dataclasses.asdict(self.prism), length_scale=self.length_scale,
            metric_override_used=False, clipping='none; incompatible eigenvalue bounds fail',
            composition='geometry + feature Loewner intersection -> prism tangent intersection -> shape -> M/scale**2')
        return stats
