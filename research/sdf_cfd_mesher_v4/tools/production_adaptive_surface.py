"""Production dispatch to the retained Authority-driven surface stages."""
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalGeometry, GeometryFailure
from sdf_cfd_mesher_v4.sdf.cgal import run_cgal
# Explicit compatibility exports; no hidden class or shape fallback.
from historical_capsule_surface import (
    OUT,V3,V4,REPO,CapsuleGeometry,SurfaceSizing,canonical,metric,seed_size,
    require_production_surface,sha,save,wsl,run_native,identity,array_hash,write_off)
from surface_binding import SurfaceBinding,bind_canonical


class ProductionAdaptiveSdfSurfaceMesher:
    def __init__(self, *, sizing_policy=None):
        self.sizing_policy=sizing_policy

    def admission(self, geometry, enclosing_region=None):
        binding=geometry if isinstance(geometry,SurfaceBinding) else bind_canonical(geometry)
        if enclosing_region is not None and binding.canonical is not None:
            enclosing_region.validate(binding.canonical)
        report=binding.inspect()
        stages={stage:dict(admitted=True) for stage in (
            'scalar_query',('stock_sdf_seed' if binding.seed_route=='CanonicalMarchingCubesSeed/1' else 'cgal_callback'),'initial_projection','carrier_refinement',
            'tensor_sizing','mmgs_adaptation','exact_projection','independent_qualification')}
        for issue in report['issues']:
            stages.setdefault(issue['stage'],dict(admitted=True))
            stages[issue['stage']]['admitted']=False
            stages[issue['stage']].setdefault('failures',[]).append(issue)
        report.update(stages=stages,production_surface_qualified=False)
        return report

    def cgal(self, geometry, invocation, enclosing_region, *, timeout_seconds, maximum_queries):
        self.admission(geometry,enclosing_region)
        return run_cgal(geometry,invocation,enclosing_region,
                        timeout_seconds=timeout_seconds,maximum_queries=maximum_queries)

    def generate(self, geometry, output, *, enclosing_region=None):
        binding=geometry if isinstance(geometry,SurfaceBinding) else bind_canonical(geometry)
        report=self.admission(binding,enclosing_region)
        if self.sizing_policy is not None and self.sizing_policy != binding.sizing_policy:
            raise GeometryFailure('SizingBindingMismatch','tensor_sizing',
                'sizing belongs to the executable binding',binding.geometry_identity)
        if not report['executable']:
            first=report['issues'][0]
            raise GeometryFailure('AuthorityOperationUnbound',first['stage'],
                'see all operation-specific binding failures',binding.geometry_identity,
                operation=first['operation'],failures=report['issues'])
        from surface_execution import execute_surface
        try:
            return execute_surface(binding,output)
        except GeometryFailure:
            raise
        except Exception as error:
            from pathlib import Path
            record=dict(code='SurfaceExecutionFailed',geometry_identity=binding.geometry_identity,
                        stage='surface_execution',operation=type(error).__name__,detail=str(error))
            if Path(output).is_dir():save(Path(output)/'FAILURE.json',record)
            raise GeometryFailure('SurfaceExecutionFailed','surface_execution',str(error),
                                  binding.geometry_identity,operation=type(error).__name__) from error


if __name__=='__main__':
    raise SystemExit('Use an explicit SurfaceBinding. Historical replay: historical_capsule_surface.py <label>.')
