"""Reuse unchanged numerical rows during a fixed-topology local search.

All cells incident to any changed coordinate and all their internal faces are
recomputed with the original metric functions. Complete certificate generation
and production admission remain outside this provisional numerical evaluator.
"""
import numpy as np
import generalize_local_remesh as g
import final_shell_incidence as incidence


def failure_order(row):
    if row['kind'] in ('exact_cell', 'cell_quality'):
        return (0, row['id'], row['kind'] == 'cell_quality')
    if row['kind'] == 'face_quality':
        return (1, tuple(sorted(row['vertices'])))
    if row['kind'] == 'final_incidence_determinant':
        return (2, row['id'])
    raise ValueError('UnsupportedLocalQualityFailure:'+row['kind'])


class LocalCorrectionValidation:
    def __init__(self, x, triangles, fronts, levels, checked):
        self.points, self.cells, self.outer = g.s.construct(x, triangles, fronts, levels)
        self.owners, self.ownership = g.t.ownership(self.cells,self.outer,[tuple(reversed(f)) for f in triangles])
        self.checked = checked
        self.vertex_cells = {}
        for cell in self.cells:
            for vertex in set(cell['vertices']):
                self.vertex_cells.setdefault(vertex,set()).add(cell['id'])

    def evaluate(self, points):
        if points.shape != self.points.shape:
            raise ValueError('LocalCorrectionTopologyChanged')
        changed=np.flatnonzero(np.any(points != self.points,axis=1))
        active=set().union(*(self.vertex_cells.get(int(v),set()) for v in changed))
        local=g.t.quality(points,self.cells,self.owners,1,active_cells=active)
        final=incidence.check(points,self.cells,self.outer,active_cells=active)
        if 'first_prism_incidence' in self.checked:
            values={r['id']:r['determinant'] for r in final['cells']}
            local['cells']=[dict(r,determinant=values[r['id']]) for r in local['cells']]
            local['failures']=[r for r in local['failures'] if r['kind']!='cell_quality']
            local['failures'] += [dict(kind='cell_quality',**r) for r in local['cells']
                if not (r['volume']>0 and r['determinant']>=.0011 and r['aspect']<=1000)]
            # The same determinant is already represented by cell_quality.
            final['failures']=[]
        old=self.checked['quality']
        def merge(kind,key):
            replacement={key(row):row for row in local[kind]}
            return [replacement.get(key(row),row) for row in old[kind]]
        failures=[]
        for row in old['failures']:
            if row['kind'] in ('sdf_target','committed_cell_mutation'):
                continue  # Recomputed by the caller on this exact candidate.
            if row.get('id') in active or any(i in active for i in row.get('owners',[])):
                continue
            failures.append(row)
        failures=sorted(failures+local['failures']+final['failures'],key=failure_order)
        quality=dict(passed=not failures,failures=failures,
                     cells=merge('cells',lambda r:r['id']),exact=merge('exact',lambda r:r['id']),
                     faces=merge('faces',lambda r:tuple(sorted(r['vertices']))))
        replacement={r['id']:r for r in final['cells']}
        combined=dict(self.checked['complete_incidence'])
        combined['cells']=[replacement.get(r['id'],r) for r in combined['cells']]
        combined['failures']=[r for r in failures if r['kind']=='final_incidence_determinant']
        combined['passed']=not combined['failures']
        result=dict(quality=quality,ownership=self.checked['ownership'],
                    vertex_links=self.checked['vertex_links'],complete_incidence=combined,passed=False)
        if 'first_prism_incidence' in self.checked:
            result['first_prism_incidence']=self.checked['first_prism_incidence']
        return result
