"""Opt-in exact prism kernels. No scientific authority or predicate changes."""
from contextlib import contextmanager
from copy import deepcopy
import sqlite3
import json
import zlib
import numpy as np
from collections.abc import Mapping
from fractions import Fraction

_INSTALLED=False


def exact_centroid(points,c):
    """Exact rational mean of binary64 inputs, with one reduction per axis."""
    ids=sorted(set(c['vertices']))
    if not ids:raise ZeroDivisionError('EmptyCellWitness')
    result=[]
    for j in range(3):
        values=[float(points[v,j]).as_integer_ratio() for v in ids]
        denominator=max(d for _,d in values)
        numerator=sum(n*(denominator//d) for n,d in values)
        result.append(Fraction(numerator,denominator*len(ids)))
    return tuple(result)


def exact_boundary(points,face):
    values=[[float(v).as_integer_ratio() for v in points[i]] for i in face]
    denominator=max(d for row in values for _,d in row)
    a,b,c=[[n*(denominator//d) for n,d in row] for row in values[:3]]
    triple=a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0])
    return Fraction(triple,6*denominator**3)


class ReadCache(Mapping):
    """Function-local reads for consumers that do not mutate returned values."""
    def __init__(self,source):self.source=source;self.values={};self.missing=set()
    def __len__(self):return len(self.source)
    def __iter__(self):return iter(self.source)
    def __getitem__(self,key):
        if key in self.missing:raise KeyError(key)
        if key not in self.values:
            try:self.values[key]=self.source[key]
            except KeyError:
                self.missing.add(key);raise
        return self.values[key]


@contextmanager
def cached_component_views(base):
    """Reuse map wrappers, not record payloads, within a guarded operation."""
    from composable_state import CommittedState
    from unittest.mock import patch
    original=CommittedState.__getitem__;views={};base.check()
    def get(state,key):
        state.check_live()
        if key not in ('cells','faces','vertices','versions'):return original(state,key)
        token=(id(state),key)
        if token not in views:views[token]=original(state,key)
        return views[token]
    with patch.object(CommittedState,'__getitem__',get):
        yield
    base.check()


def reverse_cycle(a,b):
    a=tuple(a);b=tuple(reversed(b))
    return any(a==b[-i:]+b[:-i] if i else a==b for i in range(len(a)))


def prism_cell(ids,column,layer):
    import local_termination_topology as t
    if not 0<=column<1000000 or layer<1:raise ValueError('StudyCellIdRange')
    ids=list(map(int,ids))
    if len(ids)!=6 or len(set(ids))!=6:raise ValueError('UncollapsedPrismRequired')
    return dict(id=layer*1000000+column,column=column,layer=layer,kind='prism',vertices=ids,
        faces=[tuple(ids[j] for j in role) for role in t.ROLES])


def batch_geometry(points,cells):
    """Same Foundation face kernels and ordered cell reductions, batched."""
    import local_termination_topology as t
    if not cells:return {}
    if any(c['kind']!='prism' or len(c['faces'])!=5 for c in cells):
        raise ValueError('PrismBatchRequiresFiveFacePrisms')
    fg=t.q.legacy_modules()['component_quality'].FG
    sf=np.empty((len(cells),5,3));cf=np.empty_like(sf)
    for role in range(5):
        ids=np.asarray([c['faces'][role] for c in cells],dtype=np.int64)
        fn=fg.face_area_centre_tri if ids.shape[1]==3 else fg.face_area_centre_quad
        sf[:,role],cf[:,role]=fn(points[ids])
    est=cf.mean(axis=1)
    pyr=np.einsum('nfi,nfi->nf',sf,cf-est[:,None,:])
    # Builtin sum in the qualified implementation is left-to-right.
    total=sum(pyr[:,j] for j in range(5))
    volumes=total/3
    centres=np.sum(pyr[:,:,None]*(.75*cf+.25*est[:,None,:]),axis=1)/total[:,None]
    bounds=abs(sf).sum(axis=1)
    result={}
    for j,c in enumerate(cells):
        B=bounds[j];V=volumes[j]
        aspect=max(max(B)/min(B),sum(B)/(6*abs(V)**(2/3))) if V else float('inf')
        result[c['id']]=dict(centre=centres[j],volume=float(V),areas=sf[j],face_centres=cf[j],aspect=float(aspect))
    return result


class CompactQuality:
    """Ordered Python reductions; no receipt input or scientific cache."""
    def __init__(self):
        self.cells = self.faces = self.nonpositive = 0

    def cell(self, cell_id, determinant, aspect, nonpositive):
        if not self.cells:
            self.determinant = determinant
            self.aspect = aspect
        else:
            if determinant < self.determinant: self.determinant = determinant
            if aspect > self.aspect: self.aspect = aspect
        self.cells += 1
        self.nonpositive += nonpositive

    def face(self, key, weight, skewness, ratio):
        if not self.faces:
            self.weight, self.skewness, self.ratio = weight, skewness, ratio
        else:
            if weight < self.weight: self.weight = weight
            if skewness > self.skewness: self.skewness = skewness
            if ratio < self.ratio: self.ratio = ratio
        self.faces += 1

    def result(self):
        # The caller's previous summary finalized after all calculations.
        if not self.cells or not self.faces:
            raise ValueError('min() arg is an empty sequence')
        return dict(affected_cells=self.cells, affected_faces=self.faces,
            minimum_determinant=self.determinant, maximum_aspect=self.aspect,
            minimum_weight=self.weight, maximum_skewness=self.skewness,
            minimum_symmetric_volume_ratio=self.ratio,
            exact_nonpositive_cells=self.nonpositive)

def batch_quality(points,cells,owners,base_layer,*,active_cells=None,_geometry_out=None,face_batch_size=None,mixed_cells=False,compact=False,_dense_face_values=True,_handoff=None):
    # Default retains the qualified capsule's original whole-arity call. The
    # joined core can bound only independent Foundation face evaluations; cell
    # geometry, exact predicates, neighbour closure and record order stay below.
    if face_batch_size is not None and (type(face_batch_size) is not int or face_batch_size<1):
        raise ValueError('InvalidQualityFaceBatchSize')
    if type(mixed_cells) is not bool:raise ValueError('InvalidMixedQualityExecution')
    if type(compact) is not bool or (compact and not mixed_cells):raise ValueError('InvalidCompactMixedQualityOutput')
    if type(_dense_face_values) is not bool:raise ValueError('InvalidDenseFaceValues')
    import local_termination_topology as t
    cq=t.q.legacy_modules()['component_quality'];byid={c['id']:c for c in cells}
    selected={c['id'] for c in cells if c['layer']>=base_layer and (active_cells is None or c['id'] in active_cells)}
    needed=set(selected)
    for rows in owners.values():
        if any(i in selected for i,f in rows):needed.update(i for i,f in rows)
    inherited_geometry={};inherited_exact={};inherited_determinants={}
    if _handoff is not None:
        from rolling_fast_state import BackgroundInputs
        if type(_handoff) is not BackgroundInputs or not mixed_cells:raise ValueError('ProtectedMixedQualityInputsRequired')
        inherited_geometry,inherited_exact,inherited_determinants=_handoff.quality_inputs(points,byid,owners,needed,selected)
    if mixed_cells:
        # Fresh geometry for this call, including complete one-hop neighbour
        # closure. No caller cache or stored receipt can replace the calculation.
        from background_mixed_geometry import batch_geometry as mixed_geometry
        from collections import defaultdict
        groups=defaultdict(list);geom=dict(inherited_geometry);calculated_determinants=dict(inherited_determinants)
        tensor_fast=(type(cells) is list and len(byid)==len(cells) and
                     all(type(c) is dict and type(c['id']) is int for c in cells))
        for i in sorted(needed):
            if i in inherited_geometry:continue
            groups[tuple(map(len,byid[i]['faces']))].append(byid[i])
        for group in groups.values():
            for start in range(0,len(group),4096):
                batch=group[start:start+4096]
                # Bounded selection flags from this call's own cells. Never
                # rescan the whole selected population in each small batch.
                flags=tuple(c['id'] in selected and c['id'] not in inherited_determinants for c in batch) if tensor_fast else None
                geom.update(mixed_geometry(points,batch,
                    _determinants=calculated_determinants if tensor_fast else None,
                    _owners=owners,_selected=flags))
        if _geometry_out is not None:
            if _geometry_out:raise ValueError('NonemptyQualityGeometryOutput')
            _geometry_out.update(geom)
    elif _geometry_out is None:
        # Keep the qualified capsule's guarded/persistent geometry session.
        geom={i:t.geometry(points,byid[i]) for i in sorted(needed)}
    else:
        # Output only, never a caller-supplied scientific geometry cache. The
        # incremental transaction owns these values for this points/topology.
        if _geometry_out:raise ValueError('NonemptyQualityGeometryOutput')
        ordered_cells=[byid[i] for i in sorted(needed)]
        arities={tuple(map(len,c['faces'])) for c in ordered_cells}
        if (all(c['kind']=='prism' and len(c['faces'])==5 for c in ordered_cells)
                and len(arities)<=1 and all(n in (3,4) for a in arities for n in a)):
            geom={}
            for start in range(0,len(ordered_cells),4096):
                geom.update(batch_geometry(points,ordered_cells[start:start+4096]))
        else:
            geom={i:t.geometry(points,byid[i]) for i in sorted(needed)}
        _geometry_out.update(geom)
    bad=[];records=[];exact=[]
    summary=CompactQuality() if compact else None
    for c in cells:
        if c['id'] not in selected:continue
        i=c['id'];g=geom[i]
        if i in inherited_exact:valid=inherited_exact[i]
        elif mixed_cells and c['kind']=='tetrahedron':
            from background_mixed_geometry import exact_tetrahedron
            valid=exact_tetrahedron(points,c,fallback=t.exact_cell,positive_signs_only=compact)
        else:valid=t.exact_cell(points,c)
        if not compact:exact.append(dict(id=i,**valid))
        if valid['jacobian_sign']<=0 or valid['volume_sign']<=0:bad.append(dict(kind='exact_cell',id=i,column=c['column'],detail=valid))
        if mixed_cells and i in calculated_determinants:det=calculated_determinants[i]
        else:
            mask=np.array([len(owners[tuple(sorted(f))])==2 for f in c['faces']]);S=g['areas'][mask]
            avg=np.linalg.norm(S,axis=1).mean();tensor=(S/avg).T@(S/avg);det=float(np.linalg.det(tensor))
        if not compact:
            rec=dict(id=i,column=c['column'],cell_type=c['kind'],volume=g['volume'],determinant=det,aspect=g['aspect']);records.append(rec)
            if not g['volume']>0 or not det>=.0011 or not g['aspect']<=1000:bad.append(dict(kind='cell_quality',**rec))
        else:
            column=c['column'];cell_type=c['kind'];cell_volume=g['volume'];aspect=g['aspect']
            if not g['volume']>0 or not det>=.0011 or not g['aspect']<=1000:
                rec=dict(id=i,column=column,cell_type=cell_type,volume=cell_volume,determinant=det,aspect=aspect)
                bad.append(dict(kind='cell_quality',**rec))
            summary.cell(i,det,g['aspect'],valid['volume_sign']<=0 or valid['jacobian_sign']<=0)
    ordered=[(key,rows) for key,rows in sorted(owners.items()) if len(rows)==2 and any(i in selected for i,f in rows)]
    dense=(compact and _dense_face_values and type(owners) in (dict,defaultdict)
        and all(len(rows[0][1]) in (3,4) for _,rows in ordered))
    values=np.empty((len(ordered),4),dtype=np.float64) if dense else {}
    for arity in (3,4):
        group=([j for j,(_,rows) in enumerate(ordered) if len(rows[0][1])==arity] if dense
            else [(key,rows) for key,rows in ordered if len(rows[0][1])==arity])
        if not group:continue
        size=len(group) if face_batch_size is None else face_batch_size
        for start in range(0,len(group),size):
            selected_group=group[start:start+size]
            batch=[ordered[j] for j in selected_group] if dense else selected_group
            faces=[rows[0][1] for _,rows in batch]
            a=[rows[0][0] for _,rows in batch];b=[rows[1][0] for _,rows in batch]
            vs=points[np.asarray(faces,dtype=np.int64)]
            S,C=(cq.FG.face_area_centre_tri if arity==3 else cq.FG.face_area_centre_quad)(vs)
            vals=cq.face_metrics(vs,S,C,np.asarray([geom[i]['centre'] for i in a]),np.asarray([geom[i]['centre'] for i in b]),
                np.asarray([geom[i]['volume'] for i in a]),np.asarray([geom[i]['volume'] for i in b]))
            if dense:values[selected_group]=np.column_stack(vals)
            else:
                for j,(key,_) in enumerate(batch):values[key]=[float(v[j]) for v in vals]
    faces=[]
    for j,(key,rows) in enumerate(ordered):
        (a,f),(b,_)=rows
        if dense:wt,sk,vr,no=map(float,values[j])
        else:wt,sk,vr,no=values[key]
        if not compact:
            rec=dict(vertices=f,owners=[a,b],weight=wt,skewness=sk,symmetric_volume_ratio=vr);faces.append(rec)
            if not wt>=.055 or not sk<=3.63636 or not vr>=.01:bad.append(dict(kind='face_quality',**rec))
        else:
            if not wt>=.055 or not sk<=3.63636 or not vr>=.01:
                rec=dict(vertices=f,owners=[a,b],weight=wt,skewness=sk,symmetric_volume_ratio=vr)
                bad.append(dict(kind='face_quality',**rec))
            summary.face(key,wt,sk,vr)
    if compact:return dict(passed=not bad,failures=bad,compact_output=True,summary=summary.result())
    return dict(passed=not bad,failures=bad,cells=records,faces=faces,exact=exact)


def parallel_geometry(points,cells,workers=4):
    from concurrent.futures import ThreadPoolExecutor
    if type(workers) is not int or not 1<=workers<=24:raise ValueError('InvalidGeometryWorkerCount')
    if workers==1:return batch_geometry(points,cells)
    with ThreadPoolExecutor(workers) as pool:
        parts=pool.map(lambda c:batch_geometry(points,c),[cells[i:i+4096] for i in range(0,len(cells),4096)])
        return {key:value for part in parts for key,value in part.items()}


def completed_incidence(points,cells,outer,owners,*,active_cells=None):
    import local_termination_topology as t
    interface={tuple(sorted(f)) for f in outer};rows=[];failures=[]
    for c in cells:
        if active_cells is not None and c['id'] not in active_cells:continue
        geometry=t.geometry(points,c)
        mask=np.array([len(owners[tuple(sorted(f))])==2 or tuple(sorted(f)) in interface for f in c['faces']])
        S=geometry['areas'][mask]
        avg=np.linalg.norm(S,axis=1).mean();tensor=(S/avg).T@(S/avg)
        determinant=float(np.linalg.det(tensor))
        row=dict(id=c['id'],column=c['column'],cell_type=c['kind'],determinant=determinant)
        rows.append(row)
        if not determinant>=.0011:failures.append(dict(kind='final_incidence_determinant',**row))
    return dict(passed=not failures,cells=rows,failures=failures,
        scope='Cell determinant only, with every declared shell/core interface face internal; actual core neighbour metrics still required')


@contextmanager
def cached_quality_reads(base):
    """Point queries share connections while existing deny-write guards are held.

    Every database is verified before use and after the batch. Recursive roots
    are checked at both boundaries. No old row population is materialized.
    """
    from pathlib import Path
    from unittest.mock import patch
    from composable_state import Rows
    original=Rows.__getitem__;connections={};base.check()
    def get(rows,key):
        o=rows.owner;o.check_live()
        if rows.kind!='quality' or o.kind!='recursive':return original(rows,key)
        o.stats['quality_lookups']+=1
        if key in o.delta['removed'].get('quality',[]):raise KeyError(key)
        o.stats['quality_components_touched']+=1
        if id(o) not in connections:
            path=o.verify(Path(o.body['quality_delta']))
            db=sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1',uri=True)
            connections[id(o)]=(o,path,db)
        db=connections[id(o)][2]
        o.stats['quality_point_queries']+=1
        row=db.execute('SELECT inputs,payload FROM updates WHERE key=?',(key,)).fetchone()
        if row is not None:o.stats['quality_rows_visited']+=1
        if row is None or row[1] is None:
            o.stats['quality_parent_lookups']+=1
            value=deepcopy(o.parent['quality'][key])
        else:value=json.loads(zlib.decompress(row[1]))
        if row is not None:value['inputs']=json.loads(zlib.decompress(row[0]))
        return value
    try:
        with patch.object(Rows,'__getitem__',get):yield
        for o,path,_ in connections.values():o.verify(path)
        base.check()
    finally:
        for _,_,db in connections.values():db.close()


def install():
    global _INSTALLED
    if _INSTALLED:return
    from optimized_admission import install as prior
    prior()
    import local_termination_topology as t
    import composable_transition as transition
    import composable_assembly as assembly
    import incremental_pair_storage
    from prism_proof_index import write_proof_index
    incremental_pair_storage.write_proof_index=write_proof_index
    import recursive_production_publication
    from prism_sql_profile import measured_quality_writer
    from composable_assembly import read
    publish=recursive_production_publication.publish
    def published(base,adapter,out,parent):
        info=read(out/'ASSEMBLY.json')
        with cached_component_views(base),measured_quality_writer(out,info['quality_new']+info['old_quality_rows_examined'],adaptive=True):
            return publish(base,adapter,out,parent)
    recursive_production_publication.publish=published
    from unittest.mock import patch
    t.reverse_cycle=reverse_cycle
    original_cell=t.cell
    def cell(ids,column,layer):
        ids=list(ids)
        if len(ids)==6 and len(set(ids))==6:return prism_cell(ids,column,layer)
        return original_cell(ids,column,layer)
    t.cell=cell
    produce=transition.produce;assemble=assembly.assemble
    append=transition.append_components
    def appended(before,*a,**kw):
        from collections import ChainMap
        maps=ChainMap({key:ReadCache(before[key]) for key in ('vertices','cells','faces','versions')},before)
        return append(maps,*a,**kw)
    transition.append_components=appended
    for module in (transition,assembly):
        original=module.geometry_inputs
        def inputs(state,original=original):
            return original(dict(state,vertices=ReadCache(state['vertices'])))
        module.geometry_inputs=inputs
    def generation(*args,**kwargs):
        from incremental_layer_validation import ExactVolume
        import final_shell_incidence
        import cavity_volume_audit
        from prism_degree import degree
        geometry=t.geometry;quality=t.quality;cache={};geometry_points=None
        original_ownership=t.ownership;original_incidence=final_shell_incidence.check;ownership_cache=[]
        def ownership(cells,*a,**kw):
            result=original_ownership(cells,*a,**kw)
            ownership_cache[:]=[cells,result[0]]
            return result
        def incidence(points,cells,outer,**kw):
            if ownership_cache and cells is ownership_cache[0]:
                return completed_incidence(points,cells,outer,ownership_cache[1],**kw)
            return original_incidence(points,cells,outer,**kw)
        def cached(points,c):
            nonlocal geometry_points
            if geometry_points is not points:
                cache.clear();geometry_points=points
            if c['id'] not in cache:cache[c['id']]=geometry(points,c)
            return cache[c['id']]
        def quality_batch(points,cells,*a,**kw):
            nonlocal geometry_points
            geometry_points=points
            cache.update(parallel_geometry(points,cells))
            return batch_quality(points,cells,*a,**kw)
        with cached_component_views(args[0]),patch.object(t,'geometry',cached),patch.object(t,'quality',quality_batch),patch.object(t,'ownership',ownership),patch.object(final_shell_incidence,'check',incidence),patch.object(ExactVolume,'sample',staticmethod(exact_centroid)),patch.object(ExactVolume,'contribution',staticmethod(exact_boundary)),patch.object(cavity_volume_audit,'degree',degree):
            return produce(*args,**kwargs)
    def assembled(base,*args,**kwargs):
        with cached_component_views(base),cached_quality_reads(base):return assemble(base,*args,**kwargs)
    transition.produce=generation;assembly.assemble=assembled
    _INSTALLED=True
