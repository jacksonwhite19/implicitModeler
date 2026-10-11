"""Sealed-state architecture controls. Forecast and legacy restart are denied."""
from pathlib import Path
import json, sys, time, sqlite3
from copy import deepcopy
from collections import Counter
from unittest.mock import patch
import psutil
from composable_state import CommittedState, plan_transition, publish_control, record_hash
from clean_sheet_l4_bootstrap import SURFACE, REQUEST, metrics
from production_surface_authority import load_production_wall
from geometry_family_authority import Source
from production_forecast_admission import BoundProductionForecastCache
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from path_preserving_deduplication import durable_json, file_hash
from sdf_cfd_mesher_v4.common.authority import identity

V4=Path(__file__).resolve().parents[1]
OUT=V4/'results/composable_incremental_state'
PRIOR=V4/'results/clean_sheet_incremental_l4_to_l5/resume'
L5='686c220a55965e0ee2868f2ab8502c6cda3ef6b6db8e97fca4806cf04afc0ec7'
SQL=Counter()


class CountCursor(sqlite3.Cursor):
    def execute(self,sql,*args):
        low=sql.lower(); self.group=next((x for x in ('receipts','updates','proof_keys') if 'from '+x in low),'other')
        SQL[self.group+'_queries']+=1
        return super().execute(sql,*args)
    def fetchone(self):
        row=super().fetchone()
        if row is not None: SQL[self.group+'_rows']+=1
        return row
    def fetchall(self):
        rows=super().fetchall();SQL[self.group+'_rows']+=len(rows);return rows
    def __next__(self):
        row=super().__next__();SQL[self.group+'_rows']+=1;return row


class CountConnection(sqlite3.Connection):
    def execute(self,*args): return self.cursor(factory=CountCursor).execute(*args)


def ref():
    return dict(kind='clean_sheet',directory=str((PRIOR/'candidate').resolve()),
        root='126b582745596b58a7f191f936e3a7906170c20e665ed6c9d226d52cc4566e51',
        seal=str((PRIOR/'CANDIDATE_SEAL.json').resolve()),
        seal_sha256='338b128a0a2376c36d568594c7128025667886801b888427e380dc7f31ad7ced')


def denied(*a,**kw): raise AssertionError('NoAdmissionForecastGeometryOrLegacyRestartAuthorized')


def adapter(state):
    wall,tri,surface=load_production_wall(SURFACE)
    source=Source(dict(components=[json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition']]))
    return state.initialize(wall,tri,surface,REQUEST,source)


def measured(): return time.perf_counter(),time.process_time(),psutil.Process().io_counters()


def normal():
    prior=json.loads((V4/'results/layer6_incremental_readiness/PREFLIGHT.json').read_text())
    for path,h in prior['protected_files'].items(): assert file_hash(path)==h
    with patch.object(sqlite3,'connect',denied):
        start=measured()
        with CommittedState(ref()) as state:
            assert state.scientific==L5
            a=adapter(state); plan=plan_transition(state,a,6)
            assert plan['increment_hex']=='0x1.744eb7e7e7516p-12'
            assert (plan['lower_vertex_offset'],plan['upper_vertex_offset'])==(84475,101370)
            assert (plan['first_cell_id'],plan['last_cell_id'])==(6000000,6033785)
            result=dict(plan=plan,metrics=metrics(*start),stats=dict(state.stats),
                candidate_stats=dict(state.wrapped.stats),native_stats=dict(state.wrapped.base.stats),
                sqlite_rows=0,proof_rows=0,quality_rows=0,full_prefix_hashes=0,full_prefix_deep_copies=0,
                admission=False,forecast=False,generation=False,core=False,scientific=state.scientific)
        native_root=json.loads((PRIOR.parent/'L4_ENROLLMENT.json').read_text())['root']
        with CommittedState(dict(kind='native',directory=str(PRIOR.parent/'L4_access'),root=native_root)) as state:
            native_plan=plan_transition(state,adapter(state),5)
            assert native_plan['increment_hex']=='0x1.37351a26971d3p-12'
            result['native_L4_plan']=native_plan
    durable_json(OUT/'NORMAL.json',result); print(json.dumps(result,indent=2),flush=True)


def deep():
    start=measured()
    with CommittedState(ref()) as state:
        actual=state.deep_verify(L5)
        result=dict(scientific=actual,passed=True,metrics=metrics(*start),stats=dict(state.stats),
            underlying_stats=dict(state.wrapped.stats),normal_path=False)
    durable_json(OUT/'DEEP.json',result); print(json.dumps(result,indent=2),flush=True)


def normal_recursive():
    reference=json.loads((OUT/'REFRAMED_REFERENCE.json').read_text())
    with patch.object(sqlite3,'connect',denied):
        tick=measured()
        with CommittedState(reference) as state:
            plan=plan_transition(state,adapter(state),6)
            result=dict(metrics=metrics(*tick),plan=plan,stats=dict(state.stats),
                native_stats=dict(state.parent.wrapped.stats),sql_rows=0,proof_rows=0,quality_rows=0,
                full_prefix_hashes=0,full_prefix_deep_copies=0,admission=False,forecast=False,generation=False)
    durable_json(OUT/'FRESH_RECURSIVE_NORMAL.json',result);print(json.dumps(result,indent=2),flush=True)


def controls():
    start=measured(); output={}
    with CommittedState(ref()) as base:
        base.wrapped.load_proof()
        db=sqlite3.connect(base.wrapped.index.as_uri()+'?mode=ro&immutable=1',uri=True)
        key=db.execute('SELECT hex(key) FROM proof_keys ORDER BY key LIMIT 1').fetchone()[0].lower(); db.close()
        qpath=Path(base.wrapped.body['quality_delta']); base.wrapped.verify(qpath)
        db=sqlite3.connect(qpath.as_uri()+'?mode=ro&immutable=1',uri=True)
        qkey=db.execute('SELECT key FROM updates WHERE payload IS NOT NULL ORDER BY key LIMIT 1').fetchone()[0];db.close()
        tick=measured(); counts=SQL.copy(); proof=base['certificate'][key]; quality=base['quality'][qkey]
        output['cold_lookup']=dict(**metrics(*tick),sql=dict(SQL-counts))
        tick=measured(); counts=SQL.copy(); assert base['certificate'][key]==proof; assert base['quality'][qkey]==quality
        output['warm_lookup']=dict(**metrics(*tick),sql=dict(SQL-counts))
        dep=next(k for k in proof['inputs'] if k.startswith('face:'))
        newkey=identity(['non_geometric_control_a','non_geometric_control_b'])
        delta=dict(proofs={newkey:dict(faces=['control:a','control:b'],inputs={'control:dependency':'v1'},value={'mechanics_only':True})},
            bindings={dep:dict(before=proof['inputs'][dep],after='control-binding-1',proof_keys=[key])},
            quality=[dict(key=qkey,inputs={'control:quality':'v1'},payload=None),dict(key='control:quality',inputs={},payload={'value':1})],
            removed={},before={'certificate':{newkey:None},'quality':{qkey:identity(quality),'control:quality':None}})
        child,stats=publish_control(base,OUT/'control_child_final',delta,identity(delta))
        assert publish_control(base,OUT/'control_child_final',delta,identity(delta))[0]==child
        output['child']=dict(reference=child,stats=stats)
        with CommittedState(child) as first:
            assert first['certificate'][key]['value']==proof['value']
            assert first['certificate'][key]['inputs'][dep]=='control-binding-1'
            assert first['quality'][qkey]['value']==quality['value']
            delta2=dict(proofs={},bindings={dep:dict(before='control-binding-1',after='control-binding-2',proof_keys=[key])},
                quality=[dict(key='control:quality',inputs={'v':'2'},payload={'value':2})],removed={'certificate':[newkey]},
                before={'certificate':{newkey:record_hash(first,'certificate',newkey)},'quality':{'control:quality':record_hash(first,'quality','control:quality')}})
            grand,stats=publish_control(first,OUT/'control_grandchild_final',delta2,identity(delta2))
            output['grandchild']=dict(reference=grand,stats=stats)
        with CommittedState(grand) as final:
            tick=measured();counts=SQL.copy()
            assert final['certificate'][key]['inputs'][dep]=='control-binding-2'
            assert final['certificate'][key]['value']==proof['value']
            assert final['quality']['control:quality']=={'value':2,'inputs':{'v':'2'}}
            assert final['quality'][qkey]['value']==quality['value']
            try: final['certificate'][newkey]
            except KeyError: pass
            else: raise AssertionError('RemovalFallthrough')
            output['recursive_lookup']=dict(metrics=metrics(*tick),sql=dict(SQL-counts),stats=dict(final.stats),parent_stats=dict(final.parent.stats))
            try: adapter(final)
            except ValueError as exc: assert str(exc)=='NonGeometricControlNotAdvancementBase'
            else: raise AssertionError('ControlCannotBecomePhysicalAuthority')
        output.update(passed=True,metrics=metrics(*start),sql=dict(SQL),selected_proof=key,selected_quality=qkey,
            preserved_proof_payload=identity(proof['value']),proof_depth=3,quality_depth=3,
            counts=dict(new_proofs=1,removed_proofs=1,binding_updates=2,quality_input_updates=1,new_quality=1,quality_numeric_updates=1),
            scope='Non-geometric composition mechanics only; production L5 unchanged')
    durable_json(OUT/'COMPOSITION.json',output); print(json.dumps(output,indent=2),flush=True)


def reframe():
    """Known-state metadata re-expression, not geometry or transition replay."""
    import composable_state
    from incremental_transaction import Store
    import immutable_state_access as access
    with CommittedState(ref()) as base:
        original=base.wrapped; old=original.plan
        parent=dict(kind='native',directory=old['base_access'],root=old['base_root'])
        content=dict(schema=composable_state.SCHEMA,parent_scientific=old['content']['base_scientific'],
            parent_content=old['content']['base_content'],components=deepcopy(old['components']))
        plan=dict(schema=composable_state.SCHEMA,scope='production',parent=parent,content=content,scientific=identity(content),
            references=old['references'],expected=dict(old['expected'],committed=identity(content)),output=old['output'],
            hot=old['hot'],geometry_counts=old['geometry_counts'],implementation=file_hash(composable_state.__file__),
            sealed_control=dict(root=base.root,scientific=base.scientific,seal_sha256=ref()['seal_sha256']))
        body=dict(epoch=access.epoch(),plan=plan,scientific=identity(content),produced=original.body['produced'],
            quality_delta=original.body['quality_delta'],binding_patch=original.body['binding_patch'])
        store=Store(OUT/'reframed_L5_final'); root=store.initialize(body)
        reference=dict(kind='recursive',directory=str(store.path.resolve()),root=root)
        durable_json(OUT/'REFRAMED_REFERENCE.json',reference)
    with patch.object(sqlite3,'connect',denied):
        tick=measured()
        with CommittedState(reference) as state:
            plan=plan_transition(state,adapter(state),6)
            assert plan['increment_hex']=='0x1.744eb7e7e7516p-12'
            normal=dict(metrics=metrics(*tick),stats=dict(state.stats),plan=plan,sql_rows=0,proof_rows=0,quality_rows=0)
    durable_json(OUT/'REFRAMED_NORMAL.json',normal)
    tick=measured()
    with CommittedState(reference) as state:
        actual=state.deep_verify(L5)
        result=dict(passed=True,scientific=actual,reference=reference,content_root=state.content_root,
            metrics=metrics(*tick),stats=dict(state.stats),parent_stats=dict(state.parent.stats),
            note='Same sealed scientific components, new recursive metadata; no target geometry/proofs/quality constructed or copied.')
    durable_json(OUT/'REFRAMED_DEEP.json',result); print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    with patch.object(BoundProductionForecastCache,'forecast',denied),patch.object(NativeCheckpointStore,'restart',denied):
        if sys.argv[1]=='controls':
            connect=sqlite3.connect
            def counted(*args,**kwargs):
                kwargs['factory']=CountConnection
                return connect(*args,**kwargs)
            with patch.object(sqlite3,'connect',counted): controls()
        else: globals()[sys.argv[1]]()
