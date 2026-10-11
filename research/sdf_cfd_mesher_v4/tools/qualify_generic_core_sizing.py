"""Bounded real-terminal-cell sizing characterization, never core generation."""
from pathlib import Path
import json
import time
import numpy as np
import capsule_user_domain_handoff as h
from generic_core_sizing import CoreSizingField, equal_volume_seed
from sdf_cfd_mesher_v4.common.authority import array_hash

OUT = h.V4/'results/generic_core_sizing'
ACCESS = '36f3bc9ecd8f4728fda53a3c43d686b62a0553931c0f5956604dcb35cd7afa52'


def distribution(values):
    x = np.asarray(values)
    return dict(count=int(x.size),minimum=float(x.min()),p05=float(np.quantile(x,.05)),
                median=float(np.median(x)),p95=float(np.quantile(x,.95)),maximum=float(x.max()))


def characterize():
    total = h.measured()
    qa=h.read(h.OUT/'AUTHORITY.json');h.verify_record(qa)
    h.eq(qa['identity'],'55e31c4497e0629b7951f90347a555fbb3510755c00d531d0fe4d6bdf9fe44c5','HandoffQualification')
    for name,digest in qa['files'].items():
        h.eq(h.file_hash(h.V4/name),digest,'HandoffAuthorityClosure:'+name)
    reopen = h.reopen(ACCESS)
    authority,_,wall,source_tri,_ = h.verify_sources()
    tick = h.measured()
    with h.guards(), h.CommittedState(authority['reference']) as state:
        x,tri,records,lower = h.extract(state,wall,source_tri,True)
        delta = h.read(state.verify(state.out/'GEOMETRY_DELTA.json'))
        # Only the 33,786 current terminal cells. No prior cells or quality rows.
        from local_termination_topology import geometry
        points = np.concatenate([lower,x]); n = len(x)
        def local_id(v):
            _,layer,_,source = v.split(':')
            h.need(int(layer) in (10,11),'NonterminalCellGeometry')
            return int(source)+(n if int(layer)==11 else 0)
        volumes=[]; aspects=[]
        for r in records:
            c = delta['cells'][r['prism_owner']]
            h.eq(c['source'],r['source_column'],'TerminalOwnerMapping')
            local = dict(c,vertices=[local_id(v) for v in c['vertices']],
                         faces=[[local_id(v) for v in f] for f in c['faces']])
            result = geometry(points,local)
            volumes.append(result['volume']); aspects.append(result['aspect'])
        state.check()
        stats = dict(state.stats)
    seeds = equal_volume_seed(volumes)
    t = x[tri]; lo = lower[tri]
    normals = np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0])
    double_area = np.linalg.norm(normals,axis=1)
    normals /= double_area[:,None]
    edges = np.stack([np.linalg.norm(t[:,i]-t[:,j],axis=1) for i,j in ((0,1),(1,2),(2,0))],axis=1)
    thickness = np.einsum('ij,ij->i',(t-lo).mean(axis=1),normals)
    h.need(np.all(thickness>0),'PositiveActualNormalThickness')
    edge_map={}; variations=[]
    for i,f in enumerate(tri):
        for a,b in zip(f,np.roll(f,-1)):
            key=tuple(sorted((int(a),int(b))))
            if key in edge_map: variations.append(abs(float(seeds[i]-seeds[edge_map[key]])))
            else: edge_map[key]=i
    eq_edge=np.sqrt(2*double_area/np.sqrt(3))
    result=dict(reopen=reopen,terminal_cells=len(records),terminal_geometry=h.metrics(*tick),stats=stats,
        volumes_m3=distribution(volumes),normal_thickness_m=distribution(thickness),
        thickness_definition='Mean paired upper-minus-lower displacement projected on owner-oriented upper triangle unit normal',
        all_interface_edges_m=distribution(edges),face_min_edge_m=distribution(edges.min(axis=1)),
        face_median_edge_m=distribution(np.median(edges,axis=1)),face_max_edge_m=distribution(edges.max(axis=1)),
        equivalent_triangle_edge_m=distribution(eq_edge),seed_m=distribution(seeds),
        seed_over_equivalent_triangle_edge=distribution(seeds/eq_edge),
        seed_over_normal_thickness=distribution(seeds/thickness),prism_aspect=distribution(aspects),
        neighbour_seed_difference_m=distribution(variations),
        patch_separation_m={f'{axis}_{side}':float((x[:,k]-bound).min() if side=='min' else (bound-x[:,k]).min())
            for k,axis in enumerate('xyz') for side,bound in [('min',[-.75,-.75,-1.25][k]),('max',[.75,.75,1.25][k])]},
        coordinates_hash=array_hash(x),triangles_hash=array_hash(tri),seed_hash=array_hash(seeds),
        volume_hash=array_hash(np.asarray(volumes)),owner_map_hash=h.identity(records),
        historical_proof_rows=0,historical_quality_rows=0,full_prefix_hashes=0,deep_copies=0)
    np.save(OUT/'SEEDS.npy',seeds,allow_pickle=False)
    result['total']=h.metrics(*total)
    h.durable_json(OUT/'CHARACTERIZATION.json',result)
    return result


def benchmark():
    x=np.load(h.OUT/'FRONT.npy');tri=np.load(h.OUT/'TRIANGLES.npy');s=np.load(OUT/'SEEDS.npy')
    # Mechanics parameters only: explicit reproducible controls derived for the
    # experiment. They are NOT a capsule production parameter recommendation.
    controls=dict(scope='NON_PRODUCTION_TEST_PARAMETER',g=.1,h_max=.1)
    tick=h.measured();field=CoreSizingField(x[tri],s,g=controls['g'],h_max=controls['h_max'])
    build=h.metrics(*tick)
    ids=np.linspace(0,len(tri)-1,64,dtype=int);t=x[tri[ids]]
    center=t.mean(axis=1);normal=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);normal/=np.linalg.norm(normal,axis=1)[:,None]
    samples={'interface':center,'near':center+normal*.001,
             'mid_domain':center+normal*.2,'farfield':np.column_stack([np.full(64,.75),np.linspace(-.75,.75,64),np.linspace(-1.25,1.25,64)])}
    rows={}
    for name,p in samples.items():
        tick=h.measured();a=[field.evaluate_with_work(v) for v in p];acc=h.metrics(*tick)
        tick=h.measured();b=[field.brute_force(v) for v in p];brute=h.metrics(*tick)
        av=np.array([v[0] for v in a]);bv=np.array(b)
        h.need(np.array_equal(av,bv),'AcceleratedBruteForceMismatch')
        h.need(np.array_equal(av,np.array([field.evaluate(v) for v in p])),'QueryReplayMismatch')
        rows[name]=dict(queries=len(p),bit_exact=True,max_absolute_error=float(abs(av-bv).max()),
            accelerated=acc,brute_force=brute,queries_per_second=len(p)/acc['seconds'],
            triangles_tested=distribution([v[1]['triangles'] for v in a]),
            nodes_visited=distribution([v[1]['nodes'] for v in a]))
    result=dict(controls=controls,index_build=build,index_nodes=len(field.nodes),
                index_array_bytes=field.array_bytes,benchmarks=rows)
    h.durable_json(OUT/'BENCHMARK.json',result)
    return result


if __name__=='__main__':
    characterize()
    print(json.dumps(benchmark(),indent=2))
