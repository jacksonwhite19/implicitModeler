from task4k_common import *
from task4j_common import geometry
from sdf_cfd_mesher_v4.certificate.incremental import kernel
import gzip,fractions

def polynomial_summary(record):
    coeff=[fractions.Fraction(c) for _,c in record]
    return dict(identity=scientific_hash(record),terms=record,degree=max((sum(m) for m,c in record),default=0),coefficient_count=len(coeff),numerator_bits=max((abs(c.numerator).bit_length() for c in coeff),default=0),denominator_bits=max((c.denominator.bit_length() for c in coeff),default=0))
def run():
    guard();f,t,x=geometry();E=len(t.edges);nv=len(f.coordinates);cases=[];quads=[];labels=[]
    for layer in range(1,21):
        with gzip.open(BASE/f'certificate_layer_{layer:02d}/EXACT.json.gz','rb') as z:records=json.load(z)
        for record in records:
            r=record['result']
            if r['method']!='NoncoincidentRank3ExactCAD':continue
            pair=record['pair'];q=[];ids=[];sides=[]
            for i in pair:
                k,e=divmod(i,E);k+=1;a,b=map(int,t.edges[e]);q.append([x[k-1,a],x[k-1,b],x[k,a],x[k,b]]);ids.append([(k-1)*nv+a,(k-1)*nv+b,k*nv+a,k*nv+b]);sides.append([k,e])
            q=np.asarray(q);ids=np.asarray(ids,np.uint64);proof=r['proof']
            roots=proof['critical_s_roots']+[rr for fiber in proof['fibers'] for rr in fiber['t_roots']]
            identity=dict(pair=pair,geometry_hash=array_hash(q),labels_hash=array_hash(ids),kernel=kernel.VERSION)
            cases.append(dict(index=len(cases),identity=scientific_hash(identity),cache_key=identity,pair=pair,layer=layer,sides=sides,ranks=r['ranks'],result_hash=scientific_hash(r),polynomials=[polynomial_summary(p) for p in proof['projection_polynomials']],factor_count=len(proof['factor_basis']),root_count=len(roots),roots=roots,root_indices=[v['root_index'] for v in roots if 'root_index' in v],fiber_count=len(proof['fibers']),cell_count=sum(len(v['cells']) for v in proof['fibers'])))
            quads.append(q);labels.append(ids)
    assert len(cases)==646
    np.savez_compressed(OUT/'CAD_INPUTS.npz',quads=np.asarray(quads),labels=np.asarray(labels));save('CAD_CASES.json',cases)
    save('CASE_POPULATION.json',dict(count=len(cases),case_hash=scientific_hash(cases),input_hash=file_hash(OUT/'CAD_INPUTS.npz'),preserved_geometry_only=True))
if __name__=='__main__':run()
