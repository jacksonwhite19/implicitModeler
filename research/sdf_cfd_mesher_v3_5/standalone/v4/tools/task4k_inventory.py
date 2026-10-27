from task4k_common import *
from task4k_cases import polynomial_summary
import gzip

def run():
    rows=[]
    for layer in range(1,21):
        with gzip.open(BASE/f'certificate_layer_{layer:02d}/EXACT.json.gz','rb') as f:records=json.load(f)
        for record in records:
            result=record['result']
            if result['method']!='NoncoincidentRank3ExactCAD':continue
            proof=result['proof'];families={}
            for name,polys in [('F',[proof['F']]),('factor_basis',proof['factor_basis']),('projection',proof['projection_polynomials'])]:
                families[name]=[{k:v for k,v in polynomial_summary(p).items() if k!='terms'} for p in polys]
            rows.append(dict(index=len(rows),pair=record['pair'],layer=layer,polynomials=families))
    assert len(rows)==646
    save('POLYNOMIAL_INVENTORY.json',rows)
if __name__=='__main__':run()
