from task4h_common import *
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.forecast.forensics import exact_wedge as old
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_wedge,exact_signs
def run():
    require_time();front=prod.load_sf2_wall();authority_path=ROOT/'results/nominal_forecast_forensics/SCIENTIFIC_OUTPUTS.json'
    frozen=read(authority_path)['exact_jacobian_sign_checks']
    with np.load(BASE/'INITIAL_FORECAST.npz') as z:x=z['coordinates']
    rows=[]
    for row in frozen:
        k=row['layer'];ids=front.triangles[row['column']];lo=front.coordinates[ids] if k==1 else x[k-2,ids];p=np.concatenate((lo,x[k-1,ids]))
        a=old(p);b=exact_wedge(p);assert a==b=={key:row[key] for key in a}
        assert exact_signs(p)=={key:a[key] for key in ('jacobian_sign','volume_sign')};rows.append(dict(column=row['column'],layer=k,**b))
    save('DEVELOPMENT_JACOBIANS.json',dict(source_sha256=file_hash(authority_path),rows=rows,zero=sum(r['jacobian_sign']==0 for r in rows),negative=sum(r['jacobian_sign']<0 for r in rows),exact_equal=True,scientific_hash=scientific_hash(rows)))
if __name__=='__main__':run()
