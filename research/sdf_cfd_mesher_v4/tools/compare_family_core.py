"""Scientific replay comparison; mesher tags are not geometry authority."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from sdf_cfd_mesher_v4.common.authority import array_hash,identity


def normalized(folder):
    selected=folder/'CORE_SELECTED.json'
    core=folder/(json.loads(selected.read_text())['directory'] if selected.exists() else ('repaired_core' if (folder/'repaired_core/GENERATION.json').exists() else 'core'))
    x=np.load(core/'coordinates.npy');t=np.load(core/'tets.npy')
    order=np.lexsort((x[:,2],x[:,1],x[:,0]));rank=np.empty(len(x),int);rank[order]=np.arange(len(x))
    cells=np.sort(rank[t],axis=1);sort=np.lexsort(tuple(cells[:,i] for i in (3,2,1,0)))
    return dict(coordinates=array_hash(x[order]),connectivity=array_hash(cells[sort]),cells=len(t))


def run(folder,primary='Q',replay='R'):
    a=normalized(folder/primary);b=normalized(folder/replay)
    qa=json.loads((folder/primary/'ASSEMBLY.json').read_text());ra=json.loads((folder/replay/'ASSEMBLY.json').read_text())
    result=dict(exact=a==b,primary=a,replay=b,normalization='Lexicographic binary64 coordinates and unordered tet vertex sets; no numerical tolerance',
                complete_relations_exact=qa['total_relations']==ra['total_relations'],complete_passed=qa['passed'] and ra['passed'])
    (folder/'CORE_DETERMINISM.json').write_text(json.dumps(result,indent=2)+'\n')
    assert all(result[k] for k in ('exact','complete_relations_exact','complete_passed'))

if __name__=='__main__':run(Path(sys.argv[1]),*sys.argv[2:])
