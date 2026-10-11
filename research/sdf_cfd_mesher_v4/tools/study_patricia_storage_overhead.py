"""Read-only logical payload/page overhead of preserved Patricia files."""
import json,sqlite3,time
from pathlib import Path
from contextlib import closing
from study_block_dispersion import ROOT,PRIOR
from profile_capsule_scaling import select_pointer

def run():
    result=[]
    for layer,label,scientific in [(7,'final_bench',None),(8,'final_8_9','c590ca84ca42770a2087c9335d6ea8feccc292f4bd0506d25a08f92fb627919e'),(9,'final_8_9',None),(10,'continuation_10',None)]:
        folder=PRIOR/label/'checkpoints';pointer=select_pointer(folder,scientific)
        root=json.loads((folder/pointer['checkpoint']).read_text())['components']['certificate']['authenticated']
        item=root['files'][-1];path=folder/'objects'/item['file'];tick=time.perf_counter()
        with closing(sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)) as db:
            pragmas={k:db.execute('PRAGMA '+k).fetchone()[0] for k in ('page_size','page_count','freelist_count','journal_mode','cache_size','mmap_size')}
            count,payload,leaf,branch=db.execute("SELECT COUNT(*),SUM(length(hash)+length(data)),SUM(substr(data,1,1)=x'4c'),SUM(substr(data,1,1)=x'42') FROM nodes").fetchone()
        row=dict(layer=layer,file=str(path),sha256=item['sha256'],nodes=count,leaf_nodes=leaf,internal_nodes=branch,logical_hash_and_node_payload_bytes=payload,file_bytes=path.stat().st_size,file_minus_payload_bytes=path.stat().st_size-payload,pragmas=pragmas,pragma_scope='Fresh read-only inspection connection; production writer cache is separately specified in authenticated_map.Nodes.__init__',seconds=time.perf_counter()-tick)
        result.append(row);print(json.dumps(row),flush=True)
        (ROOT/'PATRICIA_STORAGE_OVERHEAD.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':run()
