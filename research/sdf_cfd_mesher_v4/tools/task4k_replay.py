"""Certificate transactions only; never generate or change mesh coordinates."""
from task4k_common import *
from task4j_common import geometry
from sdf_cfd_mesher_v4.certificate.production import certify_immediate,verify_committed_layer
import argparse,psutil,threading

FIELDS=('candidate_pairs','completed_pairs','forbidden','unresolved','invalid_surfaces','topology','geometry_hash','labels_hash','kernel','pair_hash','code_hash','witness_hash','outcome_hash','fallback_hash','validity_hash','validity_fallback','class_counts','exact_calls','ownership','complement')
def run():
    parser=argparse.ArgumentParser();parser.add_argument('--name',default='A');parser.add_argument('--layers',default='1,2,12,20');args=parser.parse_args()
    rows=[];peak=[0];stop=threading.Event()
    def memory():
        while not stop.wait(.1):
            try:peak[0]=max(peak[0],sum(p.memory_info().rss for p in [psutil.Process(),*psutil.Process().children(recursive=True)] if p.is_running()))
            except psutil.Error:pass
    thread=threading.Thread(target=memory,daemon=True);thread.start();f,t,x=geometry()
    for layer in map(int,args.layers.split(',')):
        guard();path=OUT/f'replay_{args.name}'/f'layer_{layer:02d}'
        record,perf=certify_immediate(f,t,x[1:layer+1],path,workers=8)
        old=read(BASE/f'certificate_layer_{layer:02d}/MANIFEST.json')
        for field in FIELDS:assert record[field]==old[field],('ScientificMismatch',layer,field)
        s=time.perf_counter();count,audit=verify_committed_layer(f,t,x[1:layer+1],path,record)
        scientific={k:record[k] for k in FIELDS}
        row=dict(layer=layer,performance=perf[0],audit_seconds=time.perf_counter()-s,pairs=count,scientific_hash=scientific_hash(scientific),compact_hash=file_hash(path/'COMPACT.npz'),exact_hash=file_hash(path/'EXACT.json.gz'),peak_rss=peak[0],equivalent=True)
        rows.append(row);save(f'REPLAY_{args.name}.json',rows);print(layer,perf[0]['seconds'],row['audit_seconds'],flush=True)
    stop.set();thread.join();save(f'REPLAY_{args.name}_RESOURCE.json',dict(peak_rss=peak[0],elapsed_task=elapsed()))
if __name__=='__main__':run()
