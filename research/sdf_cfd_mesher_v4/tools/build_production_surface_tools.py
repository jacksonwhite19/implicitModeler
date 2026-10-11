"""Build only V4 surface extraction/audit adapters with installed CGAL/GCC."""
from production_adaptive_surface import OUT,V4,wsl,run_native,save,sha

def main():
    runtime=OUT/'runtime';runtime.mkdir(parents=True,exist_ok=True)
    results=[]
    for source,name in [('production_capsule_cgal_seed.cpp','seed'),('production_surface_intersections.cpp','intersections')]:
        path=V4/'tools'/source
        result,_=run_native(['g++','-O1','-DNDEBUG','-std=c++17',wsl(path),'-o',wsl(runtime/name),'-lgmp','-lmpfr'],runtime/('build_'+name),120)
        results.append(dict(source=str(path),source_sha256=sha(path),binary_sha256=sha(runtime/name),result=result))
    save(OUT/'BUILD.json',dict(tools=results,installed_tool_changes=False,volume_mesh_export=False))

if __name__=='__main__':main()
