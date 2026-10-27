// Complete CGAL exact-predicate surface self-intersection audit. No meshing.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <fstream>
#include <iostream>
#include <vector>
int main(int argc,char**argv){
 if(argc!=2)return 64;
 using K=CGAL::Exact_predicates_inexact_constructions_kernel;
 using M=CGAL::Surface_mesh<K::Point_3>;
 M m;std::ifstream in(argv[1]);in>>m;if(!in||m.is_empty())return 65;
 std::vector<std::pair<M::Face_index,M::Face_index>> pairs;
 CGAL::Polygon_mesh_processing::self_intersections(m,std::back_inserter(pairs));
 std::cout<<"{\"forbidden\":"<<pairs.size()<<",\"unresolved\":0,\"pairs\":[";
 bool first=true;for(auto p:pairs){if(!first)std::cout<<",";first=false;std::cout<<"["<<p.first.idx()<<","<<p.second.idx()<<"]";}
 std::cout<<"]}\n";return 0;
}
