// V3 held_out_geometry_cohort surface-only extraction from a live implicit domain.
// The CGAL background triangulation is a seed data structure, never a CFD core.
// No cells, prisms, layers, or core authority are materialized or published.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_domain_with_polyline_features_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/Random.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <vector>
#include <array>
#include <cmath>
using K=CGAL::Exact_predicates_inexact_constructions_kernel;
using Point=K::Point_3;
// Exact same analytic definition as geometry_family_authority.primitive(capsule).
struct CapsuleSdf {
  double radius, half;
  double operator()(const Point& p) const {
    double z=p.z()-std::clamp(double(p.z()),-half,half);
    return std::sqrt(p.x()*p.x()+p.y()*p.y()+z*z)-radius;
  }
};
int main(int argc,char**argv) {
  if(argc!=5) return 64;
  const double radius=std::stod(argv[1]),half=std::stod(argv[2]);
  const double facet_size=std::stod(argv[4]);
  if(!(radius>0 && half>=0 && facet_size>0)) return 65;
  using Base=CGAL::Labeled_mesh_domain_3<K>;
  using Domain=CGAL::Mesh_domain_with_polyline_features_3<Base>;
  using Tr=CGAL::Mesh_triangulation_3<Domain>::type;
  using C3t3=CGAL::Mesh_complex_3_in_triangulation_3<Tr>;
  using VH=Tr::Vertex_handle;
  // V3 cohort reference policy (scale=1), not SF2/airfoil settings.
  const double bound=std::nextafter(radius+half,INFINITY);
  CGAL::get_default_random()=CGAL::Random(49944);
  Base base=Base::create_implicit_mesh_domain(CapsuleSdf{radius,half},K::Sphere_3(Point(0,0,0),bound*bound),K::FT(1e-6));
  Domain domain(base);
  CGAL::Mesh_criteria_3<Tr> criteria(CGAL::parameters::edge_size=K::FT(.08),
    CGAL::parameters::facet_angle=K::FT(25.),CGAL::parameters::facet_size=K::FT(facet_size),
    CGAL::parameters::facet_distance=K::FT(.02),CGAL::parameters::cell_radius_edge_ratio=K::FT(3.),
    CGAL::parameters::cell_size=K::FT(.25));
  C3t3 mesh=CGAL::make_mesh_3<C3t3>(domain,criteria,CGAL::parameters::no_perturb(),CGAL::parameters::no_exude());
  std::map<VH,int> ids;std::vector<std::array<VH,3>> faces;
  for(auto it=mesh.facets_in_complex_begin();it!=mesh.facets_in_complex_end();++it) {
    auto cell=it->first;int side=it->second;
    std::array<VH,3> f={cell->vertex((side+1)&3),cell->vertex((side+2)&3),cell->vertex((side+3)&3)};
    auto a=mesh.subdomain_index(cell),b=mesh.subdomain_index(cell->neighbor(side));
    if((a>b)==(side%2==1))std::swap(f[0],f[1]);
    for(auto v:f)if(!ids.count(v)){int i=ids.size();ids[v]=i;}
    faces.push_back(f);
  }
  if(faces.empty())return 66;
  std::vector<Point> points(ids.size());for(auto [v,i]:ids)points[i]=v->point().point();
  std::ofstream out(argv[3]);out<<std::setprecision(17)<<"OFF\n"<<points.size()<<" "<<faces.size()<<" 0\n";
  for(auto p:points)out<<p.x()<<" "<<p.y()<<" "<<p.z()<<"\n";
  for(auto f:faces)out<<"3 "<<ids[f[0]]<<" "<<ids[f[1]]<<" "<<ids[f[2]]<<"\n";
}
