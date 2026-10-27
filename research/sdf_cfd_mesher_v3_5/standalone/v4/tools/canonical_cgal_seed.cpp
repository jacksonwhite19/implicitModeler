// Generic field connection. No shape constructor or shape equation is defined here.
#include "canonical_kernel_callback.hpp"
#include <CGAL/Labeled_mesh_domain_3.h>
#include <CGAL/Mesh_domain_with_polyline_features_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Surface_mesh_complex_2_in_triangulation_3.h>
#include <CGAL/Surface_mesh_default_criteria_3.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Random.h>
#include <array>
#include <chrono>
#include <fstream>
#include <map>
#include <vector>

int main(int argc, char** argv) {
  try {
    if (argc < 9) throw std::runtime_error("Usage_mode_file_centerXYZ_radius_relativeError_repetitions_or_criteria");
    const std::string mode=argv[1];
    const double cx=std::stod(argv[3]),cy=std::stod(argv[4]),cz=std::stod(argv[5]),radius=std::stod(argv[6]),error=std::stod(argv[7]);
    if (!std::isfinite(cx)||!std::isfinite(cy)||!std::isfinite(cz)||!std::isfinite(radius*radius)||radius<=0||!std::isfinite(error)||error<=0||error>=1)
      throw std::runtime_error("InvalidEnclosingRegionOrRelativeError");
    using K=CanonicalK; using Point=K::Point_3;
    using Base=CGAL::Labeled_mesh_domain_3<K>;
    using Domain=CGAL::Mesh_domain_with_polyline_features_3<Base>;
    CanonicalKernelCallback callback;
    Base base=Base::create_implicit_mesh_domain(callback,K::Sphere_3(Point(cx,cy,cz),radius*radius),K::FT(error));
    Domain domain(base);
    if (mode=="--probe") {
      const int repeats=std::stoi(argv[8]);if(repeats<1||repeats>100000)throw std::runtime_error("InvalidRepetitions");
      std::ifstream in(argv[2]); if(!in)throw std::runtime_error("CannotOpenPoints");
      std::vector<Point> points;double x,y,z;
      while(in>>x>>y>>z)points.emplace_back(x,y,z);
      if(!in.eof()||points.empty())throw std::runtime_error("InvalidPointBuffer");
      const auto start=std::chrono::steady_clock::now();
      for(int r=0;r<repeats;++r) for(auto p:points) {
        const double value=callback(p);
        // This is CGAL's actual domain classification functor, invoking the same callback.
        const auto label=domain.is_in_domain_object()(p);
        if(r==0)std::cout<<std::setprecision(17)<<"V "<<value<<' '<<(label?*label:0)<<std::endl;
      }
      const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      std::cout<<std::setprecision(17)<<"DONE {\"mode\":\"callback_diagnostic\",\"calls\":"<<*callback.calls<<",\"seconds\":"<<seconds<<",\"points\":"<<points.size()<<",\"repeats\":"<<repeats<<"}"<<std::endl;
    } else if(mode=="--extract-surface") {
      if(argc!=15)throw std::runtime_error("ExtractionCriteriaRequired");
      // Retain the six-entry case format. Only facet criteria apply here;
      // edge/cell criteria are explicitly reported as inactive, not relaxed.
      std::array<double,6> c{};for(int i=0;i<6;++i){c[i]=std::stod(argv[8+i]);if(!std::isfinite(c[i])||c[i]<=0)throw std::runtime_error("InvalidCriteria");}
      const auto seed=std::stoul(argv[14]); CGAL::get_default_random()=CGAL::Random(seed);
      using Tr=CGAL::Surface_mesh_default_triangulation_3;
      using GT=Tr::Geom_traits;
      using C2t3=CGAL::Surface_mesh_complex_2_in_triangulation_3<Tr>;
      using Surface=CGAL::Implicit_surface_3<GT,CanonicalKernelCallback>;
      using VH=Tr::Vertex_handle;
      Tr tr; C2t3 mesh(tr);
      Surface surface(callback,GT::Sphere_3(Point(cx,cy,cz),radius*radius),GT::FT(error));
      CGAL::Surface_mesh_default_criteria_3<Tr> criteria(c[1],c[2],c[3]);
      // Restricted surface-Delaunay refinement. No make_mesh_3, volume size
      // criterion or interior-cell quality refinement is used in this branch.
      CGAL::make_surface_mesh(mesh,surface,criteria,CGAL::Manifold_tag());
      std::map<VH,int> ids;std::vector<std::array<VH,3>> faces;
      for(auto it=mesh.facets_begin();it!=mesh.facets_end();++it){
        auto cell=it->first;int side=it->second;
        std::array<VH,3> f={cell->vertex((side+1)&3),cell->vertex((side+2)&3),cell->vertex((side+3)&3)};
        // Orientation is a proposal. The existing source owner projects and
        // orients every returned face before refinement and final acceptance.
        for(auto v:f)if(!ids.count(v)){int i=ids.size();ids[v]=i;}faces.push_back(f);
      }
      if(faces.empty())throw std::runtime_error("EmptyExtraction");
      std::vector<Point> points(ids.size());for(auto [v,i]:ids)points[i]=v->point();
      std::ofstream out(argv[2]);if(!out)throw std::runtime_error("CannotWriteOutput");
      out<<std::setprecision(17)<<"OFF\n"<<points.size()<<' '<<faces.size()<<" 0\n";
      for(auto p:points)out<<p.x()<<' '<<p.y()<<' '<<p.z()<<'\n';
      for(auto f:faces)out<<"3 "<<ids[f[0]]<<' '<<ids[f[1]]<<' '<<ids[f[2]]<<'\n';
      out.close();if(!out)throw std::runtime_error("OutputWriteFailed");
      std::cout<<"DONE {\"mode\":\"unqualified_surface_seed\",\"route\":\"CGALImplicitSurfaceExtraction/1\",\"volume_refinement\":false,\"inactive_legacy_criteria_indices\":[0,4,5],\"calls\":"<<*callback.calls<<",\"vertices\":"<<points.size()<<",\"triangles\":"<<faces.size()<<"}"<<std::endl;
    } else if(mode=="--extract") {
      if(argc!=15)throw std::runtime_error("ExtractionCriteriaRequired");
      // All criteria and seed are supplied by the caller, independently of geometry.
      std::array<double,6> c{};for(int i=0;i<6;++i){c[i]=std::stod(argv[8+i]);if(!std::isfinite(c[i])||c[i]<=0)throw std::runtime_error("InvalidCriteria");}
      const auto seed=std::stoul(argv[14]); CGAL::get_default_random()=CGAL::Random(seed);
      using Tr=CGAL::Mesh_triangulation_3<Domain>::type;
      using C3t3=CGAL::Mesh_complex_3_in_triangulation_3<Tr>;using VH=Tr::Vertex_handle;
      CGAL::Mesh_criteria_3<Tr> criteria(CGAL::parameters::edge_size=K::FT(c[0]),
        CGAL::parameters::facet_angle=K::FT(c[1]),CGAL::parameters::facet_size=K::FT(c[2]),
        CGAL::parameters::facet_distance=K::FT(c[3]),CGAL::parameters::cell_radius_edge_ratio=K::FT(c[4]),
        CGAL::parameters::cell_size=K::FT(c[5]));
      auto mesh=CGAL::make_mesh_3<C3t3>(domain,criteria,CGAL::parameters::no_perturb(),CGAL::parameters::no_exude());
      std::map<VH,int> ids;std::vector<std::array<VH,3>> faces;
      for(auto it=mesh.facets_in_complex_begin();it!=mesh.facets_in_complex_end();++it){
        auto cell=it->first;int side=it->second;
        std::array<VH,3> f={cell->vertex((side+1)&3),cell->vertex((side+2)&3),cell->vertex((side+3)&3)};
        auto a=mesh.subdomain_index(cell),b=mesh.subdomain_index(cell->neighbor(side));
        if((a>b)==(side%2==1))std::swap(f[0],f[1]);
        for(auto v:f)if(!ids.count(v)){int i=ids.size();ids[v]=i;}faces.push_back(f);
      }
      if(faces.empty())throw std::runtime_error("EmptyExtraction");
      std::vector<Point> points(ids.size());for(auto [v,i]:ids)points[i]=v->point().point();
      std::ofstream out(argv[2]);if(!out)throw std::runtime_error("CannotWriteOutput");
      out<<std::setprecision(17)<<"OFF\n"<<points.size()<<' '<<faces.size()<<" 0\n";
      for(auto p:points)out<<p.x()<<' '<<p.y()<<' '<<p.z()<<'\n';
      for(auto f:faces)out<<"3 "<<ids[f[0]]<<' '<<ids[f[1]]<<' '<<ids[f[2]]<<'\n';
      out.close();if(!out)throw std::runtime_error("OutputWriteFailed");
      std::cout<<"DONE {\"mode\":\"unqualified_seed\",\"calls\":"<<*callback.calls<<",\"vertices\":"<<points.size()<<",\"triangles\":"<<faces.size()<<"}"<<std::endl;
    } else throw std::runtime_error("UnknownMode");
    return 0;
  } catch(const std::exception& e) {
    std::cout<<"FAIL "<<e.what()<<std::endl;return 2;
  } catch(...) {std::cout<<"FAIL UnknownNativeException"<<std::endl;return 3;}
}
