// Execution translation of the existing NumPy 1.26.4 SSE2 sizing leaf.
// Same policy/tree. Exceptional arithmetic replays the whole Python call.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cfenv>
#include <limits>
using V=std::array<double,3>;
static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559);
struct Flags {
 std::fexcept_t saved;
 Flags(){std::fegetexceptflag(&saved,FE_ALL_EXCEPT);std::feclearexcept(FE_ALL_EXCEPT);}
 ~Flags(){std::fesetexceptflag(&saved,FE_ALL_EXCEPT);}
 bool exceptional(){return std::fetestexcept(FE_OVERFLOW|FE_DIVBYZERO|FE_INVALID)!=0;}
};
static V read(const double* p) {return {p[0],p[1],p[2]};}
static bool finite(V a) {return std::isfinite(a[0])&&std::isfinite(a[1])&&std::isfinite(a[2]);}
static V sub(V a,V b) {return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
static V cross(V a,V b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
static double dot(V a,V b) {return 0.+(((a[0]*b[0]+0.)+a[2]*b[2])+((a[1]*b[1]+0.)+0.));}
extern "C" std::uint8_t values(const double* position,const std::uint64_t* ids,std::size_t count,std::size_t n,
 const double* triangles,const double* normals,const double* normal2,const double* e0,const double* e1,const double* e2,
 const double* s0,const double* s1,const double* s2,const double* seeds,double g,double maximum,double* output) noexcept {
 if(!count)return 0;
 if(!position||!ids||!triangles||!normals||!normal2||!e0||!e1||!e2||!s0||!s1||!s2||!seeds||!output||std::fegetround()!=FE_TONEAREST)return 1;
 Flags flags;
 const V p=read(position);const double* edges[]={e0,e1,e2};const double* squares[]={s0,s1,s2};
 if(!finite(p)||!std::isfinite(g)||!std::isfinite(maximum))return 1;
 for(std::size_t j=0;j<count;++j){
  const auto i=ids[j];if(i>=n||normal2[i]<=0.||!std::isfinite(normal2[i]))return 1;
  const V normal=read(normals+3*i);const V a=read(triangles+9*i);const double plane=dot(sub(p,a),normal);const double ratio=plane/normal2[i];
  const V projected={p[0]-ratio*normal[0],p[1]-ratio*normal[1],p[2]-ratio*normal[2]};
  if(!std::isfinite(plane)||!finite(projected))return 1;
  bool inside=true;double best=std::numeric_limits<double>::infinity();
  for(std::size_t k=0;k<3;++k){
   const V u=read(triangles+9*i+3*k);const V e=read(edges[k]+3*i);const double square=squares[k][i];
   if(square<=0.||!std::isfinite(square))return 1;
   const double side=dot(cross(e,sub(projected,u)),normal);const double numerator=dot(sub(p,u),e);
   if(!std::isfinite(side)||!std::isfinite(numerator))return 1;
   inside=inside&&side>=0.;double t=std::clamp(numerator/square,0.,1.);if(t==0.)t=0.;
   const V delta=sub(p,{u[0]+t*e[0],u[1]+t*e[1],u[2]+t*e[2]});const double d=dot(delta,delta);
   if(!std::isfinite(d))return 1;best=std::min(best,d);
  }
  // np.where evaluates both arguments, including this expression outside.
  const volatile double plane_distance=plane*plane/normal2[i];
  if(inside)best=plane_distance;if(!std::isfinite(best)||flags.exceptional())return 1;
  const double distance=std::sqrt(std::max(best,0.));const double value=seeds[i]+g*distance;
  if(std::isnan(value))return 1;output[j]=std::min(maximum,value);
 }
 return flags.exceptional()?1:0;
}
