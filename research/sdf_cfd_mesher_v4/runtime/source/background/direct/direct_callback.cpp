// Unselected direct Gmsh callback. Existing whole-query source stays unchanged.
#include "query.cpp"
#include <atomic>
#include <thread>
#include <stdexcept>
#include <cstring>

using Fallback=int(*)(double,double,double,double*);
struct DirectContext {
 QueryContext query;
 Fallback fallback;
 std::atomic<bool> active{true};
 std::atomic<int> failure{0},forced{0};
 std::atomic<std::uint64_t> calls{0},native{0},fallbacks{0},nodes{0},triangles{0};
 DirectContext(const QueryContext& q,Fallback f):query(q),fallback(f){}
 void fail(int code) noexcept {int clear=0;failure.compare_exchange_strong(clear,code);}
};

extern "C" void* direct_clone(const void* source,Fallback fallback,int* status) noexcept {
 if(!status)return nullptr;*status=1;if(!source||!fallback)return nullptr;
 try {auto q=new DirectContext(*static_cast<const QueryContext*>(source),fallback);*status=0;return q;}
 catch(...){*status=2;return nullptr;}
}
extern "C" void direct_destroy(void* p) noexcept {delete static_cast<DirectContext*>(p);}
extern "C" void direct_expire(void* p) noexcept {if(p)static_cast<DirectContext*>(p)->active=false;}
extern "C" int direct_info(const void* p,std::uint64_t* n,double* policy) noexcept {
 if(!p||!n||!policy)return 1;const auto& q=static_cast<const DirectContext*>(p)->query;
 n[0]=q.n;n[1]=q.nn;policy[0]=q.g;policy[1]=q.maximum;return 0;
}
// Typed bounded export. The context copy owns every precomputed byte.
extern "C" int direct_export(const void* p,int id,void* out,std::size_t bytes) noexcept {
 if(!p||!out)return 1;const auto& q=static_cast<const DirectContext*>(p)->query;
 const void* source=nullptr;std::size_t size=0;
 const std::vector<double>* arrays[]={&q.triangles,&q.normals,&q.normal2,&q.e0,&q.e1,&q.e2,&q.s0,&q.s1,&q.s2,&q.seeds,&q.boxes};
 if(id>=0&&id<11){source=arrays[id]->data();size=arrays[id]->size()*sizeof(double);}
 else if(id==11){source=q.children.data();size=q.children.size()*sizeof(std::int64_t);}
 else if(id==12){source=q.offsets.data();size=q.offsets.size()*sizeof(std::uint64_t);}
 else if(id==13){source=q.ids.data();size=q.ids.size()*sizeof(std::uint64_t);}
 else return 1;
 if(bytes!=size)return 1;std::memcpy(out,source,size);return 0;
}

extern "C" double direct_callback(int,int,double x,double y,double z,double,void* userdata) noexcept {
 // A null userdata is malformed raw ABI use; the typed installer never admits it.
 if(!userdata)return std::numeric_limits<double>::quiet_NaN();
 auto& c=*static_cast<DirectContext*>(userdata);++c.calls;
 const double unwind=c.query.maximum;
 if(c.failure.load())return unwind;
 try {
  if(!c.active.load()){c.fail(1);return unwind;}
  if(!finite({x,y,z})){c.fail(2);return unwind;}
  if(c.forced.load())throw std::runtime_error("DirectCallbackControlFailure");
  const double p[]={x,y,z};double value=0.;std::uint64_t work[2]={};
  const auto status=query_value(&c.query,p,&value,work);
  if(status){
   ++c.fallbacks;
   if(c.fallback(x,y,z,&value)!=0){c.fail(3);return unwind;}
  }else {++c.native;c.nodes+=work[0];c.triangles+=work[1];}
  if(!std::isfinite(value)||value<=0.){c.fail(4);return unwind;}
  return value;
 }catch(...){c.fail(5);return unwind;}
}
extern "C" int direct_report(const void* p,std::uint64_t* counts,int* failure) noexcept {
 if(!p||!counts||!failure)return 1;const auto& c=*static_cast<const DirectContext*>(p);
 counts[0]=c.calls;counts[1]=c.native;counts[2]=c.fallbacks;counts[3]=c.nodes;counts[4]=c.triangles;
 *failure=c.failure;return 0;
}
// Controls use allocated, live contexts only; never invoke a freed pointer.
extern "C" void direct_control_force(void* p,int code) noexcept {if(p)static_cast<DirectContext*>(p)->forced=code;}
extern "C" int direct_control_threads(void* p,const double* positions,std::size_t count,
 double* output,unsigned threads,int rounding) noexcept {
 if(!p||!positions||!output||!threads||threads>16)return 1;
 try {
  std::vector<std::thread> workers;
  try {
   for(unsigned t=0;t<threads;++t)workers.emplace_back([=]{
    const int old=std::fegetround();std::fesetround(rounding);
    for(std::size_t i=t;i<count;i+=threads)output[i]=direct_callback(3,1,positions[3*i],positions[3*i+1],positions[3*i+2],1.,p);
    std::fesetround(old);
   });
  }catch(...){for(auto& w:workers)if(w.joinable())w.join();return 2;}
  for(auto& w:workers)w.join();return 0;
 }catch(...){return 2;}
}
