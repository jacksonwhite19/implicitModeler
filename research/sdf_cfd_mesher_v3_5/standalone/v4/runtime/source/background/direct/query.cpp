// Unselected whole-query traversal. Existing qualified scalar leaf unchanged.
// No Gmsh callback installation. Python owns lifetime and whole-query fallback.
#define values v4_owned_sizing_leaf_values
#include "../sizing_values.cpp"
#undef values
#include <vector>
#include <memory>

struct QueryContext {
 std::size_t n,nn;
 double g,maximum;
 std::vector<double> triangles,normals,normal2,e0,e1,e2,s0,s1,s2,seeds,boxes;
 std::vector<std::int64_t> children;
 std::vector<std::uint64_t> offsets,ids;
};

extern "C" void* query_create(std::size_t n,std::size_t nn,
 const double* triangles,const double* normals,const double* normal2,
 const double* e0,const double* e1,const double* e2,const double* s0,const double* s1,const double* s2,
 const double* seeds,const double* boxes,const std::int64_t* children,
 const std::uint64_t* offsets,const std::uint64_t* ids,double g,double maximum,int* status) noexcept {
 if(!status)return nullptr;*status=1;
 if(!n||!nn||n>std::numeric_limits<std::size_t>::max()/9||nn>std::numeric_limits<std::size_t>::max()/7
    ||!triangles||!normals||!normal2||!e0||!e1||!e2||!s0||!s1||!s2||!seeds||!boxes||!children||!offsets||!ids
    ||!std::isfinite(g)||g<=0.||!std::isfinite(maximum)||maximum<=0.||offsets[0]!=0||offsets[nn]!=n)return nullptr;
 try {
  auto q=std::make_unique<QueryContext>();q->n=n;q->nn=nn;q->g=g;q->maximum=maximum;
  q->triangles.assign(triangles,triangles+9*n);q->normals.assign(normals,normals+3*n);q->normal2.assign(normal2,normal2+n);
  q->e0.assign(e0,e0+3*n);q->e1.assign(e1,e1+3*n);q->e2.assign(e2,e2+3*n);
  q->s0.assign(s0,s0+n);q->s1.assign(s1,s1+n);q->s2.assign(s2,s2+n);q->seeds.assign(seeds,seeds+n);
  q->boxes.assign(boxes,boxes+7*nn);q->children.assign(children,children+2*nn);
  q->offsets.assign(offsets,offsets+nn+1);q->ids.assign(ids,ids+n);
  for(const auto* v:{&q->triangles,&q->normals,&q->normal2,&q->e0,&q->e1,&q->e2,&q->s0,&q->s1,&q->s2,&q->seeds,&q->boxes})
   for(double x:*v)if(!std::isfinite(x))return nullptr;
  for(std::size_t i=0;i<n;++i)if(q->normal2[i]<=0.||q->s0[i]<=0.||q->s1[i]<=0.||q->s2[i]<=0.||q->seeds[i]<=0.)return nullptr;
  std::vector<unsigned char> parents(nn,0),seen(n,0);
  for(std::size_t i=0;i<nn;++i){
   if(q->offsets[i]>q->offsets[i+1]||q->offsets[i+1]>n)return nullptr;
   const auto count=q->offsets[i+1]-q->offsets[i];const auto left=q->children[2*i],right=q->children[2*i+1];
   if(q->boxes[7*i+6]<=0.)return nullptr;
   for(int k=0;k<3;++k)if(q->boxes[7*i+k]>q->boxes[7*i+3+k])return nullptr;
   if(left<0){
    if(right!=-1||left!=-1||count==0||count>16)return nullptr;
   }else{
    if(count||right<0||std::uint64_t(left)>=nn||std::uint64_t(right)>=nn||left==right)return nullptr;
    if(++parents[left]!=1||++parents[right]!=1)return nullptr;
   }
   for(auto j=q->offsets[i];j<q->offsets[i+1];++j){const auto id=q->ids[j];if(id>=n||seen[id]++)return nullptr;}
  }
  if(parents[0]||std::any_of(parents.begin()+1,parents.end(),[](auto v){return v!=1;})
     ||std::any_of(seen.begin(),seen.end(),[](auto v){return v!=1;}))return nullptr;
  // Verify reachability and a stack bound; malformed cycles cannot query.
  std::vector<std::pair<std::size_t,std::size_t>> queue{{0,0}};std::vector<unsigned char> visited(nn,0);
  for(std::size_t j=0;j<queue.size();++j){
   const auto [i,depth]=queue[j];if(depth>=64||visited[i]++)return nullptr;
   if(q->children[2*i]>=0){queue.emplace_back(q->children[2*i],depth+1);queue.emplace_back(q->children[2*i+1],depth+1);}
  }
  if(queue.size()!=nn)return nullptr;
  *status=0;return q.release();
 }catch(...){*status=2;return nullptr;}
}

extern "C" void query_destroy(void* context) noexcept {delete static_cast<QueryContext*>(context);}

extern "C" std::uint8_t query_value(const void* context,const double* position,double* output,std::uint64_t* work) noexcept {
 if(!context||!position||!output||!work||std::fegetround()!=FE_TONEAREST)return 1;
 Flags flags;
 try{
  const auto& q=*static_cast<const QueryContext*>(context);const V p=read(position);if(!finite(p))return 1;
  double best=q.maximum;std::array<std::size_t,128> stack{};std::size_t used=1;stack[0]=0;
  std::uint64_t visited=0,tested=0;std::array<double,16> leaf{};
  while(used){
   const auto i=stack[--used];const auto* box=q.boxes.data()+7*i;double distance=0.,scale=1.;
   for(int k=0;k<3;++k){
    const double delta=std::max({box[k]-p[k],p[k]-box[3+k],0.});distance=std::max(distance,delta);
    scale=std::max({scale,std::abs(box[k]),std::abs(box[3+k]),std::abs(p[k])});
   }
   distance=std::max(0.,distance-128*std::numeric_limits<double>::epsilon()*scale);
   const double bound=std::nextafter(box[6]+q.g*distance,-std::numeric_limits<double>::infinity());
   if(flags.exceptional()||std::isnan(bound))return 1;
   if(bound>best)continue;++visited;
   const auto left=q.children[2*i];
   if(left<0){
    const auto start=q.offsets[i],count=q.offsets[i+1]-start;
    const auto status=v4_owned_sizing_leaf_values(position,q.ids.data()+start,count,q.n,
     q.triangles.data(),q.normals.data(),q.normal2.data(),q.e0.data(),q.e1.data(),q.e2.data(),
     q.s0.data(),q.s1.data(),q.s2.data(),q.seeds.data(),q.g,q.maximum,leaf.data());
    if(status)return 1;
    for(std::size_t j=0;j<count;++j){if(!std::isfinite(leaf[j])||leaf[j]<=0.)return 1;best=std::min(best,leaf[j]);}
    tested+=count;
   }else{
    if(used+2>stack.size())return 1;
    stack[used++]=q.children[2*i+1];stack[used++]=left;
   }
  }
  if(flags.exceptional()||!std::isfinite(best)||best<=0.)return 1;
  *output=best;work[0]=visited;work[1]=tested;return 0;
 }catch(...){return 1;}
}

// Control-only floating environment access; no query invokes these setters.
extern "C" int query_control_round(int mode) noexcept {return std::fesetround(mode);}
extern "C" int query_control_flags(int flags) noexcept {return std::feraiseexcept(flags);}
extern "C" int query_environment(int which) noexcept {
 switch(which){case 0:return std::fegetround();case 1:return FE_TONEAREST;case 2:return FE_UPWARD;
 case 3:return FE_INVALID;case 4:return std::fetestexcept(FE_ALL_EXCEPT);case 5:return FE_ALL_EXCEPT;default:return -1;}
}
