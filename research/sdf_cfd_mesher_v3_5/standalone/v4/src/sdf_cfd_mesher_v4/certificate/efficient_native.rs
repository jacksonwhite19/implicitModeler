// Conservative filters only. Every uncertain sign returns to the frozen kernel.
// No fast-math, fused reductions, tolerance, topology-only exclusion, or root work.
use std::slice;
#[derive(Clone,Copy)] struct I { l:f64,h:f64 }
impl I {
 fn exact(x:f64)->Self { Self{l:x,h:x} }
 fn add(self,b:Self)->Self { if self.l==0. && self.h==0.{return b} if b.l==0. && b.h==0.{return self} Self{l:(self.l+b.l).next_down(),h:(self.h+b.h).next_up()} }
 fn sub(self,b:Self)->Self { if self.l==self.h && b.l==b.h && self.l==b.l {return Self::exact(0.)} Self{l:(self.l-b.h).next_down(),h:(self.h-b.l).next_up()} }
 fn mul(self,b:Self)->Self {
  if (self.l==0. && self.h==0.) || (b.l==0. && b.h==0.) {return Self::exact(0.)}
  let v=[self.l*b.l,self.l*b.h,self.h*b.l,self.h*b.h];
  if v.iter().any(|x|!x.is_finite()) {return Self{l:f64::NEG_INFINITY,h:f64::INFINITY}}
  Self{l:v.iter().copied().fold(f64::INFINITY,f64::min).next_down(),h:v.iter().copied().fold(f64::NEG_INFINITY,f64::max).next_up()}
 }
}
type V=[f64;3];type VI=[I;3];type Q=[V;4];
fn sub(a:V,b:V)->V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn cross(a:V,b:V)->V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn si(a:V,b:V)->VI { std::array::from_fn(|j|I::exact(a[j]).sub(I::exact(b[j]))) }
fn ci(a:VI,b:VI)->VI { [a[1].mul(b[2]).sub(a[2].mul(b[1])),a[2].mul(b[0]).sub(a[0].mul(b[2])),a[0].mul(b[1]).sub(a[1].mul(b[0]))] }
fn di(a:VI,b:VI)->I { a[0].mul(b[0]).add(a[1].mul(b[1])).add(a[2].mul(b[2])) }
fn vi(a:V)->VI {a.map(I::exact)}
fn mean(q:&Q)->V {std::array::from_fn(|j|q.iter().map(|p|p[j]*0.25).sum())}
fn unit(a:V)->V {let s=a.iter().map(|v|v*v).sum::<f64>().sqrt();if s>0. && s.is_finite(){a.map(|v|v/s)}else{a}}
const EDGES:[(usize,usize);4]=[(0,1),(1,3),(3,2),(2,0)];
fn normals(q:&Q)->[V;4] {let a=sub(q[1],q[0]);let b=sub(q[3],q[2]);let c=sub(q[2],q[0]);let d=sub(q[3],q[1]);[cross(a,c),cross(a,d),cross(b,c),cross(b,d)]}
fn valid(q:&Q)->bool {
 let a=si(q[1],q[0]);let b=si(q[3],q[2]);let c=si(q[2],q[0]);let d=si(q[3],q[1]);
 let ns=[ci(a,c),ci(a,d),ci(b,c),ci(b,d)];
 // Strict projected corner Jacobians certify a regular convex projection.
 (0..3).any(|j|ns.iter().all(|n|n[j].l>0.) || ns.iter().all(|n|n[j].h<0.))
}
fn proof(a:&Q,b:&Q,ai:&[u64],bi:&[u64])->(u8,u8) {
 let common:Vec<(usize,usize)>=(0..4).flat_map(|i|(0..4).filter_map(move|j|if ai[i]==bi[j]{Some((i,j))}else{None})).collect();
 if common.iter().any(|&(i,j)|a[i]!=b[j]) {return(0,255)}
 for j in 0..3 {let al=a.iter().map(|p|p[j]).fold(f64::INFINITY,f64::min);let ah=a.iter().map(|p|p[j]).fold(f64::NEG_INFINITY,f64::max);let bl=b.iter().map(|p|p[j]).fold(f64::INFINITY,f64::min);let bh=b.iter().map(|p|p[j]).fold(f64::NEG_INFINITY,f64::max);if ah<bl || bh<al{return(1,j as u8)}}
 if common.len()>2{return(0,255)}
 if common.len()==2 {
  let ((i,j),(k,l))=(common[0],common[1]);
  if !EDGES.iter().any(|&(u,v)|(u==i&&v==k)||(v==i&&u==k)) || !EDGES.iter().any(|&(u,v)|(u==j&&v==l)||(v==j&&u==l)){return(0,255)}
 }
 let ma=mean(a);let mb=mean(b);let origin=if common.is_empty(){a[0]}else{a[common[0].0]};
 let na=normals(a);let nb=normals(b);
 let mut candidates=vec![sub(ma,mb),[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
 candidates.extend(na);candidates.extend(nb);
 if common.len()==2 {
  let edge=sub(a[common[1].0],origin);
  let center=std::array::from_fn(|j|(origin[j]+a[common[1].0][j])*0.5);
  candidates.insert(0,cross(sub(unit(sub(ma,center)),unit(sub(mb,center))),edge));
 } else {
  for (q,ns) in [(a,na),(b,nb)] {for &(i,j) in &EDGES {for n in ns {candidates.push(cross(sub(q[j],q[i]),n));}}}
 }
 let mut exact_planes:Vec<(VI,[V;3])>=vec![];
 if !common.is_empty(){for q in [a,b]{for tri in [[0,1,2],[0,1,3],[0,2,3],[1,2,3]]{
  let pts=[q[tri[0]],q[tri[1]],q[tri[2]]];
  if !pts.contains(&origin) || (common.len()==2 && !pts.contains(&a[common[1].0])){continue}
  let mut others=pts.iter().copied().filter(|p|*p!=origin);if let (Some(u),Some(v))=(others.next(),others.next()){exact_planes.push((ci(si(u,origin),si(v,origin)),pts));}
 }}}
 let mut accum_a=[I::exact(0.);4];let mut accum_b=accum_a;
 for w in 0..candidates.len()+exact_planes.len() {
  let n=if w>=candidates.len(){exact_planes[w-candidates.len()].0}else{
   let normal=candidates[w];if normal.iter().any(|v|!v.is_finite()) || normal.iter().all(|v|*v==0.){continue}
   if common.len()==2 {ci(si(a[common[1].0],origin),vi(normal))}else{vi(normal)}
  };
  let known=if w>=candidates.len(){Some(exact_planes[w-candidates.len()].1)}else{None};
  let mut va=[I::exact(0.);4];let mut vb=va;
  for i in 0..4 {if !common.iter().any(|&(j,_)|i==j) && !known.is_some_and(|p|p.contains(&a[i])){va[i]=di(n,si(a[i],origin));}}
  for i in 0..4 {if !common.iter().any(|&(_,j)|i==j) && !known.is_some_and(|p|p.contains(&b[i])){vb[i]=di(n,si(b[i],origin));}}
  if common.is_empty() {
   if va.iter().map(|x|x.h).fold(f64::NEG_INFINITY,f64::max)<vb.iter().map(|x|x.l).fold(f64::INFINITY,f64::min) || vb.iter().map(|x|x.h).fold(f64::NEG_INFINITY,f64::max)<va.iter().map(|x|x.l).fold(f64::INFINITY,f64::min){return(2,w as u8)}
  } else {
   let aa:Vec<I>=(0..4).filter(|i|!common.iter().any(|&(j,_)|*i==j)).map(|i|va[i]).collect();
   let bb:Vec<I>=(0..4).filter(|i|!common.iter().any(|&(_,j)|*i==j)).map(|i|vb[i]).collect();
   let forward=aa.iter().all(|v|v.l>=0.) && bb.iter().all(|v|v.h<=0.) && (aa.iter().all(|v|v.l>0.) || bb.iter().all(|v|v.h<0.));
   let reverse=aa.iter().all(|v|v.h<=0.) && bb.iter().all(|v|v.l>=0.) && (aa.iter().all(|v|v.h<0.) || bb.iter().all(|v|v.l>0.));
   if forward || reverse{return(if common.len()==1{3}else{4},w as u8)}
   if aa.iter().all(|v|v.l>=0.) && bb.iter().all(|v|v.h<=0.) {
    for i in 0..4{accum_a[i]=accum_a[i].add(va[i]);accum_b[i]=accum_b[i].add(I::exact(0.).sub(vb[i]));}
   } else if aa.iter().all(|v|v.h<=0.) && bb.iter().all(|v|v.l>=0.) {
    for i in 0..4{accum_a[i]=accum_a[i].add(I::exact(0.).sub(va[i]));accum_b[i]=accum_b[i].add(vb[i]);}
   }
   if (0..4).filter(|i|!common.iter().any(|&(j,_)|*i==j)).all(|i|accum_a[i].l>0.) || (0..4).filter(|i|!common.iter().any(|&(_,j)|*i==j)).all(|i|accum_b[i].l>0.) {return(if common.len()==1{5}else{6},w as u8)}
  }
 }
 (0,255)
}
#[no_mangle] pub unsafe extern "C" fn screen(q:*const f64,ids:*const u64,n:usize,pairs:*const u32,m:usize,out:*mut u8,witness:*mut u8) {
 let quads=slice::from_raw_parts(q as *const Q,n);let labels=slice::from_raw_parts(ids,n*4);let pp=slice::from_raw_parts(pairs,m*2);let codes=slice::from_raw_parts_mut(out,m);let ww=slice::from_raw_parts_mut(witness,m);
 for k in 0..m {let a=pp[k*2] as usize;let b=pp[k*2+1] as usize;let (c,w)=proof(&quads[a],&quads[b],&labels[a*4..a*4+4],&labels[b*4..b*4+4]);codes[k]=c;ww[k]=w;}
}
#[no_mangle] pub unsafe extern "C" fn validity(q:*const f64,n:usize,out:*mut u8){let qq=slice::from_raw_parts(q as *const Q,n);let o=slice::from_raw_parts_mut(out,n);for i in 0..n{o[i]=valid(&qq[i]) as u8;}}

#[derive(Clone)] struct Node {lo:V,hi:V,start:usize,end:usize,left:usize,right:usize}
pub struct Forest {lo:Vec<V>,hi:Vec<V>,ids:Vec<u32>,nodes:Vec<Node>,roots:Vec<usize>,width:usize}
impl Forest {
 fn build(&mut self,start:usize,end:usize)->usize {
  let mut lo=[f64::INFINITY;3];let mut hi=[f64::NEG_INFINITY;3];
  for &id in &self.ids[start..end] {for j in 0..3 {lo[j]=lo[j].min(self.lo[id as usize][j]);hi[j]=hi[j].max(self.hi[id as usize][j]);}}
  let idx=self.nodes.len();self.nodes.push(Node{lo,hi,start,end,left:usize::MAX,right:usize::MAX});
  if end-start>8 {let axis=(0..3).max_by(|&a,&b|(hi[a]-lo[a]).total_cmp(&(hi[b]-lo[b]))).unwrap();let mid=(end-start)/2;let l=&self.lo;let h=&self.hi;
   self.ids[start..end].select_nth_unstable_by(mid,|&a,&b|((l[a as usize][axis]+h[a as usize][axis])*0.5).total_cmp(&((l[b as usize][axis]+h[b as usize][axis])*0.5)).then(a.cmp(&b)));
   let left=self.build(start,start+mid);let right=self.build(start+mid,end);self.nodes[idx].left=left;self.nodes[idx].right=right;
  } idx
 }
 fn query(&self,node:usize,b:usize,out:&mut Vec<u64>){let n=&self.nodes[node];
  if (0..3).any(|j|n.hi[j]<self.lo[b][j]||self.hi[b][j]<n.lo[j]){return}
  if n.left==usize::MAX {for &a in &self.ids[n.start..n.end] {if (a as usize)<b && (0..3).all(|j|self.hi[a as usize][j]>=self.lo[b][j]&&self.hi[b][j]>=self.lo[a as usize][j]){out.push(((a as u64)<<32)|(b as u64));}}}
  else {self.query(n.left,b,out);self.query(n.right,b,out);}
 }
}
#[no_mangle] pub unsafe extern "C" fn forest_new(q:*const f64,n:usize,width:usize)->*mut Forest {
 let qq=slice::from_raw_parts(q as *const Q,n);let lo=qq.iter().map(|q|std::array::from_fn(|j|q.iter().map(|p|p[j]).fold(f64::INFINITY,f64::min))).collect();
 let hi=qq.iter().map(|q|std::array::from_fn(|j|q.iter().map(|p|p[j]).fold(f64::NEG_INFINITY,f64::max))).collect();
 let mut f=Forest{lo,hi,ids:(0..n as u32).collect(),nodes:vec![],roots:vec![],width};
 for start in (0..n).step_by(width){let root=f.build(start,(start+width).min(n));f.roots.push(root);} Box::into_raw(Box::new(f))
}
#[no_mangle] pub unsafe extern "C" fn forest_free(p:*mut Forest){drop(Box::from_raw(p));}
#[no_mangle] pub unsafe extern "C" fn forest_append(p:*mut Forest,q:*const f64) {
 let f=&mut *p;let qq=slice::from_raw_parts(q as *const Q,f.width);let start=f.ids.len();
 for (i,q) in qq.iter().enumerate(){f.lo.push(std::array::from_fn(|j|q.iter().map(|p|p[j]).fold(f64::INFINITY,f64::min)));f.hi.push(std::array::from_fn(|j|q.iter().map(|p|p[j]).fold(f64::NEG_INFINITY,f64::max)));f.ids.push((start+i) as u32);}
 let root=f.build(start,start+f.width);f.roots.push(root);
}
#[no_mangle] pub extern "C" fn ieee_environment()->u8 {
 let x=std::hint::black_box(f64::MIN_POSITIVE);let y=std::hint::black_box(0.5);((x*y)>0. && 0f64.next_up()>0.) as u8
}
#[repr(C)] pub struct Buffer {pub data:*mut u64,pub len:usize,pub cap:usize}
#[no_mangle] pub unsafe extern "C" fn forest_pairs(p:*const Forest,layer:usize)->Buffer {
 let f=&*p;let mut pairs=vec![];for b in (layer-1)*f.width..layer*f.width {for root in &f.roots[..layer]{f.query(*root,b,&mut pairs);}}
 pairs.sort_unstable();let result=Buffer{data:pairs.as_mut_ptr(),len:pairs.len(),cap:pairs.capacity()};std::mem::forget(pairs);result
}
#[no_mangle] pub unsafe extern "C" fn buffer_free(b:Buffer){drop(Vec::from_raw_parts(b.data,b.len,b.cap));}
