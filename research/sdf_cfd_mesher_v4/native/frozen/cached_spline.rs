//! Smooth NACA section authority. Stored preview samples are never distance geometry.
//! A single clamped C2 cubic spline runs upper TE -> LE -> lower TE. The closing
//! trailing face is a separate straight branch, with no continuity imposed there.
use glam::DVec2;
use serde::{Deserialize, Serialize};

pub const SPAN_COUNT: usize = 4096;
pub const QUERY_BUDGET: usize = 200_000;
pub const DISTANCE_TOLERANCE: f64 = 2.0e-12;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CubicSpan { pub control: [[f64; 2]; 4] }

impl CubicSpan {
    fn points(&self) -> [DVec2; 4] { self.control.map(DVec2::from_array) }
    pub fn evaluate(&self, t: f64) -> (DVec2, DVec2) {
        let p=self.points(); let s=1.0-t;
        (p[0]*(s*s*s)+p[1]*(3.0*s*s*t)+p[2]*(3.0*s*t*t)+p[3]*(t*t*t),
         (p[1]-p[0])*(3.0*s*s)+(p[2]-p[1])*(6.0*s*t)+(p[3]-p[2])*(3.0*t*t))
    }
    fn split(&self) -> (Self, Self) {
        let p=self.points(); let a=(p[0]+p[1])*0.5; let b=(p[1]+p[2])*0.5;
        let c=(p[2]+p[3])*0.5; let d=(a+b)*0.5; let e=(b+c)*0.5; let f=(d+e)*0.5;
        (Self{control:[p[0],a,d,f].map(|v|v.to_array())},Self{control:[f,e,c,p[3]].map(|v|v.to_array())})
    }
    fn bounds(&self) -> (DVec2,DVec2) {
        let p=self.points();(p.iter().copied().fold(DVec2::splat(f64::INFINITY),DVec2::min),p.iter().copied().fold(DVec2::splat(f64::NEG_INFINITY),DVec2::max))
    }
    fn squared_distance_monotone(&self,q:DVec2)->bool {
        // Bernstein product of (B-q), degree 3, with B', degree 2.
        // Its coefficient hull encloses (B-q).B' over the complete interval.
        let p=self.points();let c3=[1.0,3.0,3.0,1.0];let c2=[1.0,2.0,1.0];let c5=[1.0,5.0,10.0,10.0,5.0,1.0];
        let mut v=[0.0;6];let mut err=[0.0;6];
        for i in 0..4 {for j in 0..3 {let a=p[i]-q;let b=(p[j+1]-p[j])*3.0;let f=c3[i]*c2[j]/c5[i+j];v[i+j]+=f*a.dot(b);err[i+j]+=f*(a.x.abs()*b.x.abs()+a.y.abs()*b.y.abs());}}
        v.iter().zip(err).all(|(v,e)|*v>e*64.0*f64::EPSILON) || v.iter().zip(err).all(|(v,e)|*v< -e*64.0*f64::EPSILON)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct NacaSpline { pub spans: Vec<CubicSpan> }

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum SectionBranch { Upper, Lower, TrailingFace }

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SectionClosestPoint {
    pub signed_distance: f64,
    /// Upper/lower: LE=0, TE=1. Trailing face: lower=0, upper=1.
    pub parameter: f64,
    pub point: [f64;2],
    pub tangent: [f64;2],
    pub outward_normal: [f64;2],
    pub branch: SectionBranch,
    pub distance_error_bound: f64,
    pub visited_nodes: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SectionQueryError { InvalidInput, ExhaustedCoverageBudget, UnresolvedCoverage { depth:usize, nodes:usize, gap:f64 }, InvalidTangent }

/// Direct NACA formula in a regular leading-edge parameter. This function does
/// not depend on the historical polygon or its sampling count.
pub fn naca_point(m:f64,p:f64,thickness:f64,u:f64)->DVec2 {
    let x=u*u; let upper=u<=0.0;
    let yt=5.0*thickness*(0.2969*u.abs()-0.1260*x-0.3516*x*x+0.2843*x*x*x-0.1015*x*x*x*x);
    let (yc,dy)=if m==0.0 || p<0.0001 {(0.0,0.0)} else if x<p {
        (m/(p*p)*(2.0*p*x-x*x),2.0*m/(p*p)*(p-x))
    } else {(m/((1.0-p)*(1.0-p))*(1.0-2.0*p+2.0*p*x-x*x),2.0*m/((1.0-p)*(1.0-p))*(p-x))};
    let cs=1.0/(1.0+dy*dy).sqrt();let sn=dy*cs;let s=if upper{1.0}else{-1.0};
    DVec2::new(x-s*yt*sn,yc+s*yt*cs)
}

impl NacaSpline {
    pub fn new(m:f64,p:f64,t:f64,upper_te:[f64;2],lower_te:[f64;2])->Result<Self,SectionQueryError> {
        if ![m,p,t].iter().all(|v|v.is_finite()) || t<=0.0 || !(0.0..1.0).contains(&p) {return Err(SectionQueryError::InvalidInput)}
        let n=SPAN_COUNT;let h=2.0/n as f64;
        let mut q:Vec<DVec2>=(0..=n).map(|i|naca_point(m,p,t,-1.0+h*i as f64)).collect();
        // Preserve the existing formula's endpoint values (including binary32
        // rounding). No smoothing or blending is applied across the trailing face.
        q[0]=DVec2::from_array(upper_te);q[n]=DVec2::from_array(lower_te);
        let eps=1e-5;
        let d0=(-naca_point(m,p,t,-1.0+2.0*eps)+naca_point(m,p,t,-1.0+eps)*4.0-naca_point(m,p,t,-1.0)*3.0)/(2.0*eps);
        let dn=(naca_point(m,p,t,1.0)*3.0-naca_point(m,p,t,1.0-eps)*4.0+naca_point(m,p,t,1.0-2.0*eps))/(2.0*eps);
        // Tridiagonal clamped cubic interpolation, solved once at construction.
        let mut diag=vec![4.0;n+1];let mut rhs=vec![DVec2::ZERO;n+1];
        diag[0]=2.0;diag[n]=2.0;rhs[0]=((q[1]-q[0])/h-d0)*(6.0/h);
        rhs[n]=(dn-(q[n]-q[n-1])/h)*(6.0/h);
        for i in 1..n {rhs[i]=(q[i+1]-q[i]*2.0+q[i-1])*(6.0/(h*h));}
        for i in 1..=n {let a=1.0/diag[i-1];diag[i]-=a;let r=rhs[i-1];rhs[i]-=r*a;}
        let mut dd=vec![DVec2::ZERO;n+1];dd[n]=rhs[n]/diag[n];
        for i in (0..n).rev(){dd[i]=(rhs[i]-dd[i+1])/diag[i];}
        let spans=(0..n).map(|i|{
            let da=(q[i+1]-q[i])/h-(dd[i]*2.0+dd[i+1])*(h/6.0);
            let db=(q[i+1]-q[i])/h+(dd[i]+dd[i+1]*2.0)*(h/6.0);
            CubicSpan{control:[q[i],q[i]+da*(h/3.0),q[i+1]-db*(h/3.0),q[i+1]].map(|v|v.to_array())}
        }).collect();
        Ok(Self{spans})
    }
    pub fn evaluate(&self,upper:bool,parameter:f64)->(DVec2,DVec2) {
        let u=if upper{-parameter.clamp(0.0,1.0)}else{parameter.clamp(0.0,1.0)};
        let s=(u+1.0)*0.5*self.spans.len() as f64;let i=(s.floor() as usize).min(self.spans.len()-1);
        let (p,d)=self.spans[i].evaluate(s-i as f64);
        (p,d*(self.spans.len() as f64*0.5)*if upper{-1.0}else{1.0})
    }
    pub fn blended(&self,other:&Self,t:f64)->Option<Self> {
        if self.spans.len()!=other.spans.len(){return None}
        Some(Self{spans:self.spans.iter().zip(&other.spans).map(|(a,b)|CubicSpan{control:std::array::from_fn(|i|std::array::from_fn(|j|a.control[i][j]+t*(b.control[i][j]-a.control[i][j])))}).collect()})
    }
    pub fn bounds(&self)->(DVec2,DVec2) {
        self.spans.iter().map(CubicSpan::bounds).fold((DVec2::splat(f64::INFINITY),DVec2::splat(f64::NEG_INFINITY)),|(lo,hi),(a,b)|(lo.min(a),hi.max(b)))
    }
    /// Ray parity against the actual cubic spans. Split at every y-extremum,
    /// then bracket the unique crossing on each monotone interval. Half-open
    /// endpoints prevent double counting knots and tangencies.
    pub fn contains(&self,q:DVec2)->bool {
        let mut crossings=0;
        for span in &self.spans {
            let (lo,hi)=span.bounds();if q.y<lo.y || q.y>hi.y || q.x>=hi.x {continue}
            let p=span.points(); let a=-p[0].y+3.0*p[1].y-3.0*p[2].y+p[3].y;
            let b=2.0*(p[0].y-2.0*p[1].y+p[2].y);let c=p[1].y-p[0].y;
            let mut cuts=vec![0.0,1.0];
            if a.abs()<1e-30 {if b!=0.0 {let r=-c/b;if r>0.0&&r<1.0{cuts.push(r)}}}
            else {let disc=b*b-4.0*a*c;if disc>=0.0 {let d=disc.sqrt();for r in [(-b-d)/(2.0*a),(-b+d)/(2.0*a)] {if r>0.0&&r<1.0{cuts.push(r)}}}}
            cuts.sort_by(f64::total_cmp);cuts.dedup();
            for w in cuts.windows(2) {
                let mut l=w[0];let mut r=w[1];let yl=span.evaluate(l).0.y;let yr=span.evaluate(r).0.y;
                if (yl>q.y)==(yr>q.y){continue}
                for _ in 0..52 {let mid=(l+r)*0.5;let y=span.evaluate(mid).0.y;if (y>q.y)==(yl>q.y){l=mid}else{r=mid}}
                if span.evaluate((l+r)*0.5).0.x>q.x{crossings+=1;}
            }
        }
        let a=DVec2::from_array(self.spans.last().unwrap().control[3]);let b=DVec2::from_array(self.spans[0].control[0]);
        if (a.y>q.y)!=(b.y>q.y) && q.x<(b.x-a.x)*(q.y-a.y)/(b.y-a.y)+a.x {crossings+=1;}
        crossings%2==1
    }
    pub fn closest_point(&self,q:DVec2)->Result<SectionClosestPoint,SectionQueryError>{self.closest_point_with_budget(q,QUERY_BUDGET)}
    pub fn closest_skin_point(&self,q:DVec2)->Result<SectionClosestPoint,SectionQueryError>{self.closest_impl(q,QUERY_BUDGET,false)}
    pub fn closest_point_with_budget(&self,q:DVec2,budget:usize)->Result<SectionClosestPoint,SectionQueryError> {
        self.closest_impl(q,budget,true)
    }
    fn closest_impl(&self,q:DVec2,budget:usize,include_trailing_face:bool)->Result<SectionClosestPoint,SectionQueryError> {
        if !q.is_finite() || self.spans.is_empty() || budget==0 {return Err(SectionQueryError::InvalidInput)}
        let a=DVec2::from_array(self.spans.last().unwrap().control[3]);let b=DVec2::from_array(self.spans[0].control[0]);let edge=b-a;
        if edge.length_squared()==0.0{return Err(SectionQueryError::InvalidInput)}
        let te=((q-a).dot(edge)/edge.length_squared()).clamp(0.0,1.0);let mut best_p=a+edge*te;
        let mut best_d=if include_trailing_face{q.distance(best_p)}else{f64::INFINITY};let mut best_i=self.spans.len();let mut best_t=te;
        // Each retained box contains its entire Bezier span by the convex-hull
        // property. No span is excluded by a nearest-sample heuristic.
        let mut stack=Vec::new();
        for (i,s) in self.spans.iter().enumerate() {
            for t in [0.0,0.5,1.0] {let p=s.evaluate(t).0;let d=q.distance(p);if d<best_d {best_d=d;best_p=p;best_i=i;best_t=t;}}
            stack.push((s.clone(),i,0.0,1.0,0usize));
        }
        let tol=DISTANCE_TOLERANCE*(1.0+q.length());let mut nodes=0;
        while let Some((s,i,l,r,depth))=stack.pop() {
            nodes+=1;if nodes>budget{return Err(SectionQueryError::ExhaustedCoverageBudget)}
            let (lo,hi)=s.bounds();let pad=16.0*f64::EPSILON*(1.0+q.length()+lo.length()+hi.length())*(depth+1) as f64;
            let lower=(q.distance(q.clamp(lo,hi))-pad).max(0.0);
            if lower>=best_d-tol {continue}
            if s.squared_distance_monotone(q) {
                for t in [l,r] {let p=self.spans[i].evaluate(t).0;let d=q.distance(p);if d<best_d{best_d=d;best_p=p;best_i=i;best_t=t;}}
                continue;
            }
            let mid=(l+r)*0.5;let p=self.spans[i].evaluate(mid).0;let d=q.distance(p);
            if d<best_d {best_d=d;best_p=p;best_i=i;best_t=mid;}
            if depth>=48 {return Err(SectionQueryError::UnresolvedCoverage{depth,nodes,gap:best_d-lower})}
            let (left,right)=s.split();stack.push((right,i,mid,r,depth+1));stack.push((left,i,l,mid,depth+1));
        }
        let (branch,parameter,tangent)=if best_i==self.spans.len(){(SectionBranch::TrailingFace,best_t,edge.normalize())}else{
            let u=2.0*(best_i as f64+best_t)/self.spans.len() as f64-1.0;
            (if u<=0.0{SectionBranch::Upper}else{SectionBranch::Lower},u.abs(),self.spans[best_i].evaluate(best_t).1.normalize())
        };
        if !tangent.is_finite(){return Err(SectionQueryError::InvalidTangent)}
        Ok(SectionClosestPoint{signed_distance:if self.contains(q){-best_d}else{best_d},parameter,point:best_p.to_array(),tangent:tangent.to_array(),outward_normal:DVec2::new(tangent.y,-tangent.x).to_array(),branch,distance_error_bound:tol,visited_nodes:nodes})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn section()->NacaSpline {NacaSpline::new(0.0,0.0,0.12,[1.0,0.00126],[1.0,-0.00126]).unwrap()}
    #[test] fn smooth_leading_edge_and_knots() {
        let s=section();let (p,a)=s.evaluate(true,0.0);let(q,b)=s.evaluate(false,0.0);
        assert_eq!(p,q);assert!((a+b).length()<1e-12);assert!(a.length()>0.1);
        for w in s.spans.windows(2) {let (_,da)=w[0].evaluate(1.0);let (_,db)=w[1].evaluate(0.0);assert!((da-db).length()<1e-14);}
    }
    #[test] fn coverage_budget_fails_closed() {
        assert!(matches!(section().closest_point_with_budget(DVec2::new(0.4,0.1),1),Err(SectionQueryError::ExhaustedCoverageBudget)));
        assert!(section().closest_point(DVec2::splat(f64::NAN)).is_err());
    }
    #[test] fn closed_sign_and_real_trailing_face() {
        let s=section();assert!(s.closest_point(DVec2::new(0.5,0.0)).unwrap().signed_distance<0.0);
        let q=s.closest_point(DVec2::new(1.001,0.0)).unwrap();assert_eq!(q.branch,SectionBranch::TrailingFace);assert!((q.signed_distance-0.001).abs()<1e-12);
    }
    #[test] fn distance_gradient_and_normals() {
        let s=section();
        for upper in [true,false] {for u in [0.0,0.02,0.4,0.9] {
            let(p,d)=s.evaluate(upper,u);let d=if upper{-d}else{d};let n=DVec2::new(d.y,-d.x).normalize();let q=p+n*1e-4;
            let r=s.closest_point(q).unwrap();let eps=1e-6;let mut g=DVec2::ZERO;
            for k in 0..2 {let mut step=DVec2::ZERO;step[k]=eps;g[k]=(s.closest_point(q+step).unwrap().signed_distance-s.closest_point(q-step).unwrap().signed_distance)/(2.0*eps);}
            assert!((g-n).length()<2e-5);assert!((g.length()-1.0).abs()<2e-5);assert!((r.signed_distance-1e-4).abs()<2e-11);
        }}
    }
}

#[derive(Clone)]
pub struct SpanCache {pub bounds:Vec<(DVec2,DVec2)>,pub seed_bounds:Vec<(DVec2,DVec2)>,pub seeds:Vec<[DVec2;3]>,pub valid:bool}
pub struct QueryStats {pub exact_spans:usize,pub rejected:usize,pub refined_roots:usize,pub winner:usize,pub nodes:usize,pub fallback:usize,pub retained:Vec<u8>}
impl QueryStats {pub fn new(n:usize)->Self{Self{exact_spans:0,rejected:0,refined_roots:0,winner:0,nodes:0,fallback:0,retained:vec![0;n]}}}
impl SpanCache {pub fn new(s:&NacaSpline)->Self{let bounds:Vec<_>=s.spans.iter().map(CubicSpan::bounds).collect();let seeds:Vec<[DVec2;3]>=s.spans.iter().map(|span|[span.evaluate(0.0).0,span.evaluate(0.5).0,span.evaluate(1.0).0]).collect();let valid=s.spans.iter().all(|sp|sp.control.iter().flatten().all(|x|x.is_finite()))&&bounds.iter().all(|(lo,hi)|lo.is_finite()&&hi.is_finite());let seed_bounds=bounds.iter().zip(&seeds).map(|((lo,hi),ps)|{let mut a=*lo;let mut b=*hi;for p in ps {a=a.min(*p);b=b.max(*p);} (a,b)}).collect();Self{bounds,seed_bounds,seeds,valid}}}
impl NacaSpline {
    fn contains_cached(&self,q:DVec2,cache:&SpanCache)->bool {
        let mut crossings=0;
        for (i,span) in self.spans.iter().enumerate() {
            let (lo,hi)=cache.bounds[i];if q.y<lo.y || q.y>hi.y || q.x>=hi.x {continue}
            let p=span.points(); let a=-p[0].y+3.0*p[1].y-3.0*p[2].y+p[3].y;
            let b=2.0*(p[0].y-2.0*p[1].y+p[2].y);let c=p[1].y-p[0].y;
            let mut cuts=vec![0.0,1.0];
            if a.abs()<1e-30 {if b!=0.0 {let r=-c/b;if r>0.0&&r<1.0{cuts.push(r)}}}
            else {let disc=b*b-4.0*a*c;if disc>=0.0 {let d=disc.sqrt();for r in [(-b-d)/(2.0*a),(-b+d)/(2.0*a)] {if r>0.0&&r<1.0{cuts.push(r)}}}}
            cuts.sort_by(f64::total_cmp);cuts.dedup();
            for w in cuts.windows(2) {
                let mut l=w[0];let mut r=w[1];let yl=span.evaluate(l).0.y;let yr=span.evaluate(r).0.y;
                if (yl>q.y)==(yr>q.y){continue}
                for _ in 0..52 {let mid=(l+r)*0.5;let y=span.evaluate(mid).0.y;if (y>q.y)==(yl>q.y){l=mid}else{r=mid}}
                if span.evaluate((l+r)*0.5).0.x>q.x{crossings+=1;}
            }
        }
        let a=DVec2::from_array(self.spans.last().unwrap().control[3]);let b=DVec2::from_array(self.spans[0].control[0]);
        if (a.y>q.y)!=(b.y>q.y) && q.x<(b.x-a.x)*(q.y-a.y)/(b.y-a.y)+a.x {crossings+=1;}
        crossings%2==1
    }
    pub fn closest_cached(&self,q:DVec2,budget:usize,include_trailing_face:bool,cache:&SpanCache,fast:bool,stats:&mut QueryStats)->Result<SectionClosestPoint,SectionQueryError> {
        if fast && !cache.valid {stats.fallback=1;return self.closest_impl(q,budget,include_trailing_face);}
        if !q.is_finite() || self.spans.is_empty() || budget==0 {return Err(SectionQueryError::InvalidInput)}
        let a=DVec2::from_array(self.spans.last().unwrap().control[3]);let b=DVec2::from_array(self.spans[0].control[0]);let edge=b-a;
        if edge.length_squared()==0.0{return Err(SectionQueryError::InvalidInput)}
        let te=((q-a).dot(edge)/edge.length_squared()).clamp(0.0,1.0);let mut best_p=a+edge*te;
        let mut best_d=if include_trailing_face{q.distance(best_p)}else{f64::INFINITY};let mut best_i=self.spans.len();let mut best_t=te;
        // Each retained box contains its entire Bezier span by the convex-hull
        // property. No span is excluded by a nearest-sample heuristic.
        let mut stack=Vec::new();
        for (i,s) in self.spans.iter().enumerate() {
            let (lo,hi)=cache.seed_bounds[i];
            let pad=16.0*f64::EPSILON*(1.0+q.length()+lo.length()+hi.length());
            let lower=(q.distance(q.clamp(lo,hi))-pad).max(0.0);
            if !fast || !cache.valid || !(lower>best_d) {
                stats.exact_spans+=1;stats.retained[i]=1;
                for (j,t) in [0.0,0.5,1.0].into_iter().enumerate() {let p=if fast && cache.valid {cache.seeds[i][j]}else{s.evaluate(t).0};let d=q.distance(p);if d<best_d {best_d=d;best_p=p;best_i=i;best_t=t;}}
            } else {stats.rejected+=1;}
            stack.push((s.clone(),i,0.0,1.0,0usize));
        }
        let tol=DISTANCE_TOLERANCE*(1.0+q.length());let mut nodes=0;
        while let Some((s,i,l,r,depth))=stack.pop() {
            nodes+=1;if nodes>budget{return Err(SectionQueryError::ExhaustedCoverageBudget)}
            let (lo,hi)=if fast && cache.valid && depth==0 {cache.bounds[i]}else{s.bounds()};let pad=16.0*f64::EPSILON*(1.0+q.length()+lo.length()+hi.length())*(depth+1) as f64;
            let lower=(q.distance(q.clamp(lo,hi))-pad).max(0.0);
            if lower>=best_d-tol {continue}
            stats.refined_roots+=usize::from(depth==0);
            if s.squared_distance_monotone(q) {
                for t in [l,r] {let p=self.spans[i].evaluate(t).0;let d=q.distance(p);if d<best_d{best_d=d;best_p=p;best_i=i;best_t=t;}}
                continue;
            }
            let mid=(l+r)*0.5;let p=self.spans[i].evaluate(mid).0;let d=q.distance(p);
            if d<best_d {best_d=d;best_p=p;best_i=i;best_t=mid;}
            if depth>=48 {return Err(SectionQueryError::UnresolvedCoverage{depth,nodes,gap:best_d-lower})}
            let (left,right)=s.split();stack.push((right,i,mid,r,depth+1));stack.push((left,i,l,mid,depth+1));
        }
        stats.winner=best_i;stats.nodes=nodes;stats.fallback=usize::from(fast && !cache.valid);
        let (branch,parameter,tangent)=if best_i==self.spans.len(){(SectionBranch::TrailingFace,best_t,edge.normalize())}else{
            let u=2.0*(best_i as f64+best_t)/self.spans.len() as f64-1.0;
            (if u<=0.0{SectionBranch::Upper}else{SectionBranch::Lower},u.abs(),self.spans[best_i].evaluate(best_t).1.normalize())
        };
        if !tangent.is_finite(){return Err(SectionQueryError::InvalidTangent)}
        Ok(SectionClosestPoint{signed_distance:if if fast && cache.valid {self.contains_cached(q,cache)}else{self.contains(q)}{-best_d}else{best_d},parameter,point:best_p.to_array(),tangent:tangent.to_array(),outward_normal:DVec2::new(tangent.y,-tangent.x).to_array(),branch,distance_error_bound:tol,visited_nodes:nodes})
    }
}
