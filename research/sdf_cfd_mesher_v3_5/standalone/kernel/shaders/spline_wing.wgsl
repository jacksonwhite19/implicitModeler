// The stored canonical cubic controls, rounded to binary32. No polygon proxy.
struct CurveNode { bounds:vec4<f32>, links:vec4<u32>, ab:vec4<f32>, cd:vec4<f32> }
@group(0) @binding(3) var<storage,read> curves:array<CurveNode>;
@group(0) @binding(4) var<storage,read_write> query_failed:atomic<u32>;
struct Curve {a:vec2<f32>,b:vec2<f32>,c:vec2<f32>,d:vec2<f32>}
fn bez(s:Curve,t:f32)->vec2<f32> {
    let u=1.0-t;
    return s.a*(u*u*u)+s.b*(3.0*u*u*t)+s.c*(3.0*u*t*t)+s.d*(t*t*t);
}
fn curve_bounds(s:Curve)->vec4<f32>{return vec4<f32>(min(min(s.a,s.b),min(s.c,s.d)),max(max(s.a,s.b),max(s.c,s.d)));}
fn box_distance(q:vec2<f32>,b:vec4<f32>)->f32{return length(q-clamp(q,b.xy,b.zw));}
fn unresolved(code:u32)->f32{atomicOr(&query_failed,code);return 0.0;}
fn monotonic_distance(q:vec2<f32>,s:Curve)->bool {
    var p=array<vec2<f32>,4>(s.a,s.b,s.c,s.d);
    var c3=array<f32,4>(1.0,3.0,3.0,1.0);var c2=array<f32,3>(1.0,2.0,1.0);
    var c5=array<f32,6>(1.0,5.0,10.0,10.0,5.0,1.0);
    var v:array<f32,6>;var error:array<f32,6>;
    for(var i=0u;i<4u;i++){for(var j=0u;j<3u;j++){
        let a=p[i]-q;let b=(p[j+1u]-p[j])*3.0;let f=c3[i]*c2[j]/c5[i+j];
        v[i+j]+=f*dot(a,b);error[i+j]+=f*dot(abs(a),abs(b));
    }}
    var positive=true;var negative=true;
    for(var i=0u;i<6u;i++){let e=error[i]*64.0*1.1920928955078125e-7;positive=positive && v[i]>e;negative=negative && v[i]< -e;}
    return positive || negative;
}
fn cubic_distance(q:vec2<f32>,s:Curve,initial:f32)->f32 {
    var best=initial;
    let edge=s.d-s.a;
    let t=clamp(dot(q-s.a,edge)/max(dot(edge,edge),1e-30),0.0,1.0);
    best=min(best,min(length(q-s.a),min(length(q-s.d),length(q-bez(s,t)))));
    if monotonic_distance(q,s){return best;}
    var stack:array<Curve,64>;stack[0]=s;var count=1u;
    // Terminate subdivisions at binary32 coordinate resolution, not a new
    // geometry tolerance. CPU/GPU errors are qualified separately.
    let resolution=1.1920928955078125e-7*max(1.0,max(abs(q.x),abs(q.y)));
    for(var iter=0u;iter<4096u;iter++) {
        if count==0u{return best;}
        count-=1u;let v=stack[count];let bounds=curve_bounds(v);
        if box_distance(q,bounds)>=best {continue;}
        if monotonic_distance(q,v){best=min(best,min(length(q-v.a),length(q-v.d)));continue;}
        let a=(v.a+v.b)*0.5;let b=(v.b+v.c)*0.5;let c=(v.c+v.d)*0.5;
        let d=(a+b)*0.5;let e=(b+c)*0.5;let f=(d+e)*0.5;
        best=min(best,length(q-f));
        if max(bounds.z-bounds.x,bounds.w-bounds.y)<=resolution {continue;}
        if count+2u>64u{return unresolved(2u);}
        stack[count]=Curve(v.a,a,d,f);stack[count+1u]=Curve(f,e,c,v.d);count+=2u;
    }
    return unresolved(4u);
}
fn cubic_crossings(q:vec2<f32>,s:Curve)->u32 {
    // Split at all derivative roots, matching the CPU half-open parity rule.
    let d0=s.b.y-s.a.y;let d1=s.c.y-s.b.y;let d2=s.d.y-s.c.y;
    let a=d0-2.0*d1+d2;let b=2.0*(d1-d0);let c=d0;
    var cuts:array<f32,4>;cuts[0]=0.0;cuts[1]=1.0;var n=2u;
    if abs(a)<1e-30 {if b!=0.0 {let r=-c/b;if r>0.0 && r<1.0 {cuts[n]=r;n+=1u;}}}
    else {let disc=b*b-4.0*a*c;if disc>=0.0 {let h=sqrt(disc);let r0=(-b-h)/(2.0*a);let r1=(-b+h)/(2.0*a);if r0>0.0 && r0<1.0 {cuts[n]=r0;n+=1u;}if r1>0.0 && r1<1.0 {cuts[n]=r1;n+=1u;}}}
    for(var i=0u;i<n;i++){for(var j=i+1u;j<n;j++){if cuts[j]<cuts[i]{let tmp=cuts[i];cuts[i]=cuts[j];cuts[j]=tmp;}}}
    var result=0u;
    for(var i=0u;i+1u<n;i++){
        var lo=cuts[i];var hi=cuts[i+1u];let yl=bez(s,lo).y;let yr=bez(s,hi).y;
        if (yl>q.y)==(yr>q.y){continue;}
        for(var k=0u;k<26u;k++){let mid=(lo+hi)*0.5;if (bez(s,mid).y>q.y)==(yl>q.y){lo=mid;}else{hi=mid;}}
        if bez(s,(lo+hi)*0.5).x>q.x{result+=1u;}
    }
    return result;
}
fn spline_distance(q:vec2<f32>)->f32 {
    let a=curves[0].ab.xy;let b=curves[0].ab.zw;let edge=b-a;
    if dot(edge,edge)==0.0{return unresolved(1u);}
    var best=length(q-(a+edge*clamp(dot(q-a,edge)/dot(edge,edge),0.0,1.0)));
    var crossings=0u;
    if (a.y>q.y)!=(b.y>q.y) {if q.x<(b.x-a.x)*(q.y-a.y)/(b.y-a.y)+a.x{crossings+=1u;}}
    var pending:array<u32,32>;pending[0]=0u;var count=1u;
    loop {
        if count==0u{break;}
        count-=1u;let i=pending[count];
        let node=curves[i];let hit_ray=q.y>=node.bounds.y && q.y<=node.bounds.w && q.x<node.bounds.z;
        let near=box_distance(q,node.bounds)<best;
        if !near && !hit_ray {continue;}
        if node.links.y==1u {
            let s=Curve(node.ab.xy,node.ab.zw,node.cd.xy,node.cd.zw);
            if near {let d=cubic_distance(q,s,best);if d!=d{return d;}best=d;}
            if hit_ray {crossings+=cubic_crossings(q,s);}
        }else{
            let left=i+1u;let right=curves[left].links.x;
            let dl=box_distance(q,curves[left].bounds);let dr=box_distance(q,curves[right].bounds);
            if count+2u>32u{return unresolved(8u);}
            pending[count]=select(left,right,dl<dr);
            pending[count+1u]=select(right,left,dl<dr);count+=2u;
        }
    }
    return select(best,-best,(crossings%2u)==1u);
}
fn spline_wing(p:vec3<f32>,s0:vec4<f32>,s1:vec4<f32>,s2:vec4<f32>)->f32 {
    let y=clamp(p.y,s0.y,s2.y);var a=s0;var b=s1;
    if y>s1.y {a=s1;b=s2;}
    let t=(y-a.y)/(b.y-a.y);let chord=a.w+t*(b.w-a.w);
    let local=vec2<f32>(p.x-(a.x+t*(b.x-a.x)),p.z-(a.z+t*(b.z-a.z)));
    let ds=max(s0.y-p.y,p.y-s2.y);let d=spline_distance(local/chord)*chord;
    if d!=d{return d;}
    return length(max(vec2<f32>(d,ds),vec2<f32>(0.0)))+min(max(d,ds),0.0);
}
