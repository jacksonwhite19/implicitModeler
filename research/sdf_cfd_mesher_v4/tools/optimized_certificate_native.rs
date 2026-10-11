// The qualified interval/exact-safe kernel is included unchanged.
include!("../src/sdf_cfd_mesher_v4/certificate/efficient_native.rs");

fn active_query(f: &Forest, node: usize, b: usize, active: &[u8], out: &mut Vec<u64>) {
    let n = &f.nodes[node];
    if (0..3).any(|j| n.hi[j] < f.lo[b][j] || f.hi[b][j] < n.lo[j]) { return; }
    if n.left == usize::MAX {
        for &a in &f.ids[n.start..n.end] {
            let a = a as usize;
            if a != b && (a < b || active[a] == 0) &&
                (0..3).all(|j| f.hi[a][j] >= f.lo[b][j] && f.hi[b][j] >= f.lo[a][j]) {
                out.push(((a.min(b) as u64) << 32) | a.max(b) as u64);
            }
        }
    } else {
        active_query(f,n.left,b,active,out); active_query(f,n.right,b,active,out);
    }
}

#[no_mangle]
pub unsafe extern "C" fn active_pairs(p: *const Forest, mask: *const u8, workers: usize) -> Buffer {
    let f = &*p;
    let active = slice::from_raw_parts(mask,f.ids.len());
    let ids: Vec<usize> = (0..active.len()).filter(|&i| active[i]!=0).collect();
    let chunks = (ids.len().max(1)+workers-1)/workers;
    let mut pairs = std::thread::scope(|scope| {
        let mut jobs=vec![];
        for part in ids.chunks(chunks) {
            jobs.push(scope.spawn(move || {
                let mut result=vec![];
                for &b in part {for &root in &f.roots {active_query(f,root,b,active,&mut result);}}
                result
            }));
        }
        let mut result=vec![];
        for job in jobs {result.extend(job.join().unwrap());}
        result
    });
    pairs.sort_unstable();
    let result=Buffer{data:pairs.as_mut_ptr(),len:pairs.len(),cap:pairs.capacity()};
    std::mem::forget(pairs); result
}

fn rotated(q: &Q, labels: &[u64], other: &[u64]) -> (Q,[u64;4],u8) {
    let mut q=*q; let mut ids=[labels[0],labels[1],labels[2],labels[3]];
    let mut count=0;
    if labels[2]==labels[3] {
        for _ in 0..3 {
            if !other.contains(&ids[2]) {break;}
            ids=[ids[1],ids[2],ids[0],ids[0]];
            q=[q[1],q[2],q[0],q[0]]; count+=1;
        }
    }
    (q,ids,count)
}

#[no_mangle]
pub unsafe extern "C" fn parallel_screen(q: *const f64, ids: *const u64, n: usize,
    pairs: *const u32, m: usize, output: *mut u8, workers: usize) {
    let quads=slice::from_raw_parts(q as *const Q,n);
    let labels=slice::from_raw_parts(ids,n*4);
    let pairs=slice::from_raw_parts(pairs,m*2);
    let output=slice::from_raw_parts_mut(output,m*3);
    let chunk=(m.max(1)+workers-1)/workers;
    std::thread::scope(|scope| {
        for (part,out) in output.chunks_mut(chunk*3).enumerate() {
            let start=part*chunk;
            scope.spawn(move || {
                for (offset,row) in out.chunks_mut(3).enumerate() {
                    let k=start+offset;let a=pairs[2*k] as usize;let b=pairs[2*k+1] as usize;
                    let ai=&labels[a*4..a*4+4];let bi=&labels[b*4..b*4+4];
                    let (mut c,mut w)=proof(&quads[a],&quads[b],ai,bi);
                    let mut rotation=255;
                    if c==0 {
                        let (aq,al,ar)=rotated(&quads[a],ai,bi);
                        let (bq,bl,br)=rotated(&quads[b],bi,ai);
                        (c,w)=proof(&aq,&bq,&al,&bl);rotation=ar*4+br;
                    }
                    row.copy_from_slice(&[c,w,rotation]);
                }
            });
        }
    });
}

fn point_query(f: &Forest, node: usize, p: &V, out: &mut Vec<u64>) {
    let n=&f.nodes[node];
    if (0..3).any(|j| p[j]<n.lo[j] || p[j]>n.hi[j]) {return;}
    if n.left==usize::MAX {
        for &a in &f.ids[n.start..n.end] {
            let i=a as usize;
            if (0..3).all(|j| p[j]>=f.lo[i][j] && p[j]<=f.hi[i][j]) {out.push(a as u64);}
        }
    } else {point_query(f,n.left,p,out);point_query(f,n.right,p,out);}
}

#[no_mangle]
pub unsafe extern "C" fn containing_point(f: *const Forest, p: *const f64) -> Buffer {
    let f=&*f;let p=&*(p as *const V);let mut ids=vec![];
    for &root in &f.roots {point_query(f,root,p,&mut ids);}
    ids.sort_unstable();
    let result=Buffer{data:ids.as_mut_ptr(),len:ids.len(),cap:ids.capacity()};
    std::mem::forget(ids);result
}

#[repr(C)] pub struct Bytes {pub data:*mut u8,pub len:usize,pub cap:usize}
fn decimal(mut value:u64,out:&mut Vec<u8>) {
    let mut digits=[0u8;20];let mut n=20;
    loop {n-=1;digits[n]=b'0'+(value%10) as u8;value/=10;if value==0 {break;}}
    out.extend_from_slice(&digits[n..]);
}
#[no_mangle]
pub unsafe extern "C" fn encode_rows(pairs:*const u64,codes:*const u64,n:usize,
    text:*const u8,text_len:usize,offsets:*const u64,payload_count:usize,
    separator:*const u8,separator_len:usize)->Bytes {
    let pairs=slice::from_raw_parts(pairs,n*2);let codes=slice::from_raw_parts(codes,n);
    let text=slice::from_raw_parts(text,text_len);let bounds=slice::from_raw_parts(offsets,payload_count*3);
    let separator=slice::from_raw_parts(separator,separator_len);let mut out=Vec::with_capacity(n*256);
    for i in 0..n {
        if i!=0 {out.extend_from_slice(separator);}
        let k=codes[i] as usize;let start=bounds[k*3] as usize;let mid=bounds[k*3+1] as usize;let end=bounds[k*3+2] as usize;
        out.extend_from_slice(&text[start..mid]);decimal(pairs[i*2],&mut out);out.push(b',');decimal(pairs[i*2+1],&mut out);
        out.extend_from_slice(&text[mid..end]);
    }
    let result=Bytes{data:out.as_mut_ptr(),len:out.len(),cap:out.capacity()};std::mem::forget(out);result
}
#[no_mangle]
pub unsafe extern "C" fn bytes_free(b:Bytes) {drop(Vec::from_raw_parts(b.data,b.len,b.cap));}

#[no_mangle]
pub unsafe extern "C" fn indent_json(text:*const u8,n:usize)->Bytes {
    let text=slice::from_raw_parts(text,n);let mut out=Vec::with_capacity(n*2);
    let mut depth=0usize;let mut quoted=false;let mut escaped=false;let mut i=0;
    fn line(out:&mut Vec<u8>,depth:usize) {out.push(b'\n');out.resize(out.len()+2*depth,b' ');}
    while i<n {
        let c=text[i];
        if quoted {
            out.push(c);
            if escaped {escaped=false;} else if c==b'\\' {escaped=true;} else if c==b'"' {quoted=false;}
        } else {
            match c {
                b'"'=>{quoted=true;out.push(c);}
                b'{'|b'['=>{
                    out.push(c);let end=if c==b'{' {b'}'} else {b']'};
                    if i+1<n && text[i+1]==end {out.push(end);i+=1;}
                    else {depth+=1;line(&mut out,depth);}
                }
                b'}'|b']'=>{depth-=1;line(&mut out,depth);out.push(c);}
                b','=>{out.push(c);line(&mut out,depth);}
                b':'=>out.extend_from_slice(b": "),
                _=>out.push(c),
            }
        }
        i+=1;
    }
    out.push(b'\n');let result=Bytes{data:out.as_mut_ptr(),len:out.len(),cap:out.capacity()};
    std::mem::forget(out);result
}
