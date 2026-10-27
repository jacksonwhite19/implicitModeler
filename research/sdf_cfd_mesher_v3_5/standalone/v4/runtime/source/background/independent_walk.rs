//! Execution-only translation of the existing independent lower-ID AABB walk.
//! The caller supplies the existing Python tree, never the producer's forest.
//! Finite binary64 inclusive comparisons; no distance or rounded predicate.
use std::{ptr, slice};

#[repr(C)]
pub struct Buffer { data: *mut u64, length: usize, capacity: usize }
struct Node { low: [f64;3], high: [f64;3], children: Option<(usize,usize)>, ids: Vec<usize>, active: bool, maximum: usize }
struct Walk { lo: Vec<[f64;3]>, hi: Vec<[f64;3]>, nodes: Vec<Node>, active: Vec<u8>, face: usize, hits: Vec<usize>, cursor: usize, failed: bool }
fn overlap(a: &[f64;3], b: &[f64;3], c: &[f64;3], d: &[f64;3]) -> bool {
    b[0]>=c[0] && b[1]>=c[1] && b[2]>=c[2] && d[0]>=a[0] && d[1]>=a[1] && d[2]>=a[2]
}
fn valid(a: &[f64;3],b: &[f64;3]) -> bool {(0..3).all(|k|a[k].is_finite()&&b[k].is_finite()&&a[k]<=b[k])}

#[no_mangle]
pub unsafe extern "C" fn walk_new(lo: *const [f64;3], hi: *const [f64;3], count: usize,
    low: *const [f64;3], high: *const [f64;3], descriptors: *const [i64;4], node_count: usize,
    leaves: *const u64, leaf_count: usize, active: *const u8) -> *mut Walk {
    std::panic::catch_unwind(||{
        if count==0 || node_count==0 {return ptr::null_mut();}
        let (lo,hi)=(slice::from_raw_parts(lo,count),slice::from_raw_parts(hi,count));
        if !lo.iter().zip(hi).all(|(a,b)|valid(a,b)){return ptr::null_mut();}
        let (lower,upper)=(slice::from_raw_parts(low,node_count),slice::from_raw_parts(high,node_count));
        let desc=slice::from_raw_parts(descriptors,node_count);let ids=slice::from_raw_parts(leaves,leaf_count);
        let flags=slice::from_raw_parts(active,count);
        if flags.iter().any(|&v|v>1) || ids.iter().any(|&v|v>=count as u64){return ptr::null_mut();}
        let mut nodes=Vec::with_capacity(node_count);let mut seen=vec![false;count];let mut parents=vec![0usize;node_count];
        for i in 0..node_count {
            if !valid(&lower[i],&upper[i]){return ptr::null_mut();}
            let row=desc[i];
            let (children,values)=if row[1]<0 {
                if row[2]<0 || row[3]<=0{return ptr::null_mut();}
                let (start,len)=(row[2] as usize,row[3] as usize);
                if start.checked_add(len).map_or(true,|end|end>leaf_count){return ptr::null_mut();}
                for &id in &ids[start..start+len]{
                    let id=id as usize;
                    if seen[id] || (0..3).any(|k|lo[id][k]<lower[i][k] || hi[id][k]>upper[i][k]){return ptr::null_mut();}
                    seen[id]=true;
                }
                (None,ids[start..start+len].iter().map(|&v|v as usize).collect())
            } else {
                if row[0]<=i as i64 || row[1]<=i as i64 || row[0]>=node_count as i64 || row[1]>=node_count as i64{return ptr::null_mut();}
                for child in [row[0] as usize,row[1] as usize]{
                    parents[child]+=1;
                    if parents[child]!=1 || (0..3).any(|k|lower[child][k]<lower[i][k] || upper[child][k]>upper[i][k]){return ptr::null_mut();}
                }
                (Some((row[0] as usize,row[1] as usize)),Vec::new())
            };
            nodes.push(Node{low:lower[i],high:upper[i],children,ids:values,active:false,maximum:0});
        }
        if seen.iter().any(|&v|!v) || parents[0]!=0 || parents[1..].iter().any(|&v|v!=1){return ptr::null_mut();}
        for i in (0..node_count).rev(){
            let (flag,max)=if let Some((a,b))=nodes[i].children{
                (nodes[a].active||nodes[b].active,nodes[a].maximum.max(nodes[b].maximum))
            }else{(nodes[i].ids.iter().any(|&v|flags[v]!=0),*nodes[i].ids.iter().max().unwrap())};
            nodes[i].active=flag;nodes[i].maximum=max;
        }
        Box::into_raw(Box::new(Walk{lo:lo.to_vec(),hi:hi.to_vec(),nodes,active:flags.to_vec(),face:0,hits:Vec::new(),cursor:0,failed:false}))
    }).unwrap_or(ptr::null_mut())
}

impl Walk {
    fn query(&mut self,i:usize){
        self.hits.clear();self.cursor=0;let mut stack=vec![0];let own=self.active[i]!=0;
        while let Some(slot)=stack.pop(){
            let node=&self.nodes[slot];
            if node.maximum<=i || (!own&&!node.active) || !overlap(&self.lo[i],&self.hi[i],&node.low,&node.high){continue;}
            if let Some((left,right))=node.children {stack.push(right);stack.push(left);}else{
                for &j in &node.ids{
                    if j>i && (own||self.active[j]!=0) && overlap(&self.lo[i],&self.hi[i],&self.lo[j],&self.hi[j]){self.hits.push(j);}
                }
            }
        }
        self.hits.sort_unstable();
    }
    fn block(&mut self,limit:usize)->Vec<u64>{
        let mut out=Vec::with_capacity(2*limit);
        while out.len()/2<limit{
            if self.cursor==self.hits.len(){
                if self.face==self.lo.len(){break;}
                let i=self.face;self.face+=1;self.query(i);
                if self.hits.is_empty(){continue;}
            }
            let count=(limit-out.len()/2).min(self.hits.len()-self.cursor);
            for &j in &self.hits[self.cursor..self.cursor+count]{out.push((self.face-1) as u64);out.push(j as u64);}
            self.cursor+=count;
        }
        out
    }
}
#[no_mangle]
pub unsafe extern "C" fn walk_next(handle:*mut Walk,limit:usize)->Buffer{
    let empty=Buffer{data:ptr::null_mut(),length:0,capacity:0};
    if handle.is_null(){return empty;}
    let walk=&mut *handle;
    if limit==0 || limit>16384{walk.failed=true;return empty;}
    if walk.failed{return empty;}
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(||walk.block(limit))){
        Ok(mut rows)=>{let b=Buffer{data:rows.as_mut_ptr(),length:rows.len(),capacity:rows.capacity()};std::mem::forget(rows);b},
        Err(_)=>{walk.failed=true;empty}
    }
}
#[no_mangle]
pub unsafe extern "C" fn walk_failed(handle:*mut Walk)->u8{if handle.is_null(){1}else{(*handle).failed as u8}}
#[no_mangle]
pub unsafe extern "C" fn walk_free(handle:*mut Walk){if !handle.is_null(){drop(Box::from_raw(handle));}}
#[no_mangle]
pub unsafe extern "C" fn walk_buffer_free(b:Buffer){if b.capacity!=0{drop(Vec::from_raw_parts(b.data,b.length,b.capacity));}}
