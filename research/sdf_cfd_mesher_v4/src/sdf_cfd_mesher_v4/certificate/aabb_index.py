"""Exact inclusive AABB candidate enumeration in legacy lexicographic order.

Node bounds are extrema of input binary64 bounds. Median ordering affects only
work partitioning, never pair admission. No padded/rounded distance predicate.
"""
import numpy as np


class AABBIndex:
    def __init__(self, lo, hi, leaf_size=32):
        self.lo = np.asarray(lo)
        self.hi = np.asarray(hi)
        if self.lo.shape != self.hi.shape or self.lo.ndim != 2 or self.lo.shape[1] != 3:
            raise ValueError('InvalidAABBShape')
        if not np.isfinite(self.lo).all() or not np.isfinite(self.hi).all() or np.any(self.lo > self.hi):
            raise ValueError('InvalidAABBBounds')
        self.nodes = []
        def build(ids):
            lower = self.lo[ids].min(0); upper = self.hi[ids].max(0)
            slot = len(self.nodes); self.nodes.append(None)
            if len(ids) <= leaf_size:
                self.nodes[slot] = (lower, upper, ids, None)
            else:
                axis = int(np.argmax(upper-lower))
                order = np.argsort(self.lo[ids, axis], kind='stable')
                ids = ids[order]; mid = len(ids)//2
                left, right = build(ids[:mid]), build(ids[mid:])
                self.nodes[slot] = (lower, upper, left, right)
            return slot
        self.root = build(np.arange(len(lo))) if len(lo) else None
        # Private traversal tables derived afresh from the same tree. The
        # scalar tree remains the independent regression reference.
        self._node_lo=np.array([n[0] for n in self.nodes]).reshape(-1,3)
        self._node_hi=np.array([n[1] for n in self.nodes]).reshape(-1,3)
        self._left=np.array([n[2] if n[3] is not None else -1 for n in self.nodes],dtype=np.intp)
        self._right=np.array([n[3] if n[3] is not None else -1 for n in self.nodes],dtype=np.intp)
        leaves=[n[2] for n in self.nodes if n[3] is None]
        width=max((len(ids) for ids in leaves),default=0)
        self._leaf_ids=np.full((len(leaves),width),-1,dtype=np.intp)
        self._leaf_rows=np.full(len(self.nodes),-1,dtype=np.intp)
        row=0
        for node,n in enumerate(self.nodes):
            if n[3] is None:
                self._leaf_ids[row,:len(n[2])]=n[2];self._leaf_rows[node]=row;row+=1

    def overlaps(self, i, minimum=-1):
        found = []; stack = [] if self.root is None else [self.root]
        lo, hi = self.lo[i], self.hi[i]
        while stack:
            lower, upper, left, right = self.nodes[stack.pop()]
            if np.any(hi < lower) or np.any(upper < lo):
                continue
            if right is None:
                ids = left[left > minimum]
                mask = np.all(hi >= self.lo[ids], axis=1) & np.all(self.hi[ids] >= lo, axis=1)
                found.extend(ids[mask].tolist())
            else:
                stack.extend((right, left))
        return sorted(found)

    def after(self, i):
        return self.overlaps(i, i)

    def overlaps_batch(self, indices, after=False):
        """Same inclusive predicates, grouped by tree node, in bounded batches.

        At most 256 query IDs and4096 node/query pairs share a work block.
        Pending tree work partitions query IDs; it is not a dense all-pairs
        matrix. Sort each query's results to preserve the scalar walk's order.
        The scalar walk stays available as an independent regression control.
        """
        # Preserve valid mixed signed/unsigned integer IDs. NumPy's implicit
        # common dtype can otherwise promote those IDs to binary64.
        indices = np.asarray(indices, dtype=object)
        if (indices.ndim != 1 or len(indices) > 256
                or any(not isinstance(i, (int, np.integer))
                       or isinstance(i, (bool, np.bool_)) for i in indices)):
            raise ValueError('InvalidAABBBatch')
        if np.any(indices < 0) or np.any(indices >= len(self.lo)):
            raise ValueError('InvalidActiveAABB')
        indices = indices.astype(np.intp, copy=False)
        found = [[] for _ in indices]
        # Bound each work block to4096 node/query pairs. This is the same
        # tree traversal and inclusive comparisons, grouped across nodes to
        # avoid a separate NumPy dispatch for every visited tree node.
        stack=[] if self.root is None else [(np.full(len(indices),self.root,dtype=np.intp),np.arange(len(indices)))]
        lo,hi=self.lo[indices],self.hi[indices]
        while stack:
            nodes,queries=stack.pop()
            mask=(hi[queries]>=self._node_lo[nodes]).all(axis=1) & (self._node_hi[nodes]>=lo[queries]).all(axis=1)
            nodes,queries=nodes[mask],queries[mask]
            leaf=self._right[nodes]<0
            leaf_nodes,leaf_queries=nodes[leaf],queries[leaf]
            if len(leaf_nodes):
                # Preserve the old default's8192 query/box comparison budget.
                # Large custom leaves take one row at a time rather than a
                # new dense multi-leaf allocation. Work stays call-local.
                limit=max(1,8192//self._leaf_ids.shape[1])
                for start in range(0,len(leaf_nodes),limit):
                    leaf_block=leaf_nodes[start:start+limit]
                    query_block=leaf_queries[start:start+limit]
                    ids=self._leaf_ids[self._leaf_rows[leaf_block]]
                    hits=(ids>=0) & (hi[query_block,None]>=self.lo[ids]).all(axis=2) & (self.hi[ids]>=lo[query_block,None]).all(axis=2)
                    if after:hits &= ids>indices[query_block,None]
                    rows,columns=np.nonzero(hits)
                    for query,match in zip(query_block[rows].tolist(),ids[rows,columns].tolist()):
                        found[query].append(match)
            nodes,queries=nodes[~leaf],queries[~leaf]
            if len(nodes):
                children=np.concatenate((self._left[nodes],self._right[nodes]))
                child_queries=np.concatenate((queries,queries))
                for start in range(0,len(children),4096):
                    stack.append((children[start:start+4096],child_queries[start:start+4096]))
        return [sorted(row) for row in found]

    def pairs(self, active=None):
        for block in self.pair_blocks(active):
            for i, j in block:
                yield int(i), int(j)

    def pair_blocks(self, active=None):
        """Same independent walk and pair order, without per-pair handoff.

        Use the existing 256-query traversal. Return at most 16,384 ordered
        pairs per block, as in the qualified packed proof encoder. Partial
        active queries keep the existing compact delta sort, not old-old pairs.
        Bounds, inclusive predicates and tree construction are unchanged.
        """
        limit = 16384
        if active is None:
            block = np.empty((limit, 2), dtype=np.uint64); used = 0
            for start in range(0, len(self.lo), 256):
                indices = range(start, min(start + 256, len(self.lo)))
                for i, matches in zip(indices, self.overlaps_batch(indices, after=True)):
                    for offset in range(0, len(matches), limit):
                        values = matches[offset:offset + limit]
                        consumed = 0
                        while consumed < len(values):
                            count = min(limit - used, len(values) - consumed)
                            block[used:used + count, 0] = i
                            block[used:used + count, 1] = values[consumed:consumed + count]
                            used += count; consumed += count
                            if used == limit:
                                yield block
                                block = np.empty((limit, 2), dtype=np.uint64); used = 0
            if used:
                yield block[:used]
            return
        active = set(active)
        if any(i < 0 or i >= len(self.lo) for i in active):
            raise ValueError('InvalidActiveAABB')
        # Query only changed boxes, against the complete historical tree.
        # Canonical pairs preserve the legacy exact-kernel order. The compact
        # sort holds only required delta pairs, never historical old-old pairs.
        from array import array
        pending = array('Q')
        ordered = sorted(active)
        for start in range(0, len(ordered), 256):
            indices = ordered[start:start + 256]
            for i, matches in zip(indices, self.overlaps_batch(indices)):
                # Keep the same active/contact filter before vectorizing only
                # integer canonicalization and buffer copying. No new predicate.
                selected = [j for j in matches if i != j and not (j in active and j < i)]
                for offset in range(0, len(selected), limit):
                    values = np.asarray(selected[offset:offset + limit], dtype=np.uint64)
                    block = np.empty((len(values), 2), dtype=np.uint64)
                    block[:, 0] = np.minimum(np.uint64(i), values)
                    block[:, 1] = np.maximum(np.uint64(i), values)
                    pending.frombytes(block.tobytes())
        pairs = np.frombuffer(pending, dtype=np.uint64).reshape(-1, 2)
        order = np.lexsort((pairs[:, 1], pairs[:, 0]))
        for start in range(0, len(order), limit):
            yield pairs[order[start:start + limit]]
