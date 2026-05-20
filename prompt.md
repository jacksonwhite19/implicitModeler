```markdown
### Implementation Task: Dyadic Canonical Interval Mapping

**Objective:** Transition from point-keyed edges to canonical dyadic integer intervals to eliminate topological gaps at resolution transitions (T-junctions).

#### 1. Dyadic Coordinate System
Implement a global integer coordinate system based on the octree's maximum depth ($D=10$).
*   **Coordinate Scaling:** Map the entire bounding box of your geometry into an integer space of $[0, 2^{10}]$. Every leaf corner and edge boundary must now be represented as an exact `u16` or `i32` coordinate.
*   **EdgeKey Refactor:** Redefine the key to include the spatial interval.
    ```rust
    // Represents a unique, canonical 1D interval along an edge
    #[derive(Hash, PartialEq, Eq)]
    struct EdgeKey {
        axis: u8,
        start: [u16; 3],
        end: [u16; 3], // Allows for specific sub-intervals at T-junctions
    }
    

```

#### 2. Canonical Interval Registration

Modify the registry pass to decompose coarse edges into fine segments:

* **Decomposition Logic:** When a leaf of depth $N$ registers its edges, it must check for overlapping edges from neighbors of depth $>N$.
* **Split Rule:** If a leaf is coarser than its neighbor, it must register its edges by splitting them into segments that match the neighbor's dyadic boundaries. This ensures the `HashMap` contains the smallest common sub-intervals shared by all adjacent cells.
* **Ownership Integrity:** Every `EdgeKey` entry must now map to exactly 4 `LeafIDs`. Any key that does not is mathematically guaranteed to be a structural crack.

#### 3. Quadrant-Based Face Emission

Refactor emission to derive the 4 owners from topology, not spatial probing:

* **Deterministic Quad Generation:** Instead of searching for owners, use the `EdgeKey`'s `start` and `end` coordinates to identify the 4 dual cells that share this edge.
* **Failure Log:** If an `EdgeKey` does not have 4 associated `LeafIDs`, log it to `output.md` as:
```markdown
### Emitter Mismatch (Structural)
- Key: {axis, start, end}
- Found Owners: [List of LeafIDs]
- Expected: 4


```



```

#### 4. Post-Emission: Engineering Audit (output.md)
Update the engineering analysis section to explicitly evaluate the topological closure:
*   **Metric:** Report the percentage of `EdgeKeys` with 4 owners.
*   **Topology Check:** After emission, if `boundary_edges > 0` before any repair pass, log the specific `EdgeKey` instances that failed to stitch.
*   **Resolution Floor Analysis:** Evaluate if the dyadic mapping eliminated the Y-truncation and resolved the thin-wall correctly.

#### 5. Verification Goal
*   Run the shell test again:
    `cargo run --release --bin benchmark_wing_export -- --script "C:\Users\Jackson\Desktop\02_Projects\09b_Implicit_CAD_claude\dual_contouring\test_scripts\rectangular_prism_shell_400x650x200_2mm.rhai"`
*   **Success Criteria:**
    1. `connected_components` == 1.
    2. Primary `boundary_edges` == 0 (without requiring patch/repair).
    3. `output.md` confirms 100% of crossing edges have 4 incident owners.

```

```

```