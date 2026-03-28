# GPU Compute Transition Plan

## Goal

Move the expensive preview and sampling work from CPU to GPU in stages without breaking the existing CPU SDF pipeline.

The key constraint is that the current modeling system evaluates dynamic Rust SDF trait objects and closures. That representation cannot run directly on the GPU. The transition therefore needs a shared intermediate representation for supported operations, with CPU fallback for anything not yet lowered.

## Success Criteria

- Section cuts and preview grids become materially faster on large models.
- GPU and CPU results match within a small tolerance for supported operations.
- Unsupported SDF nodes still render correctly through CPU fallback.
- The app remains debuggable, with the CPU path retained as ground truth.

## Current Bottlenecks

- Repeated scalar field evaluation on dense 3D grids for preview.
- Repeated scalar field evaluation on 2D section planes.
- Marching-cubes mesh extraction on CPU after field evaluation.
- Small features are especially sensitive to preview resolution and sampling density.

## Recommended Migration Strategy

Do this in four phases:

1. GPU section sampling
2. GPU 3D scalar grid evaluation
3. Hybrid preview pipeline with CPU meshing fallback
4. Full GPU meshing and long-term shared IR

This keeps risk controlled and gives useful wins early.

## Phase 1: GPU Section Sampling

### Scope

Move 2D field sampling for section cuts from CPU to GPU compute.

### Why First

- Lowest-risk entry point
- Easy to validate against current CPU outputs
- Directly improves debugging workflows
- No need to solve full GPU meshing yet

### Implementation

- Add a GPU compute path that samples an `N x M` section grid into a storage buffer.
- Start with a limited SDF subset:
  - primitives
  - transforms
  - boolean ops
  - offset
  - shell
- Read the sampled distances back to CPU for plotting and validation.
- Keep the current CPU `sample_section` path as reference.

### Deliverables

- `gpu_sample_section(...)` backend
- validation harness comparing CPU vs GPU section outputs
- debug toggle in app/CLI to switch section sampling backend

## Phase 2: GPU Scalar Grid Evaluation

### Scope

Move 3D preview-grid SDF evaluation to GPU, while keeping marching cubes on CPU.

### Why Second

- Major preview speedup
- Reuses most of the section-sampling infrastructure
- Much easier than implementing GPU marching cubes immediately

### Implementation

- Reuse the same SDF lowering path from Phase 1
- Dispatch compute over the preview voxel grid
- Fill a 3D scalar buffer or flattened storage buffer
- Read back the scalar grid to CPU
- Continue using current CPU marching cubes on that field

### Deliverables

- GPU-backed preview scalar grid generation
- tolerance comparison against CPU scalar grids
- metrics capture:
  - CPU eval time
  - GPU eval time
  - readback time

## Phase 3: Shared SDF IR and Hybrid Backend

### Scope

Introduce a proper intermediate representation for GPU-supported SDF graphs.

### Why This Matters

Right now the app is built around dynamic Rust trait-object composition. GPU compute needs:

- serializable node graphs
- fixed operation sets
- explicit parameter buffers

### Implementation

- Define a compact `SdfIr`:
  - node type
  - parameters
  - child indices
- Add a lowering pass from the current scripted SDF result into `SdfIr`
- If lowering succeeds:
  - run GPU path
- If lowering fails:
  - fall back to CPU

### Initial Supported IR Nodes

- sphere
- box
- cylinder
- torus
- cone
- plane
- union
- subtract
- intersect
- smooth union
- translate
- rotate
- scale
- offset
- shell

### Deliverables

- `SdfIr` definition
- CPU evaluator for IR for debugging parity
- GPU WGSL evaluator for IR
- fallback markers for unsupported nodes

## Phase 4: GPU Marching Cubes

### Scope

Move surface extraction to GPU after grid evaluation is stable.

### Implementation

- GPU classify pass
- prefix sum / compaction
- triangle emission pass
- upload vertex/index buffers directly to renderer

### Benefits

- eliminates costly CPU readback of full scalar grids
- enables significantly faster live preview on dense scenes

### Risks

- more complex synchronization
- harder debugging
- triangle-count management and buffer sizing

## Long-Term Architecture

The final target should be:

- one shared SDF IR
- CPU backend for correctness/reference
- GPU backend for preview/performance

That avoids maintaining two separate modeling implementations.

## Validation Plan

Every phase should include comparison against the current CPU output.

### Numerical Checks

- max absolute distance error on sampled points
- RMS error on grids
- hit/miss agreement on sign transitions

### Visual Checks

- section overlays
- small-hole preview checks
- thin-wall / countersink fidelity checks

### Performance Checks

- section generation time
- scalar grid generation time
- mesh generation time
- full rebuild latency in app

## Risks and Mitigations

### Risk: Unsupported dynamic SDFs

Mitigation:

- explicit fallback to CPU
- show backend status in debug UI

### Risk: Small-feature loss in GPU preview

Mitigation:

- preserve CPU reference path
- add feature-specific validation scenes
- support higher local preview density later

### Risk: Readback overhead erases gains

Mitigation:

- stop at Phase 2 only as an intermediate step
- move to GPU meshing in Phase 4 if preview remains readback-bound

## Recommended Order of Execution

1. Build `SdfIr` for a small supported subset
2. GPU section sampling
3. CPU/GPU comparison harness
4. GPU scalar preview grid
5. App toggle and profiling
6. Expand supported node coverage
7. GPU marching cubes

## Immediate Next Task

Implement Phase 1 only:

- add `SdfIr`
- lower basic preview scenes into it
- run section sampling on GPU
- compare against current CPU section sampling

That is the best first milestone because it is useful, testable, and low-risk.
