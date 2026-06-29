# Analysis Modules

Purpose: replaceable analysis adapters used by the optimizer evaluation pipeline.

Initial module families:

- Geometry validation.
- Geometry metrics.
- Mass/CG estimate.
- Low-fidelity aero.
- Medium-fidelity aero.
- Selective high-fidelity CFD.
- Scoring.

Future module families:

- Structures.
- Propulsion.
- Mission simulation.
- Controls.
- Thermal systems.
- Surrogate inference.

Current contracts:

- `cfd_aerodynamic_conventions.md`
- `cfd_su2_adapter_contract.md`
- `cfd_openfoam_adapter_contract.md`

Status: contracts started; no CFD solver is called automatically by the optimizer skeleton yet. OpenFOAM/Gmsh is the current preferred full-aircraft sandbox path for gated tetrahedral smoke tests. Force coefficients are solver-plumbing evidence until the scoring CFD policy is finalized.
