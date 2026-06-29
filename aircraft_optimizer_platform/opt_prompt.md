# Aircraft Optimizer Platform Prompt

Act as lead software architect for a new aircraft optimization platform. Do not write production code yet. This is an architecture, requirements, and roadmap exercise.

Goal: design the foundation of a long-term aircraft optimization ecosystem focused on architecture quality, extensibility, traceability, and future MDAO integration rather than immediate implementation.

Project vision:

- Fully autonomous aircraft design and optimization environment for small unmanned aircraft.
- Generate aircraft variants.
- Evaluate aircraft performance.
- Run multiple levels of analysis.
- Perform optimization studies.
- Manage large candidate populations.
- Track optimization history.
- Explain optimization decisions.
- Build surrogate models from accumulated data.
- Support future MDAO workflows.
- Support future VTOL and tailsitter aircraft.
- Produce engineering reports and visualizations.
- Not intended for manned aircraft.

Initial focus:

- Small UAVs.
- Fixed-wing aircraft.
- Loiter/endurance optimization.

Future expansion:

- VTOL.
- Tailsitters.
- Multi-mission optimization.
- Structural optimization.
- Mission simulation.
- Surrogate-model-assisted optimization.

Important constraint:

- A separate implicit CAD/SDF aircraft modeling system already exists.
- Assume aircraft geometry can be generated from a parameterized aircraft definition.
- Do not redesign the CAD system.
- Assume geometry generation exists and the optimizer consumes it.

Design philosophy:

- Support MDAO-style workflows from the beginning.
- Support multiple analysis fidelities.
- Support replaceable analysis modules.
- Preserve complete run traceability.
- Support large optimization campaigns.
- Support future machine learning integration.
- Avoid tight coupling between geometry, CFD, scoring, optimization, and logging.
- All major subsystems should be replaceable.

Initial optimization scope:

- Continuous parameters only.
- Examples: wing span, sweep, taper, twist, root chord, tip chord, thickness ratios, wing position; fuselage length, diameter, scaling parameters, cross-section parameters; inlet size, position, and lip geometry parameters.
- No topology changes initially.
- Do not initially allow adding/removing wings, adding/removing fins, or changing aircraft family.
- Future versions may support topology and family changes.

Initial analysis philosophy:

- Use escalating fidelity: geometry validation, geometry metrics, mass/CG estimate, low fidelity aero, medium fidelity aero, high fidelity CFD, scoring.
- Avoid expensive CFD whenever possible.

Open source requirement:

- Recommended analysis tools should be free or open source.
- Potential examples: OpenFOAM, AVL, SU2, XFOIL, OpenVSP/VSPAERO, OpenMDAO, pymoo, Optuna, Nevergrad, Gmsh, PyVista, VTK.

Dashboard requirements:

- Prefer a standalone local web application.
- Do not assume integration into the CAD application.
- Eventually provide run monitoring, candidate browsing, ranking, convergence plots, Pareto front visualization, candidate comparison, screenshot galleries, failure analysis, and run history.

Screenshot requirements:

- Every candidate should generate standardized visual artifacts: top, side, front, and isometric views.
- Future artifacts include internal layout views and CFD visualizations.
- Architecture should support optimization timelapse videos, best-candidate evolution videos, and convergence galleries.

Traceability requirements:

- Every result must be reproducible.
- Preserve design variables, constraints, software versions, scoring versions, geometry versions, analysis versions, and optimization settings for every candidate and run.

Candidate lineage:

- Support ancestry tracking.
- Example: Candidate 108, parent Candidate 42, mutations sweep +3 deg and taper -0.05, reason optimizer exploration.
- Make lineage queryable later.

Failure taxonomy:

- Classify geometry, meshing, CFD, optimization, and other failures.

Analysis confidence:

- Results should carry fidelity metadata: value, source, confidence, runtime cost.
- Support future multi-fidelity workflows.

Human-in-the-loop:

- Users should be able to mark candidates as promising, investigate, rerun CFD, visually interesting, suspicious result, and similar.

Future machine learning support:

- Preserve data needed for future surrogate models, such as drag prediction, stability prediction, CFD outcome prediction, mesh failure prediction, and optimization acceleration.
- Do not design a machine learning system yet.
- Design data structures and logging strategies that maximize future ML usefulness.

Future MDAO support:

- Eventually support aerodynamics, structural analysis, mass properties, propulsion, mission simulation, controls, and thermal systems without requiring major redesign.

Requested deliverables:

1. Overall system architecture
2. Major subsystems
3. Data model recommendations
4. Run database architecture
5. Candidate lifecycle
6. Analysis module architecture
7. Optimization architecture
8. Dashboard architecture
9. Logging architecture
10. Artifact management architecture
11. Traceability/versioning strategy
12. Candidate lineage system
13. Failure taxonomy proposal
14. Future surrogate-model readiness
15. Future MDAO readiness
16. Recommended technology stack
17. Recommended development phases
18. Risks and architectural pitfalls
19. Proposed v1 MVP scope
20. Proposed roadmap from MVP to full autonomous aircraft optimizer

Also create a new folder for this work, keep it clean and minimal, and create `opt_output.md` and `opt_prompt.md` in that folder.
