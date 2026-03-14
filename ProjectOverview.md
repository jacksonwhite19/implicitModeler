# Implicit CAD Modeler — High Level Overview

## Project Purpose

The Implicit CAD Modeler is a programmable geometry system designed to create complex parametric models using **implicit geometry (signed distance fields)** instead of traditional boundary-representation (B-Rep) CAD.  

The system is inspired by tools such as **nTop**, where geometry is defined through **procedural operations and node graphs**, enabling workflows that are difficult or impossible in traditional CAD environments.

The project aims to provide a **lightweight, extensible, code-driven modeling platform** capable of generating complex geometry, supporting engineering workflows, and eventually offering a responsive real-time viewer.

---

## Core Philosophy

The system treats geometry as a **computable field**, not a collection of surfaces.  
Models are constructed by composing operations that transform or combine implicit bodies.

Key principles:

- **Code-first modeling** — geometry defined in scripts rather than manual CAD operations
- **Procedural workflows** — models are generated through a directed graph of operations
- **Parametric control** — parameters drive all geometry generation
- **Reproducibility** — models are deterministic and version-controlled
- **Extensibility** — new primitives and operations can be added easily

---

## Core Architecture

The system is structured around several major components.

### Geometry Kernel

The kernel evaluates **implicit fields (SDFs)** and defines the fundamental modeling operations.

Capabilities include:

- primitive implicit shapes
- boolean operations
- offsets and shelling
- transformations
- procedural geometry generation

Geometry is represented internally as **functions over space**, not meshes.

---

### Typed DAG Model Graph

Model construction is represented as a **typed directed acyclic graph (DAG)**.

Each node represents an operation such as:

- primitive creation
- transformation
- boolean operations
- modeling utilities

Edges represent data flow between operations.

Benefits:

- deterministic evaluation
- dependency tracking
- incremental recomputation
- parametric editing

---

### Execution / Evaluation Pipeline

The system evaluates the model graph to produce outputs such as:

- implicit bodies
- preview meshes
- analysis data
- exported geometry

Evaluation is designed to support:

- incremental updates
- caching
- partial recomputation
- future GPU acceleration

---

### Mesh Extraction & Export

Implicit geometry is converted into polygon meshes for visualization and export.

Supported outputs include:

- STL
- mesh previews
- downstream simulation geometry

The extraction system is designed to support **adaptive resolution and incremental updates**.

---

### Viewer System (Planned / In Progress)

A desktop viewer will provide interactive inspection of generated models.

Preview pipeline goals:

- coarse preview first
- refined mesh update afterward
- incremental mesh updates
- caching of previously computed regions
- real time viewing, comparable to a standard CAD program. Replicate the functionality of the nTop viewer

This architecture enables a **responsive viewer even for complex models**.

Future development may include hybrid preview modes combining mesh extraction with direct SDF visualization.

---

## Planned Capabilities

Planned modeling capabilities include:

- primitives and parametric shapes
- boolean operations
- transformations
- offsets and shell operations
- field operations
- procedural lattice generation
- parametric patterning
- custom modeling blocks
- support for **sketch-based geometry generation**, where spline or profile sketches can be converted into implicit geometry.

---

## Development Goals

Primary goals of the project:

1. Create a **fully programmable CAD system**
2. Support **complex procedural geometry workflows**
3. Enable **fast iteration through parametric models**
4. Provide a **responsive live preview viewer**
5. Maintain a **simple, extensible architecture**

The long-term objective is to build a system capable of **nTop-style computational design workflows** while remaining lightweight, scriptable, and developer-friendly.

---

## Long-Term Vision

The Implicit CAD Modeler aims to become a platform for:

- advanced procedural modeling
- engineering geometry generation
- optimization and simulation workflows
- computational design

By combining **implicit modeling, parametric graphs, and programmable workflows**, the system enables a fundamentally different approach to CAD compared to traditional surface-based modeling tools.

## User Interface

- minimal, two pane, similar to openSCAD 
- code on left, 3D viewer on right
- buttons/drop downs to add "blocks" - predefined code chunks like a "fuselage", "wing", "cube" ,etc.
