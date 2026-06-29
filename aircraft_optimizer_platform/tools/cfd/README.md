# CFD Tool Environment

Date created: 2026-06-20

## Purpose

This folder tracks external CFD/mesh tooling used by the aircraft optimizer platform.

Do not vendor CFD binaries into the repository. Install them into a user-local tool environment and record exact commands, versions, and verification status here.

## Current Installed Environment

Host environment:

```text
WSL2 Ubuntu 22.04
```

User-local micromamba:

```text
~/.local/aircraft_optimizer_platform/micromamba/bin/micromamba
```

Environment root:

```text
~/.local/aircraft_optimizer_platform/mamba_root/envs/aop-cfd
```

Installed packages:

```text
python=3.11
su2=8.5.0
gmsh
meshio
```

Verified commands:

```text
SU2_CFD --help
gmsh -version
meshio --help
```

Observed versions:

```text
SU2 v8.5.0 "Harrier"
Gmsh 4.15.2
Micromamba 2.8.1
```

## Recreate Environment

From WSL:

```bash
bash /mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/aircraft_optimizer_platform/tools/cfd/bootstrap_wsl_aop_cfd.sh
```

## Run Commands

From PowerShell:

```powershell
wsl bash -lc "export MAMBA_ROOT_PREFIX=~/.local/aircraft_optimizer_platform/mamba_root; ~/.local/aircraft_optimizer_platform/micromamba/bin/micromamba run -n aop-cfd SU2_CFD --help"
```

From WSL:

```bash
export MAMBA_ROOT_PREFIX=~/.local/aircraft_optimizer_platform/mamba_root
~/.local/aircraft_optimizer_platform/micromamba/bin/micromamba run -n aop-cfd SU2_CFD --help
```

## Notes

- WSL apt has OpenFOAM/Gmsh packages available, but non-interactive sudo is not available on this machine.
- Windows winget found `blueCFD-Core`, but native Windows CFD is not the preferred platform path for this project.
- SU2 was not available on conda-forge win-64 in the tested search, but it is available on conda-forge linux-64.
- Docker CLI exists but is old/unstable in this environment; WSL user-local micromamba is cleaner for now.
- This environment is installed only for future CFD adapter development. The optimizer skeleton still does not call CFD.
