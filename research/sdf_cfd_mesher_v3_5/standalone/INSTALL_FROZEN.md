# Frozen candidate installation and use

## Environment used for qualification

Windows PowerShell; Windows Python 3.12 with NumPy, SciPy, SymPy, CasADi and psutil; WSL Ubuntu-22.04 with the existing Linux Python/native dependencies; Foundation OpenFOAM 13 at `/opt/openfoam13`; pinned owned Gmsh/MMG/geometry consumers included in the closure. Existing `setsid`, `timeout` and `prlimit` are used. `strace` is optional for native file tracing. The package does not install Windows, WSL, OpenFOAM or system Python libraries.

## Use the frozen files

1. Extract the delivery archive and enter `Mesher_V3_5_Standalone`, keeping its sibling `qualification` folder. Alternatively clone `https://github.com/jacksonwhite19/implicitModeler.git`, branch `freeze/mesher-v3-5-candidate-2026-10-27`, and enter `research/sdf_cfd_mesher_v3_5/standalone`. The archive's root GIT_FREEZE.json records its exact published commit.
2. Preserve the included runtime, fixture and source bytes. The Git attributes prevent newline conversion of the validated standalone files. The archive manifest and frozen inventory bind their bytes.
3. In the configured environment, run `python -B mesh.py --help` to inspect the CLI. This command does not qualify a mesh.
4. Use one of the exact commands in QUALIFICATION.md with a fresh output directory. The source package excludes results, so the recorded output names are fresh in a new extraction. Change the output name for any later repeat.
5. Require complete RESULT, export/Mesh OK, independent semantic readback and clean source/resource closure before making the corresponding engineering claim. Do not substitute saved meshes or expired live authority.

Only the configured host environment has been numerically tested. A successful extraction or help check is not clean-machine portability qualification. If a new environment requires configuration or rebuild, work in a separate copy: rebuilding/configuring owned binaries or manifests creates a different unqualified runtime identity. Do not silently modify this frozen candidate or claim the previous results transfer to that new build. No automatic installer or new verification mode was added.

Operational bounds in the recorded commands are 43,200 seconds, 16 GiB sampled Windows tree RSS and 16,000,000,000 output bytes. Linux RLIMIT_AS is per-process address space, not aggregate RSS. These are finite ceilings, not runtime targets. The sphere/blend quota is the only CASE difference from their earlier accepted numerical baselines (4 GB -> 16 GB).
