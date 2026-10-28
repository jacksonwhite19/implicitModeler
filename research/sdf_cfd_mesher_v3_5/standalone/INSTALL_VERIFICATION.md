# Installation on the configured host

Extract the ZIP into a new directory. Enter `Mesher_V3_5_Verification_Candidate/standalone` in Windows PowerShell. Keep its sibling `qualification` directory and parent `RELEASE.json`. No Git checkout is required. This candidate does not overwrite the original frozen release.

The numerically tested environment is Windows x64 Python 3.12, WSL distribution `Ubuntu-22.04`, Foundation OpenFOAM 13 at `/opt/openfoam13`, and the existing Linux Python/system dependencies. The included owned Windows/Linux native binaries, Gmsh/MMG libraries, source closure and finite Python payloads are preserved byte-for-byte. Windows/WSL/system Python/OpenFOAM are external prerequisites, not installed by this ZIP. On Linux the existing `bash`, `setsid`, `timeout` and `prlimit` utilities are required; `strace` is optional for requested native file tracing.

Observed Windows Python package versions:

| Package | Version |
|---|---|
| numpy | 1.26.4 |
| scipy | 1.15.3 |
| sympy | 1.14.0 |
| casadi | 3.7.2 |
| psutil | 7.2.2 |

These versions are an environment record, not a newly tested dependency upgrade or a fresh-machine installation guarantee. Use the existing configured environment. If preparing another environment, match its dependencies separately and treat numerical portability as unqualified. No dependency install, rebuild or configuration script was run for this delivery. Do not run `build_native.py` or `configure_runtime.py` as a normal extraction step: rebuilding or changing manifests creates a different runtime needing separate validation.

Check the delivered ZIP SHA256 against the delivery receipt. `PACKAGE_MANIFEST.json` records every payload member's size/hash and Git blob binding, plus derived release metadata. The manifest itself is bound by the external package receipt and ZIP hash. The release metadata records the exact local source commit and reviewed runtime identity.

Safe packaging checks, run from `standalone`:

```powershell
python -B mesh.py --help
python -B offline_audit.py --help
python -B offline_audit.py --print-verifier-identity
```

The printed verifier identity must equal `runtime_identity` in `../RELEASE.json`. The archive check independently rehashed all 1,873 runtime files after extraction. Help success is not a mesh result or a clean-machine qualification.

Use the commands in QUALIFICATION.md with fresh output directories. Keep offline audit outputs outside the standalone runtime directory; the README example uses a sibling offline-audits directory. Keep the source/fixture/runtime bytes unchanged and use `-B` to avoid bytecode caches. The runtime identity includes local fixture/source files as well as binaries. The existing CLI supports user inputs, but other geometry/settings and changed runtime bytes are outside this qualification. Output path names may change; acceptance must still close successfully.

The tested normal limits are 43,200 seconds, 16 GiB sampled Windows process-tree RSS and 16,000,000,000 logical output bytes. Linux RLIMIT_AS is per process, not aggregate Linux RSS. Offline deadline is 10,800 seconds. These are ceilings, not performance targets. Full production evidence consumes substantially more space than this source ZIP; retain it if later audit is required.
