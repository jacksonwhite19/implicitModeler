# Install on the configured host

Extract the ZIP into a new directory. Keep standalone, examples, qualification, licenses, provenance, HOST_DEPENDENCIES.json, RELEASE.json and PACKAGE_MANIFEST.json together. No Git checkout is needed. Do not overwrite prior packages or a frozen release.

The tested host is Windows x64 Python 3.12 with WSL Ubuntu-22.04, Linux Python 3.10 and Foundation OpenFOAM 13 at /opt/openfoam13. The source closure includes owned Windows/Linux native binaries, Gmsh/MMG and finite Python payloads. Windows, WSL, system Python/OpenFOAM, existing MSVC/SDK/compiler-header dependencies and utilities remain external. HOST_DEPENDENCIES.json distinguishes exact runtime pins from historical build/source names. This is a configured-host package, not a clean-machine installer or numerical portability guarantee.

Observed Windows Python packages: numpy 1.26.4, scipy 1.15.3, sympy 1.14.0, casadi 3.7.2, psutil 7.2.2. This is an environment record, not a dependency-upgrade recommendation. Existing Linux bash, setsid, timeout and prlimit are required; strace is needed only for requested file tracing. Do not install, rebuild or run configure_runtime.py/build_native.py as a normal extraction step: changed bytes/manifests need new qualification.

The unchanged V35IsolatedBrotliRuntime/1 profile carries these two candidate-relative original assets:

| File | SHA256 |
|---|---|
| standalone/v4/runtime/isolated_brotli/libbrotlicommon.so.1 | abf86ae9362dbb413c740a55b194c74adb09a7bd92d8fbd96a293c797a4a6d08 |
| standalone/v4/runtime/isolated_brotli/libbrotlidec.so.1 | db9dbda709a46c3ae124433f47c4db71167db8aaba4eafdfd99c8a3a14584463 |

They originate in authenticated Ubuntu amd64 libbrotli1 1.0.9-2build6, package SHA256 8e6aa2dfd44df287b9fb227862f3b6062ec074dd39a247874edfe12e1dcc3841. The exact .deb, signed metadata/extraction chain and independent provenance reviews are in provenance. The MIT notice is in licenses/BROTLI-COPYRIGHT.txt. Other included runtime license/source notices remain exact.

The launcher derives the isolated library directory from the extracted standalone root, prepends it only for its child processes, rejects LD_PRELOAD, and retains early 1,916-file/loader and late actual-mapping checks. Original manifests/system packages stay unchanged. Missing or changed host dependencies refuse; there is no repin or install workaround in this package. A different host needs separate dependency/relocation qualification. Help/extraction checks do not prove a numerical run from the extracted path.

Check the ZIP SHA256 against the separate delivery receipt. PACKAGE_MANIFEST.json binds every payload member's size/SHA256 and exact local Git blob origin; RELEASE.json and the manifest are explicit derived metadata. qualification/RUNTIME_INVENTORY.json binds all 1,895 reviewed runtime files. From standalone, run:

```powershell
python -B mesh.py --help
python -B offline_audit.py --help
python -B offline_audit.py --print-verifier-identity
python -B compact_delivery.py --help
```

The verifier must print efb724a6a70273999f3e36e5873269984781de99b5eb7f306278f4001bf4d3ac. Keep source/fixture/binary bytes unchanged and use -B to avoid bytecode caches. Keep new outputs outside standalone, as the quickstart does. These are package checks, not new scientific tests.

Documented operating ceilings are 43,200 s, 16 GiB sampled Windows tree RSS, Linux RLIMIT_AS per process and 16,000,000,000 logical output B, with a 1 GB free-space floor in compact mode. They are ceilings, not performance promises. Aggregate Linux RSS, physical allocation and physical reclaimed space are unmeasured. Full temporary evidence and compact delivery coexist until optional success cleanup, so enough staging capacity is still required. Hard death of the external controller can leave detached WSL work; ordinary failure diagnostics and owned closure records must be inspected.
