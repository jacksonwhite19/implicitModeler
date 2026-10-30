# Configured-host installation

Extract to a fresh directory. Keep standalone, examples, qualification, licenses, provenance, HOST_DEPENDENCIES.json, RELEASE.json and PACKAGE_MANIFEST.json together. Do not overwrite prior packages. Use a short extraction path and long-path-aware tools on Windows.

The recorded host is Windows x64 Python 3.12, WSL Ubuntu-22.04, Linux Python 3.10 and Foundation OpenFOAM 13 under /opt/openfoam13. Recorded Windows packages include numpy 1.26.4, scipy 1.15.3, sympy 1.14.0, casadi 3.7.2 and psutil 7.2.2. Linux bash, setsid, timeout and prlimit and the exact pinned external dependencies remain required. HOST_DEPENDENCIES.json is existing host/build provenance, not a portable installation recipe.

Owned Windows/Linux binaries, finite Python payloads, Gmsh/MMG and isolated Brotli assets are included in the exact 1,906-file runtime. In particular v4/runtime/isolated_brotli/libbrotlicommon.so.1 and libbrotlidec.so.1 retain their original hashes and MIT notices. The authenticated source .deb and signed-metadata/extraction provenance are included. All other required runtime license/source notices remain in their original closure. Historical modules inside that closed inventory are retained to preserve its exact identity; external campaign experiments and large outputs are excluded.

Do not run configure_runtime.py or build_native.py as extraction steps. Do not upgrade, repin or rebuild dependencies and then assume qualification still applies. The existing early host/loader and late actual-mapping checks remain authoritative. No system changes are made by packaging, and no relocated numerical run is claimed.

Verify the ZIP against the adjacent SHA256 file and the payload against PACKAGE_MANIFEST.json. qualification/RUNTIME_INVENTORY.json lists the exact runtime. From standalone, these are help/identity checks only:

```powershell
python -B mesh.py --help
python -B offline_audit.py --help
python -B offline_audit.py --print-verifier-identity
python -B compact_delivery.py --help
```

The verifier must print `97d081d262f87756109dca1ae54bdf409382922d734f32910bf9a3d93cd4196f`. Keep outputs outside standalone. Help/identity success is not numerical portability qualification. See [usage](QUICKSTART.md) and [qualification](QUALIFICATION.md).
