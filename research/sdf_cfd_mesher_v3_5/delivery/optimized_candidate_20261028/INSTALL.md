# Install on the configured host

Extract the ZIP into a new directory. Keep the whole Mesher_V3_5_Optimized_Verification_Candidate folder. In PowerShell, enter its standalone directory. No Git checkout or rebuild is required. Do not install the bundled historical .deb into the system; it is a provenance artifact only.

The tested host uses Windows x64 Python 3.12 and WSL distribution Ubuntu-22.04, Linux /usr/bin/python3.10, Foundation OpenFOAM 13 at /opt/openfoam13, and pinned Linux packages including /home/jwhite/.local/lib/python3.10/site-packages. Windows numpy 1.26.4, scipy 1.15.3, sympy 1.14.0, casadi 3.7.2 and psutil 7.2.2 were observed. These are records, not installation or upgrade guarantees. Existing bash, setsid, timeout, prlimit and configured native tools are required; strace is optional for requested file tracing.

The ZIP contains all 1,885 selected standalone runtime/source/binary files, including original dependency manifests and owned native components. External system dependencies are not bundled. HOST_DEPENDENCIES.json separates the effective 1,916 Linux external pins (1,914 unchanged host paths plus the two qualified local Brotli replacements) from 168 historical owned-source names in the original manifests. In the supported normal V3.5 CLI path, those historical source names are provenance; current export uses the included local writer/contract sources and held native runtime. Legacy standalone internal native_tools entry points are not supported package commands and do not inherit this closure. Do not treat that historical name list as an extra installation requirement. The preserved build-provenance map also has 1,018 Windows external entries and 247 POSIX entries. Windows Python 3.12.2800, MSVC 14.44.35207, Windows SDK/Performance Toolkit and system DLL/header/library paths remain byte-pinned dependencies. The native Windows preparation owner checks its original build receipt, Python ABI/version and dependency files at run time even though no rebuild occurs. HOST_DEPENDENCIES.json includes that map as well as the Linux/native-tool pins. Those existing host files must remain present and byte-correct. The packaged copy does not replace the remaining manifest-bound external host files. Use only the existing configured environment. Do not run configure_runtime.py or build_native.py as an extraction step: changing manifests or binaries creates a different runtime.

## Isolated Brotli profile

V35IsolatedBrotliRuntime/1 includes exactly these regular files relative to standalone:

| File | SHA256 |
|---|---|
| v4/runtime/isolated_brotli/libbrotlicommon.so.1 | abf86ae9362dbb413c740a55b194c74adb09a7bd92d8fbd96a293c797a4a6d08 |
| v4/runtime/isolated_brotli/libbrotlidec.so.1 | db9dbda709a46c3ae124433f47c4db71167db8aaba4eafdfd99c8a3a14584463 |

They came from authenticated Ubuntu amd64 libbrotli1 1.0.9-2build6, package SHA256 8e6aa2dfd44df287b9fb227862f3b6062ec074dd39a247874edfe12e1dcc3841. See provenance/PERF9_SIGNED_METADATA.json and PERF9_PACKAGE_ARTIFACTS.json for the signed Ubuntu metadata verification and package extraction chain; the exact .deb is included. EXTERNAL_RUNTIME_PROFILE.json binds the original manifests, provenance receipt hashes, profile identity and two replacements. The MIT copyright and permission notice is included in licenses/BROTLI-COPYRIGHT.txt. Existing runtime license/source notices for the other bundled components are retained byte-for-byte.

The reviewed launcher derives the two library paths from its actual standalone root and prepends that directory to child LD_LIBRARY_PATH. It rejects nonempty LD_PRELOAD. It checks all 1,916 effective external file pins and ldd resolution before geometry under source/binary custody. The late runtime gate retains all original byte checks and requires actual common/decoder mappings at the candidate paths, with correct inode/device/hash; foreign or deleted mappings are refused. Do not set a global loader override, symlink these files elsewhere, weaken pins or replace system libraries.

The two assets can be carried at these same relative paths without changing runtime bytes. Their path derivation supports archive extraction, but no fresh numerical run was made from the extracted ZIP. The remaining absolute host dependencies prevent a portable-install claim. Shared system packages remain updated and unchanged; the strict original-path audit still sees the same two Brotli mismatches, while the explicit isolated candidate profile passed its qualified checks.

For another host, a separate narrow relocation/dependency qualification is required. This delivery does not repin paths, install dependencies or start that task. Missing byte-correct external files are a runnable-closure blocker on that host.

## Check extraction

Compare the ZIP SHA256 with its separate delivery receipt. PACKAGE_MANIFEST.json binds each payload's size and SHA256, with Git blob origins for committed source/docs and explicit derived metadata. RELEASE.json records the source/docs/runtime identities. Runtime inventory is qualification/RUNTIME_INVENTORY.json.

From standalone:

```powershell
python -B mesh.py --help
python -B offline_audit.py --help
python -B offline_audit.py --print-verifier-identity
```

The last command must print 3bdd14455b74a67e6fa1405efc945956f69bcb03925283ee1d6481b9815b82a8. These checks do not run a mesh or establish numerical portability. Use -B and keep source/fixture/binary bytes unchanged. Keep future audit output outside standalone so it cannot enter the source inventory.

Normal production limits are 43,200 seconds, 16 GiB sampled Windows process-tree RSS and 16,000,000,000 logical output bytes. Linux address-space limits are per-process; aggregate WSL RSS and physical allocation were not measured. Retained production evidence needs much more disk space than this source archive. Separate offline commands below use the tested 43,200-second ceiling.
