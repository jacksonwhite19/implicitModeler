# Current-profile CLI usage

Read [installation limits](INSTALL.md) and [qualification](QUALIFICATION.md). These commands document existing CLI support. They were not numerically executed from the extracted package during publication. The default example is the current-qualified physical close-spheres configuration; extraction/relocation remains unqualified.

In PowerShell, enter the extracted standalone directory. Select a fresh external output:

```powershell
$packagePath = Split-Path -Parent (Get-Location).Path
$outputPath = Join-Path $packagePath 'runs/close-spheres-01/solver'
if (Test-Path -LiteralPath $outputPath) { throw 'Choose a fresh output' }
$meshArguments = @(
  '--case', (Join-Path $packagePath 'examples/close_spheres_CASE.json'),
  '--geometry', (Join-Path $packagePath 'examples/close_spheres.rhai'),
  '--metres-per-model-unit', '0.001',
  '--prism-first-height', '0.00002', '--prism-growth', '1.3', '--prism-layers', '5',
  '--surface-size', '0.0002', '--surface-policy', 'size-led', '--surface-carrier', 'adaptive',
  '--background', (Join-Path (Get-Location).Path 'fixtures/background_direct_v4.json'),
  '--output', $outputPath, '--stop-after', 'complete',
  '--verification', 'production', '--source-workers', '8',
  '--retention', 'compact-delivery', '--immediate-audit', '--cleanup-scratch',
  '--timeout-seconds', '43200', '--memory-limit-gib', '16', '--output-limit-bytes', '16000000000'
)
python -B mesh.py @meshArguments
if ($LASTEXITCODE -ne 0) { throw 'Preserve failure diagnostics and staging' }
$status = Get-Content -LiteralPath ($outputPath + '.retention.json') -Raw | ConvertFrom-Json
if ($status.status -ne 'delivery_complete' -or $status.separate_audit -ne 'passed' -or $status.compact_publication -ne 'verified' -or $status.cleanup.status -ne 'cleanup_complete') { throw 'Requested lifecycle incomplete' }
```

Explicit eight-worker production needs the stock CPU-affinity check; serial remains the default. Existing stock limits are 43,200 seconds, 16 GiB sampled Windows tree RSS, Linux RLIMIT_AS per process, 16,000,000,000 logical output bytes and a 1,000,000,000-byte free-space floor. Windows samples, Linux per-process address space and disk accounting are different quantities. Temporary evidence and compact output coexist before cleanup.

Compact cleanup removes only the new invocation-owned successful staging after actual audit and verified delivery. Failures preserve diagnostics. Compact files are solver data and completed-check records, not a full replay bundle. For later full offline replay, retain a full-evidence run, its trusted source root and compatible verifier; put offline outputs outside standalone. Consult `python -B offline_audit.py --help` for that separate workflow. A saved owner JSON is not a live cleanup capability.

Ellipsoid/torus templates are included for explicit further work, but their full CLI qualification belongs to the earlier runtime. No separate transition-stage generator or geometry-family dispatch is selected by these example files.
