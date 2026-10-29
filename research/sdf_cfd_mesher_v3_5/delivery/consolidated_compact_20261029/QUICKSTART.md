# Run from the extracted package

First complete [installation checks](INSTALL.md). In Windows PowerShell, enter the package's standalone directory. Use an absolute output path with an existing parent. Every output name must be fresh; keep prior outputs and failures.

Prepare common sphere arguments. This is the exact accepted CASE and the original physical settings, not a new geometry-specific mesher path.

```powershell
$runtimePath = (Get-Location).Path
$packagePath = Split-Path -Parent $runtimePath
$runParent = Join-Path $packagePath 'runs'
New-Item -ItemType Directory -Path $runParent -Force | Out-Null
$meshArguments = @(
  '--case', (Join-Path $packagePath 'examples/sphere_CASE.json'),
  '--geometry', (Join-Path $runtimePath 'fixtures/sphere.rhai'),
  '--metres-per-model-unit', '0.001',
  '--prism-first-height', '0.00002', '--prism-growth', '1.3', '--prism-layers', '5',
  '--surface-size', '0.0002', '--surface-policy', 'size-led', '--surface-carrier', 'adaptive',
  '--background', (Join-Path $runtimePath 'fixtures/background_direct_v4.json'),
  '--stop-after', 'complete', '--timeout-seconds', '43200',
  '--memory-limit-gib', '16', '--output-limit-bytes', '16000000000'
)
```

For normal production with four workers and full retention:

```powershell
$fullOutput = Join-Path $runParent 'sphere-full-01'
if (Test-Path -LiteralPath $fullOutput) { throw 'Choose a fresh output name' }
python -B mesh.py @meshArguments --output $fullOutput --verification production --source-workers 4 --retention full-evidence
if ($LASTEXITCODE -ne 0) { throw 'Run failed; preserve diagnostics' }
```

Keep the entire accepted full bundle and its compatible verifier for later offline audit. Immediately capture the accepted VERIFICATION.json SHA256 outside that bundle after successful source/supervisor closure; do not establish trust by rehashing unexplained modified evidence. Production alone records separate audit not_run. Omitting --source-workers keeps serial, and omitting --verification keeps audit. Use a different fresh output for each command. These examples are help/source checked; packaging does not rerun numerical campaigns.

For the practical compact production path with immediate audit and owned cleanup:

```powershell
$compactOutput = Join-Path $runParent 'sphere-compact-01'
if (Test-Path -LiteralPath $compactOutput) { throw 'Choose a fresh output name' }
python -B mesh.py @meshArguments --output $compactOutput --verification production --source-workers 4 --retention compact-delivery --immediate-audit --cleanup-scratch
if ($LASTEXITCODE -ne 0) { throw 'Run failed; preserve diagnostics and staging' }
$status = Get-Content -LiteralPath ($compactOutput + '.retention.json') -Raw | ConvertFrom-Json
if ($status.status -ne 'delivery_complete' -or $status.scientific_status -ne 'accepted' -or $status.separate_audit -ne 'passed' -or $status.compact_publication -ne 'verified' -or $status.cleanup.status -ne 'cleanup_complete') { throw 'Requested lifecycle did not complete' }
$compactRoot = $status.delivery.manifest_sha256
python -B compact_delivery.py check --bundle $compactOutput --expected-root $compactRoot
if ($LASTEXITCODE -ne 0) { throw 'Compact integrity check failed' }
```

Capture $compactRoot from this successful invocation in a separate trusted record for future integrity checks. The checker authenticates population/bytes; it does not rerun science. Full offline audit rejects compact input. --immediate-audit is optional and supports compact production only; omitting it skips separate replay in production. --cleanup-scratch is independently optional and supports compact mode only. Omitting cleanup leaves full staging. Audit verification itself includes replay, so do not add --immediate-audit to an audit-mode command.

Temporary staging is a fresh .v35-compact-scratch-* sibling of the destination. The sibling .retention.json reports actual outcomes; .cleanup-authorization.json binds the pre-delete producer/audit/delivery roots and process closure. No historical folder can be passed as owned scratch. Do not remove these small receipts to pretend cleanup passed.

Existing accepted full bundles can be exported without new meshing or automatic deletion:

```powershell
python -B compact_delivery.py export --bundle C:\MesherRuns\accepted-full --expected-root TRUSTED_VERIFICATION_SHA256 --output C:\MesherRuns\fresh-compact
```

The paths and hash above are placeholders for your own existing accepted bundle, captured trusted root and fresh destination. A supplied audit can be attached with --audit-result and --expected-audit-root. Use compact_delivery.py export --help for the exact interface. Export of historical input never deletes it.

Generic Rhai/SDF input remains supported by the same workflow. The sample sphere uses adaptive construction. Historical blend used adaptive and passage used nodal; do not treat other carrier/settings combinations or generic inputs as qualified by the sphere run. Keep [warnings and limits](QUALIFICATION.md) with outputs.
