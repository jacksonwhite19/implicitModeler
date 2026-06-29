param(
    [Parameter(Mandatory = $true)]
    [string]$InputStl,

    [Parameter(Mandatory = $true)]
    [string]$RunDir,

    [string]$Python = "..\su2_sandbox\.venv\Scripts\python.exe",
    [string]$PvPython = "C:\Program Files\ParaView 5.10.1-Windows-Python3.9-msvc2017-AMD64\bin\pvpython.exe",
    [string]$BaseCells = "56,36,56",
    [int]$SurfaceMinLevel = 2,
    [int]$SurfaceMaxLevel = 3,
    [int]$FeatureLevel = 2,
    [int]$NCellsBetweenLevels = 4,
    [double]$SnapTolerance = 0.5,
    [int]$NSmoothPatch = 10
)

$ErrorActionPreference = "Stop"

$ScriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$ExperimentRoot = Resolve-Path (Join-Path $ScriptRoot "..")
$RunPath = Join-Path $ExperimentRoot $RunDir
$CasePath = Join-Path $RunPath "openfoam_case"
$SurfaceStl = Join-Path $RunPath "aircraft_surface.stl"
$SurfaceReport = Join-Path $RunPath "surface_report.json"
$IsoPng = Join-Path $RunPath "aircraft_iso.png"

New-Item -ItemType Directory -Force -Path $RunPath | Out-Null

& $Python (Join-Path $ScriptRoot "cfd_surface_mesher.py") `
    --input-stl $InputStl `
    --output-stl $SurfaceStl `
    --report $SurfaceReport

& $Python (Join-Path $ScriptRoot "make_openfoam_case.py") `
    --input-stl $SurfaceStl `
    --case-dir $CasePath `
    --base-cells $BaseCells `
    --surface-min-level $SurfaceMinLevel `
    --surface-max-level $SurfaceMaxLevel `
    --feature-level $FeatureLevel `
    --n-cells-between-levels $NCellsBetweenLevels `
    --snap-tolerance $SnapTolerance `
    --n-smooth-patch $NSmoothPatch

$CasePathLinux = (wsl wslpath -a "$CasePath").Trim()
$FoamCommand = @"
source /opt/openfoam13/etc/bashrc
cd '$CasePathLinux'
surfaceCheck constant/geometry/aircraft.stl > log.surfaceCheck 2>&1 &&
blockMesh > log.blockMesh 2>&1 &&
surfaceFeatures > log.surfaceFeatures 2>&1 &&
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1 &&
checkMesh > log.checkMesh 2>&1 &&
foamToVTK -constant -noInternal > log.foamToVTK 2>&1
"@

wsl bash -lc $FoamCommand

& $PvPython (Join-Path $ScriptRoot "render_aircraft_iso_screenshot.py") `
    --aircraft-vtk (Join-Path $CasePath "VTK\aircraft\aircraft_0.vtk") `
    --output-png $IsoPng `
    --representation "Surface With Edges"

& $Python (Join-Path $ScriptRoot "summarize_openfoam_run.py") `
    --run-dir $RunPath
