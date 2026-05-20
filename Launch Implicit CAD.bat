@echo off
setlocal
cd /d "%~dp0"
if exist "%~dp0target\debug\implicit-cad.exe" (
    start "" "%~dp0target\debug\implicit-cad.exe"
) else if exist "%~dp0target\release\implicit-cad.exe" (
    start "" "%~dp0target\release\implicit-cad.exe"
) else if exist "%~dp0target_hotfix5\release\implicit-cad.exe" (
    start "" "%~dp0target_hotfix5\release\implicit-cad.exe"
) else if exist "%~dp0target_hotfix4\release\implicit-cad.exe" (
    start "" "%~dp0target_hotfix4\release\implicit-cad.exe"
) else if exist "%~dp0target_hotfix3\release\implicit-cad.exe" (
    start "" "%~dp0target_hotfix3\release\implicit-cad.exe"
) else if exist "%~dp0target_hotfix2\release\implicit-cad.exe" (
    start "" "%~dp0target_hotfix2\release\implicit-cad.exe"
) else if exist "%~dp0target_hotfix\release\implicit-cad.exe" (
    start "" "%~dp0target_hotfix\release\implicit-cad.exe"
) else (
    start "" "%~dp0target\release\implicit-cad.exe"
)
