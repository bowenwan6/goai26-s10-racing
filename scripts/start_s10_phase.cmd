@echo off
setlocal
set "PHASE_GROUP=%~1"
if not defined PHASE_GROUP set "PHASE_GROUP=B"
for %%I in ("%~dp0..") do set "PHASE_ROOT=%%~fI"
wsl.exe -d Ubuntu-24.04 --cd "%PHASE_ROOT%" bash scripts/run_phase.sh "%PHASE_GROUP%"
endlocal
