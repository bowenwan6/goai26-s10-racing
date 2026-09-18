@echo off
cd /d "%~dp0.."
if exist "D:\Anaconda\pythonw.exe" (
  start "" "D:\Anaconda\pythonw.exe" "%~dp0s10_control_gui.py"
) else (
  python "%~dp0s10_control_gui.py"
)
