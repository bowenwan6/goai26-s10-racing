@echo off
setlocal
set "S10_MODEL_NAME=Turn Stop 2000 (research)"
set "S10_SSH_HOST=s10-48-golai"
set "S10_SDK_TRIAL_ROOT=/home/golai/s10-sdk-isolated-ptzjbjrx"
call "%~dp0start_s10_speedturn.cmd"
endlocal
