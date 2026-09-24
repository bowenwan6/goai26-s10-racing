$ErrorActionPreference = 'Stop'
$repoPath = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$pythonPath = Join-Path $repoPath '.venv/Scripts/python.exe'
$dataPath = Join-Path $env:LOCALAPPDATA 'S10GaitCapture/demo'
if (-not (Test-Path -LiteralPath $pythonPath)) { throw 'Create the project Python environment and install requirements.txt first.' }
New-Item -ItemType Directory -Path $dataPath -Force | Out-Null
$existing = Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -like '*s10_gait_capture*server.py*' }
if (-not $existing) {
    $serverPath = Join-Path $PSScriptRoot 'server.py'
    Start-Process -FilePath $pythonPath -ArgumentList @("`"$serverPath`"", '--demo', '--output', "`"$dataPath`"") -WindowStyle Hidden -RedirectStandardOutput (Join-Path $dataPath 'server.out.log') -RedirectStandardError (Join-Path $dataPath 'server.err.log') | Out-Null
}
Write-Host 'Page: http://127.0.0.1:8090'
Write-Host "Access token file: $dataPath\.access-token"
Write-Host 'Synthetic demo only. Robot deployment uses ROS mode.'
