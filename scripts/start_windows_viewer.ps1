param(
    [Parameter(Mandatory)][string]$Python,
    [Parameter(Mandatory)][string]$Viewer,
    [Parameter(Mandatory)][string]$Model,
    [Parameter(Mandatory)][int]$Port
)

Start-Process -FilePath $Python -ArgumentList @(
    $Viewer, "--xml-path", $Model, "--port", $Port
) -WindowStyle Hidden
