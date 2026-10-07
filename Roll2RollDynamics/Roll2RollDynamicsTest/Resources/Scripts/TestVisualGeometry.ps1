$ErrorActionPreference = "Stop"

$omcCommand = Get-Command omc -ErrorAction SilentlyContinue
$isWindowsPlatform = $env:OS -eq "Windows_NT"
if ($null -ne $omcCommand) {
    $omc = $omcCommand.Source
} elseif ($env:OPENMODELICAHOME) {
    $executable = if ($isWindowsPlatform) { "omc.exe" } else { "omc" }
    $omc = Join-Path (Join-Path $env:OPENMODELICAHOME "bin") $executable
} elseif ($isWindowsPlatform) {
    $pattern = Join-Path $env:ProgramFiles "OpenModelica*\bin\omc.exe"
    $omc = Get-ChildItem $pattern |
        Sort-Object FullName -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $omc -or -not (Test-Path -LiteralPath $omc)) {
    throw "OpenModelica compiler not found"
}

$repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$probePath = Join-Path $PSScriptRoot "VisualGeometryProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "Visual geometry probe exited with code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$log = $output -join "`n"
$successCount = ([regex]::Matches(
    $log, "The simulation finished successfully")).Count
if ($successCount -ne 8) {
    throw "Expected 8 successful visual geometry checks: $log"
}
if ($log -match "LOG_ASSERT\s+\|\s+error|assertion has been violated") {
    throw "Visual geometry assertion failed: $log"
}

Write-Output "Eight visual geometry checks passed."
