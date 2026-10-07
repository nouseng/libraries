$ErrorActionPreference = "Stop"

$omcCommand = Get-Command omc -ErrorAction SilentlyContinue
$isWindowsPlatform = $env:OS -eq "Windows_NT"
if ($null -ne $omcCommand) {
    $omc = $omcCommand.Source
} elseif ($env:OPENMODELICAHOME) {
    $omcExecutable = if ($isWindowsPlatform) { "omc.exe" } else { "omc" }
    $omcBin = Join-Path $env:OPENMODELICAHOME "bin"
    $omc = Join-Path $omcBin $omcExecutable
} elseif ($isWindowsPlatform) {
    $omc = Get-ChildItem (Join-Path $env:ProgramFiles "OpenModelica*\bin\omc.exe") -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $omc -or -not (Test-Path -LiteralPath $omc)) {
    throw "OpenModelica compiler not found; add omc to PATH or set OPENMODELICAHOME"
}

$repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$probePath = Join-Path $PSScriptRoot "SimulationProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica simulation probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$simulationLog = $output -join "`n"
if ($simulationLog -notmatch "The simulation finished successfully") {
    throw "MisalignedIdlerLine simulation did not finish successfully: $simulationLog"
}
if ($simulationLog -match "(?m)^Warning:") {
    throw "MisalignedIdlerLine simulation reported initialization warnings: $simulationLog"
}

$numericLines = @($output | Where-Object {
    ([string]$_).Trim() -match '^-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?$'
})
if ($numericLines.Count -lt 2) {
    throw "MisalignedIdlerLine simulation returned no dancer displacement: $simulationLog"
}
$initialPosition = [double]([string]$numericLines[-2]).Trim()
$finalPosition = [double]([string]$numericLines[-1]).Trim()
if ([double]::IsNaN($initialPosition) -or [double]::IsInfinity($initialPosition) -or
    [double]::IsNaN($finalPosition) -or [double]::IsInfinity($finalPosition)) {
    throw "Dancer displacement is not finite: $initialPosition -> $finalPosition"
}
if ([Math]::Abs($finalPosition - $initialPosition) -le 1e-6) {
    throw "Dancer prismatic joint did not move: $initialPosition -> $finalPosition"
}

Write-Output "MisalignedIdlerLine simulated without warnings and the external dancer moved."
