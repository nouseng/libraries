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
$probePath = Join-Path $PSScriptRoot "SupportReactionProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica support-reaction probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$simulationLog = $output -join "`n"
if (([regex]::Matches($simulationLog, "The simulation finished successfully")).Count -ne 4 -or
    $simulationLog -match 'LOG_ASSERT\s+\|\s+error|assertion has been violated') {
    throw "Support-reaction simulation failed: $simulationLog"
}

$numericLines = @($output | Where-Object {
    ([string]$_).Trim() -match '^-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?$'
})
if ($numericLines.Count -lt 3) {
    throw "Support-reaction probe returned fewer than three forces: $simulationLog"
}
$winderSupportForceX = [double]([string]$numericLines[-3]).Trim()
$rollerSupportForceX = [double]([string]$numericLines[-2]).Trim()
$rollBodySupportForceX = [double]([string]$numericLines[-1]).Trim()
if ([Math]::Abs($winderSupportForceX + 100) -gt 1e-6) {
    throw "Winder support force is $winderSupportForceX N; expected -100 N"
}
if ([Math]::Abs($rollerSupportForceX + 100) -gt 1e-6) {
    throw "Roller support force is $rollerSupportForceX N; expected -100 N"
}
if ([Math]::Abs($rollBodySupportForceX + 100) -gt 1e-6) {
    throw "Roll body support force is $rollBodySupportForceX N; expected -100 N"
}
if ($output[-1] -ne '""') {
    throw "OpenModelica reported support-reaction errors: $($output[-1])"
}

Write-Output "Support reactions passed, including applied forces, yaw and shell weight."
