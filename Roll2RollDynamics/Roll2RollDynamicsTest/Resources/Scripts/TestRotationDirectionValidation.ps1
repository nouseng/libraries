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
$probePath = Join-Path $PSScriptRoot "InvalidRotationDirectionCheck.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica direction-validation probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$probeLog = $output -join "`n"
if ($probeLog -notmatch "Roller: rotationDirection must be -1 or \+1") {
    throw "Zero Roller rotationDirection did not report the intended validation: $probeLog"
}
if ($probeLog -notmatch "Winder: rotationDirection must be -1 or \+1") {
    throw "Zero Winder rotationDirection did not report the intended validation: $probeLog"
}
if ($probeLog -match "The simulation finished successfully") {
    throw "A component with rotationDirection = 0 must not simulate successfully: $probeLog"
}

Write-Output "Roller and Winder reject zero rotationDirection with clear messages."
