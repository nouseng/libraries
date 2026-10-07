$ErrorActionPreference = "Stop"

$omcCommand = Get-Command omc -ErrorAction SilentlyContinue
$isWindowsPlatform = $env:OS -eq "Windows_NT"
if ($null -ne $omcCommand) {
    $omc = $omcCommand.Source
} elseif ($env:OPENMODELICAHOME) {
    $omcExecutable = if ($isWindowsPlatform) { "omc.exe" } else { "omc" }
    $omc = Join-Path (Join-Path $env:OPENMODELICAHOME "bin") $omcExecutable
} elseif ($isWindowsPlatform) {
    $omc = Get-ChildItem (Join-Path $env:ProgramFiles "OpenModelica*\bin\omc.exe") -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $omc -or -not (Test-Path -LiteralPath $omc)) {
    throw "OpenModelica compiler not found; add omc to PATH or set OPENMODELICAHOME"
}

$repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$probePath = Join-Path $PSScriptRoot "NipIntegrationProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica nip integration probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$probeLog = $output -join "`n"
$successCount = ([regex]::Matches(
    $probeLog, "The simulation finished successfully")).Count
if ($successCount -ne 8) {
    throw "Expected eight successful nip integration simulations: $probeLog"
}
if ($probeLog -match 'LOG_ASSERT\s+\|\s+error|assertion has been violated') {
    throw "Nip integration assertion failed: $probeLog"
}
# Only the nip model is warning-free; MisalignedIdlerLine already warns about
# unspecified initial conditions before this feature, so its getErrorString is
# left out of the probe.
if ($probeLog -match "(?m)^Warning:") {
    throw "Nip integration simulation reported warnings: $probeLog"
}

$values = @($output | Where-Object {
    ([string]$_).Trim() -match '^-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?$'
})
if ($values.Count -lt 11) {
    throw "Nip integration probe returned fewer than eleven values: $probeLog"
}
$loadedBearingTorque = [double]([string]$values[-11]).Trim()
if ($loadedBearingTorque -le 0) {
    throw "Rotating top roller must have positive viscous bearing drag: $loadedBearingTorque"
}
$values = @($values[-10..-1] | ForEach-Object {
    [double]([string]$_).Trim()
})

$openLoad = $values[0]
$closedLoad = $values[1]
$capacityIncrement = $values[2]
$expectedIncrement = $values[3]
$speedBeforeContact = $values[4]
$speedUnderContact = $values[5]
$releasedTraction = $values[6]
$speedAtRelease = $values[7]
$speedAfterRelease = $values[8]
$disabledLoad = $values[9]

if ([Math]::Abs($openLoad) -gt 1e-8) {
    throw "Open nip carries load: $openLoad"
}
if ($closedLoad -le 1) {
    throw "Closed nip did not develop load: $closedLoad"
}
if ([Math]::Abs($capacityIncrement - $expectedIncrement) -gt 1e-8) {
    throw "Nip traction-capacity increment is wrong: $capacityIncrement vs $expectedIncrement"
}
if ([Math]::Abs($speedUnderContact) -le [Math]::Abs($speedBeforeContact) + 1e-3) {
    throw "Nip drum did not accelerate toward web speed: $speedBeforeContact -> $speedUnderContact"
}
if ([Math]::Abs($releasedTraction) -gt 1e-8) {
    throw "Released nip still applies traction: $releasedTraction"
}
if ([Math]::Abs($speedAfterRelease) -ge [Math]::Abs($speedAtRelease) - 1e-5) {
    throw "Bearing drag did not slow the released nip: $speedAtRelease -> $speedAfterRelease"
}
if ([Math]::Abs($disabledLoad) -gt 1e-12) {
    throw "Roller with a disabled nip port carries load: $disabledLoad"
}

Write-Output "Eight nip integration checks passed: momentum in both directions, normal load, friction ownership, capacity, drum response, release, partial and full actuator commands, disabled and unconnected behavior."
