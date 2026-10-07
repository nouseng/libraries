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

Push-Location (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)))
try {
    $output = foreach ($probe in @("SkewNipProbe.mos", "SkewNipIntegrationProbe.mos")) {
        & $omc (Join-Path $PSScriptRoot $probe)
        if ($LASTEXITCODE -ne 0) {
            throw "OpenModelica failed for $probe with exit code $LASTEXITCODE"
        }
    }
} finally {
    Pop-Location
}

$probeLog = $output -join "`n"
if (([regex]::Matches($probeLog, "The simulation finished successfully")).Count -ne 3 -or
    $probeLog -match '(?m)^Warning:|^Error:|LOG_ASSERT\s+\|\s+error|assertion has been violated') {
    throw "Skew nip validation failed: $probeLog"
}
$values = @($output | Where-Object {
    ([string]$_).Trim() -match '^-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?$'
})
if ($values.Count -lt 5) {
    throw "Skew nip probe returned too few measurements: $probeLog"
}
$skew, $skewPeak, $skewMinimum, $runoutPeak, $runoutMinimum = @(
    $values[-5..-1] | ForEach-Object {
        [double]::Parse(([string]$_).Trim(), [Globalization.CultureInfo]::InvariantCulture)
    }
)
if ([Math]::Abs($skew - 0.06) -gt 1e-10) {
    throw "Relative skew is wrong: skew=$skew"
}
# NipLoading uses a 0.66 m face, 0.10/0.12 m roll radii and 1 mm held penetration.
$edgeLift = [Math]::Pow(0.33*[Math]::Tan(0.06), 2)/0.44
$expectedSkewLoad = 1e6*(0.001 - $edgeLift/3)
if ([Math]::Abs($skewMinimum - $expectedSkewLoad) -gt 0.5 -or
    [Math]::Abs($skewPeak - $expectedSkewLoad) -gt 0.5) {
    throw "Expected a steady, reduced load from the crossed nip: peak=$skewPeak minimum=$skewMinimum"
}
if ($runoutMinimum -le 0 -or ($runoutPeak - $runoutMinimum) -lt 200) {
    throw "Expected the eccentric drum to ripple the held load: peak=$runoutPeak minimum=$runoutMinimum"
}
Write-Output "Three skew nip simulations passed: static crossed, parallel, released and wedged contacts; steady crossed load; runout ripple."
Write-Output "Crossed nip holds $skewMinimum to $skewPeak N; eccentric drum swings $runoutMinimum to $runoutPeak N."
