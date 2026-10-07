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
$probePath = Join-Path $PSScriptRoot "TripleSFrictionCheck.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica Triple-S probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$probeLog = $output -join "`n"
if ($probeLog -notmatch "Check of Roll2RollDynamicsTest.TripleSFrictionCheck completed successfully") {
    throw "Triple-S component check failed: $probeLog"
}
if ([regex]::Matches($probeLog, 'The simulation finished successfully\.').Count -ne 1) {
    throw "Triple-S simulation did not finish successfully: $probeLog"
}
$numericLines = @($output | Where-Object {
    ([string]$_).Trim() -match '^-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?$'
})
if ($numericLines.Count -lt 3) {
    throw "Triple-S probe returned fewer than three coefficients: $probeLog"
}
$actual = @(
    [double]([string]$numericLines[-3]).Trim(),
    [double]([string]$numericLines[-2]).Trim(),
    [double]([string]$numericLines[-1]).Trim()
)
$expected = @(0.0, 0.35, 0.22)
for ($i = 0; $i -lt $expected.Count; $i++) {
    if ([Math]::Abs($actual[$i] - $expected[$i]) -gt 1e-12) {
        throw "Triple-S coefficient $i is $($actual[$i]); expected $($expected[$i])"
    }
}
if ($output[-1] -ne '""') {
    throw "OpenModelica reported Triple-S errors: $($output[-1])"
}

Write-Output "Triple-S coefficients match zero, adhesion peak, and sliding saturation values."
