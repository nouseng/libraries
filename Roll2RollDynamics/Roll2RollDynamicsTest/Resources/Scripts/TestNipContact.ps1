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
$probePath = Join-Path $PSScriptRoot "NipContactProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica nip contact probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$probeLog = $output -join "`n"
if (([regex]::Matches($probeLog, "The simulation finished successfully")).Count -ne 2) {
    throw "Nip contact simulation failed: $probeLog"
}
if ($probeLog -match "(?m)^Warning:") {
    throw "Nip contact simulation reported warnings: $probeLog"
}

$values = @($output | Where-Object {
    ([string]$_).Trim() -match '^(?:true|false|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)$'
})
if ($values.Count -lt 9) {
    throw "Nip contact probe returned fewer than nine values: $probeLog"
}

$separated = [double]([string]$values[-9]).Trim()
$shallow = [double]([string]$values[-8]).Trim()
$deep = [double]([string]$values[-7]).Trim()
$forwardSlip = [double]([string]$values[-6]).Trim()
$forwardForce = [double]([string]$values[-5]).Trim()
$reverseSlip = [double]([string]$values[-4]).Trim()
$reverseForce = [double]([string]$values[-3]).Trim()
$forwardLoss = [double]([string]$values[-2]).Trim()
$reverseLoss = [double]([string]$values[-1]).Trim()
$expectedShallow = 100.0

if ([Math]::Abs($separated) -gt 1e-10) {
    throw "Separated contact load is $separated; expected zero"
}
if ([Math]::Abs($shallow - $expectedShallow) -gt 1e-5) {
    throw "Shallow contact load is $shallow; expected $expectedShallow"
}
if ([Math]::Abs($deep - 10000.0) -gt 1e-4) {
    throw "Deep contact load is $deep; expected uncapped compliance load 10000 N"
}
if ($forwardSlip -ge 0 -or $forwardForce -ge 0) {
    throw "Forward web motion has wrong nip slip/force signs: slip=$forwardSlip force=$forwardForce"
}
if ($reverseSlip -le 0 -or $reverseForce -le 0) {
    throw "Reverse web motion has wrong nip slip/force signs: slip=$reverseSlip force=$reverseForce"
}
if ($forwardLoss -lt -1e-10 -or $reverseLoss -lt -1e-10) {
    throw "Nip friction generated energy: forward=$forwardLoss reverse=$reverseLoss"
}

Write-Output "Nip contact separation, uncapped compliance, friction signs, dissipation and the disabled endpoint are correct."
