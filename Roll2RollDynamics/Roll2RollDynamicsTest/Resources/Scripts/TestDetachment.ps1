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
$logPath = Join-Path $repositoryRoot "build/detachment-checks/verification.log"
New-Item -ItemType Directory -Path (Split-Path -Parent $logPath) -Force | Out-Null
Push-Location $repositoryRoot
try {
    $output = @(& $omc (Join-Path $PSScriptRoot "DetachmentChecks.mos"))
    $compilerExit = $LASTEXITCODE
} finally {
    Pop-Location
}

$probeLog = $output -join "`n"
$probeLog | Set-Content -LiteralPath $logPath
if ($compilerExit -ne 0) {
    throw "OpenModelica detachment probe failed with exit code $compilerExit; see $logPath"
}

# getErrorString() follows each record and carries the initialization diagnostic.
$results = [regex]::Matches($probeLog,
    '(?ms)^record SimulationResult\b.*?(?=^record SimulationResult\b|\z)')
$expected = @("RigidRegression", "Detached", "EngagementContinuity", "RoundTrip",
    "StartDetached", "NipConflict", "LateralTransition")
if ($results.Count -ne $expected.Count) {
    throw "Expected seven detachment simulation results; found $($results.Count); see $logPath"
}

$nipDiagnostic = "Roller: a detachable roll cannot carry a nip, because a nip load has no meaning with no web between the rolls"
for ($index = 0; $index -lt $expected.Count; $index++) {
    $case = $expected[$index]
    $section = $results[$index].Value
    $record = [regex]::Match($section,
        '(?s)^record SimulationResult.*?end SimulationResult;').Value
    if (-not $record.Contains("fileNamePrefix = 'DetachmentChecks.$case'")) {
        throw "Missing simulation result for $case; see $logPath"
    }
    if ($case -eq "NipConflict") {
        if ($record -notmatch 'resultFile\s*=\s*""' -or
            $record -match 'The simulation finished successfully\.' -or
            -not $section.Contains($nipDiagnostic)) {
            throw "NipConflict did not produce its expected rejection; see $logPath"
        }
    } elseif ($record -notmatch 'resultFile\s*=\s*"[^"]+"' -or
        $record -notmatch 'The simulation finished successfully\.' -or
        $section -match '\b(?:Error|Warning):') {
        throw "$case failed; see $logPath"
    }
}

Write-Output "Six detachment simulations passed; NipConflict failed with the intended diagnostic."
