$ErrorActionPreference = "Stop"

$omcCommand = Get-Command omc -ErrorAction SilentlyContinue
$isWindowsPlatform = $env:OS -eq "Windows_NT"
if ($null -ne $omcCommand) {
    $omc = $omcCommand.Source
} elseif ($env:OPENMODELICAHOME) {
    $omcExecutable = if ($isWindowsPlatform) { "omc.exe" } else { "omc" }
    $omc = Join-Path (Join-Path $env:OPENMODELICAHOME "bin") $omcExecutable
} elseif ($isWindowsPlatform) {
    $omc = Get-ChildItem (Join-Path $env:ProgramFiles "OpenModelica*\bin\omc.exe") |
        Sort-Object FullName -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $omc -or -not (Test-Path -LiteralPath $omc)) {
    throw "OpenModelica compiler not found; add omc to PATH or set OPENMODELICAHOME"
}

$repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
Push-Location $repositoryRoot
try {
    $output = @(& $omc (Join-Path $PSScriptRoot "MassConservationChecks.mos"))
    $compilerExit = $LASTEXITCODE
    $probeLog = $output -join "`n"
    $logPath = Join-Path $repositoryRoot "build/mass-conservation-checks/verification.log"
    $probeLog | Set-Content -LiteralPath $logPath
    if ($compilerExit -ne 0 -or $probeLog -match 'Error:|Warning:|LOG_ASSERT|LOG_NLS.*error') {
        throw "Material conservation checks failed; see $logPath"
    }
    $expectedRuns = 16
    if ([regex]::Matches($probeLog, 'The simulation finished successfully\.').Count -ne $expectedRuns) {
        throw "Expected $expectedRuns successful simulations; see $logPath"
    }
} finally {
    Pop-Location
}
Write-Output "All $expectedRuns material conservation and derivative simulations passed without warnings."
