$ErrorActionPreference = "Stop"

$omcCommand = Get-Command omc -ErrorAction SilentlyContinue
if ($null -ne $omcCommand) {
    $omc = $omcCommand.Source
} elseif ($env:OPENMODELICAHOME) {
    $executable = if ($env:OS -eq "Windows_NT") { "omc.exe" } else { "omc" }
    $omc = Join-Path (Join-Path $env:OPENMODELICAHOME "bin") $executable
} elseif ($env:OS -eq "Windows_NT") {
    $omc = Get-ChildItem (Join-Path $env:ProgramFiles "OpenModelica*\bin\omc.exe") |
        Sort-Object FullName -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}
if (-not $omc -or -not (Test-Path -LiteralPath $omc)) {
    throw "OpenModelica compiler not found; add omc to PATH or set OPENMODELICAHOME"
}

$repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
$probePath = Join-Path $PSScriptRoot "RunoutChecks.mos"
$checkerPath = Join-Path $PSScriptRoot "check_runout_spectrum.py"
Push-Location $repositoryRoot
try {
    $logDirectory = Join-Path $repositoryRoot "build/runout-checks"
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    $output = @(& $omc $probePath)
    $compilerExit = $LASTEXITCODE
    $log = $output -join "`n"
    $logPath = Join-Path $logDirectory "verification.log"
    $log | Set-Content -LiteralPath $logPath
    # The line example already falls back to the pivoting linear solver at t = 0
    $checked = ($log -split "`n" | Where-Object { $_ -notmatch 'fallback solver with total pivoting' }) -join "`n"
    if ($compilerExit -ne 0 -or $checked -match 'Error:|Warning:|LOG_ASSERT|\| warning \|') {
        throw "Runout checks failed; see $logPath"
    }
    # Seven idler cases, one run-time override, two closed loops, a driven roll and one nip.
    $expectedRuns = 11
    if ([regex]::Matches($log, 'The simulation finished successfully\.').Count -ne $expectedRuns) {
        throw "Expected $expectedRuns successful runout simulations; see $logPath"
    }
    & python $checkerPath
    if ($LASTEXITCODE -ne 0) {
        throw "Runout spectrum and phase checks failed; see $logPath"
    }
} finally {
    Pop-Location
}
Write-Output "Runout checks passed: the swing, its once-per-turn rate and its phase follow the drum orbit, the ripple is proportional to the runout, the closed loop stays in balance and shares the ripple, and only radial runout swings a held nip load."

