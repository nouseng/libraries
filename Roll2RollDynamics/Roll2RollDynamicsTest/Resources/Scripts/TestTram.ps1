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
Push-Location $repositoryRoot
try {
    $logDirectory = Join-Path $repositoryRoot "build/tram-checks"
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    $output = @(& $omc (Join-Path $PSScriptRoot "TramChecks.mos"))
    $compilerExit = $LASTEXITCODE
    $log = $output -join "`n"
    $logPath = Join-Path $logDirectory "verification.log"
    $log | Set-Content -LiteralPath $logPath
    # The line example already falls back to the pivoting linear solver at t = 0
    $checked = ($log -split "`n" | Where-Object { $_ -notmatch 'fallback solver with total pivoting' }) -join "`n"
    if ($compilerExit -ne 0 -or $checked -match 'Error:|Warning:|LOG_ASSERT|\| warning \|') {
        throw "Tram checks failed; see $logPath"
    }
    if ([regex]::Matches($log, 'The simulation finished successfully\.').Count -ne 3) {
        throw "Expected three successful tram simulations; see $logPath"
    }
} finally {
    Pop-Location
}
Write-Output "Three tram checks passed: wedge-only nip, crossed nip unchanged, tilted idler steering."
