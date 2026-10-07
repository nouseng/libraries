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
    $logDirectory = Join-Path $repositoryRoot "build/lateral-steering"
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    $output = @(& $omc (Join-Path $PSScriptRoot "LateralSteeringChecks.mos"))
    $compilerExit = $LASTEXITCODE
    $log = $output -join "`n"
    $logPath = Join-Path $logDirectory "verification.log"
    $log | Set-Content -LiteralPath $logPath
    if ($compilerExit -ne 0 -or $log -match 'Error:|Warning:|LOG_ASSERT|\| warning \|') {
        throw "Lateral steering checks failed; see $logPath"
    }
    if ([regex]::Matches($log, 'The simulation finished successfully\.').Count -ne 5) {
        throw "Expected five successful lateral steering simulations; see $logPath"
    }
} finally {
    Pop-Location
}
Write-Output "Five lateral steering checks passed: scene invariance, local slip, routing signs, reversal, friction and normal entry."
