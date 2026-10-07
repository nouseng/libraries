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
    New-Item -ItemType Directory -Force "build/shelton-checks" | Out-Null
    $output = @(& $omc (Join-Path $PSScriptRoot "SheltonChecks.mos"))
    $compilerExit = $LASTEXITCODE
    $probeLog = $output -join "`n"
    $logPath = Join-Path $repositoryRoot "build/shelton-checks/verification.log"
    $probeLog | Set-Content -LiteralPath $logPath
    if ($compilerExit -ne 0 -or $probeLog -match 'Error:|Warning:|LOG_ASSERT|LOG_NLS.*error') {
        throw "Shelton steering checks failed; see $logPath"
    }
    if ([regex]::Matches($probeLog, 'The simulation finished successfully\.').Count -ne 3) {
        throw "Expected three successful simulations; see $logPath"
    }
} finally {
    Pop-Location
}
Write-Output "All three Shelton analytical steering simulations passed without warnings."
