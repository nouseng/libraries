$ErrorActionPreference = "Stop"

$omcCommand = Get-Command omc -ErrorAction SilentlyContinue
if ($null -ne $omcCommand) {
    $omc = $omcCommand.Source
} elseif ($env:OPENMODELICAHOME) {
    $omcExecutable = if ($env:OS -eq "Windows_NT") { "omc.exe" } else { "omc" }
    $omc = Join-Path (Join-Path $env:OPENMODELICAHOME "bin") $omcExecutable
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
    $logDirectory = Join-Path $repositoryRoot "build/belt-dynamics"
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    foreach ($suite in @("LocalBeltDynamicsChecks", "TangentFrameLoadCheck", "BeltDampingChecks")) {
        $output = @(& $omc (Join-Path $PSScriptRoot "$suite.mos"))
        $compilerExit = $LASTEXITCODE
        $log = $output -join "`n"
        $logPath = Join-Path $logDirectory "$suite.log"
        $log | Set-Content -LiteralPath $logPath
        if ($compilerExit -ne 0 -or $log -match 'Error:|Warning:|LOG_ASSERT|\| warning \|') {
            throw "Belt dynamics checks failed; see $logPath"
        }
        $expectedRuns = if ($suite -eq "BeltDampingChecks") { 7 } else { 6 }
        if ([regex]::Matches($log, 'The simulation finished successfully\.').Count -ne $expectedRuns) {
            throw "Expected $expectedRuns successful $suite simulations; see $logPath"
        }
    }
} finally {
    Pop-Location
}
Write-Output "Nineteen belt damping, acceleration, conservation and support-load checks passed without warnings."
