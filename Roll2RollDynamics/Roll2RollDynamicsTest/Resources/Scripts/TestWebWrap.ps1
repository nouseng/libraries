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
    $output = @(& $omc (Join-Path $PSScriptRoot "WebWrapChecks.mos"))
    $compilerExit = $LASTEXITCODE
    $log = $output -join "`n"
    $logPath = Join-Path $repositoryRoot "build/webwrap-checks/verification.log"
    $log | Set-Content -LiteralPath $logPath
    if ($compilerExit -ne 0 -or $log -match 'Error:|Warning:') {
        throw "WebWrap compilation failed; see $logPath"
    }
    $records = [regex]::Matches($log, '(?s)record SimulationResult.*?end SimulationResult;')
    $expected = [ordered]@{
        SpatialResultant = $null
        CoarseReverseWrap = $null
        MovingTangency = $null
        ZeroResultant = $null
        SpanGradient = $null
        InvalidDirection = "WebWrap: rotationDirection must be -1 or +1"
        InvalidRadius = "WebWrap requires positive radius, webWidth and beltThickness"
        ZeroTension = "WebWrap requires taut entry and exit spans"
        AxialOnlyForce = "WebWrap requires nonzero entry and exit tension projected into the roll plane"
    }
    if ($records.Count -ne $expected.Count) {
        throw "Expected $($expected.Count) simulation results; see $logPath"
    }
    $index = 0
    foreach ($case in $expected.Keys) {
        $record = $records[$index++].Value
        if (-not $record.Contains("WebWrapChecks.$case")) {
            throw "Missing simulation result for $case; see $logPath"
        }
        if ($null -eq $expected[$case]) {
            if ($record -notmatch 'The simulation finished successfully\.' -or
                $record -match 'LOG_ASSERT|resultFile = ""') {
                throw "$case failed; see $logPath"
            }
        } elseif ($record -notmatch 'resultFile = ""' -or
            $record -notmatch 'LOG_ASSERT\s+\| error' -or
            -not $record.Contains($expected[$case])) {
            throw "$case did not produce its expected diagnostic; see $logPath"
        }
    }
} finally {
    Pop-Location
}
Write-Output "Five WebWrap geometry/load/colour simulations and four invalid-input checks passed."
