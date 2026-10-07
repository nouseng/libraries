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
$probePath = Join-Path $PSScriptRoot "AnnotationProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica annotation probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

# The probe emits one line per query after the three load results and the first
# getErrorString, so name the offsets once here rather than counting at each use.
$icons = @{ "Belt icon base" = $output[4]; "Roll icon base" = $output[5]
            "Winder icon base" = $output[6]; MisalignedIdlerLine = $output[7]
            WebWrap = $output[8]; RollBody = $output[9]
            WebFriction = $output[10] }
$diagrams = @{ Belt = $output[11]; Roller = $output[12]
               Winder = $output[13]; MisalignedIdlerLine = $output[14]
               WebWrap = $output[15]; RollBody = $output[16]
               WebFriction = $output[17] }
$components = @{ Belt = $output[18]; Roller = $output[19]
                 Winder = $output[20]; WebWorld = $output[21]
                 MisalignedIdlerLine = $output[22]; WebWrap = $output[23]
                 RollBody = $output[24]; WebFriction = $output[25] }

foreach ($name in $icons.Keys) {
    if ($icons[$name] -eq "{}") {
        throw "$name has no direct Icon annotation"
    }
    if ($icons[$name] -match "\bArc\(") {
        throw "$name uses the non-standard Arc graphical primitive"
    }
}

foreach ($name in $diagrams.Keys) {
    if ($diagrams[$name] -eq "{}") {
        throw "$name has no Diagram annotation"
    }
    if ($diagrams[$name] -match "\b(Rectangle|Ellipse|Polygon|Bitmap)\(") {
        throw "$name Diagram contains decorative graphics"
    }
}

# Dialog(tab, group, ...) is how OpenModelica serializes the annotation, so a
# group declared without a tab reads back as "General".
$parameterDialogs = @{
    Belt = @("`"General`",`"Geometry`"", "`"General`",`"Dynamics`"", "`"Animation`",`"Parameters`"")
    Roller = @("`"General`",`"Geometry`"", "`"General`",`"Mounting`"", "`"General`",`"Dynamics`"",
               "`"Initialization`",`"Parameters`"", "`"Animation`",`"Parameters`"")
    Winder = @("`"General`",`"Geometry`"", "`"General`",`"Mounting`"", "`"General`",`"Dynamics`"",
               "`"Initialization`",`"Parameters`"")
    WebWorld = @("`"General`",`"Geometry`"", "`"General`",`"Process`"", "`"General`",`"Out-of-plane`"")
}
foreach ($component in $parameterDialogs.Keys) {
    foreach ($dialog in $parameterDialogs[$component]) {
        if (-not $components[$component].Contains("Dialog($dialog")) {
            throw "$component parameter dialog is missing the $dialog group"
        }
    }
}
if ($components["Roller"] -notmatch 'Dialog\("Animation","Dynamic coating",useDynamicColor') {
    throw "Roller coating transition parameters must follow dynamic-color mode"
}

# Every connector is placed, and one whose diagram position falls outside the
# icon canvas needs an iconTransformation or it lands off the icon. Pinning the
# coordinates instead only breaks when a diagram is widened, which it did.
foreach ($file in @(
    "Components/Roller.mo", "Components/NipRoller.mo", "Components/Winder.mo", "Components/Belt.mo",
    "Utilities/Parts/WebWrap.mo", "Utilities/Parts/RollBody.mo",
    "Utilities/Parts/WebFriction.mo", "Utilities/Parts/NipContact.mo"
)) {
    $text = (Get-Content -LiteralPath (Join-Path $repositoryRoot "Roll2RollDynamics/$file") -Raw) -replace "`r`n", "`n"
    $pattern = "(?m)^\s*(?:(?:public|protected)\s+)?(?:Modelica\.Mechanics\.\S*Interfaces\.(?:Frame_a|Frame_b|Flange_a|Flange_b|Support)|Roll2RollDynamics\.Utilities\.Interfaces\.NipPort)\s+(?<name>\w+)"
    foreach ($match in [regex]::Matches($text, $pattern)) {
        $name = $match.Groups["name"].Value
        $declaration = $text.Substring($match.Index)
        $declaration = $declaration.Substring(0, $declaration.IndexOf(";") + 1)
        if ($declaration -notmatch "Placement\(") {
            throw "$file connector $name has no Placement annotation"
        }
        $origin = [regex]::Match($declaration, "transformation\(origin = \{(?<x>-?[\d.]+), (?<y>-?[\d.]+)\}")
        if ($origin.Success) {
            $offCanvas = ([Math]::Abs([double]$origin.Groups["x"].Value) -gt 100) -or
                         ([Math]::Abs([double]$origin.Groups["y"].Value) -gt 100)
            if ($offCanvas -and $declaration -notmatch "iconTransformation\(") {
                throw "$file connector $name sits off the icon canvas with no iconTransformation"
            }
        }
    }
}

$placementMinimums = @{ MisalignedIdlerLine = 25; Winder = 4 }
foreach ($name in $placementMinimums.Keys) {
    $placementCount = ([regex]::Matches($components[$name], "Placement\(")).Count
    if ($placementCount -lt $placementMinimums[$name]) {
        throw "$name exposes $placementCount placements; expected at least $($placementMinimums[$name])"
    }
}
$rollerPlacementCount = ([regex]::Matches($components["Roller"], "Placement\(")).Count
if ($rollerPlacementCount -ne 10) {
    throw "Roller exposes $rollerPlacementCount placements; expected exactly 10"
}
$requiredFacadePlacements = @(
    "Placement(true,0.0,-150.0,",
    "Placement(true,0.0,120.0,",
    "Placement(true,0.0,-40.0,"
)
foreach ($placement in $requiredFacadePlacements) {
    if (-not $components["Roller"].Contains($placement)) {
        throw "Roller facade is missing planned part placement $placement"
    }
}

# Every connect must be routed, or the diagram draws it as a straight line
# through whatever lies between. Read from the source rather than by asking for
# the nth connection, so adding a connect cannot silently skip the check.
$sourceFiles = @(
    Get-ChildItem -Path (Join-Path $repositoryRoot "Roll2RollDynamics") -Filter *.mo -Recurse
    Get-Item -LiteralPath (Join-Path $repositoryRoot "Roll2RollDynamicsTest/TripleSFrictionCheck.mo")
    Get-Item -LiteralPath (Join-Path $repositoryRoot "Roll2RollDynamicsTest/ComponentChecks.mo")
    Get-Item -LiteralPath (Join-Path $repositoryRoot "Roll2RollDynamicsTest/VisualGeometryChecks.mo")
)
$unrouted = @()
foreach ($file in $sourceFiles) {
    $text = (Get-Content -LiteralPath $file.FullName -Raw) -replace "`r`n", "`n"
    if ($file.Name -eq "ComponentChecks.mo") {
        $text = $text.Substring($text.IndexOf("model RollBodyComponentCheck"))
    }
    if ($file.Name -eq "VisualGeometryChecks.mo") {
        $start = $text.IndexOf("model RollerWrapVisualGeometryCheck")
        $endMarker = "end RollerFacadeCompositionCheck;"
        $text = $text.Substring(
            $start,
            $text.IndexOf($endMarker, $start) + $endMarker.Length - $start)
    }
    foreach ($match in [regex]::Matches($text, "(?s)\bconnect\s*\(.*?\)\s*(?<tail>[^;]*);")) {
        if ($match.Groups["tail"].Value -notmatch "annotation\s*\(\s*Line\(") {
            $line = ($text.Substring(0, $match.Index) -split "`n").Count
            $unrouted += "$($file.Name):$line"
        }
    }
}
if ($unrouted.Count -gt 0) {
    throw "Connections without a graphical Line annotation: $($unrouted -join ', ')"
}

foreach ($diagram in @($output[-5], $output[-4])) {
    if ($diagram -notmatch '\bRectangle\(' -or $diagram -notmatch '%name') {
        throw 'Combined web ports must have terminal graphics and a name in the diagram layer'
    }
}
foreach ($portPlacements in @($output[-3], $output[-2])) {
    if ($portPlacements -notmatch 'Placement\(false,0\.0,0\.0,' -or
        $portPlacements -notmatch 'Placement\(false,0\.0,-60\.0,') {
        throw 'Combined web ports must place distinct frame and web connection endpoints'
    }
}

if ($output[-1] -ne '""') {
    throw "OpenModelica reported annotation errors: $($output[-1])"
}

Write-Output "Icon, diagram, dialog, placement and connection-routing checks passed for $($sourceFiles.Count) files."
