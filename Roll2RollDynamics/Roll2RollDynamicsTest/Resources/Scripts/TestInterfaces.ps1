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
$probePath = Join-Path $PSScriptRoot "InterfaceProbe.mos"
Push-Location $repositoryRoot
try {
    $output = @(& $omc $probePath)
    if ($LASTEXITCODE -ne 0) {
        throw "OpenModelica interface probe failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

$beltComponents = $output[4]
$rollerComponents = $output[5]
$winderComponents = $output[6]
$webWrapComponents = $output[7]
$rollBodyComponents = $output[8]
$webFrictionComponents = $output[9]
$nipRollComponents = $output[10]
$nipContactComponents = $output[11]

foreach ($name in @("spanLength", "web_a", "web_b", "v_a", "v_b", "v_belt")) {
    if ($beltComponents -match ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Belt still exposes redundant connector $name"
    }
}
foreach ($name in @("frame_a", "frame_b", "tension", "tensionIn", "tensionOut",
    "webMass", "momentum", "physicalVelocity", "aBelt")) {
    if ($beltComponents -notmatch ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Belt compiled interface is missing $name"
    }
}

foreach ($portComponents in @($output[-4], $output[-3])) {
    foreach ($name in @("frame", "web", "spanDirection")) {
        if ($portComponents -notmatch ('[,{]\s*"?' + $name + '"?\s*[,}]')) {
            throw "Composite web port is missing $name"
        }
    }
    if ($portComponents -match '[,{]\s*"?halfSpanMass"?\s*[,}]') {
        throw "Composite web ports must transfer support reactions through their frame"
    }
}

foreach ($name in @("T_in", "T_out", "tension", "web_a", "web_b")) {
    if ($rollerComponents -match ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Roller still exposes redundant signal connector $name"
    }
}
foreach ($name in @(
    "position", "kinematic", "movable", "dancerTravel", "dancerGain",
    "tauDancer", "tensionSetpoint", "guideColor", "guideAnimation",
    "wrapAngle"
)) {
    $pattern = '[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]'
    if ($rollerComponents -match $pattern) {
        throw "Roller still exposes redundant member $name"
    }
}
foreach ($name in @("v_belt", "v")) {
    if ($rollerComponents -match ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Roller still exposes redundant speed connector $name"
    }
}
foreach ($name in @("T", "frame_t_b", "web", "r", "mass0")) {
    if ($winderComponents -match ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Winder still exposes redundant member $name"
    }
}

$requiredRollerParameters = @(
    "radius", "width", "beltThickness", "yaw",
    "rotationDirection", "useSupport", "xPosition", "zPosition",
    "useFlange", "useNip", "traction", "bearingDamping", "density", "wallThickness",
    "fixedInitialAngle", "fixedInitialSpeed",
    "entryAngleStart", "exitAngleStart", "useDynamicColor", "rollColor",
    "arcSegments", "stressColor", "stressLow", "stressHigh", "colorGamma",
    "elastomerColor", "FnThreshold", "FnSharpness"
)
foreach ($name in $requiredRollerParameters) {
    if ($rollerComponents -notmatch ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Roller compiled interface is missing parameter $name"
    }
}
$requiredRollerMembers = @(
    "frame_support", "frame_a", "frame_b", "frame_nip", "flange_a",
    "surfaceVelocity", "tensionIn", "tensionOut", "angle", "Fn", "Fcap",
    "Ft", "slipVelocity", "lateralPosition", "lateralSlip",
    "lateralForce", "bearingTorque", "nipLoad",
    "entryAngle", "exitAngle", "arcAngle"
)
foreach ($name in $requiredRollerMembers) {
    if ($rollerComponents -notmatch ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Roller compiled interface is missing $name"
    }
}
foreach ($name in @("nipTraction", "nipSlip", "penetration", "engaged")) {
    if ($rollerComponents -match ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Roller still duplicates NipRoller diagnostic $name"
    }
}
foreach ($name in @("frame_t", "radius", "surfaceVelocity", "tension", "angle")) {
    if ($winderComponents -notmatch ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Winder compiled interface is missing $name"
    }
}

foreach ($name in @("frame_support", "nipLoading")) {
    if ($nipRollComponents -notmatch ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "NipRoller compiled interface is missing $name"
    }
}
$nipFrameCount = ([regex]::Matches(
    $nipRollComponents,
    '\{Modelica\.Mechanics\.MultiBody\.Interfaces\.Frame_[ab],')).Count
if ($nipFrameCount -ne 1 -or $nipRollComponents -notmatch
    '\{Roll2RollDynamics\.Utilities\.Interfaces\.NipPort,\s*nipLoading,') {
    throw "NipRoller must expose one support frame and one combined nip port"
}
if ($rollerComponents -notmatch
    '\{Roll2RollDynamics\.Utilities\.Interfaces\.NipPort,\s*frame_nip,') {
    throw "Roller must expose the same combined nip port type"
}
foreach ($name in @("traction", "coverStiffness", "coverDamping", "maxNipLoad")) {
    if ($nipRollComponents -notmatch ('[,{]\s*"?' + $name + '"?\s*[,}]')) {
        throw "NipRoller is missing contact parameter $name"
    }
}
foreach ($name in @("nipRadius", "coverStiffness", "coverDamping", "maxNipLoad", "nipContact")) {
    if ($rollerComponents -match ('[,{]\s*"?' + $name + '"?\s*[,}]')) {
        throw "Roller still owns nip contact member $name"
    }
}
if ($nipRollComponents -notmatch
    '\{Roll2RollDynamics\.Utilities\.Parts\.NipContact,\s*nipContact,\s*.*?,\s*"protected",') {
    throw "NipRoller must own the protected NipContact instance"
}

foreach ($name in @("webWrap", "rollBody", "webFriction")) {
    $partPattern = '\{Roll2RollDynamics\.Utilities\.Parts\.[^,]+,\s*' +
        [regex]::Escape($name) + ',\s*.*?,\s*"protected",'
    if ($rollerComponents -notmatch $partPattern) {
        throw "Roller facade is missing protected internal part $name"
    }
}
$partCount = ([regex]::Matches(
    $rollerComponents,
    '\{Roll2RollDynamics\.Utilities\.Parts\.(WebWrap|RollBody|WebFriction),')).Count
if ($partCount -ne 3) {
    throw "Roller facade contains $partCount focused part instances; expected exactly 3"
}
foreach ($name in @(
    "baseTranslation", "fixedSupport", "skewRotation", "lateralJoint",
    "lateralLock", "revolute", "drumBody", "tangentA", "tangentB",
    "webAdapterA", "webAdapterB", "rollBearing",
    "webGearA", "webGearB", "wrapArc", "drumSurface", "faceMark", "webWeight"
)) {
    if ($rollerComponents -match ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
        throw "Roller facade still owns extracted component $name"
    }
}

$partInterfaces = @{
    WebWrap = @(
        $webWrapComponents,
        @("frame_center", "frame_a", "frame_b", "entryTension", "exitTension",
          "spanDirectionA", "spanDirectionB",
          "lateralFlange", "meanTension", "normalForce",
          "entryAngle", "exitAngle", "arcAngle", "arcLength",
          "drumDrawnRadius", "arcColor", "arcSegmentColor")
    )
    RollBody = @(
        $rollBodyComponents,
        @("frame_support", "frame_center", "frame_drum", "flange_a",
          "normalForce", "drumDrawnRadius", "surfaceVelocity",
          "angle", "bearingTorque",
          "drumSurfaceColor")
    )
    WebFriction = @(
        $webFrictionComponents,
        @("flange_a", "flange_b", "frame_drum", "lateralFlange",
          "normalForce", "lateralDrift", "slipVelocity", "tractionCapacity",
          "longitudinalForce", "lateralSlip", "lateralForce")
    )
    NipContact = @(
        $nipContactComponents,
        @("nipPort", "frameNip", "normalForce",
          "tangentialForce", "penetration", "slipVelocity", "engaged")
    )
}
if ([regex]::Matches($webFrictionComponents,
    '\{Modelica\.Mechanics\.MultiBody\.Interfaces\.Frame_[ab],').Count -ne 1) {
    throw "WebFriction must carry support force and contact torque through one frame"
}
foreach ($part in $partInterfaces.Keys) {
    $componentList = $partInterfaces[$part][0]
    foreach ($name in $partInterfaces[$part][1]) {
        if ($componentList -notmatch ('[,{]\s*"?' + [regex]::Escape($name) + '"?\s*[,}]')) {
            throw "$part compiled interface is missing $name"
        }
    }
}

foreach ($name in @("surfaceVelocity", "tensionIn", "tensionOut", "angle", "Fn", "Fcap", "Ft", "slipVelocity")) {
    if ($rollerComponents -match ('\{Modelica\.Blocks\.Interfaces\.RealOutput,\s*' + [regex]::Escape($name) + '\s*,')) {
        throw "Roller diagnostic $name must be an ordinary output variable, not a signal connector"
    }
}
foreach ($name in @("radius", "surfaceVelocity", "tension", "angle")) {
    if ($winderComponents -match ('\{Modelica\.Blocks\.Interfaces\.RealOutput,\s*' + [regex]::Escape($name) + '\s*,')) {
        throw "Winder diagnostic $name must be an ordinary output variable, not a signal connector"
    }
}

# Four dancer-support, twelve web-path and ten drive/control connections.
if ([int]$output[12] -ne 26) {
    throw "MisalignedIdlerLine has $($output[12]) connections; expected 26 with external roller supports"
}
if ($output[13] -ne '"false"' -or $output[14] -ne '"false"') {
    throw "Roller must leave initial angle and speed free by default for external drives"
}
if ($output[15] -ne '"false"') {
    throw "Roller.useNip must default to false"
}
$modelCheck = $output[16..($output.Count - 2)] -join "`n"
if ($modelCheck -notmatch "Check of Roll2RollDynamicsTest.ComponentChecks.ExternalDancerSupportCheck completed successfully") {
    throw "OpenModelica external dancer support check failed: $modelCheck"
}
if ($modelCheck -notmatch "Check of Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine completed successfully") {
    throw "OpenModelica model check failed: $modelCheck"
}
if ($output[-2] -ne '{"1.0.0"}') {
    throw "Roll2RollDynamics package version is $($output[-2]); expected 1.0.0"
}
if ($output[-1] -ne '""') {
    throw "OpenModelica reported interface errors: $($output[-1])"
}

Write-Output "Composite web transport, material inventory and support load, and assembled model checks passed."
