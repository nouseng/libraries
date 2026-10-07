within Roll2RollDynamics.Components;
model Roller
  "Physical roller with frame-carried web transport, regularized traction and two belt tangencies"
  extends Roll2RollDynamics.Utilities.Icons.Roll;
  //Parameters
  // Geometry
  parameter Modelica.Units.SI.Radius radius = world.rollRadius "Roller radius"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length width = world.rollWidth "Roller face width (cross-machine)"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length beltThickness = world.web.thickness "Web/belt thickness"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Angle yaw = 0
    "In-plane misalignment: roller axis turned about the vertical, ends stay level"
    annotation(Dialog(group = "Misalignment"));
  parameter Modelica.Units.SI.Angle tram = 0
    "Out-of-plane misalignment: roller axis tilted about the machine direction, one end up"
    annotation(Dialog(group = "Misalignment"));
  parameter Boolean variableTilt = false
    "= true, keep yaw and tram as run-time parameters, for FMU sweeps"
    annotation(Dialog(group = "Misalignment"));
  parameter Modelica.Units.SI.Length runout = 0
    "Radial runout: drum centre offset from its bearing axis"
    annotation(Dialog(group = "Runout"));
  parameter Modelica.Units.SI.Angle runoutPhase = 0
    "Direction of the runout high point at zero shaft angle, from the machine direction toward up"
    annotation(Dialog(group = "Runout"));
  parameter Boolean variableRunout = false
    "= true, keep runout as a run-time parameter, for FMU sweeps"
    annotation(Dialog(group = "Runout"));
  parameter Integer rotationDirection(min = -1, max = 1) = 1
    "= +1, the roll touches the web bottom face (web passes over the roll); = -1, the roll touches the web top face (web passes under the roll)"
    annotation(Dialog(group = "Geometry"), choices(
      choice = 1 "Roll touches web bottom face (web passes over)",
      choice = -1 "Roll touches web top face (web passes under)"));
  final parameter Boolean webOverTop = rotationDirection > 0
    "= true, the web passes over the top of the roll; drawn in the icon"
    annotation(Evaluate = true, HideResult = true);

  // Mounting
  parameter Boolean useSupport = false
    "= true, expose the support frame; = false, ground the support internally at the roller position"
    annotation(Dialog(group = "Mounting"));
  parameter Modelica.Units.SI.Length xPosition = 0 "Fixed roller center x-position"
    annotation(Dialog(group = "Mounting", enable = not useSupport));
  parameter Modelica.Units.SI.Length zPosition = 0 "Fixed roller center height"
    annotation(Dialog(group = "Mounting", enable = not useSupport));
  parameter Boolean useFlange = false "= true, expose a drive/brake flange"
    annotation(Dialog(group = "Mounting"));
  parameter Boolean useNip = false "= true, expose the nip connection"
    annotation(Dialog(group = "Mounting"));

  // Dynamics
  parameter Modelica.Units.SI.RotationalDampingConstant bearingDamping(min=0) = world.bearingDamping
    "Viscous bearing damping"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.Density density = 2700 "Roller shell material density"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.Length wallThickness = 0.006 "Roller shell wall thickness"
    annotation(Dialog(group = "Dynamics"));

  // Contact
  parameter Roll2RollDynamics.Utilities.Types.WebFace contactFace =
    Roll2RollDynamics.Utilities.Types.WebFace.Inner
    "Material face of the web this roll surface touches"
    annotation(Dialog(group = "Contact"));
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters traction =
    if contactFace == Roll2RollDynamics.Utilities.Types.WebFace.Inner then
      world.innerTraction else world.outerTraction
    "Web-to-roll traction characteristic of the contacting face"
    annotation(Dialog(group = "Contact"));
  parameter Boolean useDetachment = false
    "= true, the web releases when the roll leaves the free span line"
    annotation(Dialog(group = "Contact"));
  parameter Modelica.Units.SI.Length contactLengthMin =
    if useDetachment then beltThickness else 0
    "Minimum wrapped length WebFriction retains for inventory near release; zero when useDetachment = false"
    annotation(Dialog(group = "Contact", enable = useDetachment));

  // Initialization
  parameter Boolean fixedInitialAngle = false "= true, fix the initial roller angle at zero"
    annotation(Dialog(tab = "Initialization"));
  parameter Boolean fixedInitialSpeed = false "= true, fix the initial roller angular speed at zero"
    annotation(Dialog(tab = "Initialization"));
  parameter Boolean fixedInitialWebState = true
    "= true, initialize web transport position and velocity at zero; set false only when the enclosing model prescribes its own transport initial conditions"
    annotation(Dialog(tab = "Initialization"));
  parameter Boolean initiallyEngaged = true
    "= true, the model starts with the web wrapped on the roll"
    annotation(Dialog(tab = "Initialization", enable = useDetachment));
  parameter Modelica.Units.SI.Angle entryAngleStart =
    if rotationDirection > 0 then -Modelica.Constants.pi/4 else -3*Modelica.Constants.pi/4
    "Guess for the entry tangency angle, measured from straight up toward +x"
    annotation(Dialog(tab = "Initialization"));
  parameter Modelica.Units.SI.Angle exitAngleStart =
    entryAngleStart + rotationDirection*Modelica.Constants.pi/2
    "Guess for the exit tangency angle, which must lie the way the roll turns from the entry"
    annotation(Dialog(tab = "Initialization"));

  // Animation
  parameter Boolean useDynamicColor = false
    "= true, surface color transitions with the wrap force"
    annotation(Dialog(tab = "Animation"));
  parameter Real rollColor[3] = world.rollColor "Roller surface color"
    annotation(Dialog(tab = "Animation"));
  parameter Integer arcSegments(min = 1) = 16
    "Number of boxes drawn along the web wrap"
    annotation(Dialog(tab = "Animation"));
  parameter Real webColor[3] = world.webColor "Wrap color at the low stress limit"
    annotation(Dialog(tab = "Animation"));
  parameter Real nominalColor[3] = world.nominalColor
    "Wrap color midway between the stress limits"
    annotation(Dialog(tab = "Animation"));
  parameter Real stressColor[3] = world.stressColor "Wrap color at the high stress limit"
    annotation(Dialog(tab = "Animation"));
  parameter Modelica.Units.SI.Stress stressLow = world.stressLow
    "Wrap stress drawn in webColor"
    annotation(Dialog(tab = "Animation"));
  parameter Modelica.Units.SI.Stress stressHigh = world.stressHigh
    "Wrap stress drawn in stressColor"
    annotation(Dialog(tab = "Animation"));
  parameter Modelica.Units.SI.Stress stressNominal = world.stressNominal
    "Wrap stress drawn in nominalColor"
    annotation(Dialog(tab = "Animation"));
  parameter Real colorGamma(min = Modelica.Constants.small) = world.stressColorGamma
    "Contrast exponent of the stress color gradient"
    annotation(Dialog(tab = "Animation"));
  parameter Real elastomerColor[3] = {220, 80, 40}
    "High-friction elastomer coating color"
    annotation(Dialog(tab = "Animation", group = "Dynamic coating", enable = useDynamicColor));
  parameter Modelica.Units.SI.Force FnThreshold = 220
    "Wrap force at which the coating state transitions"
    annotation(Dialog(tab = "Animation", group = "Dynamic coating", enable = useDynamicColor));
  parameter Modelica.Units.SI.Force FnSharpness = 15
    "Wrap-force range over which the coating transition happens"
    annotation(Dialog(tab = "Animation", group = "Dynamic coating", enable = useDynamicColor));

  //Variables
  // Main results, grouped into one browser node
  record RollerVariables "Roller results a line engineer plots"
    Modelica.Units.SI.Force tensionIn "Entry tension derived from frame_a force";
    Modelica.Units.SI.Force tensionOut "Exit tension derived from frame_b force";
    Modelica.Units.SI.Velocity surfaceVelocity
      "Roller surface speed along the web travel direction";
    Modelica.Units.SI.Angle angle "Roller turning angle";
    Modelica.Units.SI.Angle arcAngle "Signed web wrap angle";
    Boolean webWrapped "= false, the roll has left the free span line";
    Modelica.Units.SI.Velocity slipVelocity "Web-to-roll slip velocity";
    Real tractionCoefficient(unit = "1")
      "Longitudinal component of the regularized friction coefficient";
    Modelica.Units.SI.Force Ft "Tangential friction force";
    Modelica.Units.SI.Force Fcap "Peak available traction force";
    Modelica.Units.SI.Torque bearingTorque
      "Retarding torque the bearings apply to the shaft";
    Modelica.Units.SI.Position lateralPosition "Cross-machine web position";
    Modelica.Units.SI.Force nipLoad
      "Compressive load an attached nip presses onto the wrap";
  end RollerVariables;
  RollerVariables rollerVariables "Roller results displayed in the variable browser";

  //Inputs and Outputs
  // Diagnostics, states and geometry, hidden from the result file
  output Modelica.Units.SI.Force Fn "Resultant spatial load from the wrapped web"
    annotation(HideResult = true);
  output Modelica.Units.SI.Force wrapCapacity
    "Peak dry traction supplied by the wrap alone"
    annotation(HideResult = true);
  output Modelica.Units.SI.Force centrifugalTension
    "Centrifugal reduction of wrap contact pressure"
    annotation(HideResult = true);
  output Modelica.Units.SI.Velocity lateralSlip "Cross-machine web-to-roll slip"
    annotation(HideResult = true);
  output Modelica.Units.SI.Force lateralForce "Cross-machine friction force"
    annotation(HideResult = true);
  output Modelica.Units.SI.Momentum webMomentum "Longitudinal wrapped-web momentum"
    annotation(HideResult = true);
  output Modelica.Units.SI.Velocity webVelocityIn "Physical entry speed relative to roll centre"
    annotation(HideResult = true);
  output Modelica.Units.SI.Velocity webVelocityOut "Physical exit speed relative to roll centre"
    annotation(HideResult = true);
  output Modelica.Units.SI.Mass webMass "Wrapped material inventory in conservative mode"
    annotation(HideResult = true);
  output Modelica.Units.SI.Mass wrappedMass
    "True wrapped material inventory, unfloored; drives the support-reaction weight term"
    annotation(HideResult = true);
  output Modelica.Units.SI.Force wrappedLoad[3]
    "Load the wrapped material applies at the roll centre; frame_drum.f is defined from this, so asserting on it tests the actual wiring"
    annotation(HideResult = true);
  output Modelica.Units.SI.MassFlowRate massFlowIn "Signed entry web mass flow"
    annotation(HideResult = true);
  output Modelica.Units.SI.MassFlowRate massFlowOut "Signed exit web mass flow"
    annotation(HideResult = true);
  output Modelica.Units.SI.Length gap
    "Clearance from the roll surface to the free span line; zero while wrapped"
    annotation(HideResult = true);
  Modelica.Units.SI.Angle entryAngle
    "Entry tangency angle solved from the web geometry"
    annotation(HideResult = true);
  Modelica.Units.SI.Angle exitAngle
    "Exit tangency angle solved from the web geometry"
    annotation(HideResult = true);
  output Real entrySkewSin(unit="1") "Entry span direction projected along the roll axis"
    annotation(HideResult = true);
  output Real exitSkewSin(unit="1") "Exit span direction projected along the roll axis"
    annotation(HideResult = true);
  //Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_support if useSupport "Support/housing connection"
    annotation(Placement(transformation(origin = {0, -200}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Roll2RollDynamics.Utilities.Interfaces.RollPort frame_a
    "Belt entry tangency geometry and web transport"
    annotation(Placement(transformation(origin = {-160, 120}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
  Roll2RollDynamics.Utilities.Interfaces.RollPort frame_b
    "Belt exit tangency geometry and web transport"
    annotation(Placement(transformation(origin = {160, 120}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}})));
  Roll2RollDynamics.Utilities.Interfaces.NipPort frame_nip if useNip
    "Combined roller-centre and web connection to NipRoller"
    annotation(Placement(transformation(origin = {160, -20}, extent = {{-16, -16}, {16, 16}}, rotation = -360), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a if useFlange "Drive/brake port"
    annotation(Placement(transformation(origin = {160, -150}, extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, -10}, {10, 10}})));
  //Components
  outer Roll2RollDynamics.WebWorld world "Multibody world and shared line defaults";

protected
  //Variables
  // Working names of the displayed results; rollerVariables carries them outward.
  Modelica.Units.SI.Force tensionIn "Entry tension derived from frame_a force";
  Modelica.Units.SI.Force tensionOut "Exit tension derived from frame_b force";
  Modelica.Units.SI.Velocity surfaceVelocity
    "Roller surface speed along the web travel direction";
  Modelica.Units.SI.Angle angle "Roller turning angle";
  Modelica.Units.SI.Angle arcAngle "Signed web wrap angle";
  Boolean webWrapped "= false, the roll has left the free span line";
  Modelica.Units.SI.Velocity slipVelocity "Web-to-roll slip velocity";
  Real tractionCoefficient(unit = "1")
    "Longitudinal component of the regularized friction coefficient";
  Modelica.Units.SI.Force Ft "Tangential friction force";
  Modelica.Units.SI.Force Fcap "Peak available traction force";
  Modelica.Units.SI.Torque bearingTorque
    "Retarding torque the bearings apply to the shaft";
  Modelica.Units.SI.Position lateralPosition "Cross-machine web position";
  Modelica.Units.SI.Force nipLoad
    "Compressive load an attached nip presses onto the wrap";
  //Components
  Roll2RollDynamics.Utilities.Parts.RollBody rollBody(
    animation = world.enableAnimation,
    radius = radius,
    width = width,
    beltThickness = beltThickness,
    yaw = yaw,
    tram = tram,
    variableTilt = variableTilt,
    runout = runout,
    runoutPhase = runoutPhase,
    variableRunout = variableRunout,
    rotationDirection = rotationDirection,
    useSupport = useSupport,
    xPosition = xPosition,
    zPosition = zPosition,
    useFlange = useFlange or useNip,
    bearingDamping = bearingDamping,
    density = density,
    wallThickness = wallThickness,
    fixedInitialAngle = fixedInitialAngle,
    fixedInitialSpeed = fixedInitialSpeed,
    useDynamicColor = useDynamicColor,
    rollColor = rollColor,
    elastomerColor = elastomerColor,
    FnThreshold = FnThreshold,
    FnSharpness = FnSharpness,
    normalForce = webWrap.normalForce,
    drumDrawnRadius = webWrap.drumDrawnRadius)
    "Mounted rotating roll and surface"
    annotation(Placement(transformation(origin = {0, -150}, extent = {{-80, -40}, {80, 40}})));
  Roll2RollDynamics.Utilities.Parts.WebWrap webWrap(
    spanDirectionA = frame_a.spanDirection,
    spanDirectionB = frame_b.spanDirection,
    entryTension = -webFriction.flange_a.f,
    exitTension = webFriction.flange_b.f,
    animation = world.enableAnimation,
    radius = radius,
    webWidth = world.web.width,
    beltThickness = beltThickness,
    yaw = yaw,
    tram = tram,
    rotationDirection = rotationDirection,
    lateral = world.lateralDynamics,
    useDetachment = useDetachment,
    initiallyEngaged = initiallyEngaged,
    arcSegments = arcSegments,
    entryAngleStart = entryAngleStart,
    exitAngleStart = exitAngleStart,
    stressLow = stressLow,
    stressHigh = stressHigh,
    stressNominal = stressNominal,
    colorGamma = colorGamma,
    webColor = webColor,
    nominalColor = nominalColor,
    stressColor = stressColor)
    "Solved web wrap and transport"
    annotation(Placement(transformation(origin = {0, 120}, extent = {{-80, -40}, {80, 40}})));
  Roll2RollDynamics.Utilities.Parts.WebFriction webFriction(
    fixedInitialWebState = fixedInitialWebState,
    referenceDensity = world.web.density*world.web.thickness*world.web.width,
    axialStiffness = world.web.EA,
    contactLengthMin = contactLengthMin,
    wrapLength = webWrap.arcLength,
    tensionIn = webWrap.entryTension,
    tensionOut = webWrap.exitTension,
    stretchIn = frame_a.stretch,
    stretchOut = frame_b.stretch,
    boundaryVelocityA = webWrap.boundaryVelocityA,
    boundaryVelocityB = webWrap.boundaryVelocityB,
    nipForce = nipForceSensor.force,normalForce = sqrt(max(0, nipForceSensor.force*nipForceSensor.force
      - webFriction.contactFlange.f^2)),
    meanTension = webWrap.meanTension,
    wrapAngle = webWrap.wrapAngle,
    radius = webWrap.centrelineRadius,
    entryTangent = webWrap.entryTangent,
    exitTangent = webWrap.exitTangent,
    meanTangent = webWrap.meanTangent,
    rotationDirection = rotationDirection,
    traction = traction,
    lateral = world.lateralDynamics,
    useDetachment = useDetachment,
    engaged = webWrap.engaged,
    spanDirectionA = frame_a.spanDirection,
    spanDirectionB = frame_b.spanDirection)
    "Friction contact between web transport and roll surface"
    annotation(Placement(transformation(origin = {0, -40}, extent = {{-80, -40}, {80, 40}})));
  Roll2RollDynamics.Utilities.Parts.NipGeometrySource nipGeometry(
    radius = radius, rotationDirection = rotationDirection,
    lateralDrift = webFriction.lateralDrift) if useNip
    "Publishes radius and web direction on the nip port"
    annotation(Placement(transformation(origin = {120, 20}, extent = {{-10, -10}, {10, 10}})));
  // Normal and tangential nip forces are orthogonal; only the normal load adds capacity.
  Modelica.Mechanics.MultiBody.Sensors.CutForce nipForceSensor(animation = false)
    "Nip branch force, zero when the optional port is absent or unconnected"
    annotation(Placement(transformation(origin = {160, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

equation
  // Displayed results, grouped for the variable browser
  rollerVariables.tensionIn = tensionIn;
  rollerVariables.tensionOut = tensionOut;
  rollerVariables.surfaceVelocity = surfaceVelocity;
  rollerVariables.angle = angle;
  rollerVariables.arcAngle = arcAngle;
  rollerVariables.webWrapped = webWrapped;
  rollerVariables.slipVelocity = slipVelocity;
  rollerVariables.tractionCoefficient = tractionCoefficient;
  rollerVariables.Ft = Ft;
  rollerVariables.Fcap = Fcap;
  rollerVariables.bearingTorque = bearingTorque;
  rollerVariables.lateralPosition = lateralPosition;
  rollerVariables.nipLoad = nipLoad;
  assert(abs(rotationDirection) == 1,
    "Roller: rotationDirection must be -1 or +1");
  assert(not (useDetachment and useNip),
    "Roller: a detachable roll cannot carry a nip, because a nip load has no meaning with no web between the rolls");
  assert(abs(beltThickness/world.web.thickness - 1) < 1e-12,
    "Conservative Roller must use world.web.thickness");
  assert(Modelica.Math.Vectors.length(Modelica.Mechanics.MultiBody.Frames.angularVelocity1(
    rollBody.frame_center.R)) < 1e-8,
    "Conservative Roller supports translating mounts but requires fixed support orientation");
  assert(abs(tensionIn + frame_a.frame.f[1]) < 1e-6*(1 + abs(tensionIn))
    and abs(tensionOut - frame_b.frame.f[1]) < 1e-6*(1 + abs(tensionOut)),
    "Conservative Roller boundaries must supply consistent spatial tension and transport torque");
  connect(frame_support, rollBody.frame_support)
    annotation(Line(points = {{0, -200}, {0, -190}}, color = {95, 95, 95}, thickness = 0.5));
  connect(frame_a.frame, webWrap.frame_a)
    annotation(Line(points = {{-160, 120}, {-80, 120}}, color = {95, 95, 95}, thickness = 0.5));
  connect(frame_b.frame, webWrap.frame_b)
    annotation(Line(points = {{40, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {120, 120}));
  connect(rollBody.frame_wrap, webWrap.frame_center)
    annotation(Line(points = {{37.166, -96.593}, {-0.834, -96.593}, {-0.834, 93.407}, {37.166, 93.407}}, color = {95, 95, 95}, thickness = 0.5, origin = {-109.166, -13.407}));
  connect(rollBody.frame_drum, webFriction.frame_drum)
    annotation(Line(points = {{0, -110}, {0, -80}}, color = {95, 95, 95}, thickness = 0.5));
  connect(frame_a.web, webFriction.flange_a)
    annotation(Line(points = {{-160, 108}, {-145, 108}, {-145, 30}, {-52, 30}, {-52, 0}}, color = {0, 127, 0}));
  connect(frame_b.web, webFriction.flange_b)
    annotation(Line(points = {{160, 108}, {145, 108}, {145, 30}, {52, 30}, {52, 0}}, color = {0, 127, 0}));
  connect(webWrap.lateralFlange, webFriction.lateralFlange)
    annotation(Line(points = {{0, 40}, {0, -40}}, color = {0, 127, 0}, thickness = 0.5, origin = {0, 40}));
  connect(rollBody.flange_a, flange_a)
    annotation(Line(points = {{80, -150}, {160, -150}}, color = {0, 0, 0}));
  connect(rollBody.flange_a, frame_nip.shaft)
    annotation(Line(points = {{80, -150}, {180, -150}, {180, -20}, {160, -20}}, color = {0, 0, 0}));
  connect(rollBody.frame_wrap, nipForceSensor.frame_a)
    annotation(Line(points = {{-72, -110}, {-70, -110}, {-70, -100}, {160, -100}, {160, -80}}, color = {95, 95, 95}, thickness = 0.5, pattern = LinePattern.DashDot));
  connect(nipForceSensor.frame_b, frame_nip.frame)
    annotation(Line(points = {{0, -20}, {0, 20}}, color = {95, 95, 95}, thickness = 0.5, pattern = LinePattern.DashDot, origin = {160, -40}));
  connect(webFriction.contactFlange, frame_nip.web)
    annotation(Line(points={{80,-20},{120,-20},{120, -20},{160, -20}}, color={0,127,0}));
  connect(webFriction.lateralFlange, frame_nip.lateral)
    annotation(Line(points = {{0, 0}, {0, 20}, {140, 20}, {140, -20}, {160, -20}}, color = {0, 127, 0}));
  connect(nipGeometry.port, frame_nip.geometry)
    annotation(Line(points = {{130, 20}, {160, 20}, {160, -20}}, color = {95, 95, 95}));
  surfaceVelocity = webFriction.surfaceVelocity;
  angle = rollBody.angle;
  bearingTorque = rollBody.bearingTorque;
  tensionIn = webWrap.entryTension;
  tensionOut = webWrap.exitTension;
  Fn = webWrap.normalForce;
  entryAngle = webWrap.entryAngle;
  exitAngle = webWrap.exitAngle;
  arcAngle = webWrap.arcAngle;
  webWrapped = webWrap.engaged;
  gap = webWrap.gap;
  entrySkewSin = webFriction.entrySkewSin;
  exitSkewSin = webFriction.exitSkewSin;
  slipVelocity = webFriction.slipVelocity;
  webMass = webFriction.webMass;
  wrappedMass = webFriction.wrappedMass;
  wrappedLoad = webFriction.wrappedLoad;
  webMomentum = webFriction.momentum;
  webVelocityIn = webFriction.velocityIn;
  webVelocityOut = webFriction.velocityOut;
  massFlowIn = webFriction.massFlowIn;
  massFlowOut = webFriction.massFlowOut;
  Fcap = webFriction.tractionCapacity;
  centrifugalTension = webFriction.centrifugalTension;
  wrapCapacity = webFriction.wrapCapacity;
  tractionCoefficient = webFriction.mu;
  Ft = webFriction.longitudinalForce;
  lateralPosition = webFriction.lateralFlange.s;
  lateralSlip = webFriction.lateralSlip;
  lateralForce = webFriction.lateralForce;
  nipLoad = webFriction.normalForce;

  annotation(
    Icon(
      coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Line(visible = useNip, points = {{0, 72}, {0, 100}}, color = {95, 95, 95}, thickness = 0.5),
        Line(visible = useSupport, points = {{0, -72}, {0, -100}},
          color = {95, 95, 95}, thickness = 0.5),
        Line(visible = webOverTop, points = {{-100, 0}, {-94, 0}, {-88, 2}, {-82, 8}, {-76, 18}, {-67.6, 30.1}, {-64.09, 37}, {-59.87, 43.5}, {-54.99, 49.52}, {-49.52, 54.99}, {-43.5, 59.87}, {-37, 64.09}, {-30.1, 67.6}, {-22.87, 70.38}, {-15.39, 72.38}, {-7.74, 73.59}, {0, 74}, {7.74, 73.59}, {15.39, 72.38}, {22.87, 70.38}, {30.1, 67.6}, {37, 64.09}, {43.5, 59.87}, {49.52, 54.99}, {54.99, 49.52}, {59.87, 43.5}, {64.09, 37}, {67.6, 30.1}, {76, 18}, {82, 8}, {88, 2}, {94, 0}, {100, 0}}, color = {0, 128, 180}, thickness = 5),
        Line(visible = not webOverTop, points = {{-100, 0}, {-94, 0}, {-88, -2}, {-82, -8}, {-76, -18}, {-67.6, -30.1}, {-64.09, -37}, {-59.87, -43.5}, {-54.99, -49.52}, {-49.52, -54.99}, {-43.5, -59.87}, {-37, -64.09}, {-30.1, -67.6}, {-22.87, -70.38}, {-15.39, -72.38}, {-7.74, -73.59}, {0, -74}, {7.74, -73.59}, {15.39, -72.38}, {22.87, -70.38}, {30.1, -67.6}, {37, -64.09}, {43.5, -59.87}, {49.52, -54.99}, {54.99, -49.52}, {59.87, -43.5}, {64.09, -37}, {67.6, -30.1}, {76, -18}, {82, -8}, {88, -2}, {94, 0}, {100, 0}}, color = {0, 128, 180}, thickness = 3.5)
      }
    ),
    Diagram(coordinateSystem(extent = {{-160, -240}, {160, 200}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Text(origin = {-60, 100}, textColor = {64, 64, 64}, extent = {{-100, 86}, {-40, 100}}, textString = "%name",
        horizontalAlignment = TextAlignment.Left)}),
    Documentation(info = "<html>
<h4>Roller</h4>
<p>A driven roll, idler or translating dancer carries wrapped-web mass,
momentum and mean strain, with drum inertia and bearing drag. Regularized
Triple-S friction acts within the moving-belt capstan limit; see
<a href=\"modelica://Roll2RollDynamics.Utilities.Parts.WebFriction\">WebFriction</a>
and <a href=\"modelica://Roll2RollDynamics.Utilities.Parts.RollBody\">RollBody</a>.
Tangency and surface speed use <code>radius + beltThickness/2</code>.
Figure 1 shows friction for web faster than drum; arrows reverse with slip.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Roller/roller-parts.svg\"
     width=\"900\"
     alt=\"Exploded geometric view: the wrapped belt arc and end tensions at the top, opposing web and drum friction forces in the middle, and the rotating drum and support at the bottom\">
<strong>Figure 1:</strong> Drum, wrapped web and opposing contact forces.</p>
<p><code>radius</code>, <code>width</code> = From
<code>world.rollRadius</code>/<code>world.rollWidth</code><br>
<code>rotationDirection</code> = <code>+1</code> over the roll (default),
<code>-1</code> under it<br>
<code>contactFace</code> = Selects <code>world.innerTraction</code> or
<code>world.outerTraction</code>; <code>traction</code> may override it<br>
<code>yaw</code>, <code>tram</code> = Fixed in-plane/out-of-plane alignment,
zero by default<br>
<code>runout</code>, <code>runoutPhase</code> = Radial eccentricity and phase,
zero by default<br>
<code>bearingDamping</code> = Viscous shaft drag from the world</p>
<p>With <code>world.lateralDynamics = true</code>, yaw drives the entry span
toward normal entry. In the low-slip limit [2], offset is yaw times span
length and response time is span length over speed. Large yaw or weak
traction permits sideways slip. Figure 2 shows a yaw-crossed nip's parabolic
gap and central resultant.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-gap-crossed.png\"
     width=\"450\"
     alt=\"Front view of the nip face on the roller with crossed axes: both face edges lift on a parabolic gap\">
<strong>Figure 2:</strong> Parabolic gap under crossed axes.</p>
<p>Tram skews inclined spans but leaves horizontal spans unaffected.
Figure 3 shows its wedge gap under a nip, with load shifted toward the closed
end. Yaw and tram are housing faults, independent of shaft angle.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-gap-wedge.png\"
     width=\"450\"
     alt=\"Front view of the nip face on the roller with a wedge gap: one end opens and the load centre moves toward the closed end\">
<strong>Figure 3:</strong> Wedge gap under a trammed roll.</p>
<p>Figure 4 shows <code>runout</code> orbiting the drum centre once per turn;
<code>runoutPhase</code> sets the high point at zero shaft angle.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/RollBody/runout.svg\"
     width=\"900\"
     alt=\"Radial runout as a drum circle orbiting the bearing axis\">
<strong>Figure 4:</strong> Radial runout of the drum.</p>
<p>The orbit changes wrap/span lengths and nip penetration, but has no
cross-machine component. Figure 5 uses a 75 mm idler, 0.3 mm runout and 2 m/s:
span swing is 0.283 mm and tension ripple 6.98 N peak-to-peak at 4.20 Hz,
against 4.24 Hz turning rate [1]. Eccentric rolls require a free shaft state:
drive <code>flange_a</code> by torque or a speed controller; a kinematic speed
source fails index reduction. <code>variableRunout = true</code> keeps runout
settable in an FMU; <code>variableTilt = true</code> does likewise for yaw/tram.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Roller/runout.png\"
     width=\"450\"
     alt=\"Entry span tension and span length change over one second for an idler with 0.3 mm radial runout against a true drum\">
<strong>Figure 5:</strong> Entry span tension and length with 0.3 mm idler runout.</p>
<p>Figure 6 compares harmonic-order spectra with Branca et al. [1], Figure 9.
Measured and published-model data are transcribed in <code>Resources/Data</code>.
Each spectrum is normalized to its own fundamental because the paper omits
roller eccentricity. Both place the fundamental at turning rate, but the
paper's 17%/21% second/third harmonics are not reproduced: the library's
second harmonic is 0.3% for this idler and below 0.1% for a PI-driven roll.
Library speed varies by less than 0.1%; [1] attributes its harmonics to driven
S-wrap speed feedback, whose inertias/controllers are not specified.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Roller/runout-spectrum.png\"
     width=\"600\"
     alt=\"Tension ripple spectra against harmonic order: the library idler as a single tone at order one, the Euclid Web Line measurement with harmonics at two and three and its other rotating elements, and the paper's own model with a small second harmonic\">
<strong>Figure 6:</strong> Runout ripple spectrum against Branca et al. (2011), Figure 9.</p>
<h4>Traction and connections</h4>
<p>Capstan capacity subtracts centrifugal tension; a nip adds normal load.
Regularized friction needs finite slip and has no locked state. The web must
stay pressed against the roll. With lateral dynamics enabled, longitudinal
and lateral slip share one friction budget; wrapped mass supplies lateral
inertia. Lateral slip uses crossing velocity and span direction relative to
the axis, at entry for forward flow and exit for reverse.</p>
<p>Both <a href=\"modelica://Roll2RollDynamics.Utilities.Interfaces.RollPort\">RollPort</a>
connectors are required: connect incoming/outgoing spans to
<code>frame_a</code>/<code>frame_b</code>, each through a <code>Belt</code> or
<a href=\"modelica://Roll2RollDynamics.Components.WebForce\">WebForce</a>.</p>
<p><code>useSupport = true</code> exposes <code>frame_support</code>; default
false fixes the housing at <code>xPosition</code>/<code>zPosition</code>.
Mount orientation must remain fixed, including yaw.
<code>useFlange = true</code> exposes <code>flange_a</code> for drive/brake;
default false leaves the drum undriven.</p>
<p><code>useNip = true</code> exposes <code>frame_nip</code> for
<code>NipRoller.nipLoading</code>. <code>NipRoller</code> owns contact and returns
normal, tangential and lateral reactions; only normal load raises wrap
capacity. An unconnected port gives zero <code>nipLoad</code>.</p>
<p><code>fixedInitialAngle</code>/<code>fixedInitialSpeed</code> set initial
shell rotation. <code>fixedInitialWebState</code> fixes both transport
coordinates, contact coordinate and web speed.
<code>world.enableAnimation</code> controls drum/web display;
<code>useDynamicColor</code> colours the wrap by stress.</p>
<h4>Detachment</h4>
<p><code>useDetachment = true</code> releases the web when the roll leaves
the neighbouring spans' straight path, and re-engages on return (Figure 7).
Detached, <code>arcAngle</code> and traction are exactly zero and
<code>gap</code> measures clearance; engaged contact is rigid with zero gap.
Either the roll support or neighbouring rolls may move. Detachment cannot
be combined with a nip.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Roller/detachment-geometry.svg\"
     width=\"900\"
     alt=\"Engaged roll pushed into the free span line with a nonzero wrap angle and zero gap, beside a detached roll clear of the line with zero wrap angle and a nonzero gap\">
<strong>Figure 7:</strong> Engaged and detached web geometry.</p>
<p><code>contactLengthMin</code> regularizes zero inventory in the momentum
closure only, leaving friction and static wrapped-material load zero when detached. A small
floor-bounded momentum-flux residual remains during detached dwell. Its
default is <code>beltThickness</code> with detachment enabled, zero otherwise.</p>
<h4>Displayed variables</h4>
<p>Main simulation results are grouped in <code>rollerVariables</code>:</p>
<ul>
<li><code>tensionIn</code>, <code>tensionOut</code> [N]: Entry and exit tension.</li>
<li><code>surfaceVelocity</code> [m/s]: Roll surface velocity along web travel.</li>
<li><code>angle</code> [rad]: Shaft turning angle.</li>
<li><code>arcAngle</code> [rad]: Signed web wrap angle.</li>
<li><code>webWrapped</code> [Boolean]: False when the roll has left the free span line.</li>
<li><code>slipVelocity</code> [m/s]: Mean web velocity minus roll surface velocity.</li>
<li><code>tractionCoefficient</code> [1]: Longitudinal regularized friction coefficient.</li>
<li><code>Ft</code>, <code>Fcap</code> [N]: Tangential friction force and its peak capacity.</li>
<li><code>bearingTorque</code> [N.m]: Retarding torque of the bearings.</li>
<li><code>lateralPosition</code> [m]: Cross-machine web position.</li>
<li><code>nipLoad</code> [N]: Compressive load supplied by an attached nip.</li>
</ul>
<p><code>webMass</code> reports only the wrapped inventory; sum it over the
line for the total. Mass-flow, momentum, capacity and tangency quantities are
public but carry <code>HideResult = true</code>.</p>
<h4>Limitations</h4>
<p>The mean-strain wrap applies gravity at the support but omits distributed
gravity moment and mount-acceleration coupling to an offset wrap centroid.
Asymmetric wraps on accelerating mounts are therefore approximate. Spans
must remain taut; slack and lateral bending are omitted.</p>
<p>See <a href=\"modelica://Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine\">MisalignedIdlerLine</a>
for runout/yaw, <a href=\"modelica://Roll2RollDynamics.Examples.SlipAndTractionCapacity\">SlipAndTractionCapacity</a>
for traction, <a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>
for lateral motion and <a href=\"modelica://Roll2RollDynamics.Examples.RetractingIdler\">RetractingIdler</a>
for release/return.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/321969\">1</a>]
C. Branca, P. R. Pagilla and K. N. Reid, &quot;Web Tension Behavior in the
Presence of Eccentric Rollers: Modeling and Validation&quot;, Proc.
International Conference on Web Handling, Oklahoma State University, 2011.
Eq. (10) gives the length of the span beside an eccentric roller, Eq. (11) its
rate of change, and Eq. (23) the fundamental frequency of the tension
disturbance at the roller turning rate; Figures 8 to 10 give the measured and
simulated tension spectra of the Euclid Web Line at 200, 250 and 300 FPM.
Accessed 2026-09-15.
<a href=\"https://hdl.handle.net/20.500.14446/321969\">Source</a>.</li>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/30409\">2</a>]
J. J. Shelton, &quot;Lateral Dynamics of a Moving Web&quot;, PhD thesis,
Oklahoma State University, 1968, Chapter III, eqs. (3.1.1), (3.2.1)-(3.2.4).
Normal-entry kinematics and the L/V response time of a yawed roll; compared
with the library and with measured yaw data in
<a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>.
Accessed 2026-09-07.
<a href=\"https://hdl.handle.net/20.500.14446/30409\">Source</a>.</li>
</ul>
</html>"));
end Roller;
