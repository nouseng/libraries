within Roll2RollDynamics.Examples;
model SlipAndTractionCapacity
  "An over-driven roller swept through slip, with and without a nip on its wrap"
  extends Modelica.Icons.Example;

  // Parameters
  parameter Modelica.Units.SI.Force nipLoad(min = 0) = 300
    "Static nip load on the second test roller";
  parameter Modelica.Units.SI.Velocity maximumSlip(min = Modelica.Constants.small) = 0.01
    "Commanded slip magnitude at each end of the sweep";
  parameter Modelica.Units.SI.Time startupTime(min = Modelica.Constants.small) = 0.5
    "Common surface-speed ramp from rest";
  parameter Modelica.Units.SI.Time sweepStart(min = 0) = 1
    "Time at which the test rollers begin to be over-driven";
  parameter Modelica.Units.SI.Time sweepDuration(min = Modelica.Constants.small) = 4
    "Time taken to traverse the slip characteristic";
  final parameter Modelica.Units.SI.Length nipReach =
    2*world.rollRadius + world.web.thickness
    "Centre distance at which the nip cover just touches the web";

  // Variables with Binding Equations
  output Modelica.Units.SI.Velocity commandedSlip = sweep.y
    "Commanded web-to-roll slip at the test rollers";
  output Modelica.Units.SI.Velocity wrapSlip = wrapTest.rollerVariables.slipVelocity
    "Measured slip at the wrap-only test roller";
  output Modelica.Units.SI.Velocity totalSlip = nipTest.rollerVariables.slipVelocity
    "Measured slip at the nip-loaded test roller";
  output Modelica.Units.SI.Force wrapTraction = wrapTest.rollerVariables.Ft
    "Signed traction on the wrap-only test roller";
  output Modelica.Units.SI.Force totalTraction = nipTest.rollerVariables.Ft
    "Signed traction on the nip-loaded test roller";
  output Modelica.Units.SI.Force wrapCapacity = wrapTest.rollerVariables.Fcap
    "Peak dry traction of the wrap-only test roller";
  output Modelica.Units.SI.Force totalCapacity = nipTest.rollerVariables.Fcap
    "Peak dry traction of the nip-loaded test roller";
  output Modelica.Units.SI.Force negativeWrapCapacity = -wrapCapacity
    "Negative dry traction limit of the wrap-only test roller";
  output Modelica.Units.SI.Force negativeTotalCapacity = -totalCapacity
    "Negative dry traction limit of the nip-loaded test roller";

  // Components
  inner Roll2RollDynamics.WebWorld world(
    web(EA = 2e6), tension = 300, webDampingTime = 0.1, rollRadius = 0.1, spanLength = 0.6,
    nominalLength = 0.3, g = 0, lateralDynamics = false)
    "Line settings: stiff 300 N web, 0.1 m rolls, 2 m/s"
    annotation(Placement(transformation(origin = {-150, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp startup(height = world.lineSpeed, duration = startupTime)
    "Common surface-speed ramp from rest"
    annotation(Placement(transformation(origin = {-150, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.TimeTable sweep(table = [
    0, 0;
    startupTime, 0;
    sweepStart, -maximumSlip;
    sweepStart + sweepDuration, maximumSlip;
    sweepStart + sweepDuration + 1, maximumSlip])
    "Web-to-roll slip command: zero during startup, eased to the start of the sweep"
    annotation(Placement(transformation(origin = {-150, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add testSpeed(k2 = -1) "Master surface speed minus the slip command"
    annotation(Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));

  // Wrap-only line, at z = 0
  Roll2RollDynamics.Components.WebForce wrapInlet(direction = 1)
    "Incoming horizontal web under line tension, arriving under the master"
    annotation(Placement(transformation(origin = {-60, 20}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller wrapMaster(
    traction = Roll2RollDynamics.Utilities.Types.TractionFrictionParameters(
      muAdhesion = 1, muSliding = 0.8),
    useFlange = true, fixedInitialAngle = true, fixedInitialWebState = true,
    xPosition = 0, zPosition = 0, rotationDirection = 1,
    entryAngleStart = -Modelica.Constants.pi, exitAngleStart = 0)
    "Half-wrapped high-grip roller that sets the web speed"
    annotation(Placement(transformation(origin = {-40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Belt wrapSpan "Free span between the rollers"
    annotation(Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller wrapTest(
    useFlange = true, fixedInitialAngle = true, fixedInitialWebState = true,
    xPosition = world.spanLength, zPosition = 0, rotationDirection = 1,
    entryAngleStart = 0, exitAngleStart = Modelica.Constants.pi/2)
    "Quarter-wrapped roller driven through the slip sweep, held by its wrap alone"
    annotation(Placement(transformation(origin = {80, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.WebForce wrapOutlet(direction = -1)
    "Outgoing vertical web under line tension"
    annotation(Placement(transformation(origin = {80, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Speed wrapMasterDrive(exact = true)
    "Prescribed master surface speed"
    annotation(Placement(transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Speed wrapTestDrive(exact = true)
    "Prescribed test surface speed"
    annotation(Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}})));

  // Nip-loaded line, the same geometry at z = -0.5
  Roll2RollDynamics.Components.WebForce nipInlet(direction = 1)
    "Incoming horizontal web under line tension, arriving under the master"
    annotation(Placement(transformation(origin = {-60, -70}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller nipMaster(
    traction = Roll2RollDynamics.Utilities.Types.TractionFrictionParameters(
      muAdhesion = 1, muSliding = 0.8),
    useFlange = true, fixedInitialAngle = true, fixedInitialWebState = true,
    xPosition = 0, zPosition = -0.5, rotationDirection = 1,
    entryAngleStart = -Modelica.Constants.pi, exitAngleStart = 0)
    "Half-wrapped high-grip roller that sets the web speed"
    annotation(Placement(transformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Belt nipSpan "Free span between the rollers"
    annotation(Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller nipTest(
    useFlange = true, useNip = true, fixedInitialAngle = true, fixedInitialWebState = true,
    xPosition = world.spanLength, zPosition = -0.5, rotationDirection = 1,
    entryAngleStart = 0, exitAngleStart = Modelica.Constants.pi/2)
    "Quarter-wrapped roller driven through the slip sweep, with a nip on its wrap"
    annotation(Placement(transformation(origin = {80, -40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.NipRoller nip(
    xPosition = nipTest.xPosition + nipReach*sin(Modelica.Constants.pi/4),
    zPosition = nipTest.zPosition + nipReach*cos(Modelica.Constants.pi/4),
    maxNipLoad = 1000, loadFraction = nipLoad/1000,
    contactFace = if nipTest.contactFace == Roll2RollDynamics.Utilities.Types.WebFace.Inner
      then Roll2RollDynamics.Utilities.Types.WebFace.Outer
      else Roll2RollDynamics.Utilities.Types.WebFace.Inner,
    fixedInitialAngle = true, fixedInitialSpeed = true)
    "Fixed nip pressing on the middle of the test roller's wrap"
    annotation(Placement(transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  Roll2RollDynamics.Components.WebForce nipOutlet(direction = -1)
    "Outgoing vertical web under line tension"
    annotation(Placement(transformation(origin = {80, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Speed nipMasterDrive(exact = true)
    "Prescribed master surface speed"
    annotation(Placement(transformation(origin = {-70, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Speed nipTestDrive(exact = true)
    "Prescribed test surface speed"
    annotation(Placement(transformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}})));

equation
  wrapInlet.force = {world.tension, 0, 0};
  wrapOutlet.force = {0, 0, -world.tension};
  nipInlet.force = {world.tension, 0, 0};
  nipOutlet.force = {0, 0, -world.tension};
  wrapMasterDrive.w_ref = startup.y/(world.rollRadius + world.web.thickness/2);
  nipMasterDrive.w_ref = startup.y/(world.rollRadius + world.web.thickness/2);
  wrapTestDrive.w_ref = testSpeed.y/(world.rollRadius + world.web.thickness/2);
  nipTestDrive.w_ref = testSpeed.y/(world.rollRadius + world.web.thickness/2);
  connect(startup.y, testSpeed.u1)
    annotation(Line(points = {{-119, 0}, {-110, 0}, {-110, -14}, {-102, -14}}, color = {0, 0, 127}, origin = {-20, 20}));
  connect(sweep.y, testSpeed.u2)
    annotation(Line(points = {{-119, -40}, {-110, -40}, {-110, -26}, {-102, -26}}, color = {0, 0, 127}, origin = {-20, 20}));

  connect(wrapInlet.frame_b, wrapMaster.frame_a)
    annotation(Line(points = {{-6.667, -6.667}, {3.333, -6.667}, {3.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, origin = {-43.333, 26.667}));
  connect(wrapMaster.frame_b, wrapSpan.frame_a)
    annotation(Line(points = {{-16.667, -6.667}, {-16.667, 3.333}, {33.333, 3.333}}, color = {95, 95, 95}, thickness = 0.5, origin = {-23.333, 66.667}));
  connect(wrapSpan.frame_b, wrapTest.frame_a)
    annotation(Line(points = {{-33.333, 3.333}, {16.667, 3.333}, {16.667, -6.667}}, color = {95, 95, 95}, thickness = 0.5, origin = {63.333, 66.667}));
  connect(wrapTest.frame_b, wrapOutlet.frame_b)
    annotation(Line(points = {{0, 10}, {0, -10}}, color = {95, 95, 95}, thickness = 0.5, origin = {80, 30}));
  connect(wrapMasterDrive.flange, wrapMaster.flange_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, origin = {-50, 50}));
  connect(wrapTestDrive.flange, wrapTest.flange_a)
    annotation(Line(points = {{-13.333, -6.667}, {6.667, 13.333}}, origin = {73.333, 36.667}));

  connect(nipInlet.frame_b, nipMaster.frame_a)
    annotation(Line(points = {{-6.667, -6.667}, {3.333, -6.667}, {3.333, 13.333}}, color = {95, 95, 95}, thickness = 0.5, origin = {-43.333, -63.333}));
  connect(nipMaster.frame_b, nipSpan.frame_a)
    annotation(Line(points = {{-40, -30}, {-40, -20}, {10, -20}}, color = {95, 95, 95}, thickness = 0.5));
  connect(nipSpan.frame_b, nipTest.frame_a)
    annotation(Line(points = {{30, -20}, {80, -20}, {80, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(nipTest.frame_b, nipOutlet.frame_b)
    annotation(Line(points = {{80, -50}, {80, -70}, {80, -70}, {80, -70}}, color = {95, 95, 95}, thickness = 0.5));
  connect(nipTest.frame_nip, nip.nipLoading)
    annotation(Line(points = {{-4.717, 0}, {4.717, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {94.717, -40}));
  connect(nipMasterDrive.flange, nipMaster.flange_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, origin = {-50, -40}));
  connect(nipTestDrive.flange, nipTest.flange_a)
    annotation(Line(points = {{-13.333, -7.249}, {6.667, 12.751}}, origin = {73.333, -52.751}));
  assert(abs(wrapTraction) <= wrapCapacity + 1e-6
    and abs(totalTraction) <= totalCapacity + 1e-6,
    "Dry contact traction must stay within its available capacity");

  annotation(
    experiment(StartTime = 0, StopTime = 5, Interval = 0.002, Tolerance = 1e-8),
    Diagram(coordinateSystem(extent = {{-200, -100}, {140, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})),
    Documentation(figures = {Figure(
      title = "Traction and dry capacity", identifier = "overview", preferred = true,
      caption = "Traction and dry capacity versus measured slip, as in Figure 1. The annotation includes the full trajectory; the documentation omits startup before 1.2 s.",
      plots = {
        Plot(title = "Traction and dry capacity versus slip",
          x = Axis(label = "Measured web-to-roll slip", unit = "mm/s", min = -10, max = 10),
          y = Axis(label = "Traction on roll", unit = "N"),
          curves = {
            Curve(x = wrapSlip, y = wrapTraction, legend = "Wrap only: traction"),
            Curve(x = wrapSlip, y = wrapCapacity, legend = "Wrap only: dry capacity"),
            Curve(x = wrapSlip, y = negativeWrapCapacity, legend = ""),
            Curve(x = totalSlip, y = totalTraction, legend = "Wrap + nip: traction"),
            Curve(x = totalSlip, y = totalCapacity, legend = "Wrap + nip: dry capacity"),
            Curve(x = totalSlip, y = negativeTotalCapacity, legend = "")})})}, info = "<html>
<h4>Slip and total traction capacity</h4>
<p>Two 300 N web paths compare a quarter-wrapped test
<a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a>
with and without a 300 N
<a href=\"modelica://Roll2RollDynamics.Components.NipRoller\">NipRoller</a>.
A half-wrapped high-grip master drives each path; test rollers run at its
surface speed minus the slip command. After reaching 2 m/s in 0.5 s,
the command sweeps -10 to +10 mm/s over 1–5 s.
<code>world.web.EA = 2e6</code> N makes this mainly a slip change.
The curves exercise the contact law, not measured friction.</p>
<p>Figure 1 plots traction against measured slip. The quarter wrap provides
<code>2&middot;T&middot;tanh(&mu;&theta;/2)</code> = 155 N at &mu; = 0.35;
the nip adds <code>nipLoad&middot;&mu;</code> = 105 N. Traction vanishes at zero,
peaks at <code>traction.vAdhesion</code> and falls toward sliding friction.
Negative slip raises span tension to 600 N; positive slip reduces it to
160 N, making capacity asymmetric. Elastic strain also shifts the measured
zero crossing about 0.3 mm/s from the command.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/SlipAndTractionCapacity/traction-capacity.png\"
     width=\"600\"
     alt=\"Signed traction versus measured slip for the wrap-only and nip-loaded test rollers,
compared with their positive and negative dry capacity limits\"/>
<strong>Figure 1:</strong> Traction and dry capacity versus slip.</p>
<p>Set <code>nipLoad = 0</code> to make the curves coincide; raise
<code>world.tension</code> to increase both capacities. Restoring default
<code>world.web.EA</code> makes tension absorb most of the speed command.
Restoring <code>world.webDampingTime = 0.01</code> s produces about 25 Hz
stick-slip on the nip-loaded roller's falling friction branch.</p>
<h4>Limitations</h4>
<p>Dry capacity bounds traction only while
<code>traction.viscousSlope = 0</code>; a viscous term can exceed it at
large slip. Regularized friction requires finite slip, including at the
master. See <a href=\"modelica://Roll2RollDynamics.Utilities.Parts.WebFriction\">WebFriction</a>
for the law and <a href=\"modelica://Roll2RollDynamics.Examples.NipLoading\">NipLoading</a>
for engagement and release.</p>
</html>"
    ));
end SlipAndTractionCapacity;
