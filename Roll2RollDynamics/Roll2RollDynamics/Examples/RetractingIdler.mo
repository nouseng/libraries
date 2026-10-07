within Roll2RollDynamics.Examples;
model RetractingIdler
  "An idler withdrawn from an open web line and returned to it"
  extends Modelica.Icons.Example;

  inner Roll2RollDynamics.WebWorld world
    annotation(Placement(transformation(origin = {20, 20}, extent = {{-120, 60}, {-100, 80}})));

  Roll2RollDynamics.Components.Roller entryRoll(
    radius = 0.15,
    useFlange = true,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    xPosition = 0,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = -3.14,
    exitAngleStart = -1.01) "Driven roll at the upstream corner"
    annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Roller idler(
    radius = 0.12,
    useSupport = true,
    useDetachment = true,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    rotationDirection = 1,
    entryAngleStart = -1.01,
    exitAngleStart = 1.01) "Retractable idler at the apex"
    annotation(Placement(transformation(origin = {0, 50}, extent = {{-10, 10}, {10, -10}})));
  Roll2RollDynamics.Components.Roller exitRoll(
    radius = 0.09,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = 1.0,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = 1.01,
    exitAngleStart = 3.14) "Idling roll at the downstream corner"
    annotation(Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Roll2RollDynamics.Components.Belt risingSpan "Span from the driven roll up to the apex"
    annotation(Placement(transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt fallingSpan "Span from the apex down to the downstream roll"
    annotation(Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.WebForce upstreamLoad(direction = 1)
    "Incoming web held at the nominal line tension"
    annotation(Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.WebForce downstreamLoad(direction = -1)
    "Outgoing web held at the nominal line tension"
    annotation(Placement(transformation(origin = {90, -50}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
    w_fixed = world.lineSpeed/(entryRoll.radius + entryRoll.beltThickness/2))
    "Line speed master"
    annotation(Placement(transformation(origin = {-120, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation mount(
    r = {0.5, 0, 0.8}, animation = false)
    "Nominal apex position the stroke is measured from"
    annotation(Placement(transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic lift(
    n = {0, 0, -1},
    useAxisFlange = true,
    animation = false,
    s(start = 0, fixed = false),
    v(start = 0, fixed = false)) "Retraction freedom of the apex roll"
    annotation(Placement(transformation(origin = {-20, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Position stroke(exact = true)
    "Prescribed retraction stroke"
    annotation(Placement(transformation(origin = {30, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Trapezoid sweep(
    amplitude = 0.9,
    rising = 5,
    width = 5,
    falling = 5,
    period = 40,
    startTime = 5) "Out of the line and back, once"
    annotation(Placement(transformation(origin = {70, 90}, extent = {{10, -10}, {-10, 10}})));

  // Variables with Binding Equations
  output Modelica.Units.SI.Length clearance = idler.gap
    "How far the idler surface stands clear of the line; zero while engaged";
  output Modelica.Units.SI.Angle wrap = idler.rotationDirection*idler.rollerVariables.arcAngle
    "Wrap the idler still holds; zero while detached";
equation
  upstreamLoad.force = {world.tension, 0, 0};
  downstreamLoad.force = {-world.tension, 0, 0};
  connect(drive.flange, entryRoll.flange_a)
    annotation(Line(points = {{-20, 0}, {20, 0}}, origin = {-90, 0}));
  connect(world.frame_b, mount.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-70, 90}));
  connect(mount.frame_b, lift.frame_a)
    annotation(Line(points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-35, 90}));
  connect(lift.frame_b, idler.frame_support)
    annotation(Line(points = {{-6.667, 10}, {3.333, 10}, {3.333, -20}}, color = {95, 95, 95}, thickness = 0.5, origin = {-3.333, 80}));
  connect(stroke.flange, lift.axis)
    annotation(Line(points = {{10.5, -3}, {5.5, -3}, {5.5, 3}, {-21.5, 3}}, color = {0, 127, 0}, origin = {9.5, 93}));
  connect(sweep.y, stroke.s_ref)
    annotation(Line(points = {{8.5, 0}, {-8.5, 0}}, color = {0, 0, 127}, origin = {50.5, 90}));
  connect(upstreamLoad.frame_b, entryRoll.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-73.333, -36.667}, points = {{-6.667, -13.333}, {3.333, -13.333}, {3.333, 26.667}}));
  connect(entryRoll.frame_b, risingSpan.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-66.667, 36.667}, points = {{-3.333, -26.667}, {-3.333, 13.333}, {6.667, 13.333}}));
  connect(risingSpan.frame_b, idler.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-25, 50}, points = {{-15, 0}, {15, 0}}));
  connect(idler.frame_b, fallingSpan.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {25, 50}, points = {{-15, 0}, {15, 0}}));
  connect(fallingSpan.frame_b, exitRoll.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {66.667, 36.667}, points = {{-6.667, 13.333}, {3.333, 13.333}, {3.333, -26.667}}));
  connect(exitRoll.frame_b, downstreamLoad.frame_b)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {73.333, -36.667}, points = {{-3.333, 26.667}, {-3.333, -13.333}, {6.667, -13.333}}));

  annotation(
    experiment(StartTime = 0, StopTime = 25, Interval = 0.005, Tolerance = 1e-8),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -80}, {120, 115}}, initialScale = 0.1, grid = {10, 10})),
    Documentation(figures = {Figure(
      title = "Web release and return", identifier = "overview", preferred = true,
      caption = "Wrap and clearance use separate panels instead of the documentation image's two vertical axes",
      plots = {Plot(title = "Web release and return",
        x = Axis(label = "Time", unit = "s"), y = Axis(label = "Wrap held", unit = "rad"),
        curves = {
            Curve(x = time, y = wrap, legend = "Wrap held")}),
        Plot(title = "Roll clearance from the web path",
          x = Axis(label = "Time", unit = "s"), y = Axis(label = "Clearance", unit = "mm"),
          curves = {Curve(x = time, y = clearance, legend = "Clearance")})}),
      Figure(title = "Idler speed and span tension", identifier = "speedAndTension", preferred = true,
        plots = {
          Plot(title = "Idler speed through the stroke",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Idler surface speed", unit = "m/s"),
            curves = {Curve(x = time, y = idler.rollerVariables.surfaceVelocity, legend = "Idler surface speed")}),
          Plot(title = "Span tension through the stroke",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Span tension", unit = "N"),
            curves = {
              Curve(x = time, y = risingSpan.tension, legend = "Rising span"),
              Curve(x = time, y = fallingSpan.tension, legend = "Falling span")})})}, info = "<html>
<h4>Idler release and return</h4>
<p>A prismatic slide lowers the apex roll of a taut three-roll line.
With <code>idler.useDetachment = true</code>, the roll releases when it
clears the neighbouring spans' common tangent.</p>
<p>Figure 1 shows wrap falling to zero before clearance increases;
reversing the slide restores contact. See
<a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a>
for the release geometry.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/RetractingIdler/release-and-return.png\"
     width=\"600\"
     alt=\"Wrap falling to zero as the roll withdraws, clearance rising in its
place, and both returning as the roll re-enters\">
<strong>Figure 1:</strong> Wrap and clearance over the stroke.</p>
<p>Figure 2 shows idler speed and span tension through the slide's three
phases: retracting (5&ndash;10 s), held out (10&ndash;15 s) and returning
(15&ndash;20 s). The web leaves the roll at about 9.4 s and picks it up again
at about 15.6 s.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/RetractingIdler/speed-and-tension.png\"
     width=\"600\"
     alt=\"Idler surface speed and span tensions against time, with retracting, held and returning phases shaded and the idler coasting down while clear of the web\">
<strong>Figure 2:</strong> Idler speed and span tension through the stroke.</p>
<p>While clear, the idler coasts down from about 1.98 to 1.89 m/s. Tension
holds near 150&nbsp;N, with &plusmn;17&nbsp;N disturbances at release and
pick-up. The larger swings where the slide starts and stops come from its
prescribed acceleration. Set <code>sweep.amplitude</code> below 0.8 m to retain
contact; full retraction at 0.9 m gives about 0.0998 m clearance.</p>
<h4>Limitations</h4>
<p><a href=\"modelica://Roll2RollDynamics.Components.WebForce\">WebForce</a>
boundaries maintain tension as material leaves the shortening path.
Closing this line into a fixed-inventory loop would exhaust its elastic
stretch before release. Detachment requires a taut web; slack is unresolved.
The slide fixes housing orientation; a translating compliant support can
use the same release condition.</p>
</html>"));
end RetractingIdler;
