within Roll2RollDynamics.Examples.WebLines;
model KimTwoActuatorLine
  "Two-actuator winding line compared with Kim's published benchmark"
  extends Modelica.Icons.Example;

  // Parameters
  // Paper geometry and material, Kim [1], Section 2 and Section 5.
  parameter Modelica.Units.SI.Length webThickness = 0.18e-3 "PET film thickness";
  parameter Modelica.Units.SI.Length webWidth = 0.15 "PET film width";
  parameter Modelica.Units.SI.Radius idlerRadius = 0.03 "Idle roller radius, 60 mm diameter";
  parameter Modelica.Units.SI.Radius core = 0.048 "Winder core radius, 96 mm diameter";
  parameter Modelica.Units.SI.Radius unwindRadius0 = 0.083
    "Initial unwind radius, from the 166 mm full-film diameter";
  parameter Modelica.Units.SI.Radius rewindRadius0 = core + webThickness
    "Initial rewind radius, one wrap of film on the bare core";
  parameter Modelica.Units.SI.Force tensionSetpoint = 48
    "Reference tension the line starts from";
  // The changing-reference scenario of [1], Section 6 and Figure 11: hold 48 N,
  // fall to 28 N over 20 s, hold 10 s, rise to 38 N and hold for the rest of
  // the transport. Replace with two rows to run at one tension throughout.
  // Table columns: elapsed time (s), reference tension (N).
  parameter Real tensionProfile[:, 2] =
    [0, 48; 37, 48; 57, 28; 67, 28; 77, 38; transportLength/webSpeed, 38]
    "Reference tension against time, seconds and newtons";
  parameter Modelica.Units.SI.Velocity webSpeed = 0.1 "Reference web speed";
  parameter Modelica.Units.SI.Time startupTime = 2 "Speed ramp duration";
  parameter Modelica.Units.SI.Length transportLength = 65.7
    "Web length transported in the paper's full-transport run";

  // Inverted from the 0.085 N module tension difference [1] reports for the air
  // floating rollers, because no bearing coefficient is published:
  // moduleTensionDifference = 8*idlerBearingDamping*webSpeed/idlerRadius^2.
  parameter Modelica.Units.SI.RotationalDampingConstant idlerBearingDamping = 9.6e-5
    "Idler bearing damping inverted from the paper's module tension difference";

  // Assumed, because the paper prints no value for either.
  parameter Modelica.Units.SI.Stress youngModulus = 4e9
    "Assumed PET modulus; the paper gives only the product AE";
  parameter Modelica.Units.SI.Density webDensity = 1390
    "Assumed PET density; the paper prints no value";

  // Assumed, because the paper publishes only the Ziegler-Nichols formulas
  // and neither the ultimate gain nor the oscillation period.
  parameter Real controllerGain(unit = "rad/(s.N)") = 1.5e-3
    "Assumed tension loop proportional gain, newtons of error to rad/s of unwinder trim";
  parameter Modelica.Units.SI.Time controllerTi = 0.5 "Assumed tension loop integral time";
  parameter Modelica.Units.SI.Frequency velocityLoopBandwidth = 45
    "Inner velocity loop bandwidth of the full-film winder, Kim [1], Section 4";

  final parameter Modelica.Units.SI.Force positiveMeasuredBand = 0.3
    "Upper tracking-error band reported by Kim";
  final parameter Modelica.Units.SI.Force negativeMeasuredBand = -0.3
    "Lower tracking-error band reported by Kim";

protected
  // Parameters
  // Serpentine layout, following Examples.WebLines.MisalignedIdlerLine.
  final parameter Modelica.Units.SI.Angle WebWorldAngle = 0.6
    "Inclination of the spans between alternating rollers";
  final parameter Modelica.Units.SI.Length dx = 0.24
    "Machine-direction spacing of consecutive rollers";
  final parameter Modelica.Units.SI.Length zBelt = idlerRadius + webThickness/2
    "Belt centreline radius at an idler";
  final parameter Modelica.Units.SI.Length xW = zBelt*sin(WebWorldAngle)
    "Machine-direction offset of a tangency from the roller centre";
  final parameter Modelica.Units.SI.Length zW = zBelt*cos(WebWorldAngle)
    "Height of a tangency above the roller centre";
  final parameter Modelica.Units.SI.Length zDeflected =
    zW + (idlerRadius - sin(WebWorldAngle)*(dx - xW))/cos(WebWorldAngle)
    "Centre height of the deflected rollers";
  final parameter Real referenceDensity(unit = "kg/m") =
    webDensity*webThickness*webWidth "Unstrained mass per unit length";

public
  // Variables
  // Radius estimation of [1], Eq. (14) and (15). The controller never sees the
  // true radius: it integrates the reference web speed and inverts the packing
  // law, exactly as the published algorithm does.
  Modelica.Units.SI.Length estimatedLength(start = 0, fixed = true)
    "Web length the estimator believes has been transported";
  Modelica.Units.SI.Radius estimatedRewindRadius "Rewinder radius from Kim Eq. (15)";
  Modelica.Units.SI.Radius estimatedUnwindRadius "Unwinder radius from Kim Eq. (15)";

  // Variables with Binding Equations
  // Reported comparison quantities
  output Modelica.Units.SI.Length woundLength =
    rewindDrum.winderVariables.webMass/referenceDensity
    "Unstretched material length wound onto the rewinder";
  output Modelica.Units.SI.Radius rewindRadius = rewindDrum.radius
    "Rewinder radius the model derives from mass conservation";
  output Modelica.Units.SI.Length radiusResidual =
    rewindRadius - estimatedRewindRadius
    "Model radius minus the radius the published estimator believes";
  output Modelica.Units.SI.Velocity encoderWebSpeed =
    idler8.rollerVariables.surfaceVelocity
    "Web speed at the idler carrying the incremental rotary encoder";
  output Real webSpeedError(unit = "1") = encoderWebSpeed/webSpeed - 1
    "Fractional web speed error, against the paper's 1.6 percent at 0.1 m/s";
  output Modelica.Units.SI.Force unwinderLoadCell = span1.beltVariables.tension
    "Unwinder-module load cell, the tension fed back to the controller";
  output Modelica.Units.SI.Force rewinderLoadCell = span9.beltVariables.tension
    "Rewinder-module load cell, the tracking error of [1], Figure 9a";
  output Modelica.Units.SI.Force tensionError = rewinderLoadCell - tensionReference.y
    "Tracking error against the reference tension profile";
  output Modelica.Units.SI.Force moduleTensionDifference =
    rewinderLoadCell - unwinderLoadCell
    "Tension the eight idle rollers absorb, against [1], Figure 13b";
  output Modelica.Units.SI.Length totalFreeSpan =
    span1.beltVariables.freeLength + span2.beltVariables.freeLength
    + span3.beltVariables.freeLength + span4.beltVariables.freeLength
    + span5.beltVariables.freeLength + span6.beltVariables.freeLength
    + span7.beltVariables.freeLength + span8.beltVariables.freeLength
    + span9.beltVariables.freeLength
    "Total free span, against the paper's 2260 mm";

  // Components
  inner Roll2RollDynamics.WebWorld world(
    web(thickness = webThickness, width = webWidth, density = webDensity,
      EA = youngModulus*webThickness*webWidth),
    tension = tensionSetpoint,
    lineSpeed = webSpeed,
    rollRadius = idlerRadius,
    coreRadius = core,
    bearingDamping = idlerBearingDamping,
    spanLength = 2*dx)
    "Paper material and operating point"
    annotation(Placement(transformation(origin = {-10, -140}, extent = {{-250, 60}, {-230, 80}})));

  Roll2RollDynamics.Components.Winder unwindDrum(
    winding = -1,
    radius0 = unwindRadius0,
    xPosition = -dx,
    zPosition = zDeflected,
    rotationDirection = 1,
    fixedInitialWebState = true,
    fixedInitialAngle = true)
    "Unwinder, which carries the tension loop"
    annotation(Placement(transformation(origin = {-200, 0}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Winder rewindDrum(
    winding = 1,
    radius0 = rewindRadius0,
    xPosition = 8*dx,
    zPosition = 0,
    rotationDirection = 1,
    fixedInitialWebState = true,
    fixedInitialAngle = true)
    "Rewinder, which carries the velocity loop"
    annotation(Placement(transformation(origin = {210, 0}, extent = {{10, -10}, {-10, 10}})));

  // The eight idle rollers of the tension zone. Every even roller sits below
  // the machine line and counter-rotates, so each of the eight carries a real
  // wrap instead of grazing a straight web path.
  Roll2RollDynamics.Components.Roller idler1(
    xPosition = 0*dx, zPosition = 0, rotationDirection = 1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "First idle roller"
    annotation(Placement(transformation(origin = {-150, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler2(
    xPosition = 1*dx, zPosition = zDeflected, rotationDirection = -1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Second idle roller, deflected below the machine line"
    annotation(Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler3(
    xPosition = 2*dx, zPosition = 0, rotationDirection = 1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Third idle roller"
    annotation(Placement(transformation(origin = {-70, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler4(
    xPosition = 3*dx, zPosition = zDeflected, rotationDirection = -1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Fourth idle roller, deflected below the machine line"
    annotation(Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler5(
    xPosition = 4*dx, zPosition = 0, rotationDirection = 1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Fifth idle roller"
    annotation(Placement(transformation(origin = {10, 38.007}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler6(
    xPosition = 5*dx, zPosition = zDeflected, rotationDirection = -1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Sixth idle roller, deflected below the machine line"
    annotation(Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler7(
    xPosition = 6*dx, zPosition = 0, rotationDirection = 1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Seventh idle roller"
    annotation(Placement(transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller idler8(
    xPosition = 7*dx, zPosition = zDeflected, rotationDirection = -1,
    fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true)
    "Eighth idle roller, which carries the incremental rotary encoder"
    annotation(Placement(transformation(origin = {130, -0}, extent = {{-10, -10}, {10, 10}})));

  // The nine free spans, one more than the number of idle rollers.
  Roll2RollDynamics.Components.Belt span1
    "Unwinder to idler 1, the load cell of the unwinder module"
    annotation(Placement(transformation(origin = {-170, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Belt span2 "Idler 1 to idler 2"
    annotation(Placement(transformation(origin = {-130, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.Belt span3 "Idler 2 to idler 3"
    annotation(Placement(transformation(origin = {-90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Belt span4 "Idler 3 to idler 4"
    annotation(Placement(transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.Belt span5 "Idler 4 to idler 5"
    annotation(Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Belt span6 "Idler 5 to idler 6"
    annotation(Placement(transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.Belt span7 "Idler 6 to idler 7"
    annotation(Placement(transformation(origin = {70, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Belt span8 "Idler 7 to idler 8"
    annotation(Placement(transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.Belt span9
    "Idler 8 to rewinder, the load cell of the rewinder module"
    annotation(Placement(transformation(origin = {170, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));

  Modelica.Mechanics.Rotational.Sources.Speed rewindSpeedSource(exact = true)
    "Prescribed rewinder shaft speed, the velocity loop of [1]"
    annotation(Placement(transformation(origin = {227.168, -40}, extent = {{10, -10}, {-10, 10}})));
  // The unwinder command carries the tension feedback, so its inner loop is
  // given the paper's finite bandwidth rather than being followed exactly.
  Modelica.Mechanics.Rotational.Sources.Speed unwindSpeedSource(
    exact = false, f_crit = velocityLoopBandwidth)
    "Prescribed unwinder shaft speed, the inner loop of the unwinder cascade"
    annotation(Placement(transformation(origin = {-220, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp startup(height = webSpeed, duration = startupTime)
    "Common web-speed ramp"
    annotation(Placement(transformation(origin = {-250, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression rewindSpeedRef(y = startup.y/estimatedRewindRadius)
    "Rewinder command, the feedforward on the estimated radius"
    annotation(Placement(transformation(origin = {267.168, -40}, extent = {{10, -10}, {-10, 10}})));
  // Feedforward Vref/Ru of Eq. (13); the loop output subtracts because feeding
  // the unwinder faster slackens the web.
  Modelica.Blocks.Sources.RealExpression unwindSpeedRef(
    y = startup.y/estimatedUnwindRadius - controller.y)
    "Unwinder command, the same feedforward trimmed by the tension loop"
    annotation(Placement(transformation(origin = {-250, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.TimeTable tensionReference(table = tensionProfile, y(unit = "N"))
    "Reference tension profile"
    annotation(Placement(transformation(origin = {-140, 120}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Continuous.LimPID controller(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = controllerGain,
    Ti = controllerTi,
    yMax = 0.2*webSpeed/core,
    initType = Modelica.Blocks.Types.Init.InitialState,
    xi_start = 0) "Outer tension loop, trimming the unwinder speed command"
    annotation(Placement(transformation(origin = {-180, 120}, extent = {{10, -10}, {-10, 10}})));


equation
  der(estimatedLength) = startup.y;
  estimatedRewindRadius = sqrt(rewindRadius0^2 + webThickness*estimatedLength/Modelica.Constants.pi);
  estimatedUnwindRadius = sqrt(unwindRadius0^2 - webThickness*estimatedLength/Modelica.Constants.pi);

  connect(rewindSpeedRef.y, rewindSpeedSource.w_ref)
    annotation(Line(points = {{8.5, 0}, {-8.5, 0}}, color = {0, 0, 127}, origin = {247.668, -40}));
  connect(unwindSpeedRef.y, unwindSpeedSource.w_ref)
    annotation(Line(points = {{-239, 50}, {-220, 50}, {-220, 50}, {-232, 50}}, color = {0, 0, 127}));
  connect(tensionReference.y, controller.u_s)
    annotation(Line(points = {{-151, 120}, {-168, 120}}, color = {0, 0, 127}));
  connect(span1.tension, controller.u_m)
    annotation(Line(points = {{-176, 30}, {-180, 30}, {-180, 108}}, color = {0, 0, 127}));
  connect(unwindSpeedSource.flange, unwindDrum.flange_a)
    annotation(Line(points = {{-210, 50}, {-200, 50}, {-200, -10}}));
  connect(rewindSpeedSource.flange, rewindDrum.flange_a)
    annotation(Line(points = {{4.778, -10}, {-2.389, -10}, {-2.389, 20}}, origin = {212.389, -30}));

  connect(unwindDrum.frame_t, span1.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-176.667, 3.333}, points = {{-13.333, -3.333}, {6.667, -3.333}, {6.667, 6.667}}));
  connect(span1.frame_b, idler1.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-166.667, 36.667}, points = {{-3.333, -6.667}, {-3.333, 3.333}, {6.667, 3.333}}));
  connect(idler1.frame_b, span2.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-133.333, 36.667}, points = {{-6.667, 3.333}, {3.333, 3.333}, {3.333, -6.667}}));
  connect(span2.frame_b, idler2.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-126.667, 3.333}, points = {{-3.333, 6.667}, {-3.333, -3.333}, {6.667, -3.333}}));
  connect(idler2.frame_b, span3.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-93.333, 3.333}, points = {{-6.667, -3.333}, {3.333, -3.333}, {3.333, 6.667}}));
  connect(span3.frame_b, idler3.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-86.667, 36.667}, points = {{-3.333, -6.667}, {-3.333, 3.333}, {6.667, 3.333}}));
  connect(idler3.frame_b, span4.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-53.333, 36.667}, points = {{-6.667, 3.333}, {3.333, 3.333}, {3.333, -6.667}}));
  connect(span4.frame_b, idler4.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-46.667, 3.333}, points = {{-3.333, 6.667}, {-3.333, -3.333}, {6.667, -3.333}}));
  connect(idler4.frame_b, span5.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-13.333, 3.333}, points = {{-6.667, -3.333}, {3.333, -3.333}, {3.333, 6.667}}));
  connect(span5.frame_b, idler5.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {-6.667, 35.338}, points = {{-3.333, -5.338}, {-3.333, 2.669}, {6.667, 2.669}}));
  connect(idler5.frame_b, span6.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {26.667, 35.338}, points = {{-6.667, 2.669}, {3.333, 2.669}, {3.333, -5.338}}));
  connect(span6.frame_b, idler6.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {33.333, 3.333}, points = {{-3.333, 6.667}, {-3.333, -3.333}, {6.667, -3.333}}));
  connect(idler6.frame_b, span7.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {66.667, 3.333}, points = {{-6.667, -3.333}, {3.333, -3.333}, {3.333, 6.667}}));
  connect(span7.frame_b, idler7.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {73.333, 36.667}, points = {{-3.333, -6.667}, {-3.333, 3.333}, {6.667, 3.333}}));
  connect(idler7.frame_b, span8.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {106.667, 36.667}, points = {{-6.667, 3.333}, {3.333, 3.333}, {3.333, -6.667}}));
  connect(span8.frame_b, idler8.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {113.333, 3.333}, points = {{-3.333, 6.667}, {-3.333, -3.333}, {6.667, -3.333}}));
  connect(idler8.frame_b, span9.frame_a)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {150, -0}, points = {{-10, -0}, {10, 0}}));
  connect(span9.frame_b, rewindDrum.frame_t)
    annotation(Line(color = {0, 128, 180}, thickness = 0.5, origin = {190, 0}, points = {{-10, -0}, {10, 0}}));

  annotation(
    experiment(StartTime = 0, StopTime = 657, Interval = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-280, -140}, {280, 160}})),
    Documentation(figures = {
      Figure(title = "Load-cell tensions and commanded profile", identifier = "tensionProfile", preferred = true,
        caption = "Load-cell curves are library simulation results",
        plots = {Plot(title = "Tension profile",
          x = Axis(label = "Time", unit = "s", min = 20, max = 90), y = Axis(label = "Tension", unit = "N"),
          curves = {
            Curve(x = time, y = unwinderLoadCell, legend = "Unwinder load cell"),
            Curve(x = time, y = rewinderLoadCell, legend = "Rewinder load cell"),
            Curve(x = time, y = tensionReference.y, legend = "Reference")})}),
      Figure(title = "Tension tracking", identifier = "tensionTracking", preferred = true,
        plots = {
          Plot(title = "Rewinder tension minus commanded tension",
            x = Axis(label = "Time", unit = "s", min = 20, max = 90), y = Axis(label = "Tracking error", unit = "N"),
            curves = {
              Curve(x = time, y = tensionError, legend = "Rewinder module"),
              Curve(x = time, y = positiveMeasuredBand, legend = "Kim measured band"),
              Curve(x = time, y = negativeMeasuredBand, legend = "")}),
          Plot(title = "Rewinder tension minus unwinder tension",
            x = Axis(label = "Time", unit = "s", min = 20, max = 90), y = Axis(label = "Winder difference", unit = "N"),
            curves = {Curve(x = time, y = moduleTensionDifference, legend = "Module tension difference")})}),
      Figure(title = "Wound radius and Kim's radius estimate", identifier = "rewindRadius", preferred = true,
        caption = "Both radii are calculated quantities; separate panels show the radius and difference instead of the documentation image's two vertical axes",
        plots = {
          Plot(title = "Rewinder radius",
            x = Axis(label = "Time", unit = "s", min = 5, max = 657), y = Axis(label = "Rewinder radius", unit = "mm"),
            curves = {
              Curve(x = time, y = rewindRadius, legend = "Winder radius"),
              Curve(x = time, y = estimatedRewindRadius, legend = "Kim Eq. (15)")}),
          Plot(title = "Library radius minus Kim's estimate",
            x = Axis(label = "Time", unit = "s", min = 5, max = 657), y = Axis(label = "Difference", unit = "um"),
            curves = {Curve(x = time, y = radiusResidual, legend = "Difference")})})}, info = "<html>
<h4>Two-actuator winder-to-winder line</h4>
<p>This model compares Roll to Roll Dynamics with Kim et al. [1].
The unwinder controls tension and the rewinder sets speed through nine
spans and eight idlers. Both commands use radii estimated from the paper's
packing equations, not simulated wound radii. At 0.1 m/s, tension changes
for 90 s, then holds 38 N until 657 s. Machine settings map as follows.</p>
<table border=\"1\" cellpadding=\"4\">
<tr><th>Quantity</th><th>Published value</th><th>Library setting</th></tr>
<tr><td>Idle rollers</td><td>8, 60 mm diameter</td><td><code>idler1</code> to
<code>idler8</code>, <code>idlerRadius</code></td></tr>
<tr><td>Winder core</td><td>96 mm diameter</td><td><code>core</code></td></tr>
<tr><td>Full film diameter</td><td>About 166 mm</td><td><code>unwindRadius0</code></td></tr>
<tr><td>Web</td><td>PET, 0.18 mm by 150 mm</td><td><code>webThickness</code>,
<code>webWidth</code></td></tr>
<tr><td>Reference tension</td><td>38 N</td><td><code>tensionSetpoint</code>,
<code>tensionProfile</code></td></tr>
<tr><td>Web speed</td><td>0.1 m/s</td><td><code>webSpeed</code></td></tr>
<tr><td>Total free span</td><td>About 2260 mm</td><td><code>totalFreeSpan</code></td></tr>
</table>
<p>Figure 1 shows both simulated load-cell signals following
<code>tensionProfile</code>: 48, 28, then 38 N. They nearly coincide over
20–90 s. These follow the paper's command profile; they are not digitized
experimental traces.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/KimTwoActuatorLine/tension-profile.png\"
     width=\"600\"
     alt=\"Simulated load-cell tensions following the commanded tension profile\">
<strong>Figure 1:</strong> Simulated load-cell tensions and commanded profile.</p>
<p>Figure 2 places rewinder-minus-command error within the measured
&plusmn;0.3 N band [1, Figure 11b]. The lower panel shows
<code>moduleTensionDifference</code>, rewinder minus unwinder tension.
Bearing drag causes positive bias; reference-slope changes cause brief excursions.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/KimTwoActuatorLine/tension-tracking.png\"
     width=\"600\"
     alt=\"Tracking error inside the measured band over the winder tension difference\">
<strong>Figure 2:</strong> Tracking error and the winder tension difference.</p>
<p>Figure 3 compares material-balance <code>rewindRadius</code> with
<code>estimatedRewindRadius</code> from Kim's Eq. (15), differing by about
9 &micro;m at the end. Neither is measured: the estimator counts commanded
transport, while the winder accumulates elastically transported material.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/KimTwoActuatorLine/rewind-radius.png\"
     width=\"600\"
     alt=\"Library wound radius and Kim's calculated radius estimate, with their difference in micrometres\">
<strong>Figure 3:</strong> Library wound radius and Kim's radius estimate.</p>
<h4>Comparison with published measurements</h4>
<p>The table compares separate constant-tension runs with [1]. To repeat,
set <code>webSpeed</code>, <code>tensionSetpoint = 38</code>,
<code>tensionProfile = [0, 38; transportLength/webSpeed, 38]</code> and
stop time <code>transportLength/webSpeed</code>. Increasing <code>webSpeed</code>
raises the drag-induced tension error in these runs. Values average the
final two seconds unless labelled otherwise.</p>
<table border=\"1\" cellpadding=\"4\">
<tr><th>Quantity</th><th>0.1 m/s</th><th>0.2 m/s</th><th>0.3 m/s</th>
<th>Paper</th></tr>
<tr><td>Average web speed, first 2 s (m/s)</td><td>0.10000</td><td>0.20000</td>
<td>0.30000</td><td>0.100 / 0.200 / 0.300</td></tr>
<tr><td>Average web speed, last 2 s (m/s)</td><td>0.09999</td><td>0.19998</td>
<td>0.29997</td><td>0.1016 / 0.203 / 0.304</td></tr>
<tr><td>Web speed error</td><td>-0.009%</td><td>-0.009%</td><td>-0.009%</td>
<td>1.6% / 1.53% / 1.33%</td></tr>
<tr><td>Unwinder load cell (N)</td><td>37.999</td><td>37.998</td><td>37.995</td>
<td>38 held, Figure 9b</td></tr>
<tr><td>Rewinder tracking error (N)</td><td>+0.075</td><td>+0.158</td>
<td>+0.241</td><td>within +/-0.3, Figure 9a</td></tr>
<tr><td>Rewinder tracking error (%)</td><td>0.20%</td><td>0.42%</td>
<td>0.63%</td><td>0.79% / 1.32% / 1.58%</td></tr>
<tr><td>Module tension difference (N)</td><td>0.076</td><td>0.161</td>
<td>0.245</td><td>0.085 with air rollers</td></tr>
<tr><td>Wound length (m)</td><td>65.88</td><td>65.78</td><td>65.68</td>
<td>About 65.7</td></tr>
<tr><td>Rewinder radius, model (mm)</td><td>77.967</td><td>77.930</td>
<td>77.893</td><td>78.37 / 78.38 / 79.33 measured</td></tr>
<tr><td>Rewinder radius, Eq. (15) (mm)</td><td>77.974</td><td>77.937</td>
<td>77.900</td><td>78.224 / 79.19 / 78.42 estimated</td></tr>
<tr><td>Estimator against model radius</td><td>0.009%</td><td>0.009%</td>
<td>0.009%</td><td>0.186% / 1.03% / 1.15%</td></tr>
<tr><td>Total free span (mm)</td><td>2236</td><td>2236</td><td>2236</td>
<td>About 2260</td></tr>
</table>
<p>Simulated speed error is below the published error; radius-estimator
agreement does not establish experimental accuracy.
<code>idlerBearingDamping</code> is fitted to the reported 0.085 N module
tension difference, not independently validated.</p>
<h4>Assumptions and limitations</h4>
<p><code>youngModulus</code>, <code>webDensity</code>,
<code>controllerGain</code> and <code>controllerTi</code> are assumed.
Rewinder speed is ideal; the unwinder loop is a first-order lag with
<code>velocityLoopBandwidth = 45</code> Hz. Individual span lengths and
roller inertias are unreported; the layout approximates total free span.</p>
<p>Air-floating mechanics, measurement noise, drive and structural resonances
are unresolved. Wound packs retain free-film thickness, so measured pack-radius
differences are not reproduced.</p>
<h4>References</h4>
<ul><li>[<a href=\"https://doi.org/10.3390/s22082917\">1</a>]
J. Kim, K. Kim, H. Kim, P. Park, S. Lee, T. Lee and D. Kang, &quot;Experimental
Validation of High Precision Web Handling for a Two-Actuator-Based
Roll-to-Roll System&quot;, Sensors 22(8), 2917, 2022. Section 2 gives the
machine, Section 3 the web and winder dynamics, Eq. (15) the packing law and
Section 6 the measurements.
<a href=\"https://doi.org/10.3390/s22082917\">Source</a>.
Accessed 2026-09-07.</li></ul>
</html>"));
end KimTwoActuatorLine;
