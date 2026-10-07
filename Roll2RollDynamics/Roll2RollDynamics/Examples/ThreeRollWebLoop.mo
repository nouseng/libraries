within Roll2RollDynamics.Examples;
model ThreeRollWebLoop
  "Web circulating around three rolls of different sizes, with no winder"
  extends Modelica.Icons.Example;

  inner Roll2RollDynamics.WebWorld world
    annotation(Placement(transformation(extent = {{-120, 60}, {-100, 80}})));

  Roll2RollDynamics.Components.Roller driveRoll(
    radius = 0.15,
    useFlange = true,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    xPosition = 0,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = -3.14,
    exitAngleStart = -1.01) "Largest roll, driven"
    annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Roller topRoll(
    radius = 0.12,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = 0.5,
    zPosition = 0.8,
    rotationDirection = 1,
    entryAngleStart = -1.01,
    exitAngleStart = 1.01) "Middle-sized idling roll at the apex"
    annotation(Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller rightRoll(
    radius = 0.09,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = 1.0,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = 1.01,
    exitAngleStart = 3.14) "Smallest idling roll"
    annotation(Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Roll2RollDynamics.Components.Belt risingSpan "Span from the driven roll up to the apex"
    annotation(Placement(transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt fallingSpan "Span from the apex down to the small roll"
    annotation(Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt returnSpan "Span closing the loop along the base"
    annotation(Placement(transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
    w_fixed = world.lineSpeed/(driveRoll.radius+driveRoll.beltThickness/2)) "Loop speed master"
    annotation(Placement(transformation(origin = {-120, -0}, extent = {{-10, -10}, {10, 10}})));

  // Variables with Binding Equations
  output Modelica.Units.SI.Velocity topIdlerSpeedDifference =
    topRoll.rollerVariables.surfaceVelocity - driveRoll.rollerVariables.surfaceVelocity
    "Upper idler surface velocity minus drive surface velocity";
  output Modelica.Units.SI.Velocity rightIdlerSpeedDifference =
    rightRoll.rollerVariables.surfaceVelocity - driveRoll.rollerVariables.surfaceVelocity
    "Right idler surface velocity minus drive surface velocity";
  output Modelica.Units.SI.Angle wrapSum = driveRoll.rotationDirection*driveRoll.rollerVariables.arcAngle
    + topRoll.rotationDirection*topRoll.rollerVariables.arcAngle
    + rightRoll.rotationDirection*rightRoll.rollerVariables.arcAngle
    "Total turning round the closed path";
  output Real tractionUsed(unit = "1") = abs(driveRoll.rollerVariables.Ft)/
    max(driveRoll.rollerVariables.Fcap, Roll2RollDynamics.Utilities.Types.forceTol)
    "Fraction of the driven wrap's available traction the loop is drawing on";
equation
  connect(drive.flange, driveRoll.flange_a)
    annotation(Line(points = {{-20, -0}, {20, 0}}, origin = {-90, -0}));
  connect(driveRoll.frame_b, risingSpan.frame_a)
    annotation(Line(points = {{-3.333, -26.667}, {-3.333, 13.333}, {6.667, 13.333}}, color = {0, 128, 180}, thickness = 0.5, origin = {-66.667, 36.667}));
  connect(risingSpan.frame_b, topRoll.frame_a)
    annotation(Line(points = {{-15, 0}, {15, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-25, 50}));
  connect(topRoll.frame_b, fallingSpan.frame_a)
    annotation(Line(points = {{-15, 0}, {15, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {25, 50}));
  connect(fallingSpan.frame_b, rightRoll.frame_a)
    annotation(Line(points = {{-6.667, 13.333}, {3.333, 13.333}, {3.333, -26.667}}, color = {0, 128, 180}, thickness = 0.5, origin = {66.667, 36.667}));
  connect(rightRoll.frame_b, returnSpan.frame_a)
    annotation(Line(points = {{70, -10}, {70, -60}, {-10, -60}}, color = {0, 128, 180}, thickness = 0.5));
  connect(returnSpan.frame_b, driveRoll.frame_a)
    annotation(Line(points = {{10, -60}, {-70, -60}, {-70, -10}}, color = {0, 128, 180}, thickness = 0.5));

  when terminal() then
    // A yawed roll takes its wrap out of the loop plane by O(yaw^2)
    assert(abs(wrapSum - 2*Modelica.Constants.pi)
      < 1e-6 + driveRoll.yaw^2 + topRoll.yaw^2 + rightRoll.yaw^2,
      "The wraps of a closed loop must add to a full turn");
    assert(tractionUsed < 1,
      "The driven roll must not be asked for more traction than its wrap can hold");
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 25, Interval = 0.005),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -80}, {100, 100}})),
    Documentation(figures = {Figure(title = "Idler speed differences and contact slip",
        identifier = "idlerSpeeds", preferred = true,
        caption = "Same quantities as Figure 2, on linear axes; the documentation uses symmetric logarithmic axes to show startup and small steady differences together.",
        plots = {
          Plot(title = "Idler surface velocity minus drive velocity",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Idler − drive", unit = "mm/s"),
            curves = {
              Curve(x = time, y = topIdlerSpeedDifference, legend = "Upper idler"),
              Curve(x = time, y = rightIdlerSpeedDifference, legend = "Right idler")}),
          Plot(title = "Web velocity minus idler surface velocity",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Web − idler", unit = "mm/s"),
            curves = {
              Curve(x = time, y = topRoll.rollerVariables.slipVelocity, legend = "Upper idler"),
              Curve(x = time, y = rightRoll.rollerVariables.slipVelocity, legend = "Right idler")})}),
      Figure(title = "Closed-loop span tensions", identifier = "overview", preferred = true,
        caption = "Span tensions from Figure 3; the late-run zoom remains in the documentation image's inset.",
        plots = {
          Plot(title = "Closed-loop span tensions",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Span tension", unit = "N"),
            curves = {
              Curve(x = time, y = risingSpan.beltVariables.tension, legend = "Rising span"),
              Curve(x = time, y = fallingSpan.beltVariables.tension, legend = "Falling span"),
              Curve(x = time, y = returnSpan.beltVariables.tension, legend = "Return span")})})}, info = "<html>
<h4>Tangency around a closed web path</h4>
<p>A driven roll pulls two idlers around a closed three-span loop.
Initial tension sets material inventory. This demonstrates tangency and
startup dynamics with chosen parameters, not a published experiment.</p>
<p>Figure 1 shows solved tangencies and wraps summing to <code>2*pi</code>.
Change a roller radius or position while retaining clearance and a taut,
convex path: individual wraps change but their sum remains one turn.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/ThreeRollWebLoop/loop-geometry.png\"
     width=\"600\"
     alt=\"Three rolls of unequal radius joined by three tangent spans, wraps summing to one turn\">
<strong>Figure 1:</strong> Solved three-roll web path.</p>
<p>Figure 2 separates idler-minus-drive surface speed (top) from
web-minus-idler contact slip (bottom, <code>rollerVariables.slipVelocity</code>).
Symmetric logarithmic axes show startup and small final differences in mm/s;
the drive starts at 2 m/s, the idlers at rest. At 25 s the upper idler is
5.33 mm/s faster than the drive but slips only 0.00082 mm/s. Gravity stretches
the soft 5 mm belt (<code>world.web.EA = 8000</code> N), increasing local
speed for the same material flow. With <code>world.g = 0</code>, the final
speed difference drops to about 0.032 mm/s in magnitude.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/ThreeRollWebLoop/surface-agreement.png\"
     width=\"600\"
     alt=\"Signed idler-minus-drive surface speed differences above actual
web-minus-idler contact slip, both in millimetres per second\">
<strong>Figure 2:</strong> Idler speed differences and web-to-roll contact slip.</p>
<p>Figure 3 shows span tensions transmitting idler acceleration and bearing
drag [2], with wrapped-web momentum also contributing. The late-run inset
still contains transients, not a static drag balance. Increase
<code>world.bearingDamping</code> to explore the tension distribution.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/ThreeRollWebLoop/loop-tensions.png\"
     width=\"600\"
     alt=\"Three span tensions during startup and the late-run transient\">
<strong>Figure 3:</strong> The three span tensions, with the final five seconds enlarged.</p>
<p>Figure 4 uses <code>topRoll.runout = 0.3e-3</code> m. Its 2.6 Hz orbit
changes adjacent span lengths [3], but the closed inventory and fixed drive
speed distribute tension ripple through all three spans: 1.6, 2.2 and 3.2 N,
largest in the return span. Adding 0.2 mm runout on the right idler introduces
3.4 Hz and beating. The kinematic <code>ConstantSpeed</code> source prevents
driven-roll runout from translating; use torque or speed control for an
eccentric driven roll.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/ThreeRollWebLoop/loop-runout.png\"
     width=\"600\"
     alt=\"Settled span tensions rippling once per turn with an eccentric top idler, and beating with both idlers eccentric\">
<strong>Figure 4:</strong> Span tensions with an eccentric top idler and with both idlers eccentric.</p>
<p><code>tractionUsed</code> is driven-roll force divided by dry capacity,
including centrifugal unloading. For Shigley's belt relations [1] and a
capstan-bound comparison, see
<a href=\"modelica://Roll2RollDynamics.Examples.OpenBeltDrive\">OpenBeltDrive</a>.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://dl.icdst.org/pdfs/files3/ad7608c18e740b0e402c025fa3187de8.pdf\">1</a>]
R. G. Budynas and J. K. Nisbett, &quot;Shigley&apos;s Mechanical Engineering
Design&quot;, 10th ed., McGraw-Hill Education, 2015, sec. 17&ndash;2.
Eq. (17&ndash;1) and the discussion on p. 875 cover two-pulley open-belt
contact angles and elastic creep. Ex. 17&ndash;2, p. 886, compares required
and available friction coefficients; <code>tractionUsed</code> instead
reports a force-to-capacity ratio. Accessed 2026-09-08.</li>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/321814\">2</a>]
D. P. Jones, &quot;Traction in Web Handling: A Review&quot;, Proc.
Sixth International Conference on Web Handling, Oklahoma State University,
2001, pp. 187&ndash;210. &quot;Web on a Roller: Basic Mechanics&quot;,
p. 190, explains tension differences, transmitted torque, elastic microslip
and idlers with bearing friction. Accessed 2026-09-08.</li>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/321969\">3</a>]
C. Branca, P. R. Pagilla and K. N. Reid, &quot;Web Tension Behavior in the
Presence of Eccentric Rollers: Modeling and Validation&quot;, Proc.
International Conference on Web Handling, Oklahoma State University, 2011.
Eq. (10) gives the span beside an eccentric roller as a function of its
rotation and Eq. (23) the fundamental frequency of the tension disturbance at
the roller turning rate, which is the rate the three spans ripple at here.
Accessed 2026-09-15.
<a href=\"https://hdl.handle.net/20.500.14446/321969\">Source</a>.</li>

</ul>
</html>"));
end ThreeRollWebLoop;
