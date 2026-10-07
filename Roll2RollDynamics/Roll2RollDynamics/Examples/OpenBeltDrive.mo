within Roll2RollDynamics.Examples;
model OpenBeltDrive
  "Open belt drive with unequal pulleys"
  extends Modelica.Icons.Example;

  // Parameters
  final parameter Modelica.Units.SI.Angle fullTurn = 2*Modelica.Constants.pi
    "One complete turn for the wrap-angle reference";

  inner Roll2RollDynamics.WebWorld world
    annotation(Placement(transformation(extent = {{-120, 60}, {-100, 80}})));

  Roll2RollDynamics.Components.Roller bigRoll(
    radius = 0.15,
    useFlange = true,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    xPosition = 0,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = 2.85,
    exitAngleStart = 5.99) "Driven pulley, wrapped by more than half a turn"
    annotation(Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Roller smallRoll(
    radius = 0.09,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = 1.0,
    zPosition = 0.3,
    rotationDirection = 1,
    entryAngleStart = -0.29,
    exitAngleStart = 2.85) "Idling pulley, wrapped by less than half a turn"
    annotation(Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Roll2RollDynamics.Components.Belt upperSpan "Span above the line of centres"
    annotation(Placement(transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt lowerSpan "Span below the line of centres"
    annotation(Placement(transformation(origin = {0, -40}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
    w_fixed = world.lineSpeed/(bigRoll.radius+bigRoll.beltThickness/2)) "Drive speed"
    annotation(Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  output Modelica.Units.SI.Angle largePulleyWrap = abs(bigRoll.rollerVariables.arcAngle)
    "Wrap angle on the large pulley";
  output Modelica.Units.SI.Angle smallPulleyWrap = abs(smallRoll.rollerVariables.arcAngle)
    "Wrap angle on the small pulley";
  output Modelica.Units.SI.Angle wrapSum = bigRoll.rotationDirection*bigRoll.rollerVariables.arcAngle
    + smallRoll.rotationDirection*smallRoll.rollerVariables.arcAngle "Total turning round the loop";
  output Real tensionRatio(unit = "1") =
    (max(smallRoll.rollerVariables.tensionIn, smallRoll.rollerVariables.tensionOut) - smallRoll.centrifugalTension)/
    max(min(smallRoll.rollerVariables.tensionIn, smallRoll.rollerVariables.tensionOut) - smallRoll.centrifugalTension, Roll2RollDynamics.Utilities.Types.forceTol)
    "Ratio of tight/slack tensions after centrifugal correction";
  output Real capstanLimit(unit = "1") =
    exp(smallRoll.traction.muAdhesion*abs(smallRoll.rollerVariables.arcAngle))
    "Largest ratio the smaller wrap can hold, from the capstan equation";
  output Modelica.Units.SI.Force initialTension =
    (smallRoll.rollerVariables.tensionIn + smallRoll.rollerVariables.tensionOut)/2 - smallRoll.centrifugalTension
    "Shigley initial tension F_i: mean end tension less centrifugal tension";
  output Modelica.Units.SI.Force transmittedForce =
    abs(smallRoll.rollerVariables.tensionIn - smallRoll.rollerVariables.tensionOut)
    "Net traction force carried by the smaller wrap";
  output Modelica.Units.SI.Force transmissibleForce =
    2*initialTension*tanh(smallRoll.traction.muAdhesion*abs(smallRoll.rollerVariables.arcAngle)/2)
    "Largest force the smaller wrap can transmit at the present initial tension";
equation
  connect(drive.flange, bigRoll.flange_a)
    annotation(Line(points = {{-20, 0}, {20, 0}}, origin = {-70, 0}));
  connect(bigRoll.frame_b, upperSpan.frame_a)
    annotation(Line(points = {{-13.333, -20}, {-13.333, 10}, {26.667, 10}}, color = {0, 128, 180}, thickness = 0.5, origin = {-36.667, 30}));
  connect(upperSpan.frame_b, smallRoll.frame_a)
    annotation(Line(points = {{-26.667, 10}, {13.333, 10}, {13.333, -20}}, color = {0, 128, 180}, thickness = 0.5, origin = {36.667, 30}));
  connect(smallRoll.frame_b, lowerSpan.frame_a)
    annotation(Line(points = {{70, -10}, {70, -10}, {70, -40}, {30, -40}}, color = {0, 128, 180}, thickness = 0.5, origin = {-20, 0}));
  connect(lowerSpan.frame_b, bigRoll.frame_a)
    annotation(Line(points = {{26.667, -10}, {-13.333, -10}, {-13.333, 20}}, color = {0, 128, 180}, thickness = 0.5, origin = {-36.667, -30}));

  when terminal() then
    assert(tensionRatio < capstanLimit,
      "The drive must run inside the capstan limit of its smaller wrap");
    assert(abs(wrapSum - 2*Modelica.Constants.pi) < 1e-6,
      "The two wraps of a two-pulley loop must add to a full turn");
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 5, Interval = 0.005),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -80}, {100, 100}})),
    Documentation(figures = {Figure(
      title = "Wrap angles", identifier = "wrapAngles", preferred = true,
      plots = {Plot(title = "Wrap angles and their sum",
        x = Axis(label = "Time", unit = "s"), y = Axis(label = "Wrap angle", unit = "rad"),
        curves = {
          Curve(x = time, y = largePulleyWrap, legend = "Large pulley"),
          Curve(x = time, y = smallPulleyWrap, legend = "Small pulley"),
          Curve(x = time, y = wrapSum, legend = "Total wrap"),
          Curve(x = time, y = fullTurn, legend = "Full turn")})}),
      Figure(
      title = "Capstan traction margin", identifier = "overview", preferred = true,
      plots = {Plot(title = "Capstan traction margin",
        x = Axis(label = "Time", unit = "s"), y = Axis(label = "Tension ratio", unit = "1"),
        curves = {
            Curve(x = time, y = tensionRatio, legend = "Tight-to-slack tension ratio"),
            Curve(x = time, y = capstanLimit, legend = "Quasi-static capstan limit")})}),
      Figure(title = "Roll surface velocities", identifier = "surfaceVelocities", preferred = true,
        plots = {Plot(title = "Roll surface velocities",
          x = Axis(label = "Time", unit = "s"), y = Axis(label = "Surface velocity", unit = "m/s"),
          curves = {
            Curve(x = time, y = bigRoll.rollerVariables.surfaceVelocity, legend = "Large pulley"),
            Curve(x = time, y = smallRoll.rollerVariables.surfaceVelocity, legend = "Small pulley")})})}, info = "<html>
<h4>Open belt drive with unequal pulleys</h4>
<p>This example compares common-tangent geometry and capstan traction in
a two-pulley drive. The large pulley runs at 2 m/s; web traction accelerates
the smaller one.</p>
<p>Figure 1 shows wraps of 3.25660 and 3.02659 rad, matching
<code>pi &plusmn; 2*asin((r1 - r2)/D)</code> [1], where <code>D</code> is
centre distance. Their sum is one turn. Increase <code>bigRoll.radius</code>,
retaining clearance, to increase its wrap and reduce the smaller wrap.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/OpenBeltDrive/wrap-angles.png\"
     width=\"600\"
     alt=\"Wrap angles of both pulleys and their sum against two pi\">
<strong>Figure 1:</strong> Wrap angles and their sum.</p>
<p>Figure 2 compares the tension ratio with Euler's capstan bound [1, 2]:</p>
<blockquote><pre>
(T_tight - T_c) / (T_slack - T_c) &lt;= exp(mu*beta)
F_i = (T_tight + T_slack)/2 - T_c
T_tight - T_slack &lt;= 2*F_i*tanh(mu*beta/2)
</pre></blockquote>
<p>The smaller wrap limits the ratio to 2.88437 at <code>mu = 0.35</code>.
Centrifugal tension is <code>T_c = (webMass/arcLength)*vWeb^2</code>;
<code>F_i</code> is Shigley's effective initial tension [1], equal to
<code>world.tension</code> at rest and following the end tensions thereafter.
Outputs <code>initialTension</code>,
<code>transmittedForce</code> and <code>transmissibleForce</code> report the
preload, tension difference and bound. Compare late-run values: wrapped-web
inertia also affects startup tension differences.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/OpenBeltDrive/capstan-limit.png\"
     width=\"600\"
     alt=\"Transient tension ratio against the quasi-static capstan limit\">
<strong>Figure 2:</strong> Tension ratio against the capstan limit.</p>
<p>Figure 3 shows the smaller pulley approaching the driven 2 m/s surface
speed without an imposed steady-speed constraint. Their difference includes
elastic draw and contact slip.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/OpenBeltDrive/surface-speeds.png\"
     width=\"600\"
     alt=\"Driven pulley surface speed and the smaller pulley's startup response\">
<strong>Figure 3:</strong> Surface velocities of both pulleys.</p>
<p>Figure 4 raises <code>world.tension</code> from 75 to 150 and 300 N.
Greater frictional capacity steepens acceleration: 2 m/s is reached at
0.55, 0.29 and 0.16 s, while overshoot rises from 2.07 to 2.30 m/s.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/OpenBeltDrive/tension-sweep.png\"
     width=\"600\"
     alt=\"Small pulley surface velocity for three initial tensions\">
<strong>Figure 4:</strong> Small pulley startup for three initial tensions.</p>
<p>Figure 5 varies <code>world.webDampingTime</code> from 0.001 to 0.01
and 0.1 s. Initial ramps coincide; damping suppresses subsequent ringing.
At 0.001 s the speed remains outside &plusmn;2% until 2.2 s and the tension
ratio peaks at 2.15, below the 2.88 bound. At 0.1 s the approach is nearly
critically damped. Change <code>dampingTime</code> in the model: its equation
branch cannot be changed by a run-time override.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/OpenBeltDrive/damping-sweep.png\"
     width=\"600\"
     alt=\"Small pulley surface velocity for three web damping times\">
<strong>Figure 5:</strong> Small pulley startup for three web damping times.</p>
<p>Figure 6 raises <code>world.bearingDamping</code> from 0.001 to
0.1 N&middot;m&middot;s/rad. The driven pulley pulls <code>lowerSpan</code>,
making the smaller pulley's exit tight and entry slack. At speed,
<code>bearingDamping*w = (T_tight - T_slack)*r</code> gives tension splits
of 0.2 and 23 N at 21.6 rad/s; the steady ratio rises from 1.00 to 1.18.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/OpenBeltDrive/bearing-sweep.png\"
     width=\"600\"
     alt=\"Tight and slack side tensions at the small pulley for two bearing drags\">
<strong>Figure 6:</strong> Tight and slack side tensions for two bearing drags.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://dl.icdst.org/pdfs/files3/ad7608c18e740b0e402c025fa3187de8.pdf\">1</a>]
R. G. Budynas and J. K. Nisbett, &quot;Shigley&apos;s Mechanical Engineering
Design&quot;, 10th ed. in SI units, McGraw-Hill, 2015, sec. 17&ndash;2.
Eq. (17&ndash;1), p. 875, gives the unequal-pulley wraps, Eq. (17&ndash;7),
p. 877, gives the tension ratio and Eq. (17&ndash;13), p. 878, gives the
initial tension. Ex. 17&ndash;1, pp. 882&ndash;883, works both
through for a 6 in / 18 in flat-belt drive. Accessed 2026-09-04.</li>
<li>[<a href=\"https://scholarlycommons.pacific.edu/euler-works/382/\">2</a>]
L. Euler, &quot;Remarques sur l&apos;effet du frottement dans
l&apos;equilibre&quot;, Memoires de l&apos;Academie des Sciences de Berlin,
Vol. 18, published 1769, pp. 265&ndash;278 (Enestrom E382). Original source of
the capstan relation. Accessed 2026-09-04.</li>
</ul>
</html>"));
end OpenBeltDrive;
