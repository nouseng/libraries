within Roll2RollDynamics.Examples;
model NipLoading
  "A moving nip closes on a running web, spins up, and releases"
  extends Roll2RollDynamics.Examples.ThreeRollWebLoop(topRoll(useNip = true));

  // Parameters
  final parameter Modelica.Units.SI.Mass initialWebMass(fixed=false)
    "Total web inventory at initialization, excluding roller shells";
  final parameter Modelica.Units.SI.Mass initialSpanWebMass(fixed=false)
    "Free-span web inventory at initialization";
  final parameter Modelica.Units.SI.Mass initialWrappedWebMass(fixed=false)
    "Roller-wrap web inventory at initialization";

  final parameter Real upperMassBalanceLimit(unit = "kg") = 1e-6
    "Upper conservation reference of one milligram";
  final parameter Real lowerMassBalanceLimit(unit = "kg") = -1e-6
    "Lower conservation reference of minus one milligram";

  // Variables with Binding Equations
  output Modelica.Units.SI.Mass spanWebMass =
    risingSpan.beltVariables.webMass + fallingSpan.beltVariables.webMass + returnSpan.beltVariables.webMass
    "Web material in the three free spans";
  output Modelica.Units.SI.Mass wrappedWebMass =
    driveRoll.webMass + topRoll.webMass + rightRoll.webMass
    "Web material in the three roller wraps";
  output Modelica.Units.SI.Mass totalWebMass = spanWebMass + wrappedWebMass
    "Complete closed-loop web inventory";
  output Real massChange(unit="kg") = totalWebMass - initialWebMass
    "Change in web inventory since initialization";
  output Real massBalanceError(unit="kg") = massChange
    "Closed-loop conservation residual, expected to remain zero";
  output Real spanMassChange(unit = "kg") = spanWebMass - initialSpanWebMass
    "Change in free-span web inventory since initialization";
  output Real wrappedMassChange(unit = "kg") = wrappedWebMass - initialWrappedWebMass
    "Change in roller-wrap web inventory since initialization";
  output Modelica.Units.SI.Force wrapTractionCapacity = topRoll.wrapCapacity
    "Dry traction capacity of the loaded roller's wrap acting alone";
  output Modelica.Units.SI.Velocity webSurfaceVelocity =
    (topRoll.webVelocityIn + topRoll.webVelocityOut)/2
    "Mean web velocity through the loaded roller";
  output Modelica.Units.SI.Velocity nipDrumSurfaceVelocity =
    nip.radius*nip.nipVariables.angularVelocity
    "Circumferential velocity of the nip drum";

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation loadingBase(
    r = {0.5, 0, 1.026}, animation = false)
    "Loading actuator base above the top roller"
    annotation(Placement(transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic loadingJoint(
    n = {0, 0, -1},
    useAxisFlange = true,
    animation = false,
    s(start = 0, fixed = false),
    v(start = 0, fixed = false)) "Vertical nip-loading slide"
    annotation(Placement(transformation(origin = {-40, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -360)));
  Modelica.Mechanics.Translational.Sources.Position loadingMotion(exact = true)
    "Prescribed close-and-release stroke"
    annotation(Placement(transformation(origin = {-50, 110}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Blocks.Sources.TimeTable loadingPosition(table = [
    0, 0;
    1, 0;
    2, 0.002;
    5, 0.002;
    6, 0;
    8, 0]) "Two millimetre nip stroke"
    annotation(Placement(transformation(origin = {-90, 110}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Roll2RollDynamics.Components.NipRoller nip(
    radius = 0.1,
    contactFace = if topRoll.contactFace ==
      Roll2RollDynamics.Utilities.Types.WebFace.Inner then
      Roll2RollDynamics.Utilities.Types.WebFace.Outer else
      Roll2RollDynamics.Utilities.Types.WebFace.Inner,
    coverStiffness = 1e6,
    coverDamping = 1000,
    maxNipLoad = 1000,
    useSupport = true,
    fixedInitialAngle = true,
    fixedInitialSpeed = true) "Undriven nip roller"
    annotation(Placement(transformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  output Modelica.Units.SI.Force nipCapacity =
    topRoll.rollerVariables.nipLoad*topRoll.traction.muAdhesion
    "Additional dry-adhesion capacity supplied by the nip";

initial equation
  initialWebMass = totalWebMass;
  initialSpanWebMass = spanWebMass;
  initialWrappedWebMass = wrappedWebMass;

equation
  assert(abs(massBalanceError) < 1e-6,
    "Closing and releasing the nip must preserve the closed-loop web mass");
  connect(world.frame_b, loadingBase.frame_a)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-90, 70}));
  connect(loadingBase.frame_b, loadingJoint.frame_a)
    annotation(Line(points = {{-5, -5}, {0, -5}, {0, 5}, {5, 5}}, color = {95, 95, 95}, thickness = 0.5, origin = {-55, 75}));
  connect(loadingJoint.frame_b, nip.frame_support)
    annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-20, 80}));
  connect(loadingPosition.y, loadingMotion.s_ref)
    annotation(Line(points = {{-8.5, 0}, {8.5, 0}}, color = {0, 0, 127}, origin = {-70.5, 110}));
  connect(loadingMotion.flange, loadingJoint.axis)
    annotation(Line(points = {{-9.5, 12}, {5.5, 12}, {5.5, -12}, {-1.5, -12}}, color = {0, 127, 0}, origin = {-30.5, 98}));
  connect(topRoll.frame_nip, nip.nipLoading)
    annotation(Line(points = {{-0, -4.717}, {0, 4.717}}, color = {95, 95, 95}, thickness = 0.5, origin = {0, 64.717}));

  when terminal() then
    assert(abs(topRoll.rollerVariables.nipLoad) < 1e-8,
      "The nip must be clear again at the end of the loading cycle");
    assert(abs(topRoll.rollerVariables.Fcap - topRoll.wrapCapacity) < 1e-8,
      "Released traction capacity must return to its wrap-only value");
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 8, Interval = 0.005, Tolerance = 1e-8),
    Diagram(coordinateSystem(preserveAspectRatio = false,
      extent = {{-140, -80}, {100, 130}}, initialScale = 0.1, grid = {10, 10})),
    Documentation(figures = {Figure(
      title = "Nip engagement and loading", identifier = "nipLoading", preferred = true,
      plots = {Plot(title = "Nip engagement and loading",
        x = Axis(label = "Time", unit = "s"), y = Axis(label = "Nip load", unit = "N"),
        curves = {
            Curve(x = time, y = nip.nipVariables.nipLoad, legend = "Nip load"),
            Curve(x = time, y = nip.maxNipLoad, legend = "Maximum actuator force")})}),
      Figure(title = "Nip traction capacity", identifier = "nipTractionCapacity", preferred = true,
        plots = {Plot(title = "Traction capacity at the loaded roll",
          x = Axis(label = "Time", unit = "s"), y = Axis(label = "Traction capacity", unit = "N"),
          curves = {
            Curve(x = time, y = topRoll.rollerVariables.Fcap, legend = "Traction capacity with the nip"),
            Curve(x = time, y = wrapTractionCapacity, legend = "The wrap acting alone")})}),
      Figure(title = "Nip spin-up and slip", identifier = "nipMotion", preferred = true,
        plots = {
          Plot(title = "Nip drum spin-up",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Surface velocity", unit = "m/s"),
            curves = {
              Curve(x = time, y = webSurfaceVelocity, legend = "Web surface velocity"),
              Curve(x = time, y = nipDrumSurfaceVelocity, legend = "Nip drum surface velocity"),
              Curve(x = time, y = nip.nipVariables.nipSlip, legend = "Nip slip")})}),
      Figure(title = "Closed-loop material conservation", identifier = "nipMaterialBalance", preferred = true,
        plots = {
          Plot(title = "Stored web mass change",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Inventory change", unit = "g"),
            curves = {
              Curve(x = time, y = spanMassChange, legend = "Free spans"),
              Curve(x = time, y = wrappedMassChange, legend = "Roller wraps")}),
          Plot(title = "Material balance residual",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Balance error", unit = "mg", min = -1.2, max = 1.2),
            curves = {
              Curve(x = time, y = massBalanceError, legend = ""),
              Curve(x = time, y = upperMassBalanceLimit, legend = ""),
              Curve(x = time, y = lowerMassBalanceLimit, legend = "")})})}, info = "<html>
<h4>Loading and releasing a nip</h4>
<p>An undriven <a href=\"modelica://Roll2RollDynamics.Components.NipRoller\">NipRoller</a>
adds traction to <a href=\"modelica://Roll2RollDynamics.Examples.ThreeRollWebLoop\">ThreeRollWebLoop</a>.
Enable <code>topRoll.useNip</code> and connect <code>nip.nipLoading</code> to
<code>topRoll.frame_nip</code>. Match the wrapped radius and rotation direction;
the nip contacts the opposite web face. Figure 1 shows the arrangement and
axis settings.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/nip-cases.svg\"
     width=\"900\"
     alt=\"Side view of the three-roll loop with the nip pressed down onto the top roller, and four plan views of the roller and nip axes: parallel, nip yawed, roller yawed, both yawed\">
<strong>Figure 1:</strong> Nip position on the loop and the axis settings of each case.</p>
<p>Figure 2 shows the load as the slide closes its initial 1 mm clearance
by 2 mm over 1–2 s, holds until 5 s and releases by 6 s. The 1 mm overlap
gives 1000 N, proportional to <code>nip.coverStiffness</code>; damping acts
during approach and release. The ideal position source has no force limit:
<code>nip.maxNipLoad</code> is an actuator-force reference, not a contact cap.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/nip-load.png\"
     width=\"600\"
     alt=\"Nip load rising to 1000 N, holding, and returning to zero, with the maximum actuator force shown for reference\">
<strong>Figure 2:</strong> Nip load over the loading stroke.</p>
<p>Figure 3 shows the added capacity
<code>nipLoad*traction.muAdhesion</code>: 350 N during the hold, zero after
release. The wrap contribution varies with tension and centrifugal unloading.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/traction-capacity.png\"
     width=\"600\"
     alt=\"Traction capacity increases while the nip is closed\">
<strong>Figure 3:</strong> Traction capacity with and without the nip.</p>
<p>Figure 4 shows drum spin-up on contact and coast-down under bearing drag.
Counter-rotation makes its surface velocity negative;
<code>nip.nipVariables.nipSlip</code> approaches zero but remains finite
to transmit the force overcoming drag.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/drum-spin-up.png\"
     width=\"600\"
     alt=\"Nip drum accelerating to line speed at touchdown and coasting after release\">
<strong>Figure 4:</strong> Nip drum spin-up and coast-down.</p>
<p>Figure 5 shows opposing inventory changes in spans and wraps. Their sum
stays constant in this closed loop; <code>massBalanceError</code> measures
numerical integration error. Web mass excludes roller shells and the nip drum.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/mass-conservation.png\"
     width=\"600\"
     alt=\"Opposing changes in free-span and wrapped web inventory during
nip loading, with the closed-loop mass balance residual below\"/>
<strong>Figure 5:</strong> Closed-loop material inventory and balance residual.</p>
<p>Figure 6 compares fixed misalignments at the same 1 mm middle overlap.
Crossed axes, <code>nip.yaw = -0.06</code> rad, lift both face edges by
0.89 mm through roller curvature, reducing the held load to 702 N.
With <code>nip.tram = 0.003</code> rad, one edge is 0.99 mm farther away:
full-face contact retains 1000 N but shifts its centre 109 mm toward the
closed end, which touches down earlier. Neither case varies with shaft angle.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/misalignment.png\"
     width=\"600\"
     alt=\"Steady held nip loads of the parallel, crossed and wedged nips over the same stroke, and the cross-machine load centre of each\">
<strong>Figure 6:</strong> Nip load and load centre with parallel, crossed and wedged axes.</p>
<p>Figure 7 uses <code>nip.runout = 0.3e-3</code> m. The orbit adds
&plusmn;0.3 mm to middle overlap, giving roughly 700–1300 N at the nip's
3.2 Hz turning rate, rather than the roller's 2.7 Hz. This amplitude follows
the cover law; no published nip-load measurement validates it.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/runout.png\"
     width=\"600\"
     alt=\"Nip load rippling once per nip turn above the steady load of a true drum, and the orbiting middle penetration behind it\">
<strong>Figure 7:</strong> Nip load and middle penetration with 0.3 mm drum runout.</p>
<p>Figure 8 enables <code>world.lateralDynamics</code> for the crossed nip,
closing at 1 s and releasing at 3.5 s. Its 0.12 m/s axial surface sweep
pulls with about 154 N, balanced by the aligned roller. Regularized friction
requires slip, so the web creeps about 0.22 mm toward the nip's sweep.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/steering.png\"
     width=\"600\"
     alt=\"Equal and opposite cross-machine forces of the crossed nip and the aligned roller on the web while the nip is loaded, and the resulting web creep underneath\">
<strong>Figure 8:</strong> Cross-machine forces and web offset with the crossed nip steering.</p>
<p>Figure 9 instead sets <code>topRoll.yaw = 0.06</code> rad. With no nip,
a machine-aligned nip or a nip yawed with the roller, normal-entry steering
gives nearly identical walk: 122.5–122.8 mm at 4 s. The roller's wrap plus
nip load gives it more traction than the crossed nip. A higher-grip cover,
<code>nip(traction = TractionFrictionParameters(muAdhesion = 0.8, muSliding = 0.6))</code>,
reverses the walk at about 10 mm/s with a 233 N pull while the roller slips
at 0.12 m/s. The reverse motion slows as the upstream span bends; normal
walk resumes on release. Replace the whole <code>traction</code> record:
a field modifier alone is ignored because it is bound from
<code>contactFace</code>. The touchdown spike is approach damping.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipLoading/drift.png\"
     width=\"600\"
     alt=\"Identical web walk of the yawed roller with no nip, a crossed nip and a stand-yawed nip, reversed by a crossed rubber nip, and the nip pulls of 154 N and about 233 N below\">
<strong>Figure 9:</strong> Web walk of the yawed roller with no nip, a crossed nip, a stand-yawed nip and a crossed rubber nip, with the nip's cross-machine force.</p>
<h4>Limitations</h4>
<p>Match the nip geometry to the wrapped roller. A force-limited actuator
must enforce its own limit, including approach damping. Finite face overlap
is unresolved; lateral drag is evaluated at mid-face.</p>
</html>"
    ));
end NipLoading;
