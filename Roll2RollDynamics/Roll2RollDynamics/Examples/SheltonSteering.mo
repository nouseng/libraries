within Roll2RollDynamics.Examples;
model SheltonSteering "Normal-entry steering compared with Shelton's idealized theory"
  extends Modelica.Icons.Example;

  // Parameters
  parameter Modelica.Units.SI.Length spanLength(min=Modelica.Constants.small) = 1
    "Aligned horizontal free span length";
  parameter Modelica.Units.SI.Velocity lineSpeed(min=Modelica.Constants.small) = 2
    "Running roller surface speed";
  parameter Modelica.Units.SI.Angle yaw = 0.004
    "Downstream roll yaw toward positive lateral displacement";
  parameter Modelica.Units.SI.Time startupTime(min=Modelica.Constants.small) = 0.2
    "Surface-speed ramp duration";
  final parameter Modelica.Units.SI.Time steeringTime = spanLength/lineSpeed
    "Ideal running-speed time constant L/V";
  final parameter Modelica.Units.SI.Mass initialWebMass(fixed=false)
    "Initial inventory in the span and both wraps";

  final parameter Real positiveMassTolerance(unit = "kg") = 1e-6
    "Upper one-milligram material balance reference";
  final parameter Real negativeMassTolerance(unit = "kg") = -1e-6
    "Lower one-milligram material balance reference";
  final parameter Modelica.Units.SI.Position zeroTrackingError = 0
    "Zero lateral tracking error reference";

  // Variables
  output Modelica.Units.SI.Position referencePosition(start=0, fixed=true)
    "Independent first-order prediction driven by upstream position and speed command";
  output Real netBoundaryMass(unit="kg", start=0, fixed=true)
    "Accumulated external inlet mass minus outlet mass";

  // Variables with Binding Equations
  output Modelica.Units.SI.Position upstreamPosition = span.frame_a.frame.r_0[2]
    "Upstream tangency position in world y";
  output Modelica.Units.SI.Position downstreamPosition = span.frame_b.frame.r_0[2]
    "Downstream tangency position in world y";
  output Modelica.Units.SI.Position relativeOffset = downstreamPosition - upstreamPosition
    "Measured lateral offset across the free span";
  output Modelica.Units.SI.Position referenceOffset = referencePosition - upstreamPosition
    "Predicted offset using the measured upstream position as an input";
  output Modelica.Units.SI.Position trackingError = downstreamPosition - referencePosition
    "Roll to Roll Dynamics downstream position minus Shelton analytical prediction";
  output Modelica.Units.SI.Position normalEntryOffset = spanLength*yaw
    "Small-angle relative offset for ideal steady normal entry";
  output Modelica.Units.SI.Mass totalWebMass =
    span.beltVariables.webMass + entryRoll.webMass + exitRoll.webMass
    "Material inventory in the free span and both wraps";
  output Real massChange(unit="kg") = totalWebMass - initialWebMass
    "Change in span and wrap inventory since initialization";
  output Real massBalanceError(unit="kg") =
    massChange - netBoundaryMass
    "Inventory change minus accumulated boundary flow";

  // Components
  inner Roll2RollDynamics.WebWorld world(
    web(thickness=1e-4, width=0.1, density=1400, EA=30000),
    tension=20, lineSpeed=lineSpeed, spanLength=spanLength, rollRadius=0.05,
    lateralDynamics=true, bearingDamping=0, g=0, nominalLength=0.3)
    "Chosen thin-web transport settings for the kinematic benchmark"
    annotation(Placement(transformation(extent={{-110,60},{-90,80}})));
  Roll2RollDynamics.Components.Roller entryRoll(
    useFlange=true, fixedInitialAngle=true, fixedInitialWebState=true,
    fixedInitialSpeed=false, xPosition=0, zPosition=0,
    rotationDirection=1, entryAngleStart=-Modelica.Constants.pi/2,
    exitAngleStart=0)
    "Aligned driven roller with a quarter wrap"
    annotation(Placement(transformation(origin={-60,0}, extent={{-10,-10},{10,10}})));
  Roll2RollDynamics.Components.Roller exitRoll(
    useFlange=true, fixedInitialAngle=true, fixedInitialWebState=true,
    fixedInitialSpeed=false, xPosition=spanLength, zPosition=0,
    rotationDirection=1, yaw=yaw,
    entryAngleStart=0, exitAngleStart=Modelica.Constants.pi/2)
    "Yawed driven roller with a quarter wrap"
    annotation(Placement(transformation(origin={60,0}, extent={{-10,-10},{10,10}})));
  Roll2RollDynamics.Components.Belt span
    "Existing axial belt with its default Kelvin-Voigt damping"
    annotation(Placement(transformation(extent={{-10,-10},{10,10}})));
  Roll2RollDynamics.Components.WebForce upstreamLoad(direction=1)
    "Incoming vertical web under constant tension"
    annotation(Placement(transformation(origin={-100,30}, extent={{-10,-10},{10,10}})));
  Roll2RollDynamics.Components.WebForce downstreamLoad(direction=-1)
    "Outgoing vertical web under constant tension"
    annotation(Placement(transformation(origin={100,30}, extent={{10,-10},{-10,10}})));
  Modelica.Mechanics.Rotational.Sources.Speed entryDrive(exact=true)
    "Prescribed entry surface speed"
    annotation(Placement(transformation(origin={-80,-40}, extent={{-10,-10},{10,10}})));
  Modelica.Mechanics.Rotational.Sources.Speed exitDrive(exact=true)
    "Prescribed downstream surface speed"
    annotation(Placement(transformation(origin={80,-40}, extent={{10,-10},{-10,10}})));
  Modelica.Blocks.Sources.Ramp startup(height=lineSpeed, duration=startupTime)
    "Common surface-speed ramp from rest"
    annotation(Placement(transformation(origin={0,-70}, extent={{-10,-10},{10,10}})));

initial equation
  initialWebMass = totalWebMass;
  assert(spanLength > 2*world.rollRadius,
    "Roller shells must fit within the selected centre spacing");
  assert(abs(yaw) < 0.02,
    "The analytical comparison requires small roller yaw");

equation
  // Driven transport with equal end loads.
  upstreamLoad.force = {0, 0, -world.tension};
  downstreamLoad.force = {0, 0, -world.tension};
  entryDrive.w_ref = startup.y/(world.rollRadius + world.web.thickness/2);
  exitDrive.w_ref = startup.y/(world.rollRadius + world.web.thickness/2);

  // Shelton's small-angle prediction does not constrain the physical web.
  der(referencePosition) = startup.y*(yaw
    - (referencePosition - upstreamPosition)/spanLength);
  der(netBoundaryMass) = entryRoll.massFlowIn - exitRoll.massFlowOut;
  assert(abs(massBalanceError) < 1e-6,
    "Web inventory must reconcile with net external mass flow");

  connect(upstreamLoad.frame_b, entryRoll.frame_a)
    annotation(Line(points={{-90,30},{-70,30},{-70,0}}, color={95,95,95}, thickness=0.5));
  connect(entryRoll.frame_b, span.frame_a)
    annotation(Line(points={{-50,0},{-10,0}}, color={0,128,180}, thickness=0.5));
  connect(span.frame_b, exitRoll.frame_a)
    annotation(Line(points={{10,0},{50,0}}, color={0,128,180}, thickness=0.5));
  connect(exitRoll.frame_b, downstreamLoad.frame_b)
    annotation(Line(points={{70,0},{70,30},{90,30}}, color={95,95,95}, thickness=0.5));
  connect(entryDrive.flange, entryRoll.flange_a)
    annotation(Line(points={{-70,-40},{-60,-40},{-60,-10}}));
  connect(exitDrive.flange, exitRoll.flange_a)
    annotation(Line(points={{70,-40},{60,-40},{60,-10}}));

  annotation(experiment(StopTime = 8, Interval = 0.005, Tolerance = 1e-9),
    Diagram(coordinateSystem(extent={{-120,-90},{120,90}})),
    Documentation(figures = {Figure(
      title = "Normal-entry steering", identifier = "overview", preferred = true,
      plots = {Plot(title = "Normal-entry steering",
        x = Axis(label = "Time", unit = "s"), y = Axis(label = "Relative offset", unit = "mm"),
        curves = {
            Curve(x = time, y = referenceOffset, legend = "Shelton analytical model"),
            Curve(x = time, y = relativeOffset, legend = "Roll to Roll Dynamics")}),
        Plot(title = "Library minus Shelton analytical response",
          x = Axis(label = "Time", unit = "s"), y = Axis(label = "Library minus Shelton", unit = "um"),
          curves = {
            Curve(x = time, y = trackingError, legend = "Library minus Shelton"),
            Curve(x = time, y = zeroTrackingError, legend = "")})}),
      Figure(title = "Web material conservation", identifier = "materialBalance", preferred = true,
        plots = {
          Plot(title = "Stored web mass change and boundary flow",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Inventory change", unit = "mg"),
            curves = {
              Curve(x = time, y = massChange, legend = "Inventory change"),
              Curve(x = time, y = netBoundaryMass, legend = "Accumulated inlet − outlet")}),
          Plot(title = "Material balance residual",
            x = Axis(label = "Time", unit = "s"), y = Axis(label = "Balance error", unit = "mg", min = -1.2, max = 1.2),
            curves = {
              Curve(x = time, y = massBalanceError, legend = "Stored change minus inflow"),
              Curve(x = time, y = positiveMassTolerance, legend = ""),
              Curve(x = time, y = negativeMassTolerance, legend = "")})})}, info = "<html>
<h4>Normal-entry steering</h4>
<p>This example compares Roll to Roll Dynamics with Shelton's small-angle
steering law [1]. Two driven rollers carry a horizontal span, with fixed
downstream yaw. The default 1 m span, 2 m/s speed and 0.004 rad yaw give
an ideal 4 mm relative offset and time constant <code>L/V = 0.5</code> s.</p>
<p>Figure 1 compares <code>relativeOffset</code> with Shelton's calculated
<code>referenceOffset</code>; <code>trackingError</code> is library minus
Shelton. Both offsets use the drifting upstream contact because no lateral
guide fixes it. The independent reference uses that simulated contact and
the commanded speed, including its 0.2 s ramp; it neither constrains the
model nor represents measured data.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/SheltonSteering/comparison.svg\"
     width=\"600\"
     alt=\"Roll to Roll Dynamics simulation in blue markers and Shelton's analytical model in orange, with the library-minus-Shelton difference below\">
<strong>Figure 1:</strong> Roll to Roll Dynamics and Shelton's analytical model.</p>
<p>Halve <code>yaw</code> to halve the ideal offset, or double
<code>spanLength</code> to double offset and time constant. Keep yaw small
and transport forward.</p>
<p>Figure 2 reconciles inventory change in the span and both wraps with
accumulated inlet minus outlet flow. <code>massBalanceError</code> is the
remaining material-balance residual.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/SheltonSteering/mass-conservation.png\"
     width=\"600\"
     alt=\"Web inventory change compared with accumulated boundary mass flow during steering, with the balance residual below\">
<strong>Figure 2:</strong> Open-path material inventory and balance residual.</p>
<p>Figure 3 compares <code>offset = yaw*spanLength</code> with nine measured
PET-web cases [2]: 0.01–0.03&deg; yaw at 13.2, 26.5 and 36.7 N, on
150 mm wide, 75 &micro;m film. The unreported span length is fitted once
as 0.32 m, so this tests angle linearity and tension independence rather
than absolute offset. Points lie within &plusmn;10%; tension-group means
are 0.90, 0.98 and 1.01 times the library value.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/SheltonSteering/measured-yaw.png\"
     width=\"600\"
     alt=\"Measured web-edge displacement against imposed yaw at three tensions with the library line through the origin, and the ratio of measured to library against tension\">
<strong>Figure 3:</strong> Measured yaw response [2] against the library's normal-entry law.</p>
<p>Figure 4 compares the first-order law with Shelton's second-order
Eq. (4.2.7) and measured amplitude/phase points digitized from Figure 4.7.4
[1]. His oscillated-upstream, fixed-downstream tests use polystyrene at
KL = 2 and 10. Against KL = 2, the library stays within 10% and 10&deg;
up to &omega;T<sub>1</sub> = 0.56, where T<sub>1</sub> = L/V.
Above that it under-predicts amplitude and limits phase lag to 90&deg;
the web reaches 150&deg;. Faster weave needs span bending absent from
<code>Belt</code>.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/SheltonSteering/frequency-response.png\"
     width=\"600\"
     alt=\"Amplitude ratio and phase of downstream lateral position against omega times span transit time: the library's first-order curve, Shelton's second-order theory for KL of 2 and 10, and his measured points\">
<strong>Figure 4:</strong> Lateral frequency response: library, Shelton's second-order theory and his measurements [1], Figure 4.7.4.</p>
<h4>Limitations</h4>
<p>Benchmark dimensions and material properties are chosen; gravity and
bearing drag are disabled. Shelton neglects shear and inertia; the library
retains finite slip, mass, axial elasticity and damping. Agreement checks
slow steering kinematics, not friction, bending or wrinkling.</p>
<h4>References</h4>
<ul><li>[<a href=\"https://hdl.handle.net/20.500.14446/30409\">1</a>]
J. J. Shelton, &quot;Lateral Dynamics of a Moving Web&quot;, PhD thesis,
Oklahoma State University, 1968, Chapter III, printed pp. 75-79,
eqs. (3.1.1), (3.2.1)-(3.2.4). Idealized normal-entry kinematics and L/V
response time; Section 4.7 and Figure 4.7.4 (printed p. 140), measured
frequency response with the second-order theory of Eq. (4.2.7), f<sub>1</sub>
and f<sub>2</sub> from Eqs. (4.1.13) and (4.1.24). Accessed 2026-09-07.
<a href=\"https://hdl.handle.net/20.500.14446/30409\">Source</a>.</li>
<li>[<a href=\"https://doi.org/10.3390/polym17212907\">2</a>]
J. Yun, S. Lee, J. Jang, M. Kim, C. Kim and C. Lee, &quot;Sensor-Efficient Estimation of Roll Misalignment via
Side-to-Side Tension Differences in Roll-to-Roll Polymer Film
Processing&quot;, Polymers 17(21):2907, 2025. Tables 1, 2 and 4: material,
the nine yaw and tension cases, and the measured web-edge displacements.
Open access, accessed 2026-09-16.</li></ul>
</html>"));
end SheltonSteering;
