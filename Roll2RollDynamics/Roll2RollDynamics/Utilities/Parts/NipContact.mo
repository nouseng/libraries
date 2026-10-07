within Roll2RollDynamics.Utilities.Parts;
model NipContact
  "Compliant friction contact between a rotating nip and wrapped web"

  // Parameters
  parameter Boolean enabled = true "Enable contact forces and diagnostics";
  parameter Modelica.Units.SI.Radius nipRadius(min = Modelica.Constants.small)
    "Mechanical radius of the nip roller";
  parameter Modelica.Units.SI.Length webThickness(min = 0)
    "Web thickness at the nip";
  parameter Modelica.Units.SI.TranslationalSpringConstant coverStiffness(
    min = Modelica.Constants.small) "Full-width nip-cover stiffness";
  parameter Modelica.Units.SI.TranslationalDampingConstant coverDamping(
    min = 0) = 0 "Nip cover damping";
  parameter Modelica.Units.SI.Force maxNipLoad(
    min = Modelica.Constants.small) "Maximum actuator force";
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters traction
    "Nip-to-web Triple-S friction characteristic";
  parameter Modelica.Units.SI.Length nipWidth(min = Modelica.Constants.small)
    "Nip roller face width over which the cover law is integrated";
  parameter Boolean prescribedLoad = false
    "= true, set the middle penetration from loadFraction instead of centre separation";
  parameter Real loadFraction(unit = "1", min = 0, max = 1) = 0.5
    "Nominal preload as a fraction of maximum actuator force when prescribedLoad is true";
  parameter Boolean lateral = false
    "= true, friction also acts on cross-machine slip through nipPort.lateral";

  // Variables
  Modelica.Units.SI.Radius rollRadius(min = Modelica.Constants.small)
    "Mechanical radius of the wrapped roller, read from the nip port";
  Integer rotationDirection(min = -1, max = 1)
    "Signed web travel direction of the wrapped roller, read from the nip port";
  Modelica.Units.SI.Position relativePosition[3]
    "Nip-centre position relative to the roller centre in world coordinates";
  Modelica.Units.SI.Position radialPosition[3]
    "Centre separation projected perpendicular to the roller axis";
  Modelica.Units.SI.Distance radialDistance
    "Distance between the roll axes in their radial plane";
  Real rollerAxis[3](each unit = "1")
    "Wrapped-roller axis resolved in world coordinates";
  Real nipAxis[3](each unit = "1")
    "Nip-roll axis resolved in world coordinates";
  Real normalDirection[3](each unit = "1")
    "Contact normal from the wrapped roller toward the nip";
  Real webDirection[3](each unit = "1")
    "Local direction of positive web travel at the nip";
  Modelica.Units.SI.Velocity relativeCentreVelocity[3]
    "Nip-centre velocity relative to the roller centre";
  Modelica.Units.SI.Velocity penetrationRate
    "Rate at which the two undeformed surfaces approach at the middle";
  Modelica.Units.SI.Force elasticForce
    "Elastic contribution to normal load";
  Modelica.Units.SI.Force dampingForce
    "Viscous contribution to normal load";
  Real axisAlignment(unit = "1")
    "+1 when the two roll axes point the same way, -1 when opposed";
  Real axisCross[3](each unit = "1") "Cross product of the two roll axes";
  Real crossedSkewSin(unit = "1")
    "Sine of the axis crossing about the contact normal";
  Real crossedSlope(unit = "1")
    "Tangential offset of the nip axis per unit cross-machine distance";
  Real wedgeSlope(unit = "1")
    "Signed opening of the nip gap per unit cross-machine distance";
  Real gapCurvature(unit = "1/m")
    "Quadratic coefficient of the gap across the face from the crossed axes";
  Real gapDiscriminant(unit = "1")
    "Discriminant of the gap quadratic; negative when no point of the face touches";
  Modelica.Units.SI.Position contactStart
    "Cross-machine position where contact begins along the roller axis";
  Modelica.Units.SI.Position contactEnd
    "Cross-machine position where contact ends along the roller axis";
  Modelica.Units.SI.Force elasticFootprint
    "Uncapped elastic load integrated over the contacting part of the face";
  Modelica.Units.SI.AngularVelocity nipSpinRate
    "Nip angular velocity about its own axis";
  Modelica.Units.SI.Velocity slipMagnitude
    "Regularized magnitude of the nip-to-web slip the friction law is evaluated on";
  Real frictionCoefficient(unit = "1")
    "Triple-S friction coefficient at the slip magnitude";
  Modelica.Units.SI.Force nipForce[3]
    "Contact force acting on the nip resolved in world coordinates";
  Modelica.Units.SI.Torque normalMoment[3]
    "Moment of the off-centre normal load about the nip centre";

  // Diagnostic outputs
  output Modelica.Units.SI.Angle relativeMisalignment
    "Acute angle between the roll axes, independent of signed edge offsets";
  output Modelica.Units.SI.Angle wedgeAngle
    "Part of the relative misalignment that opens one end of the nip";
  output Modelica.Units.SI.Length edgeLift
    "Gap the crossed axes open at both face edges relative to the middle";
  output Modelica.Units.SI.Length endOpening
    "Gap the wedge opens at the lifted face edge relative to the middle";
  output Modelica.Units.SI.Length contactWidth
    "Cross-machine width of the face in contact";
  output Modelica.Units.SI.Length loadCentre
    "Cross-machine position of the normal load resultant along the roller axis";
  output Modelica.Units.SI.Force normalForce "Compressive nip load";
  output Modelica.Units.SI.Force tangentialForce
    "Signed tangential force acting on the web";
  output Modelica.Units.SI.Length penetration
    "Undeformed overlap of the two surfaces at the middle of the face";
  output Modelica.Units.SI.Velocity slipVelocity
    "Nip surface velocity relative to the web";
  output Modelica.Units.SI.Velocity lateralSlip
    "Nip surface velocity relative to the web along the wrapped roller axis";
  output Modelica.Units.SI.Force lateralForce
    "Signed cross-machine force acting on the web along the wrapped roller axis";
  output Boolean engaged "True while any part of the face is in contact";
  output Modelica.Units.SI.Power frictionLoss
    "Power dissipated by nip-to-web slip";

  // Physical connectors
  Roll2RollDynamics.Utilities.Interfaces.NipPort nipPort
    "Wrapped-roller centre and web transport"
    annotation(Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameNip
    "Rotating centre frame of the nip roller"
    annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));

protected
  // Parameters
  final parameter Modelica.Units.SI.Length halfWidth = nipWidth/2
    "Half the face width, the edge position along the roller axis";
  final parameter Real coverStiffnessPerWidth(unit = "N/m2") = coverStiffness/nipWidth
    "Cover stiffness per unit face width";

equation
  // Wrapped roller geometry published on the nip port
  rollRadius = nipPort.geometry.radius;
  rotationDirection = integer(nipPort.geometry.rotationDirection);
  assert(abs(rotationDirection) == 1,
    "NipContact: rotationDirection must be -1 or +1");

  // Contact geometry in world coordinates
  relativePosition = frameNip.r_0 - nipPort.frame.r_0;
  rollerAxis = Modelica.Mechanics.MultiBody.Frames.resolve1(
    nipPort.frame.R, {0, 1, 0});
  nipAxis = Modelica.Mechanics.MultiBody.Frames.resolve1(
    frameNip.R, {0, 1, 0});
  radialPosition = relativePosition
    - rollerAxis*(rollerAxis*relativePosition);
  radialDistance = sqrt(radialPosition*radialPosition);
  normalDirection = radialPosition/noEvent(max(radialDistance,
    Roll2RollDynamics.Utilities.Types.lengthTol));
  webDirection = rotationDirection*cross(rollerAxis, normalDirection);
  axisAlignment = rollerAxis*nipAxis;
  axisCross = cross(rollerAxis, nipAxis);
  relativeMisalignment = Modelica.Math.atan2(
    sqrt(axisCross*axisCross), abs(axisAlignment));
  // Split the crossing: about the normal the axes cross (edge lift),
  // in the normal plane one end of the nip opens (wedge)
  crossedSkewSin = axisCross*normalDirection;
  wedgeAngle = Modelica.Math.atan2(
    sqrt(max(0, axisCross*axisCross - crossedSkewSin^2)), abs(axisAlignment));
  assert(not enabled or abs(axisAlignment) > 0.5,
    "NipContact: the roll axes must stay within 60 degrees of parallel");
  crossedSlope = crossedSkewSin/axisAlignment;
  wedgeSlope = nipAxis*normalDirection/axisAlignment;

  assert(not enabled or radialDistance >
    Roll2RollDynamics.Utilities.Types.lengthTol,
    "NipContact: roll centres must not coincide");

  // Undeformed overlap at the face middle
  nipPort.shaft.tau = 0;
  nipPort.geometry.radiusFlow = 0;
  nipPort.geometry.rotationDirectionFlow = 0;
  nipPort.geometry.lateralDriftFlow = 0;
  // Nominal preload sets the parallel-axis compression; skew changes the footprint
  penetration = if not enabled then 0
    elseif prescribedLoad then
      loadFraction*maxNipLoad/coverStiffness
    else rollRadius + nipRadius + webThickness - radialDistance;
  penetrationRate = der(penetration);

  // Cover law integrated across the face. The gap
  //   gap(s) = penetration - gapCurvature*s^2 - wedgeSlope*s
  // is concave in s, so the face touches on one interval between its roots,
  // clipped to the face edges; load and moment are its polynomial integrals.
  gapCurvature = crossedSlope^2/(2*(rollRadius + nipRadius));
  edgeLift = gapCurvature*halfWidth^2;
  endOpening = halfWidth*abs(wedgeSlope);
  gapDiscriminant = wedgeSlope^2 + 4*gapCurvature*penetration;
  contactStart = noEvent(
    if gapCurvature*halfWidth^2 > Roll2RollDynamics.Utilities.Types.lengthTol then
      (if gapDiscriminant < 0 then 0 else
        max(-halfWidth, (-wedgeSlope - sqrt(gapDiscriminant))/(2*gapCurvature)))
    elseif abs(wedgeSlope)*halfWidth > Roll2RollDynamics.Utilities.Types.lengthTol then
      (if wedgeSlope > 0 then -halfWidth else
        max(-halfWidth, min(halfWidth, penetration/wedgeSlope)))
    else -halfWidth);
  contactEnd = noEvent(
    if gapCurvature*halfWidth^2 > Roll2RollDynamics.Utilities.Types.lengthTol then
      (if gapDiscriminant < 0 then 0 else
        min(halfWidth, (-wedgeSlope + sqrt(gapDiscriminant))/(2*gapCurvature)))
    elseif abs(wedgeSlope)*halfWidth > Roll2RollDynamics.Utilities.Types.lengthTol then
      (if wedgeSlope > 0 then
        min(halfWidth, max(-halfWidth, penetration/wedgeSlope)) else halfWidth)
    else (if penetration > 0 then halfWidth else -halfWidth));
  contactWidth = noEvent(max(0, contactEnd - contactStart));
  elasticFootprint = noEvent(if contactWidth > 0 then coverStiffnessPerWidth*(
    penetration*(contactEnd - contactStart)
    - gapCurvature*(contactEnd^3 - contactStart^3)/3
    - wedgeSlope*(contactEnd^2 - contactStart^2)/2) else 0);
  // At first touch the moment and load both vanish; fall back to the
  // midpoint of the touching interval
  loadCentre = noEvent(if elasticFootprint > coverStiffness*Roll2RollDynamics.Utilities.Types.lengthTol
    then coverStiffnessPerWidth*(
      penetration*(contactEnd^2 - contactStart^2)/2
      - gapCurvature*(contactEnd^4 - contactStart^4)/4
      - wedgeSlope*(contactEnd^3 - contactStart^3)/3)/elasticFootprint
    else (contactStart + contactEnd)/2);
  elasticForce = if enabled then elasticFootprint else 0;
  dampingForce = if enabled then
    coverDamping*penetrationRate*contactWidth/nipWidth else 0;
  normalForce = noEvent(max(0, elasticForce + dampingForce));
  relativeCentreVelocity = der(frameNip.r_0) - der(nipPort.frame.r_0);

  nipSpinRate = Modelica.Mechanics.MultiBody.Frames.angularVelocity1(
    frameNip.R)*nipAxis;
  slipVelocity = if enabled then
    relativeCentreVelocity*webDirection
      - rotationDirection*axisAlignment*nipRadius*nipSpinRate
      - der(nipPort.web.s) else 0;
  // A cocked nip surface moves along the roller axis at
  // -nipRadius*nipSpinRate*crossedSkewSin. The web material moves at the
  // contact-line speed less the roller's normal-entry drift, which is
  // rolling across the roller, not sliding.
  lateralSlip = if enabled and lateral then
    relativeCentreVelocity*rollerAxis
      - nipRadius*nipSpinRate*crossedSkewSin
      - (der(nipPort.lateral.s) - nipPort.geometry.lateralDrift) else 0;
  slipMagnitude = sqrt(slipVelocity^2 + lateralSlip^2
    + Roll2RollDynamics.Utilities.Types.velocityTol^2);
  frictionCoefficient = Roll2RollDynamics.Functions.tripleS(
    traction.vAdhesion, traction.vSlide,
    traction.muAdhesion, traction.muSliding,
    slipMagnitude, traction.viscousSlope);
  // One friction budget shared between the two slip directions
  tangentialForce = normalForce*frictionCoefficient*slipVelocity/slipMagnitude;
  lateralForce = normalForce*frictionCoefficient*lateralSlip/slipMagnitude;
  frictionLoss = tangentialForce*slipVelocity + lateralForce*lateralSlip;

  nipForce = normalForce*normalDirection
    - tangentialForce*webDirection
    - lateralForce*rollerAxis;
  normalMoment = normalForce*loadCentre*cross(rollerAxis, normalDirection);

  // Mechanical reactions and the unused-endpoint boundary
  if enabled then
    frameNip.f = -Modelica.Mechanics.MultiBody.Frames.resolve2(
      frameNip.R, nipForce);
    frameNip.t = -Modelica.Mechanics.MultiBody.Frames.resolve2(
      frameNip.R, rotationDirection*nipRadius*tangentialForce*rollerAxis
        - rotationDirection*nipRadius*lateralForce*webDirection
        + normalMoment);
    // Spatial forces balance at both centres; the web flange supplies relative transport work.
    nipPort.frame.f = Modelica.Mechanics.MultiBody.Frames.resolve2(
      nipPort.frame.R, nipForce);
    nipPort.frame.t = Modelica.Mechanics.MultiBody.Frames.resolve2(
      nipPort.frame.R, normalMoment);
  else
    Connections.branch(nipPort.frame.R, frameNip.R);
    frameNip.r_0 = nipPort.frame.r_0;
    frameNip.R = nipPort.frame.R;
    nipPort.frame.f + frameNip.f = zeros(3);
    nipPort.frame.t + frameNip.t = zeros(3);
  end if;
  nipPort.web.f = -tangentialForce;
  nipPort.lateral.f = -lateralForce;

  engaged = enabled and contactWidth > 0;

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
      Line(points = {{42, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5),
        Line(points = {{-100, 4}, {-42, 4}}, color = {95, 95, 95}, thickness = 0.5),
        Line(points = {{-100, -4}, {-42, -4}}, color = {0, 127, 0}),
        Text(textColor = {64, 64, 64}, extent = {{-100, 62}, {100, 90}}, textString = "%name"),
        Line(origin = {11.296, 2.129}, points = {{-48.768, -0.594}, {-23.671, 5.925}, {-3.579, 5.925}, {17.359, 2.232}, {30.92, -3.862}, {33.847, -5.195}, {31.232, -1.871}, {26.826, 2.232}}, thickness = 5, smooth = Smooth.Bezier),
      Line(origin = {-12.094, -7.871}, points = {{49.201, -2.647}, {23.671, 5.925}, {3.579, 5.925}, {-17.359, 2.232}, {-30.92, -3.862}, {-33.847, -5.195}, {-25.584, -5.125}, {-21.255, -5.125}}, thickness = 5, smooth = Smooth.Bezier),
        Line(origin = {-4.568, 34.66}, rotation = 32.333, points = {{-4.149, -16.181}, {1.164, 12.945}, {2.985, 3.236}}, thickness = 5),
        Line(origin = {14.278, -26.139}, rotation = -145.853, points = {{-4.149, -16.181}, {1.164, 12.945}, {2.985, 3.236}}, thickness = 5)}),
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<h4>Nip contact</h4>
<p>This massless element connects the wrapped roller at <code>nipPort</code>
to the rotating nip at <code>frameNip</code>. Centre separation sets middle
penetration; axis misalignment sets its distribution across the face.
<code>NipRoller</code> supplies mass and inertia. The wrapped roller supplies
<code>rollRadius</code> and <code>rotationDirection</code> through
<code>nipPort.geometry</code>; match <code>webThickness</code> to the web.</p>
<p>By default, loading follows geometry. Setting <code>prescribedLoad = true</code>
instead sets middle penetration to
<code>loadFraction&middot;maxNipLoad/coverStiffness</code>: this is the nominal
parallel-axis preload, and misalignment changes the integrated load.
<code>maxNipLoad</code> specifies actuator capacity, not a contact-force cap;
an external actuator must enforce its own limit.</p>
<p>Figure 1 shows contact geometry and physical forces for
<code>rotationDirection = 1</code>. Positive nip-to-web slip pulls the web
forward and the nip backward. Surface separation, overlap and arrow lengths
are illustrative; force arrows are not connector flow signs.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-geometry.svg\"
width=\"900\" alt=\"Parallel nip and wrapped roller axes with radii, radial
centre separation, web thickness and penetration; equal and opposite normal
and tangential contact forces when the nip surface moves faster than the web\"/>
<strong>Figure 1:</strong> Nip contact geometry and forces.</p>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Symbol</th><th>Model quantity and sign</th></tr>
<tr><td>C, C<sub>n</sub></td><td>Centres at <code>nipPort.frame</code> and
<code>frameNip</code>, respectively.</td></tr>
<tr><td>R, R<sub>n</sub>, h, d</td><td><code>rollRadius</code>,
<code>nipRadius</code>, <code>webThickness</code>, <code>radialDistance</code>.
The separation d excludes any component along the roller axis.</td></tr>
<tr><td>&delta;</td><td><code>penetration</code> when enabled, at the middle
of the face. Positive values mean undeformed overlap; zero is geometric
touchdown.</td></tr>
<tr><td>a, n, t</td><td><code>rollerAxis</code>, <code>normalDirection</code>,
<code>webDirection</code>, all resolved in world coordinates. The normal points
from C toward C<sub>n</sub>; t is
<code>rotationDirection*cross(rollerAxis, normalDirection)</code>.</td></tr>
<tr><td>N, F<sub>t</sub></td><td><code>normalForce</code> and
<code>tangentialForce</code>. N is compressive; positive F<sub>t</sub>
acts on the web along t.</td></tr>
<tr><td>F<sub>nip</sub></td><td><code>nipForce</code>, the physical force
on the nip. The wrapped roller and web receive its negative.</td></tr>
<tr><td>v<sub>nip</sub>, v<sub>web</sub></td><td>Nip surface speed and
<code>der(nipPort.web.s)</code>, measured relative to the wrapped-roller centre
along t. The nip speed includes relative centre translation and surface spin.
Their difference is <code>slipVelocity</code>.</td></tr>
<tr><td>v<sub>lat</sub>, F<sub>lat</sub></td><td><code>lateralSlip</code>
and <code>lateralForce</code> along a, both zero unless <code>lateral</code>
is set. The nip surface of a crossed nip moves along a at
<code>-nipRadius&middot;nipSpinRate&middot;crossedSkewSin</code>; the web
material moves at <code>der(nipPort.lateral.s)</code> less the wrapped
roller's normal-entry <code>lateralDrift</code>, published on
<code>nipPort.geometry</code>, so a nip parallel to a yawed roller sees the
web roll past, not slide. Positive F<sub>lat</sub> acts on the
web along a.</td></tr>
</table>
<p><code>frameNip.f</code> carries <code>-nipForce</code>, resolved in the nip
frame; <code>nipPort.frame.f</code> carries its opposite. The web receives
<code>nipPort.web.f = -tangentialForce</code>. The nip's physical friction
torque is <code>rotationDirection*nipRadius*tangentialForce</code> along a;
<code>frameNip.t</code> carries its negative plus the off-centre normal-load
moment. The wrapped side receives friction work through the web flanges and
only the normal-load moment through <code>nipPort.frame.t</code>.</p>
<h4>Misalignment across the face</h4>
<p><code>relativeMisalignment</code> is the acute angle between the connected
roll axes. Crossing about the contact normal lifts both edges;
<code>wedgeAngle</code> opens one end and closes the other. At cross-machine
position s from the face middle,</p>
<p><code>gap(s) = penetration - (s&middot;tan(crossing))^2/(2&middot;(rollRadius + nipRadius)) - s&middot;tan(wedge)</code></p>
<p>Figure 2 shows the symmetric crossed-axis gap and centred resultant.
The offset <code>s&middot;tan(crossing)</code> comes from two-circle geometry;
the quadratic gap uses the near-contact approximation in [2].</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-gap-crossed.png\"
     width=\"450\"
     alt=\"Front view of the nip face on the roller with crossed axes: both face edges lift on a parabolic gap\">
<strong>Figure 2:</strong> Parabolic gap under crossed axes.</p>
<p>Figure 3 shows the wedge shifting the resultant toward the closed end.
As in the eccentric Winkler-foundation problem [3], full-width contact keeps
the elastic load unchanged; after edge lift-off, the contact width shrinks.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-gap-wedge.png\"
     width=\"450\"
     alt=\"Front view of the nip face on the roller with a wedge gap: one end opens and the load centre moves toward the closed end\">
<strong>Figure 3:</strong> Wedge gap under a trammed roller.</p>
<p><code>edgeLift</code> and <code>endOpening</code> evaluate the two gap terms
at <code>s = nipWidth/2</code>. Independent springs of stiffness
<code>coverStiffness/nipWidth</code> act on <code>max(0, gap)</code>.
Integrating over the contacting interval gives the elastic load and its
first moment; parallel axes give <code>coverStiffness&middot;penetration</code>.
<code>contactWidth</code> reports that interval and <code>loadCentre</code>
the resultant position. Its moment is
<code>normalMoment = normalForce&middot;loadCentre&middot;cross(rollerAxis, normalDirection)</code>.
The support model must resolve this centre-frame torque into individual
bearing reactions.</p>
<p>Fixed misalignment gives steady loading; runout changes it by moving the
connected frames. <code>nipPort.shaft</code> receives zero torque but must
connect to a fixed or prescribed shaft in standalone use;
<code>Roller</code> supplies that connection.</p>
<h4>Contact forces</h4>
<p><code>coverDamping</code> acts on middle approach speed, scaled by contact
width. Clamping elastic plus damping force to zero prevents tension [1];
this differs from <code>ElastoGap</code>'s bounded-damper law.</p>
<p>The <a href=\"modelica://Roll2RollDynamics.Functions.tripleS\">tripleS</a>
coefficient at slip magnitude times <code>normalForce</code> gives the shared
friction budget. With default <code>lateral = false</code>, all friction is
longitudinal. Setting it to <code>true</code> also resolves crossed-nip slip
along the roller axis through <code>nipPort.lateral</code>, sharing the web's
lateral coordinate with the wrapped-roller contact. Spatial reactions balance
linear momentum; flange efforts supply work relative to the roller centre.</p>
<p>When <code>enabled = false</code>, leave <code>frameNip</code> unconnected:
its pose is supplied internally and contact forces vanish.</p>
<h4>Limitations</h4>
<p>The independent-spring cover omits the elliptical crossed-cylinder patch,
face shear, varying contact normal, finite face overlap and wrapped-arc
membership. Friction uses middle slip and projected nip surface speed;
wedge and crossing rates do not enter damping. Axes must remain within
60 degrees of parallel. Damping can cause a touchdown force jump, and
imposed compression or transient loads can exceed actuator capacity.
Regularized friction requires finite slip.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://doc.modelica.org/Modelica%204.0.0/Resources/helpOM/Modelica.Mechanics.Translational.Components.ElastoGap.html\">1</a>]
Modelica Association, &quot;ElastoGap&quot;, Modelica Standard Library 4.0.0, 2020,
Information section. Unilateral spring-damper contact, tensile-force prevention
and damping behaviour at touchdown. Accessed 2026-09-08.</li>
<li>[2] K. L. Johnson, <i>Contact Mechanics</i>, Cambridge University Press,
1985, Ch. 3, &quot;The separation of two curved bodies in near
contact&quot;. Near-contact separation of two curved surfaces is locally
quadratic in offset; the two-circle case used here is that geometry's
elementary special case in one plane.</li>
<li>[3] J. E. Bowles, <i>Foundation Analysis and Design</i>, 5th ed.,
McGraw-Hill, 1996, Ch. 4, eccentrically loaded footings and the
middle-third rule. Same line-of-springs foundation with a linear pressure
term: full contact and unchanged resultant load below the eccentricity
threshold, edge lift-off and shrinking contact area past it.</li>
</ul>
</html>"));
end NipContact;
