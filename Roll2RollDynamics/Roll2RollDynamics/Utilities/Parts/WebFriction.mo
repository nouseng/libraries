within Roll2RollDynamics.Utilities.Parts;
model WebFriction
  "Analytic Triple-S friction between a moving web and a roll surface"
  outer Modelica.Mechanics.MultiBody.World world "Gravity field at the roll centre";
  //Parameters
  parameter Boolean fixedInitialWebState = false
    "Initialize transport coordinates and physical web speed at zero";
  parameter Real referenceDensity(unit = "kg/m") = 1 "Unstrained web mass per length";
  parameter Real axialStiffness(unit = "N") = 8000 "Web axial stiffness";
  parameter Modelica.Units.SI.Length contactLengthMin = 0
    "Minimum wrapped length retained for inventory; 0 disables the floor";
  input Modelica.Units.SI.Length wrapLength = 1 "Current mechanical wrap length";
  input Modelica.Units.SI.Force tensionIn = meanTension "Entry tension";
  input Modelica.Units.SI.Force tensionOut = meanTension "Exit tension";
  input Real stretchIn(unit="1") = 1 + tensionIn/axialStiffness
    "Actual entry stretch; elastic default for standalone use";
  input Real stretchOut(unit="1") = 1 + tensionOut/axialStiffness
    "Actual exit stretch; elastic default for standalone use";
  input Modelica.Units.SI.Velocity boundaryVelocityA = 0 "Entry tangency speed along wrap";
  input Modelica.Units.SI.Velocity boundaryVelocityB = 0 "Exit tangency speed along wrap";
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters traction
    "Triple-S web-to-roll traction characteristic";
  parameter Modelica.Units.SI.Radius radius(min = Modelica.Constants.small)
    "Roll radius the surface speed and contact moment are taken at";
  parameter Integer rotationDirection(min = -1, max = 1) = 1
    "Signed web travel direction over the roll";
  input Modelica.Units.SI.Force nipForce[3] = zeros(3)
    "Spatial force at the optional nip connection";
  input Modelica.Units.SI.Force normalForce(min = 0) = Modelica.Math.Vectors.length(nipForce)
    "Additional concentrated normal load, for example from a nip";
  input Modelica.Units.SI.Force meanTension(min = 0) = 0
    "Mean of the incoming and outgoing wrap tensions";
  input Modelica.Units.SI.Angle wrapAngle(min = 0) = 0
    "Magnitude of the web wrap angle";
  parameter Boolean lateral = false
    "= true, if the contact resolves a cross-machine slip component";
  parameter Boolean useDetachment = false
    "= true, the engaged input can toggle at run time; = false, the lateral contact equation is fixed at compile time with no conditional"
    annotation(Evaluate = true, HideResult = true);
  input Boolean engaged = true
    "= false, the web has left the roll surface and carries no contact";
  input Real spanDirectionA[3](each unit="1") = entryTangent
    "Directed entry span unit vector in world coordinates";
  input Real spanDirectionB[3](each unit="1") = exitTangent
    "Directed exit span unit vector in world coordinates";
  input Modelica.Units.SI.Velocity lateralDrift =
    -(if massFlowIn + massFlowOut >= 0 then der(flange_a.s)*entrySkewSin
      else der(flange_b.s)*exitSkewSin)
    "Roll-relative contact-line speed that gives zero axial slip";
  output Real entrySkewSin(unit="1") "Entry span direction projected along the roll axis";
  output Real exitSkewSin(unit="1") "Exit span direction projected along the roll axis";
  input Real entryTangent[3](each unit="1") = {1, 0, 0}
    "Directed entry tangent in world coordinates";
  input Real exitTangent[3](each unit="1") = {1, 0, 0}
    "Directed exit tangent in world coordinates";
  input Real meanTangent[3](each unit="1") = {1, 0, 0}
    "Arc average of the directed tangent in world coordinates";
  output Modelica.Units.SI.Force wrapCapacity "Peak dry traction supplied by the wrap alone";

  Modelica.Mechanics.Translational.Interfaces.Flange_a flange_a
    "Incoming web transport flange"
    annotation(Placement(transformation(origin = {-130, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-130, 100}, extent = {{-10, -10}, {10, 10}}))
    );
  Modelica.Mechanics.Translational.Interfaces.Flange_b flange_b
    "Outgoing web transport flange"
    annotation(Placement(transformation(origin = {130, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {130, 100}, extent = {{-10, -10}, {10, 10}}))
    );
  Modelica.Mechanics.Translational.Interfaces.Flange_a contactFlange
    "Physical mean contact velocity and additional nip force"
    annotation(Placement(transformation(origin={200,50}, extent={{-10,-10},{10,10}}),
      iconTransformation(origin={200,50}, extent={{-10,-10},{10,10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_drum
    "Rotating roll-centre frame carrying wrapped-web support force and contact torque"
    annotation(Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.Translational.Interfaces.Flange_a lateralFlange
    "Cross-machine web position in the contact patch"
    annotation(Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));
  Real drumAxis[3](each unit = "1")
    "Roll rotation axis resolved in world coordinates";
  Modelica.Units.SI.AngularVelocity spinRate
    "Roll angular velocity about its own axis";
  Modelica.Units.SI.Velocity surfaceVelocity
    "Signed roll-surface speed along web travel";
  Modelica.Units.SI.Velocity vWeb(start = 0, fixed = false,
    stateSelect = StateSelect.default)
    "Web velocity through the contact patch";
  Modelica.Units.SI.Momentum momentum(start=0, fixed=false,
    stateSelect=StateSelect.prefer)
    "Stored longitudinal momentum of the wrapped web";
  Modelica.Units.SI.Velocity velocityIn "Physical entry velocity relative to roll centre";
  Modelica.Units.SI.Velocity velocityOut "Physical exit velocity relative to roll centre";
  output Modelica.Units.SI.Mass webMass
    "Wrapped material inventory, floored by contactLengthMin for the momentum closure";
  output Modelica.Units.SI.Mass wrappedMass
    "True wrapped material inventory, unfloored; identical to webMass unless the floor is active";
  output Modelica.Units.SI.MassFlowRate massFlowIn "Signed entry material flux";
  output Modelica.Units.SI.MassFlowRate massFlowOut "Signed exit material flux";
  Modelica.Units.SI.Acceleration aWeb
    "Web acceleration through the contact patch";
  Real mu(unit = "1") "Signed regularized friction coefficient";
  Modelica.Units.SI.Force centrifugalTension
    "Mean line density times squared contact speed";
  Modelica.Units.SI.Force contactTension
    "Mean tension remaining to press the moving web against the roll";
  Modelica.Units.SI.Velocity vLatWeb(start = 0, fixed = false)
    "Cross-machine web velocity in the contact patch";
  Modelica.Units.SI.Acceleration aLatWeb
    "Cross-machine web acceleration in the contact patch";
  Modelica.Units.SI.Velocity vMag "Slip magnitude the friction law is evaluated on";
  Real muMag(unit = "1") "Regularized friction coefficient at the slip magnitude";
  Modelica.Units.SI.Force frictionMagnitude
    "Combined wrap and concentrated-contact traction at the slip magnitude";
  output Modelica.Units.SI.Velocity slipVelocity
    "Web velocity relative to the roll surface";
  output Modelica.Units.SI.Force tractionCapacity =
    wrapCapacity
      + max(0, normalForce)*traction.muAdhesion
    "Peak dry adhesion force before the additional viscous term";
  output Modelica.Units.SI.Force longitudinalForce
    "Longitudinal friction force acting on the roll surface";
  output Modelica.Units.SI.Force wrappedLoad[3]
    "Load the wrapped material applies at the roll centre, before resolution into frame_drum";
  output Modelica.Units.SI.Velocity lateralSlip
    "Cross-machine web-to-roll slip";
  output Modelica.Units.SI.Force lateralForce
    "Cross-machine friction force acting on the web";

protected
  //Variables
  Modelica.Units.SI.Velocity centerVelocity[3]
    "Roll-centre translational velocity resolved in world coordinates";

initial equation
  if fixedInitialWebState then
    flange_a.s = 0;
    flange_b.s = 0;
    contactFlange.s = 0;
    vWeb = 0;
  end if;

equation
  assert(traction.vSlide > traction.vAdhesion,
    "WebFriction requires vSlide > vAdhesion");
  assert(traction.muAdhesion >= traction.muSliding,
    "WebFriction requires muAdhesion >= muSliding");
  velocityIn = der(flange_a.s) + boundaryVelocityA;
  velocityOut = der(flange_b.s) + boundaryVelocityB;
  vWeb = (velocityIn + velocityOut)/2;
  der(contactFlange.s) = vWeb;
  // Interpolate actual endpoint stretch; viscous force is not stored strain.
  // The inventory floor keeps webMass off zero so the momentum closure
  // (der(webMass), der(momentum), momentum = webMass*vWeb below) stays
  // well posed; contactLengthMin = 0 reproduces wrapLength exactly.
  webMass = 2*referenceDensity*max(wrapLength, contactLengthMin)/(stretchIn + stretchOut);
  // The true, unfloored inventory. Equal to webMass except while the floor
  // is holding webMass above a fully or nearly detached wrapLength; used
  // wherever the support reaction should report the roll's actual carried
  // weight rather than the closure's regularized state.
  wrappedMass = 2*referenceDensity*wrapLength/(stretchIn + stretchOut);
  massFlowIn = referenceDensity*der(flange_a.s)/stretchIn;
  massFlowOut = referenceDensity*der(flange_b.s)/stretchOut;
  assert(stretchIn > 0 and stretchOut > 0,
    "WebFriction requires positive boundary material stretches");
  der(webMass) = massFlowIn - massFlowOut;
  der(momentum) = flange_a.f + flange_b.f + contactFlange.f
    - longitudinalForce + massFlowIn*velocityIn - massFlowOut*velocityOut;
  if useDetachment then
    // Zero wrap is valid during the contact event. Allow only root-finding
    // roundoff in its length, while keeping the taut-span check an error.
    assert(tensionIn > 0 and tensionOut > 0
      and wrapLength >= -Roll2RollDynamics.Utilities.Types.lengthTol,
      "Detachable WebFriction requires nonnegative wrap length and taut spans");
  else
    assert(tensionIn > 0 and tensionOut > 0 and (wrapLength > 0 or not engaged),
      "Conservative WebFriction requires positive wrap length and taut entry/exit spans");
  end if;
  momentum = webMass*vWeb;
  // Diagnostic only; nothing in the library reads aWeb.
  aWeb = if noEvent(webMass > Modelica.Constants.small)
    then (der(momentum) - vWeb*(massFlowIn - massFlowOut))/webMass else 0;
  drumAxis = Modelica.Mechanics.MultiBody.Frames.resolve1(
    frame_drum.R, {0, 1, 0});
  entrySkewSin = spanDirectionA*drumAxis;
  exitSkewSin = spanDirectionB*drumAxis;
  spinRate = Modelica.Mechanics.MultiBody.Frames.angularVelocity1(
    frame_drum.R)*drumAxis;
  surfaceVelocity = rotationDirection*spinRate*radius;
  slipVelocity = vWeb - surfaceVelocity;
  // webMass/wrapLength is exactly 2*referenceDensity/(stretchIn + stretchOut);
  // cancelling keeps it finite at zero wrap instead of evaluating 0/0.
  centrifugalTension = 2*referenceDensity/(stretchIn + stretchOut)*vWeb^2;
  contactTension = max(0, meanTension - centrifugalTension);
  wrapCapacity = 2*contactTension*tanh(traction.muAdhesion*wrapAngle/2);
  assert(min(tensionIn,tensionOut) > centrifugalTension,
    "Wrapped web lost contact: tension must exceed the centrifugal tension");
  vLatWeb = der(lateralFlange.s);
  aLatWeb = der(vLatWeb);
  if lateral then
    // u_web = u_boundary + crossingSpeed*spanDirection. Project u_web-u_surface
    // along drumAxis: common mount translation cancels, and circumferential spin
    // and tangency migration have no axial component. The boundary contributes vLatWeb.
    lateralSlip = vLatWeb - lateralDrift;
    vMag = sqrt(slipVelocity^2 + lateralSlip^2
      + Roll2RollDynamics.Utilities.Types.velocityTol^2);
    muMag = Roll2RollDynamics.Functions.tripleS(
      traction.vAdhesion, traction.vSlide,
      traction.muAdhesion, traction.muSliding, vMag,
      traction.viscousSlope);
    mu = muMag*slipVelocity/vMag;
    frictionMagnitude = 2*contactTension*tanh(muMag*wrapAngle/2)
      + max(0, normalForce)*muMag;
    longitudinalForce = frictionMagnitude*slipVelocity/vMag;
    lateralForce = -frictionMagnitude*lateralSlip/vMag;
    // Retain lateral momentum through release. Zero wrap removes contact
    // friction; freezing the velocity instead would impose an impulse and
    // change the differentiation index. While detached, webMass is the
    // small regularized inventory set by contactLengthMin.
    webMass*aLatWeb = lateralFlange.f + lateralForce;
  else
    lateralSlip = 0;
    vMag = 0;
    muMag = 0;
    frictionMagnitude = 0;
    lateralForce = 0;
    lateralFlange.s = 0;
    mu = Roll2RollDynamics.Functions.tripleS(
      traction.vAdhesion, traction.vSlide,
      traction.muAdhesion, traction.muSliding, slipVelocity,
      traction.viscousSlope);
    longitudinalForce = 2*contactTension*tanh(mu*wrapAngle/2)
      + max(0, normalForce)*mu;
  end if;
  // Wrapped material support load, resolved from world into the rotating frame.
  // The weight-and-acceleration term uses wrappedMass (the true, unfloored
  // inventory): it is a static reaction to how much material the roll is
  // actually carrying, and must go to exactly zero once wrapLength does,
  // even while the floor is holding webMass above zero for the momentum
  // closure below. der(momentum*meanTangent) is the rate of change of the
  // momentum ODE's own state and cannot be de-floored independently of it
  // without reopening the division-by-webMass singularity the floor exists
  // to close; the mass-flux terms use massFlowIn/massFlowOut, which never
  // depended on webMass in the first place.
  centerVelocity = der(frame_drum.r_0);
  // wrappedLoad is the single expression the model uses; frame_drum.f is
  // defined FROM it rather than restating its own copy, so the two cannot
  // drift apart, and a check outside this component can assert on exactly
  // the quantity the physics actually computes.
  wrappedLoad = wrappedMass*(der(centerVelocity) - world.gravityAcceleration(frame_drum.r_0))
    + der(momentum*meanTangent)
    - massFlowIn*velocityIn*entryTangent + massFlowOut*velocityOut*exitTangent;
  frame_drum.f = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_drum.R, wrappedLoad);
  // The force acts at the centre; only contact friction adds shaft torque
  frame_drum.t = -Modelica.Mechanics.MultiBody.Frames.resolve2(
    frame_drum.R, rotationDirection*radius*longitudinalForce*drumAxis);

  annotation(
    Icon(coordinateSystem(extent = {{-200, -100}, {200, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Text(origin = {-4.218, -120}, textColor = {64, 64, 64}, extent = {{-100, 36}, {100, 70}}, textString = "%name"),
      Line(origin = {11.296, 2.129}, points = {{-48.768, -0.594}, {-23.671, 5.925}, {-3.579, 5.925}, {17.359, 2.232}, {30.92, -3.862}, {33.847, -5.195}, {31.232, -1.871}, {26.826, 2.232}}, thickness = 5, smooth = Smooth.Bezier),
      Line(origin = {-12.094, -7.871}, points = {{49.201, -2.647}, {23.671, 5.925}, {3.579, 5.925}, {-17.359, 2.232}, {-30.92, -3.862}, {-33.847, -5.195}, {-25.584, -5.125}, {-21.255, -5.125}}, thickness = 5, smooth = Smooth.Bezier),
        Line(origin = {-4.568, 34.66}, rotation = 32.333, points = {{-4.149, -16.181}, {1.164, 12.945}, {2.985, 3.236}}, thickness = 5),
        Line(origin = {14.278, -26.139}, rotation = -145.853, points = {{-4.149, -16.181}, {1.164, 12.945}, {2.985, 3.236}}, thickness = 5),
      Text(origin = {-130, 77.537}, extent = {{-14.915, -9.679}, {14.915, 9.679}}, textString = "Fx_a"),
        Text(origin = {2.915, 76.568}, extent = {{-14.915, -9.679}, {14.915, 9.679}}, textString = "Ft"),
        Rectangle(origin = {-2.322, 0}, lineColor = {128, 128, 128}, fillColor = {0, 116, 0}, lineThickness = 5, extent = {{-197.678, -97}, {197.678, 97}}, radius = 25), Text(origin = {130, 75.413}, extent = {{-14.915, -9.679}, {14.915, 9.679}}, textString = "Fx_b")}),
    Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}}), graphics = {
      Text(extent = {{-100, 70}, {100, 40}}, textString = "%name",
        textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Wrapped material and friction</h4>
<p>This component conserves wrapped-web mass and momentum while exchanging
friction with the drum [1]. <code>webMass</code> follows from
<code>wrapLength</code>, <code>referenceDensity</code> and mean endpoint stretch.
Adjacent spans supply <code>stretchIn</code> and <code>stretchOut</code>;
standalone defaults use elastic tension. This uniform-strain approximation
adds no damping, and each <code>Belt</code> retains its own mass.</p>
<p>Boundary flows use local strain and speeds relative to moving tangencies.
<code>boundaryVelocityA</code> and <code>boundaryVelocityB</code> recover
centre-relative speeds, whose mean <code>vWeb</code> sets momentum.
Tension, friction and <code>contactFlange.f</code> determine its rate.
<code>fixedInitialWebState</code> initializes transport and web speed at zero.</p>
<p>Supply curved-wrap geometry from <code>WebWrap</code>; default tangents
describe a straight path. <code>frame_drum</code> carries friction torque,
wrapped-material weight, mount-acceleration load and vector momentum reaction.
Shell and web move independently during slip.</p>
<p>Figure 1 shows a planar wrap with <code>rotationDirection = 1</code>.
Blue arrows give A-to-B span directions; orange arrows act on the web.
Positive longitudinal and axial slip produce opposing friction.
Arrow lengths are illustrative; contact pressure is not distributed.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/WebFriction/contact-forces.svg\"
width=\"900\" alt=\"Wrapped web with outward entry and exit tensions, outward drum pressure,
optional inward nip load, wrap angle and roller axis; local tangent-plane friction
opposing longitudinal and lateral web slip\"/>
<strong>Figure 1:</strong> Wrapped material and contact forces.</p>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Symbol</th><th>Model quantity and sign</th></tr>
<tr><td>T<sub>A</sub>, T<sub>B</sub></td><td><code>tensionIn</code>,
<code>tensionOut</code>. In <code>Roller</code>, these equal
<code>-flange_a.f</code> and <code>flange_b.f</code>, respectively.</td></tr>
<tr><td>e<sub>A</sub>, e<sub>B</sub></td><td><code>spanDirectionA</code>,
<code>spanDirectionB</code>, resolved in world coordinates. Both remain
directed from A to B during reverse transport.</td></tr>
<tr><td>a, t</td><td><code>drumAxis</code> and the local forward circumferential
tangent. The span directions can have a component along a when skewed.</td></tr>
<tr><td>r, &theta;</td><td><code>radius</code>, <code>wrapAngle</code>.</td></tr>
<tr><td>dN, N<sub>nip</sub></td><td>Illustrative local drum pressure force
(not a separate model variable) and the additional load <code>normalForce</code>.
The wrap's own normal loading is accounted for in the capstan traction law.</td></tr>
<tr><td>F<sub>t</sub>, F<sub>y</sub></td><td><code>longitudinalForce</code>
acts on the drum; the web receives its negative. <code>lateralForce</code>
acts on the web along a. The slip vector's components are
<code>slipVelocity</code> and <code>lateralSlip</code>.</td></tr>
</table>
<p>Drum contact torque is <code>rotationDirection*radius*longitudinalForce</code>
along the roller axis; <code>frame_drum.t</code> carries its negative in the
drum frame. Optional tangential nip force enters through <code>contactFlange</code>.</p>
<h4>Traction and lateral motion</h4>
<p>The capstan law subtracts centrifugal tension from contact tension [2].
<a href=\"modelica://Roll2RollDynamics.Functions.tripleS\">tripleS</a>
supplies the regularized coefficient; <code>normalForce</code> adds concentrated
nip loading. When lateral motion is enabled, both slip directions share one
friction budget and the wrapped mass supplies lateral inertia.</p>
<p>Axial slip is contact-line speed relative to the mount plus signed boundary
crossing speed projected on the roller axis. Supply unit
<code>spanDirectionA</code> and <code>spanDirectionB</code> from the spans;
default circumferential tangents give normal entry. Common mount translation
cancels, and shaft spin and tangency migration have no axial velocity.
A custom boundary may override <code>lateralDrift</code> in roller-axis coordinates.</p>
<p>Mean mass-flux sign selects entering end A forward and B in reverse;
span vectors remain A-to-B. <code>entrySkewSin</code> and <code>exitSkewSin</code>
measure axial alignment. Zero axial slip recovers normal-entry kinematics [3];
the friction and inertia closure is specific to this component.</p>
<h4>Limitations</h4>
<p>The single control volume uses mean strain and speed. It omits distributed
strain, lateral momentum flux, a full energy balance, gravity moments and
accelerating-offset-centroid coupling. Support orientation must be fixed.
Both end tensions must exceed centrifugal tension; air lubrication is omitted
and traction requires finite slip. Simultaneous inflow or outflow at both ends
makes mean-flux boundary selection approximate and potentially discontinuous.
Finite-skew longitudinal speed and torque remain approximations.</p>
<p><code>contactLengthMin</code> defaults to zero. A positive value floors
<code>webMass</code> in the momentum closure to avoid singularity as wrap vanishes:
<code>der(webMass) = massFlowIn - massFlowOut</code> and
<code>momentum = webMass*vWeb</code>. While the floor is active, equal boundary
flows give pass-through transport. It persists throughout detached dwell,
not only at switching, and lateral velocity continues to evolve.</p>
<p>The floor never enters <code>wrapCapacity</code>,
<code>longitudinalForce</code> or <code>centrifugalTension</code>.
Zero wrap therefore gives exactly zero wrap traction.
The true inventory
<code>wrappedMass = 2*referenceDensity*wrapLength/(stretchIn+stretchOut)</code>
supplies weight and mount-acceleration load, so a detached roll carries no
static web load. <code>frame_drum.f</code> resolves <code>wrappedLoad</code>,
whose <code>der(momentum*meanTangent)</code> term remains on the floored state.
Detached momentum reactions and lateral dynamics therefore depend on the floor
and require sensitivity checks. Transition tension peaks and detached support
forces remain provisional.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/resources/mit2_25f13_fundam_law-son/\">1</a>]
A. A. Sonin, &quot;Fundamental Laws of Motion for Particles, Material Volumes,
and Control Volumes&quot;, MIT, 2003, secs. 3.1&ndash;3.2,
eqs. (25), (26A), (27A). Moving-boundary mass and momentum balances.
Accessed 2026-09-06.</li>
<li>[<a href=\"https://normalentry.com/images/Papers/Getting_and_Losing_Traction/Getting_and_Losing_Traction_-_ppt.pdf\">2</a>]
J. Brown, &quot;Getting and Losing Traction&quot;, AIMCAL Web Coating and Handling
Conference, 2012, slides 15 and 19. Capstan relation and centrifugal tension
correction; the regularization is supplied by <code>tripleS</code>.
Accessed 2026-09-06.</li>
<li>[<a href=\"https://normalentry.com/images/Papers/The_Connection_Between_Longitudinal_and_Lateral_Web_Dynamics/The_Connection_Between_Longitudinal_and_Lateral_Web_Dynamics_9-15-2019_paper.pdf\">3</a>]
J. Brown, &quot;The Connection Between Longitudinal and Lateral Web Dynamics&quot;,
2019, p. 2, eq. (1) and footnotes 1&ndash;2. Normal-entry contact-line kinematics
and its no-slip assumption; the vector projection above extends the kinematic
description to the library's moving tangencies. Accessed 2026-09-06.</li>
</ul>
</html>"));
end WebFriction;
