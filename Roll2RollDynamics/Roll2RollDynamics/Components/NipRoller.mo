within Roll2RollDynamics.Components;
model NipRoller   "Undriven rotating nip roller with an optional external housing support"

  //Parameters
  // Geometry
  parameter Modelica.Units.SI.Radius radius = world.rollRadius
    "Nip roller radius"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length width = world.rollWidth
    "Nip roller face width"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Angle yaw = 0
    "In-plane misalignment: nip axis turned about the vertical, ends stay level"
    annotation(Dialog(group = "Misalignment"));
  parameter Modelica.Units.SI.Angle tram = 0
    "Out-of-plane misalignment: nip axis tilted about the machine direction, one end up"
    annotation(Dialog(group = "Misalignment"));
  parameter Boolean variableTilt = false
    "= true, keep yaw and tram as run-time parameters, for FMU sweeps"
    annotation(Dialog(group = "Misalignment"));
  parameter Modelica.Units.SI.Length runout = 0
    "Radial runout: nip drum centre offset from its bearing axis"
    annotation(Dialog(group = "Runout"));
  parameter Modelica.Units.SI.Angle runoutPhase = 0
    "Direction of the runout high point at zero shaft angle, from the machine direction toward up"
    annotation(Dialog(group = "Runout"));
  parameter Boolean variableRunout = false
    "= true, keep runout as a run-time parameter, for FMU sweeps"
    annotation(Dialog(group = "Runout"));

  // Mounting
  parameter Boolean useSupport = false
    "Expose the housing support frame instead of grounding the nip internally"
    annotation(Dialog(group = "Mounting"));
  parameter Modelica.Units.SI.Length xPosition = 0
    "Fixed nip centre x-position"
    annotation(Dialog(group = "Mounting", enable = not useSupport));
  parameter Modelica.Units.SI.Length zPosition = 0
    "Fixed nip centre height"
    annotation(Dialog(group = "Mounting", enable = not useSupport));

  // Dynamics
  parameter Modelica.Units.SI.RotationalDampingConstant bearingDamping(min=0) =
    world.bearingDamping "Viscous nip-bearing damping"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.Density density = 2700
    "Nip shell material density"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.Length wallThickness = 0.006
    "Nip shell wall thickness"
    annotation(Dialog(group = "Dynamics"));

  // Contact
  parameter Roll2RollDynamics.Utilities.Types.WebFace contactFace =
    Roll2RollDynamics.Utilities.Types.WebFace.Outer
    "Material face of the web the nip surface touches"
    annotation(Dialog(group = "Contact"));
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters traction =
    if contactFace == Roll2RollDynamics.Utilities.Types.WebFace.Inner then
      world.innerTraction else world.outerTraction
    "Nip surface-to-web friction characteristic of the contacting face"
    annotation(Dialog(group = "Contact"));
  parameter Modelica.Units.SI.TranslationalSpringConstant coverStiffness = 1e6
    "Full-width nip-cover stiffness"
    annotation(Dialog(group = "Contact"));
  parameter Modelica.Units.SI.TranslationalDampingConstant coverDamping = 1000
    "Nip-cover damping during contact"
    annotation(Dialog(group = "Contact"));
  parameter Modelica.Units.SI.Force maxNipLoad = 1000
    "Maximum actuator force"
    annotation(Dialog(group = "Contact"));
  parameter Real loadFraction(unit = "1", min = 0, max = 1) = 0.5
    "Nominal preload as a fraction of maximum actuator force"
    annotation(Dialog(group = "Contact", enable = not useSupport));

  // Initialization
  parameter Boolean fixedInitialAngle = false
    "Fix the initial nip angle at zero"
    annotation(Dialog(tab = "Initialization"));
  parameter Boolean fixedInitialSpeed = false
    "Fix the initial nip angular speed at zero"
    annotation(Dialog(tab = "Initialization"));

  // Animation
  parameter Real rollColor[3] = {205, 120, 55}
    "Nip roller surface colour"
    annotation(Dialog(tab = "Animation"));

  //Variables
  // Main results, grouped into one browser node

  record NipVariables "Nip results a line engineer plots"
    Modelica.Units.SI.Force nipLoad "Compressive nip load";
    Modelica.Units.SI.Force nipTraction "Tangential force acting on the web";
    Modelica.Units.SI.Velocity nipSlip "Nip surface speed relative to the web";
    Modelica.Units.SI.Length penetration "Middle cover penetration";
    Modelica.Units.SI.AngularVelocity angularVelocity "Nip shaft angular velocity";
  end NipVariables;
  NipVariables nipVariables "Nip results displayed in the variable browser";

  //Components
  outer Roll2RollDynamics.WebWorld world
    "Multibody world and shared line defaults" annotation(
      Placement(transformation(origin = {-60, 60}, extent = {{-10, -10}, {10, 10}}))
    );

protected
  //Variables
  // Working names of the displayed results; nipVariables carries them outward.
  Modelica.Units.SI.Force nipLoad "Compressive nip load";
  Modelica.Units.SI.Force nipTraction "Tangential force acting on the web";
  Modelica.Units.SI.Velocity nipSlip "Nip surface speed relative to the web";
  Modelica.Units.SI.Length penetration "Cover penetration";
  Modelica.Units.SI.AngularVelocity angularVelocity "Nip shaft angular velocity";
  //Components
  Roll2RollDynamics.Utilities.Parts.RollBody rollBody(
    animation = world.enableAnimation,
    radius = radius,
    width = width,
    beltThickness = world.web.thickness,
    yaw = yaw,
    tram = tram,
    variableTilt = variableTilt,
    runout = runout,
    runoutPhase = runoutPhase,
    variableRunout = variableRunout,
    rotationDirection = 1,
    useSupport = useSupport,
    xPosition = xPosition,
    zPosition = zPosition,
    useFlange = false,
    bearingDamping = bearingDamping,
    density = density,
    wallThickness = wallThickness,
    fixedInitialAngle = fixedInitialAngle,
    fixedInitialSpeed = fixedInitialSpeed,
    useDynamicColor = false,
    rollColor = rollColor,
    normalForce = 0,
    drumDrawnRadius = radius) "Mounted nip body, shaft inertia and bearing drag" annotation(
      Placement(transformation(origin = {-20, 0}, extent = {{-40, -20}, {40, 20}}, rotation = -90)));
public
  Roll2RollDynamics.Utilities.Parts.NipContact nipContact(
    nipRadius = radius,
    webThickness = world.web.thickness,
    coverStiffness = coverStiffness,
    coverDamping = coverDamping,
    maxNipLoad = maxNipLoad,
    prescribedLoad = not useSupport,
    loadFraction = loadFraction,
    nipWidth = width,
    traction = traction,
    lateral = world.lateralDynamics) "Contact between the nip surface and web"
    annotation(Placement(transformation(origin = {40, 0},
      extent = {{25, -25}, {-25, 25}})));

  //Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_support if useSupport
    "Nip housing support connection"
    annotation(Placement(transformation(origin = {-80, 0}, extent = {{-16, -16}, {16, 16}}, rotation = -180), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Roll2RollDynamics.Utilities.Interfaces.NipPort nipLoading
    "Combined connection to the wrapped roller centre and web"
    annotation(Placement(transformation(origin = {80, 0}, extent = {{-16, -16}, {16, 16}}, rotation = -180), iconTransformation(origin = {105.665, 0}, extent = {{-20, -20}, {20, 20}}, rotation = -180)));

equation
  // Displayed results, grouped for the variable browser
  nipVariables.nipLoad = nipLoad;
  nipVariables.nipTraction = nipTraction;
  nipVariables.nipSlip = nipSlip;
  nipVariables.penetration = penetration;
  nipVariables.angularVelocity = angularVelocity;
  connect(frame_support, rollBody.frame_support)
    annotation(Line(points = {{-20, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-60, 0}));
  connect(rollBody.frame_drum, nipContact.frameNip)
    annotation(Line(points = {{-7.5, 0}, {7.5, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {7.5, 0}));
  connect(nipLoading, nipContact.nipPort)
    annotation(Line(points = {{7.5, 0}, {-7.5, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {72.5, 0}));

  angularVelocity = der(rollBody.angle);
  nipLoad = nipContact.normalForce;
  nipTraction = nipContact.tangentialForce;
  nipSlip = nipContact.slipVelocity;
  penetration = nipContact.penetration;

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
      Line(rotation = -90, points = {{0, 50}, {0, 100}}, color = {95, 95, 95}, thickness = 0.5),
      Ellipse(lineColor = {64, 64, 64}, fillColor = {205, 120, 55}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-50, 50}, {50, -50}}),
        Ellipse(lineColor = {95, 95, 95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-22, 22}, {22, -22}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-9, 9}, {9, -9}}),
      Text(origin = {0, 166}, textColor = {64, 64, 64}, extent = {{-100, -96}, {100, -68}}, textString = "%name"),
        Polygon(origin = {-81.151, 3.043}, fillColor = {0, 116, 0}, fillPattern = FillPattern.Solid, points = {{29.745, -3.574}, {4.396, -26.746}, {4.396, -13.574}, {-27.444, -13.574}, {-27.444, 6.426}, {4.396, 6.426}, {4.396, 24.034}})}),
    Diagram(coordinateSystem(extent = {{-80, -80}, {80, 80}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {
      Text(origin = {50, 70}, textColor = {64, 64, 64}, extent = {{-50, -10}, {50, 10}}, textString = "%name")}),
    Documentation(info = "<html>
<h4>Nip roller</h4>
<p>An undriven drum presses on a roller's wrapped web. Cover compression
supplies normal load; friction spins the drum and pulls the web. Drum mass,
shaft inertia and viscous bearing drag are included.</p>
<p>Figure 1 defines R as the wrapped radius, R<sub>n</sub> as
<code>radius</code>, h as <code>world.web.thickness</code>, d as axis distance
and delta as <code>nipVariables.penetration</code>. N and F<sub>t</sub> are
<code>nipLoad</code> and <code>nipTraction</code>. Positive nip-to-web slip
pulls the web forward and brakes the drum. Integrated cover compression and
approach damping give a nonnegative load; the contact cannot pull. See
<a href=\"modelica://Roll2RollDynamics.Utilities.Parts.NipContact\">NipContact</a>
for the load and regularized friction laws.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-geometry.svg\"
     width=\"900\"
     alt=\"Nip and wrapped roller geometry beside equal and opposite normal and tangential contact forces for positive nip-to-web slip\">
<strong>Figure 1:</strong> Nip contact geometry and forces.</p>
<p><code>coverStiffness</code> = Full-width stiffness, 1e6 N/m<br>
<code>coverDamping</code> = Contact damping, 1000 N.s/m<br>
<code>maxNipLoad</code>, <code>loadFraction</code> = Actuator/preload settings,
1000 N and 0.5; not a contact-force cap<br>
<code>contactFace</code> = <code>Outer</code> by default, opposite the wrapped
roller's face; selects shared <code>traction</code>, overridable per cover<br>
<code>yaw</code>, <code>tram</code> = Fixed in-plane/out-of-plane axis angles,
zero by default<br>
<code>runout</code>, <code>runoutPhase</code> = Radial eccentricity and phase,
zero by default</p>
<p>Misalignment uses both actual axes: opposite 3-degree yaw gives 6 degrees
relative. Figure 2 shows the parabolic crossed-axis gap, with edge lift
<code>((width/2)*tan(crossing))^2/(2*(wrapped radius + radius))</code>
and a central resultant.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-gap-crossed.png\"
     width=\"450\"
     alt=\"Front view of the nip face on the roller with crossed axes: both face edges lift on a parabolic gap\">
<strong>Figure 2:</strong> Parabolic gap under crossed axes.</p>
<p>Figure 3 shows a tram difference opening one end by
<code>(width/2)*tan(wedge)</code>. Contact shortens toward that end; the
resultant moves toward the closed end.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipContact/nip-gap-wedge.png\"
     width=\"450\"
     alt=\"Front view of the nip face on the roller with a wedge gap: one end opens and the load centre moves toward the closed end\">
<strong>Figure 3:</strong> Wedge gap under a trammed roller.</p>
<p>The face-integrated cover law permits partial contact. Its resultant acts
at <code>nipContact.loadCentre</code>, producing a support torque. With
<code>world.lateralDynamics = true</code>, crossed axes also steer: axial
surface speed is <code>radius*angularVelocity*sin(crossing)</code>.
Longitudinal and lateral slip share one Triple-S budget. The stronger contact
controls tracking; equal covers favour the wrapped roller, which carries
nip load plus wrap load. Co-yawed axes add no relative steering.</p>
<p>Figure 4 shows radial runout: the drum centre orbits once per turn,
changing penetration and bearing unbalance. <code>runoutPhase</code> sets
the high point at zero shaft angle. <code>variableRunout = true</code> keeps
runout settable in an FMU; <code>variableTilt = true</code> does likewise for
yaw and tram.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/RollBody/runout.svg\"
     width=\"900\"
     alt=\"Radial runout as a drum circle orbiting the bearing axis\">
<strong>Figure 4:</strong> Radial runout of the nip drum.</p>
<h4>Connections and initialization</h4>
<p>Connect <code>nipLoading</code> to <code>Roller.frame_nip</code> with
<code>useNip = true</code>. The required port supplies centre geometry, web
transport, shaft angle, radius and rotation direction. Position the nip
inside the web-covered arc.</p>
<p><code>useSupport = true</code> exposes non-spinning
<code>frame_support</code>; external separation sets compression and the
actuator must limit its own force. The default false uses
<code>xPosition</code>/<code>zPosition</code> for contact direction and sets
middle compression to <code>loadFraction*maxNipLoad/coverStiffness</code>.
Parallel-axis static load is <code>loadFraction*maxNipLoad</code>;
misalignment changes it.</p>
<p><code>fixedInitialAngle</code>/<code>fixedInitialSpeed</code> fix zero
shaft angle/speed when enabled; both default false.
<code>world.enableAnimation</code> controls the drum display.</p>
<h4>Displayed variables</h4>
<p>Results are grouped in <code>nipVariables</code>:</p>
<ul>
<li><code>nipLoad</code> [N]: Compressive contact load.</li>
<li><code>nipTraction</code> [N]: Tangential force on the web, positive along web travel.</li>
<li><code>nipSlip</code> [m/s]: Nip contact-point velocity minus web velocity along web travel.</li>
<li><code>penetration</code> [m]: Undeformed overlap at the middle of the face; negative means clearance.</li>
<li><code>angularVelocity</code> [rad/s]: Nip shaft angular velocity.</li>
</ul>
<p>Contact diagnostics such as <code>relativeMisalignment</code>,
<code>wedgeAngle</code>, <code>edgeLift</code>, <code>endOpening</code>,
<code>contactWidth</code>, <code>loadCentre</code> and
<code>frictionLoss</code> stay on the public <code>nipContact</code> part.</p>
<h4>Limitations</h4>
<p>Independent cover springs omit the elliptical crossed-cylinder patch and
face shear. Damping uses middle approach speed scaled by contact width;
lateral drag acts at mid-face. Finite face overlap and placement inside the
wrapped arc are unchecked. Touchdown can produce a damping-force jump;
compression and transient loads can exceed <code>maxNipLoad</code>.
Regularized friction requires finite slip.</p>
<p><a href=\"modelica://Roll2RollDynamics.Examples.NipLoading\">NipLoading</a>
demonstrates engagement, spin-up, traction, misalignment, runout and steering.</p>
</html>"));
end NipRoller;
