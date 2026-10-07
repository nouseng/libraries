within Roll2RollDynamics.Components;
model Winder
  "Winding and unwinding with variable radius and conservative material transport"
  extends Roll2RollDynamics.Utilities.Icons.Winder;
  //Parameters
  // Geometry
  parameter Modelica.Units.SI.Radius radius0 = 0.15 "Initial wound roll radius"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length beltThickness = world.web.thickness "Web/belt thickness"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length width = world.rollWidth "Roll face width (cross-machine)"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Radius coreRadius = world.coreRadius "Core/hub radius"
    annotation(Dialog(group = "Geometry"));
  parameter Integer rotationDirection(min = -1, max = 1) = 1
    "= +1, the roll turns so its upper surface moves with the web; = -1, the web leaves underneath and the roll counter-rotates"
    annotation(Dialog(group = "Geometry"));

  // Mounting
  parameter Boolean useSupport = false
    "= true, expose the support frame; = false, ground the support internally at the roll position"
    annotation(Dialog(group = "Mounting"));
  parameter Modelica.Units.SI.Length xPosition = 0
    "Roll center machine-direction position"
    annotation(Dialog(group = "Mounting", enable = not useSupport));
  parameter Modelica.Units.SI.Length zPosition = 0 "Roll center height"
    annotation(Dialog(group = "Mounting", enable = not useSupport));

  // Dynamics
  parameter Integer winding(min = -1, max = 1) = 1
    "= +1, the roll rewinds and the radius grows; = -1, it unwinds and the radius shrinks"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.Density density = world.web.density "Wound web density"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.RotationalDampingConstant bearingDamping(min=0) = world.bearingDamping
    "Viscous bearing damping"
    annotation(Dialog(group = "Dynamics"));

  // Initialization
  parameter Boolean fixedInitialAngle = false "= true, initialize shaft angle at zero"
    annotation(Dialog(tab = "Initialization"));
  parameter Boolean fixedInitialWebState=true
    "= true, initialize the boundary transport coordinate at zero; set false only when the enclosing model prescribes its own transport initial conditions"
    annotation(Dialog(tab="Initialization"));
  parameter Boolean fixedInitialSpeed = false
    "= true, start the shaft at w0 instead of letting the initial system choose"
    annotation(Dialog(tab = "Initialization"));
  parameter Modelica.Units.SI.AngularVelocity w0 = 0 "Initial shaft speed"
    annotation(Dialog(tab = "Initialization", enable = fixedInitialSpeed));
  parameter Modelica.Units.SI.Angle tangencyAngleStart=0
    "Initial guess selecting the directed tangency route"
    annotation(Dialog(tab="Initialization"));

  //Variables
  // Main results, grouped into one browser node; radius stays flat because
  // drive references such as w_ref = speed/unwind.radius read it directly.
  record WinderVariables "Winder results a line engineer plots"
    Modelica.Units.SI.Force tension "Web tension derived from frame_t force";
    Modelica.Units.SI.Mass webMass "Wound material inventory";
    Modelica.Units.SI.MassFlowRate massFlow "Signed mass flow into the wound roll";
    Modelica.Units.SI.Velocity surfaceVelocity "Roll surface speed";
    Modelica.Units.SI.Velocity crossingVelocity "Web velocity relative to moving tangency";
    Modelica.Units.SI.Inertia J "Instantaneous roll inertia from the wound geometry";
  end WinderVariables;
  WinderVariables winderVariables "Winder results displayed in the variable browser";

  //Inputs and Outputs
  // The radius is the wound-roll geometry state. Tangency angles are
  // algebraic: the span-perpendicularity constraint is insensitive to the
  // radius at the tangency, so a tool that picks an angle as state and solves
  // the radius from geometry meets a singular Jacobian.
  output Modelica.Units.SI.Radius radius(start = radius0, fixed = true, stateSelect = StateSelect.always)
    "Instantaneous wound roll radius";
  output Modelica.Units.SI.Angle angle "Roll turning angle"
    annotation(HideResult = true);
  output Modelica.Units.SI.Angle tangencyAngle(start=tangencyAngleStart)
    "Tangency angle solved from the web geometry"
    annotation(HideResult = true);
  output Modelica.Units.SI.Torque bearingTorque
    "Retarding torque the bearings apply to the shaft"
    annotation(HideResult = true);

  //Components
  outer Roll2RollDynamics.WebWorld world "Multibody world and shared line defaults";

protected
  //Parameters
  final parameter Modelica.Units.SI.Length markThickness = beltThickness/2
    "Thickness of the slices lying on the roll faces";
  final parameter Real rollColor[3] = {115, 115, 120} "Core/hub and mark color";

  //Variables
  Real webColor[3] "Wound web color, same stress gradient as the belt";

  // Working names of the displayed results; winderVariables carries them outward.
  Modelica.Units.SI.Force tension "Web tension derived from frame_t force";
  Modelica.Units.SI.Mass webMass "Wound material inventory";
  Modelica.Units.SI.MassFlowRate massFlow "Signed mass flow into the wound roll";
  Modelica.Units.SI.Velocity surfaceVelocity "Roll surface speed";
  Modelica.Units.SI.Velocity crossingVelocity "Web velocity relative to moving tangency";
  Modelica.Units.SI.Inertia J "Instantaneous roll inertia from the wound geometry";
  Modelica.Units.SI.Velocity radiusRate "Wound radius growth rate";
public
  //Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_support if useSupport "Support connection"
    annotation(Placement(transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Roll2RollDynamics.Utilities.Interfaces.RollPort frame_t
    "Web tangency geometry and material transport"
    annotation(Placement(transformation(origin = {200, 0}, extent = {{-16, 16}, {16, -16}}), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a "Drive/brake port"
    annotation(Placement(transformation(origin = {-10, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -180), iconTransformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}})));

protected
  //Components
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedSupport(animation = false, r = {xPosition, 0, zPosition}) if not useSupport
    "Internally grounded support at the roll position"
    annotation(Placement(transformation(extent = {{-100, 20}, {-90, 30}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation baseTranslation(r = {0, 0, 0}, animation = false)
    "Roll center junction (position is set by the support)"
    annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(
    n = {0, 1, 0},
    useAxisFlange = true,
    phi(start = 0, fixed = fixedInitialAngle),
    w(start = w0, fixed = fixedInitialSpeed))
    "Roll rotation about the cross-machine axis"
    annotation(Placement(transformation(origin = {-10, 20}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.WebWeight rollBody(
    mass = webMass,
    transportForce = -massFlow*rotationDirection*surfaceVelocity*
      Modelica.Mechanics.MultiBody.Frames.resolve1(baseTranslation.frame_b.R,
        {cos(tangencyAngle),0,-sin(tangencyAngle)})) "Wound material weight and support acceleration"
    annotation(Placement(transformation(origin={50,20},extent={{-10,-10},{10,10}})));
  Roll2RollDynamics.Utilities.Parts.VariableInertia rollInertia(J = J,
    momentumFlow = massFlow*radius^2*revolute.w) "Wound roll rotational inertia"
    annotation(Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Damper rollBearing(d = bearingDamping)
    "Drag of the bearings the roll turns in"
    annotation(Placement(transformation(origin = {-40, 50}, extent = {{10, -10}, {-10, 10}})));
  Roll2RollDynamics.Utilities.Parts.TangentFrame tangencyTranslation(direction=rotationDirection, angle=tangencyAngle,
    spanDirection=frame_t.spanDirection,
    transmitAxialMoment=false,
    r = {radius*sin(tangencyAngle), 0, radius*cos(tangencyAngle)})
    "Roll-center to tangency load path"
    annotation(Placement(transformation(origin = {90, -0}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.VariableGearR2T surfaceGear(radius = rotationDirection*radius,
    velocityOffset = -rotationDirection*radius*tangencyTranslation.angleRate)
    "Variable-radius web/shaft transformer"
    annotation(Placement(transformation(origin = {60, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape winder(
    shapeType = "cylinder",
    length = world.web.width,
    width = 2*radius,
    height = 2*radius,
    color = webColor,
    lengthDirection = {0, 1, 0},
    widthDirection = {1, 0, 0},
    r_shape = {0, -world.web.width/2, 0},
    R = revolute.frame_b.R,
    r = revolute.frame_b.r_0) if world.enableAnimation "Wound web roll";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape hub(
    shapeType = "cylinder",
    length = width,
    width = 2*coreRadius,
    height = 2*coreRadius,
    color = rollColor,
    lengthDirection = {0, 1, 0},
    widthDirection = {1, 0, 0},
    r_shape = {0, -width/2, 0},
    R = revolute.frame_b.R,
    r = revolute.frame_b.r_0) if world.enableAnimation "Core hub";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape faceMark[2](
    each shapeType = "box",
    each length = 0.95*radius,
    each width = radius/8,
    each height = markThickness,
    each color = rollColor,
    each lengthDirection = {1, 0, 0},
    each widthDirection = {0, 0, 1},
    r_shape = {{0, s*(world.web.width + markThickness + beltThickness/10)/2, 0}
      for s in {-1, 1}},
    each R = revolute.frame_b.R,
    each r = revolute.frame_b.r_0) if world.enableAnimation
    "Rotation marks on the roll faces";

initial equation
  if fixedInitialWebState then
    frame_t.web.s=0;
  end if;

equation
  // Displayed results, grouped for the variable browser
  winderVariables.tension = tension;
  webColor = Roll2RollDynamics.Functions.stressColor(
    tension/(beltThickness*world.web.width),
    world.stressLow, world.stressHigh, world.stressNominal, world.stressColorGamma,
    world.webColor, world.nominalColor, world.stressColor);
  winderVariables.webMass = webMass;
  winderVariables.massFlow = massFlow;
  winderVariables.surfaceVelocity = surfaceVelocity;
  winderVariables.crossingVelocity = crossingVelocity;
  winderVariables.J = J;
  connect(tangencyTranslation.frame_b, frame_t.frame)
    annotation(Line(points={{-50, 0},{50,0}},color={95,95,95}, origin = {150, 0}));
  connect(surfaceGear.flangeT, frame_t.web)
    annotation(Line(points = {{70, 70}, {180, 70}, {180, 9.6}, {200, 9.6}}, color = {0, 127, 0}));
  assert(rotationDirection == 1 or rotationDirection == -1,
    "Winder: rotationDirection must be -1 or +1");
  if useSupport then
    connect(frame_support, baseTranslation.frame_a)
      annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-90, 0}));
  else
    connect(fixedSupport.frame_b, baseTranslation.frame_a)
      annotation(Line(points = {{-90, 25}, {-70, 25}, {-70, 30}}, color = {95, 95, 95}, thickness = 0.5));
  end if;
  connect(baseTranslation.frame_b, revolute.frame_a)
    annotation(Line(points = {{-12.5, -10}, {-7.5, -10}, {-7.5, 10}, {27.5, 10}}, color = {95, 95, 95}, thickness = 0.5, origin = {-47.5, 10}));
  connect(revolute.frame_b, rollBody.frame_a)
    annotation(Line(points = {{-20, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {20, 20}));
  connect(revolute.axis, rollInertia.flange_a)
    annotation(Line(points = {{-3.333, -13.333}, {-3.333, 6.667}, {6.667, 6.667}}, origin = {-6.667, 43.333}));
  connect(revolute.axis, surfaceGear.flangeR)
    annotation(Line(points = {{-20, -26.667}, {-20, 13.333}, {40, 13.333}}, origin = {10, 56.667}));
  connect(baseTranslation.frame_b, tangencyTranslation.frame_a)
    annotation(Line(points = {{-70, -0}, {70, 0}}, color = {95, 95, 95}, thickness = 0.5, origin = {10, 0}));
  connect(revolute.axis, flange_a)
    annotation(Line(points = {{0, -35}, {-0, 35}}, origin = {-10, 65}));
  connect(revolute.axis, rollBearing.flange_a)
    annotation(Line(points = {{-10, 30}, {-10, 50}, {-30, 50}}, color = {0, 0, 0}));
  connect(rollBearing.flange_b, revolute.support)
    annotation(Line(points = {{-11.6, 10}, {-16.6, 10}, {-16.6, -5}, {22.4, -5}, {22.4, -10}}, origin = {-38.4, 40}));

  // Material inventory and boundary flow
  crossingVelocity = der(surfaceGear.flangeT.s);
  webMass = density*Modelica.Constants.pi*(radius^2 - coreRadius^2)*world.web.width;
  massFlow = winding*world.web.density*world.web.thickness*world.web.width
    *crossingVelocity/frame_t.stretch;
  // The explicit rate keeps der(radius) out of the differentiated tangency
  // geometry; the belt needs tangency velocity and acceleration.
  radiusRate = der(radius);
  radiusRate = massFlow/(2*density*Modelica.Constants.pi*radius*world.web.width);

  // Operating limits
  assert(radius > coreRadius, "Conservative Winder has exhausted the wound web");
  assert(tension > 0, "Conservative Winder requires a taut web");
  assert(abs(frame_t.frame.f[1]-frame_t.web.f) < 1e-6*(1+abs(tension)),
    "Winder requires consistent directed spatial and transport effort");
  assert(winding == 1 or winding == -1, "Conservative Winder requires winding = -1 or +1");
  assert(Modelica.Math.Vectors.length(Modelica.Mechanics.MultiBody.Frames.angularVelocity1(
    baseTranslation.frame_b.R)) < 1e-8,
    "Conservative Winder requires fixed support orientation");
  assert(abs(beltThickness/world.web.thickness - 1) < 1e-12
    and abs(density/world.web.density - 1) < 1e-12,
    "Conservative Winder must use the shared reference material");
  // Shaft inertia, support load and motion
  tension = -winding*surfaceGear.flangeT.f;
  J = webMass*(radius^2 + coreRadius^2)/2;
  bearingTorque = rollBearing.flange_a.tau;
  angle = revolute.phi;
  surfaceVelocity = rotationDirection*revolute.w*radius;

  annotation(
    Diagram(coordinateSystem(extent = {{-100, -100}, {200, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {
      Text(textColor = {64, 64, 64}, extent = {{-100, 86}, {-40, 100}}, textString = "%name",
        horizontalAlignment = TextAlignment.Left)}),
    Documentation(info = "<html>
<h4>Winder</h4>
<p>A wound roll stores web; its radius, mass and annular inertia follow the
inventory. Figure 1 shows its geometry and connections.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Winder/winder-geometry.svg\"
     width=\"600\"
     alt=\"Wound roll with growing radius, drive flange, web tangency and rotation direction\">
<strong>Figure 1:</strong> Wound-roll geometry and connections.</p>
<p>Variable inertia includes stored angular momentum and material momentum
flux [1]: material arriving at surface speed adds no acceleration torque
merely because inertia grows. <code>crossingVelocity</code> includes tangency
migration; <code>frame_t</code> stretch converts it to reference-material
flow. Wound-web weight, support acceleration and momentum-flow reaction
act on the support; the adjoining span carries its own loads.</p>
<p><code>winding</code> = <code>+1</code> rewinds, <code>-1</code> unwinds<br>
<code>rotationDirection</code> = Positive shaft surface-travel direction,
independent of <code>winding</code><br>
<code>radius0</code>, <code>coreRadius</code> = Initial and empty-core radii<br>
<code>width</code> = Visible hub only; mass/inertia use
<code>world.web.width</code><br>
<code>tangencyAngleStart</code> = Initial route guess<br>
<code>bearingDamping</code> = Viscous shaft drag</p>
<h4>Connections and initialization</h4>
<p>Connect <code>frame_t</code> to <code>Belt</code> or <code>WebForce</code>
and <code>flange_a</code> to the drive/brake. <code>useSupport = true</code>
exposes <code>frame_support</code>; default false fixes the housing at
<code>xPosition</code>/<code>zPosition</code>.</p>
<p><code>radius0</code> fixes initial radius. <code>fixedInitialAngle</code>,
<code>fixedInitialSpeed</code> and <code>fixedInitialWebState</code> control
the remaining states. On speed-driven lines, retain the adjacent span's
<code>fixedInitialTension = true</code>, including startup from rest.</p>
<h4>Displayed variables</h4>
<p>Wound-roll results are grouped in <code>winderVariables</code>:</p>
<ul>
<li><code>tension</code> [N]: Web tension at <code>frame_t</code>.</li>
<li><code>webMass</code> [kg]: Wound material inventory.</li>
<li><code>massFlow</code> [kg/s]: Signed material flow into the wound roll; positive adds inventory.</li>
<li><code>surfaceVelocity</code> [m/s]: Roll surface velocity with the selected <code>rotationDirection</code>.</li>
<li><code>crossingVelocity</code> [m/s]: Web velocity relative to the moving tangency point.</li>
<li><code>J</code> [kg.m2]: Rotational inertia of the wound material.</li>
</ul>
<p><code>radius</code> stays at model level so drive references can read it
directly, as in <code>w_ref = speed/unwind.radius</code>;
<code>angle</code>, <code>tangencyAngle</code> and
<code>bearingTorque</code> carry <code>HideResult = true</code>.</p>
<h4>Limitations</h4>
<p>The web must stay taut and radius above <code>coreRadius</code>. The core
has no separate mass/inertia. Relaxed winding omits wound-in stress,
compaction, deposition-layer offsets and contact slip. Mounts may translate
but must keep fixed orientation.</p>
<p><a href=\"modelica://Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine\">MisalignedIdlerLine</a>
shows an unwind-to-rewind line.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/resources/mit2_25f13_fundam_law-son/\">1</a>]
A. A. Sonin, &quot;Fundamental Laws of Motion for Particles, Material Volumes,
and Control Volumes&quot;, MIT, 2003, secs. 3.1&ndash;3.3,
eqs. (26A), (27A), (29A). Mass and momentum balances for moving boundaries;
the annular winding approximation is derived here. Accessed 2026-09-06.</li>
</ul>
</html>"));
end Winder;
