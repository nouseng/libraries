within Roll2RollDynamics.Utilities.Parts;
model RollBody
  "Mounted rotating roll body with bearing drag and surface animation"

  // Parameters
  parameter Modelica.Units.SI.Radius radius "Mechanical roll radius";
  parameter Modelica.Units.SI.Length width "Roll face width";
  parameter Modelica.Units.SI.Length beltThickness "Web thickness";
  parameter Modelica.Units.SI.Angle yaw = 0
    "In-plane misalignment: axis turned about the vertical, ends stay level";
  parameter Modelica.Units.SI.Angle tram = 0
    "Out-of-plane misalignment: axis tilted about the machine direction, one end up";
  parameter Boolean variableTilt = false
    "= true, keep yaw and tram as run-time parameters at some simulation cost";
  parameter Modelica.Units.SI.Length runout = 0
    "Radial runout: drum centre offset from the bearing axis";
  parameter Modelica.Units.SI.Angle runoutPhase = 0
    "Direction of the runout high point at zero shaft angle, from the machine direction toward up";
  parameter Boolean variableRunout = false
    "= true, keep runout as a run-time parameter at some simulation cost";
  parameter Integer rotationDirection(min = -1, max = 1) = 1
    "Signed web travel direction around the roll";
  parameter Boolean useSupport = false "Expose the support frame";
  parameter Modelica.Units.SI.Length xPosition = 0 "Fixed roll x-position";
  parameter Modelica.Units.SI.Length zPosition = 0 "Fixed roll height";
  parameter Boolean useFlange = false "Expose the drive/brake flange";
  parameter Boolean animation = true "= true, draw the roll drum and its face marks";
  parameter Modelica.Units.SI.RotationalDampingConstant bearingDamping(min=0)
    "Viscous bearing damping";
  parameter Modelica.Units.SI.Density density = 2700 "Shell density";
  parameter Modelica.Units.SI.Length wallThickness = 0.006
    "Shell wall thickness";
  parameter Boolean fixedInitialAngle = false "Fix initial shaft angle";
  parameter Boolean fixedInitialSpeed = false "Fix initial shaft speed";
  parameter Boolean useDynamicColor = false "Animate coating state";
  parameter Real rollColor[3] = {150, 150, 150} "Base roll colour";
  parameter Real elastomerColor[3] = {220, 80, 40}
    "High-friction coating colour";
  parameter Modelica.Units.SI.Force FnThreshold = 220
    "Normal force at the coating transition";
  parameter Modelica.Units.SI.Force FnSharpness = 15
    "Normal-force width of the coating transition";

  // Inputs and Outputs
  input Modelica.Units.SI.Force normalForce "Tangency-load magnitude used for coating colour";
  input Modelica.Units.SI.Radius drumDrawnRadius
    "Displayed drum radius below the segmented web";
  output Modelica.Units.SI.Velocity surfaceVelocity
    "Roll surface velocity along web travel";
  output Modelica.Units.SI.Angle angle "Shaft angle";
  output Modelica.Units.SI.Torque bearingTorque "Bearing drag torque";
  output Real drumSurfaceColor[3] "Displayed drum colour";

protected
  // Parameters
  final parameter Modelica.Units.SI.Radius innerRadius = max(0, radius - wallThickness)
    "Roll shell inner radius";
  final parameter Modelica.Units.SI.Mass mass = density*Modelica.Constants.pi*(radius^2 - innerRadius^2)*width
    "Roll mass from the shell geometry";
  final parameter Modelica.Units.SI.Inertia axialInertia = 0.5*mass*(radius^2 + innerRadius^2)
    "Inertia about the cross-machine rotation axis";
  final parameter Modelica.Units.SI.Inertia transverseInertia = mass*(3*(radius^2 + innerRadius^2) + width^2)/12
    "Inertia about the two transverse axes";
  final parameter Real markColor[3] = {50, 50, 55} "Registration mark colour";
  final parameter Modelica.Units.SI.Length markThickness = beltThickness/2
    "Thickness of the slices lying on the roll faces";

  // Components
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation baseTranslation(
    r = {0, 0, 0}, animation = false)
    "Roll centre junction"
    annotation(Placement(transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedSupport(
    animation = false, r = {xPosition, 0, zPosition}) if not useSupport
    "Internally grounded roll support"
    annotation(Placement(transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation skewRotation(
    n = {0, 0, 1},
    angle = yaw*180/Modelica.Constants.pi,
    animation = false) if not variableTilt
    "Yaw of the roll axis about the vertical"
    annotation(Placement(transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -450)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation tramRotation(
    n = {1, 0, 0},
    angle = tram*180/Modelica.Constants.pi,
    animation = false) if not variableTilt
    "Tilt of the roll axis about the machine direction"
    annotation(Placement(transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = -450)));
  Roll2RollDynamics.Utilities.Parts.AxisTilt axisTilt(
    yaw = yaw,
    tram = tram) if variableTilt
    "Run-time yaw and tram of the roll axis"
    annotation(Placement(transformation(origin = {-80, -25}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(
    n = {0, 1, 0},
    useAxisFlange = true,
    animation = false,
    phi(fixed = fixedInitialAngle),
    w(fixed = fixedInitialSpeed))
    "Roll rotation about the cross-machine axis"
    annotation(Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation eccentricity(
    r = runout*{cos(runoutPhase), 0, sin(runoutPhase)},
    animation = false) if not variableRunout
    "Drum centre offset from the bearing axis, turning with the shaft"
    annotation(Placement(transformation(origin = {25, -30}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.DrumRunout drumRunout(
    runout = runout,
    runoutPhase = runoutPhase) if variableRunout
    "Run-time drum offset, turning with the shaft"
    annotation(Placement(transformation(origin = {35, -55}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.VariableTranslation orbit(
    r = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_center.R,
      frame_drum.r_0 - frame_center.r_0))
    "Non-spinning follower of the orbiting drum centre"
    annotation(Placement(transformation(origin = {-90, -75}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body drumBody(
    m = mass,
    r_CM = {0, 0, 0},
    I_11 = transverseInertia,
    I_22 = axialInertia,
    I_33 = transverseInertia,
    animation = false)
    "Roll shell mass and inertia"
    annotation(Placement(transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Damper rollBearing(d = bearingDamping)
    "Drag of the roll bearings"
    annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -540)));
  // Alias anchors for the drum pose. OpenModelica keeps the shallowest
  // non-connector name of an alias set; without these it keeps a shape's
  // R.T or r, and the derivative it then needs of that shape variable shows
  // up in the 3D viewer as a stray "Unknown type DUMMY" capsule.
  Real drumTransform[3, 3](each unit = "1") = frame_drum.R.T
    "Drum orientation, transformation matrix from world to drum frame";
  Modelica.Units.SI.Position drumPosition[3] = frame_drum.r_0
    "Drum centre position resolved in the world frame";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape drumSurface(
    shapeType = "cylinder",
    length = width,
    width = 2*drumDrawnRadius,
    height = 2*drumDrawnRadius,
    color = drumSurfaceColor,
    specularCoefficient = 0.3,
    lengthDirection = {0, 1, 0},
    widthDirection = {1, 0, 0},
    r_shape = {0, -width/2, 0},
    R = frame_drum.R,
    r = frame_drum.r_0) if animation "Roll drum surface";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape faceMark[2](
    each shapeType = "box",
    each length = 0.95*radius,
    each width = radius/8,
    each height = markThickness,
    each color = markColor,
    each lengthDirection = {1, 0, 0},
    each widthDirection = {0, 0, 1},
    r_shape = {{0, s*(width + markThickness + beltThickness/10)/2, 0}
      for s in {-1, 1}},
    each R = frame_drum.R,
    each r = frame_drum.r_0) if animation
    "Rotation marks on the roll faces";

  // Physical connectors
public Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_support if useSupport
    "Support/housing connection"
    annotation(Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_center
    "Housing-aligned centre frame for web geometry and normal reaction"
    annotation(Placement(transformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = -360), iconTransformation(origin = {-180, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_wrap
    "Housing-aligned frame at the orbiting drum centre, for wrap and nip geometry"
    annotation(Placement(transformation(origin = {-120, -75}, extent = {{-20, -20}, {20, 20}}, rotation = -360), iconTransformation(origin = {-100, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_drum
    "Drum centre frame rotating with the shaft, carrying contact force and torque"
    annotation(Placement(transformation(origin = {50, -100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a if useFlange
    "Drive/brake port"
    annotation(Placement(transformation(origin = {120, -10}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {200, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(frame_support, baseTranslation.frame_a)
    annotation(Line(points = {{-46.16, 3.333}, {-46.16, -36.667}, {23.587, -36.667}, {23.84, -46.667}}, color = {95, 95, 95}, thickness = 0.5, origin = {-73.84, 76.667}));
  connect(fixedSupport.frame_b, baseTranslation.frame_a)
    annotation(Line(points = {{0, 10}, {-0, -10}}, color = {95, 95, 95}, thickness = 0.5, origin = {-50, 40}));
  connect(baseTranslation.frame_b, skewRotation.frame_a)
    annotation(Line(points = {{0, 5}, {0, -5}}, color = {95, 95, 95}, thickness = 0.5, origin = {-50, 5}));
  connect(skewRotation.frame_b, tramRotation.frame_a)
    annotation(Line(points = {{0, 5}, {0, -5}}, color = {95, 95, 95}, thickness = 0.5, origin = {-50, -25}));
  connect(tramRotation.frame_b, revolute.frame_a)
    annotation(Line(points = {{-50, -50}, {-50, -60}, {-20, -60}, {-20, -30}, {-10, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(tramRotation.frame_b, frame_center)
    annotation(Line(points = {{-50, -50}, {-50, -60}, {-90, -60}, {-90, -30}, {-120, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(baseTranslation.frame_b, axisTilt.frame_a)
    annotation(Line(points = {{-50, 10}, {-50, 5}, {-80, 5}, {-80, -15}}, color = {95, 95, 95}, thickness = 0.5));
  connect(axisTilt.frame_b, revolute.frame_a)
    annotation(Line(points = {{-80, -35}, {-80, -60}, {-20, -60}, {-20, -30}, {-10, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(axisTilt.frame_b, frame_center)
    annotation(Line(points = {{-80, -35}, {-80, -60}, {-90, -60}, {-90, -30}, {-120, -30}}, color = {95, 95, 95}, thickness = 0.5));
  // The wrap frame follows the drum centre while keeping the housing
  // orientation; with zero runout it coincides with the centre frame. The
  // follower is unconditional so that a structural switch on runout does
  // not fold the parameter at translation and break run-time runout
  connect(frame_center, orbit.frame_a)
    annotation(Line(points = {{-120, -30}, {-110, -30}, {-110, -75}, {-100, -75}}, color = {95, 95, 95}, thickness = 0.5));
  connect(orbit.frame_b, frame_wrap)
    annotation(Line(points = {{-80, -75}, {-80, -90}, {-120, -90}, {-120, -75}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute.frame_b, eccentricity.frame_a)
    annotation(Line(points = {{10, -30}, {15, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(eccentricity.frame_b, frame_drum)
    annotation(Line(points = {{-10, 23.333}, {5, 23.333}, {5, -46.667}}, color = {95, 95, 95}, thickness = 0.5, origin = {45, -53.333}));
  connect(revolute.frame_b, drumRunout.frame_a)
    annotation(Line(points = {{10, -30}, {15, -30}, {15, -55}, {25, -55}}, color = {95, 95, 95}, thickness = 0.5));
  connect(drumRunout.frame_b, frame_drum)
    annotation(Line(points = {{-3.333, 15}, {1.667, 15}, {1.667, -30}}, color = {95, 95, 95}, thickness = 0.5, origin = {48.333, -70}));
  connect(frame_drum, drumBody.frame_a)
    annotation(Line(points = {{-3.333, -46.667}, {-3.333, 23.333}, {6.667, 23.333}}, color = {95, 95, 95}, thickness = 0.5, origin = {53.333, -53.333}));
  connect(revolute.axis, flange_a)
    annotation(Line(points = {{0, -20}, {0, -10}, {120, -10}, {120, -10}}));
  connect(revolute.axis, rollBearing.flange_a)
    annotation(Line(points = {{-8, -10}, {-8, -5}, {7, -5}, {7, 10}, {2, 10}}, origin = {8, -10}));
  connect(rollBearing.flange_b, revolute.support)
    annotation(Line(points = {{0.4, 10}, {-4.6, 10}, {-4.6, -5}, {4.4, -5}, {4.4, -10}}, origin = {-10.4, -10}));

  surfaceVelocity = rotationDirection*revolute.w*(radius+beltThickness/2);
  angle = revolute.phi;
  bearingTorque = rollBearing.flange_a.tau;
  if useDynamicColor then
    drumSurfaceColor = rollColor + (elastomerColor - rollColor)
      /(1 + exp(-(normalForce - FnThreshold)/FnSharpness));
  else
    drumSurfaceColor = rollColor;
  end if;

  annotation(
    Line(points = {{-13.333, -13.333}, {-13.333, 6.667}, {26.667, 6.667}}, origin = {13.333, -6.667}),
    Line(points = {{60, 0}, {120, 0}, {120, 60}, {0, 60}, {0, 100}}, color = {0, 127, 0}),
    Icon(coordinateSystem(extent = {{-200, -100}, {200, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Line(origin = {42, 0}, rotation = -90, points = {{0, 158}, {0, 35.517}}, color = {95, 95, 95}, thickness = 0.5),
        Line(points = {{0, -44}, {0, -100}}),
        Ellipse(origin = {0, -2}, lineColor = {64, 64, 64}, fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-78, 78}, {78, -78}}),
        Ellipse(lineColor = {95, 95, 95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid,extent = {{-22, 22}, {22, -22}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-9, 9}, {9, -9}}),
        Rectangle(origin = {-0, 0}, lineColor = {128, 128, 128}, fillColor = {0, 116, 0}, lineThickness = 5, extent = {{-197.678, -97}, {197.678, 97}}, radius = 25)}),
    Diagram(coordinateSystem(extent = {{-120, -100}, {120, 80}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
      Text(origin = {40, -30}, textColor = {64, 64, 64}, extent = {{-40, 70}, {40, 90}}, textString = "%name")}),
    Documentation(info = "<html>
<h4>Roll body</h4>
<p>This component supplies roll mounting, shaft dynamics, viscous bearing drag
and drum animation. Hollow-shell mass and inertia follow from
<code>radius</code>, <code>width</code>, <code>wallThickness</code> and
<code>density</code>; separate parts supply wrap geometry and friction.</p>
<p><code>yaw</code> turns the axis about vertical; subsequent <code>tram</code>
tilts it about the machine direction. Setting <code>variableTilt = true</code>
uses <a href=\"modelica://Roll2RollDynamics.Utilities.Parts.AxisTilt\">AxisTilt</a>
to keep both angles settable in an exported FMU.
<code>bearingDamping</code> sets the viscous
<a href=\"modelica://Modelica.Mechanics.Rotational.Components.Damper\">Damper</a>.</p>
<p>Setting <code>useSupport = true</code> exposes <code>frame_support</code>
for external mounting; otherwise <code>xPosition</code> and <code>zPosition</code>
ground the housing. Setting <code>useFlange = true</code> exposes drive/brake
shaft <code>flange_a</code>. <code>fixedInitialAngle</code> and
<code>fixedInitialSpeed</code> select shaft initialization.
<code>rotationDirection</code> sets the sign of <code>surfaceVelocity</code>.</p>
<p><code>frame_center</code> is the non-spinning bearing-axis frame.
<code>frame_wrap</code> shares its orientation at the drum centre and carries
wrap/nip geometry. Rotating <code>frame_drum</code> carries the shell and
contact moments; attaching those moments to the housing would bypass drum
inertia. Contacts obtain surface speed from this frame's spin.</p>
<p>Figure 1 shows radial <code>runout</code>, zero by default: drum and wrap
centres orbit once per turn, and eccentric shell mass loads the bearings.
<code>runoutPhase</code> locates the high point at zero shaft angle.
Default <code>FixedTranslation</code> folds the offset during translation;
<code>variableRunout = true</code> uses
<a href=\"modelica://Roll2RollDynamics.Utilities.Parts.DrumRunout\">DrumRunout</a>
to retain FMU tunability at some simulation cost.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/RollBody/runout.svg\"
     width=\"900\" alt=\"Radial runout as a drum circle orbiting the bearing axis\">
<strong>Figure 1:</strong> Radial runout of the drum.</p>
<p><code>bearingTorque</code> reports viscous drag. The drum and face marks
follow shaft motion; <code>useDynamicColor</code> changes coating from
<code>rollColor</code> to <code>elastomerColor</code> around <code>FnThreshold</code>.</p>
<h4>Limitations</h4>
<p>The body is a cylindrical shell; crowning, spokes, shafts and layered or
segmented construction need a specialized model. Angular runout is omitted.
Viscous drag is independent of radial load and omits seal friction,
breakaway stiction and detailed bearing losses.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/321814\">1</a>]
D. P. Jones, &quot;Traction in Web Handling: A Review&quot;, Proc.
International Conference on Web Handling, Oklahoma State University, 2001. Roll
and idler behaviour and the traction a wrap allows. Accessed 2026-09-04.</li>
</ul>
</html>"));
end RollBody;
