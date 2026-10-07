within Roll2RollDynamics.Utilities.Parts;
model WebWrap
  "Directed tangency geometry, spatial web loading and wrap animation"
  //Parameters
  parameter Modelica.Units.SI.Radius radius "Mechanical roll radius";
  parameter Modelica.Units.SI.Length webWidth "Web width";
  parameter Modelica.Units.SI.Length beltThickness "Web thickness";
  parameter Modelica.Units.SI.Angle yaw = 0 "Roll-axis yaw";
  parameter Modelica.Units.SI.Angle tram = 0 "Roll-axis tilt";
  parameter Integer rotationDirection(min = -1, max = 1) = 1
    "Directed winding sense";
  parameter Boolean lateral = false "Enable cross-machine contact motion";
  parameter Boolean useDetachment = false
    "= true, the web releases when the roll leaves the free span line"
    annotation(Evaluate = true, HideResult = true);
  parameter Boolean initiallyEngaged = true
    "= true, the model starts with the web wrapped on the roll";
  parameter Boolean animation = true "= true, draw the wrapped web";
  parameter Integer arcSegments(min = 1) = 16 "Requested wrap segments, with at least four used";
  final parameter Integer drawingSegments = max(4, arcSegments)
    "Segment count that also accommodates wraps exceeding half a turn";
  final parameter Modelica.Units.SI.Radius centrelineRadius = radius + beltThickness/2
    "Radius used by web geometry, transport and contact torque";
  parameter Modelica.Units.SI.Angle entryAngleStart =
    if rotationDirection > 0 then -Modelica.Constants.pi/4
    else -3*Modelica.Constants.pi/4
    "Initial entry tangency guess";
  parameter Modelica.Units.SI.Angle exitAngleStart =
    entryAngleStart + rotationDirection*Modelica.Constants.pi/2
    "Initial exit tangency guess";
  parameter Modelica.Units.SI.Stress stressLow "Low colour limit";
  parameter Modelica.Units.SI.Stress stressHigh "High colour limit";
  parameter Modelica.Units.SI.Stress stressNominal "Stress drawn in nominalColor";
  parameter Real colorGamma(min = Modelica.Constants.small)
    "Stress-colour contrast exponent";
  parameter Real webColor[3] = {60, 110, 200} "Low-stress colour";
  parameter Real nominalColor[3] = {245, 210, 70} "Nominal colour";
  parameter Real stressColor[3] = {205, 50, 35} "High-stress colour";
  //Inputs and Outputs
  input Real spanDirectionA[3](each unit="1") "Entry material-travel direction in world coordinates";
  input Real spanDirectionB[3](each unit="1") "Exit material-travel direction in world coordinates";
  input Modelica.Units.SI.Force entryTension "Entry transport tension";
  input Modelica.Units.SI.Force exitTension "Exit transport tension";
  output Modelica.Units.SI.Force meanTension "Mean entry and exit transport tension";
  output Modelica.Units.SI.Angle wrapAngle "Magnitude of the mechanical wrap angle";
  output Modelica.Units.SI.Velocity boundaryVelocityA
    "Entry tangency speed along the directed wrap";
  output Modelica.Units.SI.Velocity boundaryVelocityB
    "Exit tangency speed along the directed wrap";
  output Real entryTangent[3](each unit="1") "Directed entry tangent in world coordinates";
  output Real exitTangent[3](each unit="1") "Directed exit tangent in world coordinates";
  output Real meanTangent[3](each unit="1") "Arc average of the directed tangent in world coordinates";
  output Modelica.Units.SI.Force normalForce "Resultant spatial load at the two tangencies";
  output Modelica.Units.SI.Angle entryAngle(start = entryAngleStart)
    "Solved entry tangency angle";
  output Modelica.Units.SI.Angle exitAngle(start = exitAngleStart)
    "Solved exit tangency angle";
  output Modelica.Units.SI.Angle arcShortest
    "Shortest signed arc between tangencies";
  output Modelica.Units.SI.Angle arcAngle "Signed mechanical wrap angle";
  output Modelica.Units.SI.Angle arcDrawn
    "Signed displayed wrap angle including overhang";
  output Modelica.Units.SI.Angle overhangAngle
    "Displayed arc past each tangency";
  output Modelica.Units.SI.Angle entryDrawn "Displayed wrap start angle";
  output Modelica.Units.SI.Length chord "Chord of one wrap segment";
  output Modelica.Units.SI.Length arcRadius "Segment-midpoint radius";
  output Modelica.Units.SI.Length arcLength "Mechanical wrapped-web length";
  output Modelica.Units.SI.Radius drumDrawnRadius
    "Drum display radius below the segmented web";
  output Real arcColor[3]
    "Wrapped-web stress colour at the mean transport tension";
  output Boolean engaged(start = initiallyEngaged, fixed = false)
    "= false, the roll has left the free span line";
  Modelica.Units.SI.Length tangencyRadius(start = centrelineRadius)
    "Centre-to-tangency distance; equals centrelineRadius while engaged";
  output Modelica.Units.SI.Length gap = tangencyRadius - centrelineRadius
    "Clearance from the roll surface to the free span line; zero while wrapped";
  //Tension at the midpoint of each drawn segment. A wrapped web carries the
  //entry tension at one tangency and the exit tension at the other, so a single
  //mean colour steps against the span ribbon where they meet. Colouring each
  //segment from its own midpoint tension makes both ends of the band agree with
  //the neighbouring span. The interpolation is linear in tension: the stored
  //end tensions are exact, the friction distribution between them is not
  //resolved by this display model.
  output Modelica.Units.SI.Force arcSegmentTension[drawingSegments] =
    {entryTension + (exitTension - entryTension)*(i - 0.5)/drawingSegments
      for i in 1:drawingSegments}
    "Transport tension at the midpoint of each drawn segment";
  output Real arcSegmentColor[drawingSegments, 3] =
    {Roll2RollDynamics.Functions.stressColor(
      arcSegmentTension[i]/(beltThickness*webWidth),
      stressLow, stressHigh, stressNominal, colorGamma,
      webColor, nominalColor, stressColor)
      for i in 1:drawingSegments}
    "Wrapped-web stress colour of each drawn segment";

protected
  //Parameters
  final parameter Modelica.Units.SI.Angle overhangMax =
    Modelica.Math.acos(max(-1, 1 - beltThickness/(2*centrelineRadius)))
    "Overhang at which the drawn band sinks half a web thickness below the span";
  //Components
  Modelica.Mechanics.MultiBody.Joints.Prismatic lateralJoint(
    n = {0, 1, 0},
    stateSelect = StateSelect.always,
    useAxisFlange = true,
    animation = false,
    s(start = 0, fixed = true),
    v(start = 0, fixed = true)) if lateral
    "Cross-machine freedom of the web contact line"
    annotation(Placement(transformation(origin = {-55, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lateralLock(
    r = {0, 0, 0}, animation = false) if not lateral
    "Rigid cross-machine contact-line constraint"
    annotation(Placement(transformation(origin = {55, -30}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.TangentFrame tangentA(direction=rotationDirection, angle=entryAngle,
    spanDirection=spanDirectionA,
    transmitAxialMoment=false,
    r = {tangencyRadius*sin(entryAngle), 0, tangencyRadius*cos(entryAngle)})
    "Roll-centre to entry tangency load path"
    annotation(Placement(transformation(origin = {-100, 40}, extent = {{10, -10}, {-10, 10}})));
  Roll2RollDynamics.Utilities.Parts.TangentFrame tangentB(direction=rotationDirection, angle=exitAngle,
    spanDirection=spanDirectionB,
    transmitAxialMoment=false,
    r = {tangencyRadius*sin(exitAngle), 0, tangencyRadius*cos(exitAngle)})
    "Roll-centre to exit tangency load path"
    annotation(Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape wrapArc[drawingSegments](
    each shapeType = "box",
    each length = chord,
    each width = webWidth,
    each height = beltThickness,
    color = arcSegmentColor,
    each lengthDirection = {1, 0, 0},
    each widthDirection = {0, 1, 0},
    each r_shape = {-chord/2, 0, 0},
    R = {Modelica.Mechanics.MultiBody.Frames.absoluteRotation(
      frame_center.R,
      Modelica.Mechanics.MultiBody.Frames.planarRotation({0, 1, 0},
        entryDrawn + arcDrawn*(i - 0.5)/drawingSegments, 0))
      for i in 1:drawingSegments},
    r = {tangentA.frame_a.r_0
      + Modelica.Mechanics.MultiBody.Frames.resolve1(
        frame_center.R,
        {arcRadius*sin(entryDrawn + arcDrawn*(i - 0.5)/drawingSegments),
          0,
          arcRadius*cos(entryDrawn + arcDrawn*(i - 0.5)/drawingSegments)})
      for i in 1:drawingSegments}) if animation
    "Web wrapped on the roll surface";
  //Physical connectors
public Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_center
    "Non-spinning yawed roll-centre frame"
    annotation(Placement(transformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-180, -100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    "Entry tangency frame"
    annotation(Placement(transformation(origin = {-200, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-200, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
    "Exit tangency frame"
    annotation(Placement(transformation(origin = {200, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {200, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_a lateralFlange if lateral
    "Cross-machine contact-line coordinate"
    annotation(Placement(transformation(origin = {200, -60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}})));

initial equation
  assert(radius > 0 and webWidth > 0 and beltThickness > 0,
    "WebWrap requires positive radius, webWidth and beltThickness");
  assert(abs(rotationDirection) == 1,
    "WebWrap: rotationDirection must be -1 or +1");
  assert(arcSegments >= 1, "WebWrap requires at least one requested segment");
  assert(stressHigh > stressLow and colorGamma > 0,
    "WebWrap requires stressHigh > stressLow and colorGamma > 0");
  pre(engaged) = initiallyEngaged or not useDetachment;

equation
  //Tangency and mechanical geometry
  assert(entryTension > 0 and exitTension > 0,
    "WebWrap requires taut entry and exit spans; zero tension does not define tangency");
  assert(noEvent(entryTension*cos(tangentA.skew) > Roll2RollDynamics.Utilities.Types.forceTol
    and exitTension*cos(tangentB.skew) > Roll2RollDynamics.Utilities.Types.forceTol),
    "WebWrap requires nonzero entry and exit tension projected into the roll plane");
  connect(tangentA.frame_b, frame_a)
    annotation(Line(points={{-110,40},{-160,40},{-160,0},{-200,0}},color={95,95,95}));
  connect(tangentB.frame_b, frame_b)
    annotation(Line(points={{110,40},{160,40},{160,0},{200,0}},color={95,95,95}));
  arcShortest = Modelica.Math.atan2(sin(exitAngle - entryAngle),
    cos(exitAngle - entryAngle));
  // Leaving contact is watched on the arc, which is free while engaged;
  // re-entering is watched on the radius, which is free while detached.
  // Both cross their threshold continuously, so this is a release, not
  // an impact.
  // Both comparisons must stay strict. pre() advances once per event
  // iteration, and the watched quantity sits exactly on its threshold at the
  // instant of switching, so a non-strict test would let the opposite
  // condition rise in the very next iteration and switch the mode straight
  // back, forever. Strictness gives the threshold value to neither trigger.
  // This is an exact-threshold argument, not a numerical tolerance: do not
  // relax it back to <=, and do not add hysteresis or a deadband.
  when {rotationDirection*arcShortest < 0 and pre(engaged) and useDetachment,
        tangencyRadius < centrelineRadius and not pre(engaged)} then
    engaged = not pre(engaged);
  end when;
  if not useDetachment then
    // useDetachment = false: engaged is still bound to a discrete
    // variable that can never actually toggle (the release edge above
    // requires useDetachment), but that variable-ness alone is enough to
    // make the branch below a genuine runtime switch between two equations
    // that constrain different unknowns (tangencyRadius vs. exitAngle),
    // which forces a dynamic state-set through Pantelides and produces a
    // structural singularity elsewhere in a full line. Fixing this branch
    // at compile time restores the single original equation with no
    // conditional.
    tangencyRadius = centrelineRadius;
  elseif pre(engaged) then
    // Read the mode through pre() to keep the discrete state out of the
    // continuous algebraic loop that also carries the event conditions.
    // The two branches agree at the switching threshold; pre() selects
    // the mode from the preceding event iteration.
    tangencyRadius = centrelineRadius;
  else
    exitAngle = entryAngle;
  end if;
  // Without detachment a negative shortest arc means the web took the long
  // way round. With it, that same sign means the roll has left the line.
  arcAngle = if useDetachment then (if engaged then arcShortest else 0)
    else noEvent(if rotationDirection*arcShortest >= 0 then arcShortest
    else arcShortest + rotationDirection*2*Modelica.Constants.pi);
  wrapAngle = rotationDirection*arcAngle;
  arcLength = wrapAngle*centrelineRadius;
  boundaryVelocityA = rotationDirection*centrelineRadius*tangentA.angleRate;
  boundaryVelocityB = rotationDirection*centrelineRadius*tangentB.angleRate;
  entryTangent = rotationDirection*Modelica.Mechanics.MultiBody.Frames.resolve1(
    frame_center.R, {cos(entryAngle), 0, -sin(entryAngle)});
  exitTangent = rotationDirection*Modelica.Mechanics.MultiBody.Frames.resolve1(
    frame_center.R, {cos(exitAngle), 0, -sin(exitAngle)});
  meanTangent = if noEvent(arcLength > Roll2RollDynamics.Utilities.Types.lengthTol)
    then (frame_b.r_0 - frame_a.r_0)/arcLength else entryTangent;
  assert(wrapAngle < 1.5*Modelica.Constants.pi,
    "WebWrap: the web wraps almost a full turn, which usually means rotationDirection contradicts where the roll is placed",
    AssertionLevel.warning);
  //Display geometry, independent of the mechanical arc
  // A detached roll must draw no wrap band. Without this the overhang alone
  // leaves a stub of web rendered in the line with the roll parked away.
  overhangAngle = if engaged then
    min(wrapAngle/drawingSegments + abs(yaw) + abs(tram), overhangMax) else 0;
  arcDrawn = arcAngle + 2*rotationDirection*overhangAngle;
  entryDrawn = entryAngle - rotationDirection*overhangAngle;
  chord = 2*centrelineRadius*sin(rotationDirection*arcDrawn/(2*drawingSegments));
  arcRadius = centrelineRadius*cos(arcDrawn/(2*drawingSegments));
  drumDrawnRadius =
    radius - (centrelineRadius - arcRadius) - beltThickness/20;
  assert(not animation or drumDrawnRadius > 0,
    "WebWrap display clearance requires more arcSegments for this radius and thickness");
  arcColor = Roll2RollDynamics.Functions.stressColor(
    meanTension/(beltThickness*webWidth), stressLow, stressHigh, stressNominal, colorGamma,
    webColor, nominalColor, stressColor);

  //Spatial loading and transport tension
  meanTension = (entryTension + exitTension)/2;
  normalForce = sqrt(sum((sum(frame_a.R.T[j,i]*frame_a.f[j] + frame_b.R.T[j,i]*frame_b.f[j] for j in 1:3))^2 for i in 1:3));
  connect(frame_center, lateralJoint.frame_a)
    annotation(Line(points = {{0, -100}, {0, -70}, {-90, -70}, {-90, -30}, {-65, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lateralJoint.frame_b, tangentA.frame_a)
    annotation(Line(points = {{-45, -30}, {-30, -30}, {-30, 40}, {-90, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lateralJoint.frame_b, tangentB.frame_a)
    annotation(Line(points = {{-45, -30}, {-30, -30}, {-30, 40}, {90, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lateralJoint.axis, lateralFlange)
    annotation(Line(points = {{-47, -24}, {-15, -24}, {-15, -60}, {200, -60}}, color = {0, 127, 0}));
  connect(frame_center, lateralLock.frame_a)
    annotation(Line(points = {{0, -100}, {0, -70}, {25, -70}, {25, -30}, {45, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lateralLock.frame_b, tangentA.frame_a)
    annotation(Line(points = {{65, -30}, {80, -30}, {80, 40}, {-90, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lateralLock.frame_b, tangentB.frame_a)
    annotation(Line(points = {{65, -30}, {80, -30}, {80, 40}, {90, 40}}, color = {95, 95, 95}, thickness = 0.5));

  annotation(
    Icon(coordinateSystem(extent = {{-200, -100}, {200, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Polygon(rotation = if rotationDirection > 0 then 0 else 180, lineColor = {35, 65, 95}, fillColor = {60, 110, 200},
          fillPattern = FillPattern.Solid,
          points = {{-180,-35},{-135,35},{-65,65},{0,72},{65,65},{135,35},{180,-35},
            {135,5},{65,35},{0,42},{-65,35},{-135,5},{-180,-35}},
          smooth = Smooth.Bezier),
        Text(textColor = {64, 64, 64}, extent = {{-140, 115}, {140, 145}},
          textString = "%name"),
        Rectangle(origin = {-2.322, 0}, lineColor = {128, 128, 128}, fillColor = {0, 116, 0}, lineThickness = 5, extent = {{-197.678, -97}, {197.678, 97}}, radius = 25)}),
    Diagram(coordinateSystem(extent = {{-200, -100}, {200, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {
      Text(origin = {90, 8}, textColor = {64, 64, 64}, extent = {{-50, 72}, {50, 92}}, textString = "%name")}),
    Documentation(info = "<html>
<h4>Web wrap</h4>
<p>This component locates taut-web tangencies on a circular roll.
Boundary-frame orientations set tangent radii; <code>rotationDirection</code>
selects the directed arc. Supply world-resolved unit material-travel vectors
<code>spanDirectionA</code> and <code>spanDirectionB</code> to determine skew.
<code>frame_center</code> locates the non-spinning axis and carries reactions.</p>
<p>The web-centreline radius <code>radius + beltThickness/2</code> sets wrap
length and tangency migration. <code>entryTangent</code>, <code>exitTangent</code>
and arc-averaged <code>meanTangent</code> use world coordinates [1]; the mean
is generally shorter than a unit vector. Figure 1 shows directed wrap and
skew for <code>rotationDirection = 1</code>. Orange arrows are span loads;
the centre balances their resultant. Arrow lengths are illustrative.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/WebWrap/wrap-geometry.svg\"
width=\"900\" alt=\"Directed web wrap with entry and exit tangency loads,
centre reaction, centreline radius and wrap angle; a skewed span resolved
along the circumferential tangent and roller axis\"/>
<strong>Figure 1:</strong> Directed wrap geometry and skewed span.</p>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Symbol</th><th>Model quantity and sign</th></tr>
<tr><td>R<sub>c</sub>, &theta;, L</td><td><code>centrelineRadius</code>,
<code>wrapAngle</code>, <code>arcLength</code>. The blue arc follows the web
centreline, outside the mechanical radius by half <code>beltThickness</code>.</td></tr>
<tr><td>t<sub>A</sub>, t<sub>B</sub></td><td><code>entryTangent</code>,
<code>exitTangent</code>: directed circumferential unit vectors in world coordinates.</td></tr>
<tr><td>e<sub>A</sub>, &beta;<sub>A</sub></td><td><code>spanDirectionA</code>
and <code>tangentA.skew</code>. The tangency frame's local x axis follows
e<sub>A</sub>. End B uses the corresponding B quantities.</td></tr>
<tr><td>a</td><td>The y axis of <code>frame_center</code>, resolved in world
coordinates. Positive <code>lateralFlange.s</code> moves the contact line along
this axis when <code>lateral = true</code>.</td></tr>
<tr><td>F<sub>A</sub>, F<sub>B</sub>, F<sub>C</sub></td><td>
<code>frame_a.f</code>, <code>frame_b.f</code>, <code>frame_center.f</code>,
each resolved into the same world frame before comparing or summing.
The end forces may contain transverse loads as well as tension.</td></tr>
</table>
<p>With <code>lateral = true</code>, the optional joint exchanges axial effort
through <code>lateralFlange</code>. Tangency adapters transmit spatial loads
but omit the shaft-axis moment of longitudinal span force;
<code>WebFriction</code> supplies contact torque and wrapped-material inertia.</p>
<p><code>normalForce</code> is the magnitude of summed spatial tangency forces,
including neighbouring <code>Belt</code> weight and inertia; it is neither
integrated contact pressure nor total bearing load. Parent-fed
<code>entryTension</code> and <code>exitTension</code> supply material and colour
calculations and must match the connected local x force components.</p>
<p>Animation uses at least four segments and small tangency overhangs without
changing mechanics. Thick webs may need more segments for visible drum clearance.</p>
<h4>Limitations</h4>
<p>The web must remain taut with nonzero roll-plane tangent projections.
Angle start values select the route; full-turn branch-cut crossings and
slack web are excluded. An almost complete turn produces a warning.</p>
<p>With <code>useDetachment = true</code>, leaving the free-span line releases
the web: wrap angle becomes exactly zero and <code>tangencyRadius</code> grows
past <code>centrelineRadius</code>. Rigid engagement keeps <code>gap</code> zero;
wrap angle measures penetration into the line. At zero wrap,
<code>meanTangent</code> uses the entry tangent, but the coupled
<code>WebFriction</code> still requires positive momentum-closure inventory.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://modelica.org/events/Conference2003/papers/h37_Otter_multibody.pdf\">1</a>]
M. Otter, H. Elmqvist and S. E. Mattsson, &quot;The New Modelica MultiBody
Library&quot;, Modelica Conference, 2003, secs. 3 and 6.2.
Frame orientation, vector resolution and massless frame transformations.
Accessed 2026-09-06.</li>
</ul>
</html>"));
end WebWrap;
