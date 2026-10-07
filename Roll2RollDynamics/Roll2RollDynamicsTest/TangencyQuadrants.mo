within Roll2RollDynamicsTest;
model TangencyQuadrants
  "Four rolls on a diamond, placing one solved tangency point in each Cartesian quadrant"
  extends Modelica.Icons.Example;
  import Modelica.Constants.pi;

  inner Roll2RollDynamics.WebWorld world
    annotation(Placement(transformation(extent = {{-120, 60}, {-100, 80}})));

protected
  final parameter Modelica.Units.SI.Length a = 0.7
    "Half diagonal of the diamond the rolls sit on";

public
  Roll2RollDynamics.Components.Roller west(
    radius = 0.15,
    useFlange = true,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    xPosition = 0,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = -3*pi/4,
    exitAngleStart = -pi/4) "Driven roll at the west corner"
    annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Roll2RollDynamics.Components.Roller north(
    radius = 0.10,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = a,
    zPosition = a,
    rotationDirection = 1,
    entryAngleStart = -pi/4,
    exitAngleStart = pi/4) "Idling roll at the north corner"
    annotation(Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Roller east(
    radius = 0.18,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = 2*a,
    zPosition = 0,
    rotationDirection = 1,
    entryAngleStart = pi/4,
    exitAngleStart = 3*pi/4) "Idling roll at the east corner"
    annotation(Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Roll2RollDynamics.Components.Roller south(
    radius = 0.12,
    fixedInitialAngle = true,
    fixedInitialWebState = true,
    fixedInitialSpeed = true,
    xPosition = a,
    zPosition = -a,
    rotationDirection = 1,
    entryAngleStart = 3*pi/4,
    exitAngleStart = 5*pi/4) "Idling roll at the south corner"
    annotation(Placement(transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));

  Roll2RollDynamics.Components.Belt northWest "Span carrying the upper-left tangency"
    annotation(Placement(transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt northEast "Span carrying the upper-right tangency"
    annotation(Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Components.Belt southEast "Span carrying the lower-right tangency"
    annotation(Placement(transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  Roll2RollDynamics.Components.Belt southWest "Span carrying the lower-left tangency"
    annotation(Placement(transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));

  Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
    w_fixed = world.lineSpeed/(west.radius+west.beltThickness/2)) "Loop speed master"
    annotation(Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));

  // The tangency point of each span, resolved in the roll frame it sits on
  output Modelica.Units.SI.Length quadrant1[2] = east.radius*{sin(east.entryAngle), cos(east.entryAngle)}
    "Upper-right tangency, on the east roll";
  output Modelica.Units.SI.Length quadrant2[2] = west.radius*{sin(west.exitAngle), cos(west.exitAngle)}
    "Upper-left tangency, on the west roll";
  output Modelica.Units.SI.Length quadrant3[2] = west.radius*{sin(west.entryAngle), cos(west.entryAngle)}
    "Lower-left tangency, on the west roll";
  output Modelica.Units.SI.Length quadrant4[2] = east.radius*{sin(east.exitAngle), cos(east.exitAngle)}
    "Lower-right tangency, on the east roll";
equation
  connect(drive.flange, west.flange_a)
    annotation(Line(points = {{-15, 0}, {15, 0}}, origin = {-85, 0}));
  connect(west.frame_b, northWest.frame_a)
    annotation(Line(points = {{-3.333, -26.667}, {-3.333, 13.333}, {6.667, 13.333}}, color = {0, 128, 180}, thickness = 0.5, origin = {-66.667, 36.667}));
  connect(northWest.frame_b, north.frame_a)
    annotation(Line(points = {{-15, 0}, {15, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-25, 50}));
  connect(north.frame_b, northEast.frame_a)
    annotation(Line(points = {{-15, 0}, {15, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {25, 50}));
  connect(northEast.frame_b, east.frame_a)
    annotation(Line(points = {{-6.667, 13.333}, {3.333, 13.333}, {3.333, -26.667}}, color = {0, 128, 180}, thickness = 0.5, origin = {66.667, 36.667}));
  connect(east.frame_b, southEast.frame_a)
    annotation(Line(points = {{3.333, 26.667}, {3.333, -13.333}, {-6.667, -13.333}}, color = {0, 128, 180}, thickness = 0.5, origin = {66.667, -36.667}));
  connect(southEast.frame_b, south.frame_a)
    annotation(Line(points = {{15, 0}, {-15, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {25, -50}));
  connect(south.frame_b, southWest.frame_a)
    annotation(Line(points = {{15, 0}, {-15, 0}}, color = {0, 128, 180}, thickness = 0.5, origin = {-25, -50}));
  connect(southWest.frame_b, west.frame_a)
    annotation(Line(points = {{6.667, -13.333}, {-3.333, -13.333}, {-3.333, 26.667}}, color = {0, 128, 180}, thickness = 0.5, origin = {-66.667, -36.667}));

  assert(quadrant1[1] > 0 and quadrant1[2] > 0, "Upper-right tangency left the first quadrant");
  assert(quadrant2[1] < 0 and quadrant2[2] > 0, "Upper-left tangency left the second quadrant");
  assert(quadrant3[1] < 0 and quadrant3[2] < 0, "Lower-left tangency left the third quadrant");
  assert(quadrant4[1] > 0 and quadrant4[2] < 0, "Lower-right tangency left the fourth quadrant");

  annotation(
    experiment(StartTime = 0, StopTime = 1, Interval = 0.001),
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-140, -80}, {100, 100}})),
    Documentation(figures = {Figure(
      title = "Tangency points in local roll frames", identifier = "quadrants", preferred = true,
      plots = {Plot(title = "Solved tangency points in local roll frames",
        x = Axis(label = "Local x position", unit = "m"), y = Axis(label = "Local z position", unit = "m"),
        curves = {
            Curve(x = quadrant1[1], y = quadrant1[2], legend = "Quadrant I, east entry"),
            Curve(x = quadrant2[1], y = quadrant2[2], legend = "Quadrant II, west exit"),
            Curve(x = quadrant3[1], y = quadrant3[2], legend = "Quadrant III, west entry"),
            Curve(x = quadrant4[1], y = quadrant4[2], legend = "Quadrant IV, east exit")})})}, info = "<html>
<h4>Tangency in all four quadrants</h4>
<p>Four unequal rolls on a diamond solve their common tangencies from
position, radius and turning sense. <code>entryAngleStart</code> and
<code>exitAngleStart</code> select the branch without prescribing the solution.</p>
<p>Figure 1 shows the route and solved surface points: east entry/exit in
quadrants I/IV, west exit/entry in II/III. The 0.15 and 0.18 m circles locate
the roll surfaces; all points match the analytical common tangent to machine
precision.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/TangencyQuadrants/quadrants.png\"
     width=\"600\"
     alt=\"Four-roll web path beside solved tangency points in local roll coordinates\">
<strong>Figure 1:</strong> Web path and the four solved local tangencies.</p>
<p>The default guesses, <code>-pi/4</code> and <code>pi/4</code>, fit only
<code>north</code>. Using them on the other rolls fails initialization with
<code>TangentFrame requires the span to follow the selected circumferential direction</code>.
Within the correct quadrant, <code>west</code> converges to
&minus;136.8&deg;/&minus;42.1&deg; even from guesses 45&deg; away.</p>
<p>The tangent direction is <code>asin((r1 - r2)/D)</code> [1], with
centre distance <code>D = 0.99</code> m. Raise <code>east.radius</code>
by 0.02 m: both east tangencies should rotate about 1.2&deg; without
changing the start angles.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://dl.icdst.org/pdfs/files3/ad7608c18e740b0e402c025fa3187de8.pdf\">1</a>]
R. G. Budynas and J. K. Nisbett, &quot;Shigley&apos;s Mechanical Engineering
Design&quot;, 10th ed. in SI units, McGraw-Hill, 2015, sec. 17&ndash;2.
Fig. 17&ndash;1 and Eq. (17&ndash;1), pp. 873 and 875, give the common-tangent
geometry for unequal pulleys; Ex. 17&ndash;1, pp. 882&ndash;883, works it through
for a 6 in / 18 in drive. Accessed 2026-09-04.</li>
</ul>
</html>"));
end TangencyQuadrants;
