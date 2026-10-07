within Roll2RollDynamics;
model WebWorld "Multibody world and machine-wide roll-to-roll defaults in one component"
  extends Modelica.Mechanics.MultiBody.World(n = {0, 0, -1});

  // Geometry
  parameter Roll2RollDynamics.Utilities.Types.WebParameters web
    "Default web thickness, width and axial elasticity"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Radius rollRadius = 0.15 "Default roll radius"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length rollWidth = 1.1*web.width
    "Default roll face width, 10% wider than the web"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Radius coreRadius = 0.05 "Default wound roll core radius"
    annotation(Dialog(group = "Geometry"));
  parameter Modelica.Units.SI.Length spanLength = 1.2
    "Machine-direction spacing between rolls"
    annotation(Dialog(group = "Geometry"));

  // Process
  parameter Modelica.Units.SI.Velocity lineSpeed = 2.0 "Line surface speed setpoint"
    annotation(Dialog(group = "Process"));
  parameter Modelica.Units.SI.Force tension = 150 "Nominal web tension"
    annotation(Dialog(group = "Process"));
  parameter Modelica.Units.SI.Time webDampingTime(min = 0) = 0.01
    "Default Kelvin-Voigt damping time for free web spans; zero gives elastic spans"
    annotation(Dialog(group = "Process"));
  parameter Modelica.Units.SI.Stress stressLow = 0.8*stressNominal
    "Web stress drawn at the cool end of the span color gradient"
    annotation(Dialog(group = "Process"));
  parameter Modelica.Units.SI.Stress stressHigh = 1.2*stressNominal
    "Web stress drawn at the hot end of the span color gradient"
    annotation(Dialog(group = "Process"));
  parameter Real stressColorGamma(min = Modelica.Constants.small) = 1
    "Contrast exponent of the span color gradient; one is proportional, below one exaggerates small deviations"
    annotation(Dialog(group = "Process"));
  final parameter Modelica.Units.SI.Stress stressNominal =
    tension/(web.thickness*web.width)
    "Web stress at the nominal tension, from which the color limits follow";
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters innerTraction
    "Default traction of the inner web face"
    annotation(Dialog(group = "Process"));
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters outerTraction
    "Default traction of the outer web face"
    annotation(Dialog(group = "Process"));
  parameter Modelica.Units.SI.RotationalDampingConstant bearingDamping(min=0) = 0.001
    "Default viscous bearing damping"
    annotation(Dialog(group = "Process"));
  parameter Boolean lateralDynamics = false
    "= true, if the web may move across the machine"
    annotation(Dialog(group = "Out-of-plane"));

  // Animation
  parameter Real webColor[3] = {60, 110, 200}
    "Default web color at the low stress limit"
    annotation(Dialog(group = "Animation"));
  parameter Real nominalColor[3] = {245, 210, 70}
    "Default web color at nominal stress"
    annotation(Dialog(group = "Animation"));
  parameter Real stressColor[3] = {205, 50, 35}
    "Default web color at the high stress limit"
    annotation(Dialog(group = "Animation"));
  parameter Real rollColor[3] = {0, 120, 0}
    "Default roller surface color"
    annotation(Dialog(group = "Animation"));
  final parameter Integer stressBarSegments = 11
    "Number of colored surfaces in the stress bar";
  final parameter Real stressBarLowColor[3] = webColor
    "Color at the minimum stress";
  final parameter Real stressBarNominalColor[3] = nominalColor
    "Color at nominal stress";
  final parameter Real stressBarHighColor[3] = stressColor
    "Color at the maximum stress";
  final parameter Modelica.Units.SI.Diameter stressBarDiameter =
    0.08*nominalLength
    "Diameter of the drawn stress-scale column";
  final parameter Modelica.Units.SI.Length stressBarSegmentLength =
    nominalLength/stressBarSegments
    "Length of one colored segment of the column";
  final parameter Real stressBarNominalFraction(unit = "1") =
    min(1, max(0, (stressNominal - stressLow)/max(
      stressHigh - stressLow, Modelica.Constants.small)))
    "Height of the nominal stress up the column, as a fraction of its length";
  final parameter Real stressBarColors[stressBarSegments, 3] = {
    Roll2RollDynamics.Functions.stressColor(
      stressLow + (stressHigh - stressLow)*(i - 1)/(stressBarSegments - 1),
      stressLow,
      stressHigh,
      stressNominal,
      stressColorGamma,
      stressBarLowColor,
      stressBarNominalColor,
      stressBarHighColor) for i in 1:stressBarSegments}
    "Colors drawn from the minimum to the maximum stress";

  // Variables with Binding Equations
  output Modelica.Units.SI.Stress stressBarMinimum = stressLow
    "Minimum stress represented by the animation bar";
  output Modelica.Units.SI.Stress stressBarMaximum = stressHigh
    "Maximum stress represented by the animation bar";

  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape
    stressBar[stressBarSegments](
      each shapeType = "cylinder",
      each r_shape = {0, 0, 0},
      each lengthDirection = {0, 0, 1},
      each widthDirection = {1, 0, 0},
      each length = nominalLength/stressBarSegments,
      each width = stressBarDiameter,
      each height = stressBarDiameter,
      color = stressBarColors,
      each R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
      r = {{-nominalLength,
            -0.6*rollWidth,
            (i - 1)*nominalLength/stressBarSegments}
        for i in 1:stressBarSegments}) if enableAnimation
    "Fixed stress-color scale from stressBarMinimum to stressBarMaximum";

  // Three constant caps mark where the scale starts, ends and reads nominal.
  // Everything about them is a parameter, so they cost nothing to simulate.
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape stressBarMinimumCap(
    shapeType = "cone",
    r_shape = {0, 0, 0},
    lengthDirection = {0, 0, -1},
    widthDirection = {1, 0, 0},
    length = 0.6*stressBarSegmentLength,
    width = stressBarDiameter,
    height = stressBarDiameter,
    color = stressBarLowColor,
    R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
    r = {-nominalLength, -0.6*rollWidth, 0}) if enableAnimation
    "Cone marking the minimum end of the stress scale";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape stressBarMaximumCap(
    shapeType = "cone",
    r_shape = {0, 0, 0},
    lengthDirection = {0, 0, 1},
    widthDirection = {1, 0, 0},
    length = 0.6*stressBarSegmentLength,
    width = stressBarDiameter,
    height = stressBarDiameter,
    color = stressBarHighColor,
    R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
    r = {-nominalLength, -0.6*rollWidth, nominalLength}) if enableAnimation
    "Cone marking the maximum end of the stress scale";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape stressBarNominalRing(
    shapeType = "cylinder",
    r_shape = {0, 0, 0},
    lengthDirection = {0, 0, 1},
    widthDirection = {1, 0, 0},
    length = 0.12*stressBarSegmentLength,
    width = 1.5*stressBarDiameter,
    height = 1.5*stressBarDiameter,
    color = stressBarNominalColor,
    R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
    r = {-nominalLength,
         -0.6*rollWidth,
         stressBarNominalFraction*nominalLength}) if enableAnimation
    "Ring marking the nominal stress on the scale";

  annotation(
    Icon(
      coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Rectangle(origin = {-0.254, -0}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-100.254, -100}, {100.254, 100}}, radius = 1),
        Ellipse(origin = {-48, 30}, lineColor = {64, 64, 64},
          fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
          lineThickness = 0.6, extent = {{-26, -26}, {26, 26}}),
        Ellipse(origin = {-48, 30}, lineColor = {100, 110, 120},
          fillColor = {245, 247, 249}, fillPattern = FillPattern.Solid,
          extent = {{-8, -8}, {8, 8}}),
        Ellipse(origin = {8, 64}, lineColor = {64, 64, 64},
          fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
          lineThickness = 0.6, extent = {{-17, -17}, {17, 17}}),
        Ellipse(origin = {8, 64}, lineColor = {100, 110, 120},
          fillColor = {245, 247, 249}, fillPattern = FillPattern.Solid,
          extent = {{-5, -5}, {5, 5}}),
        Ellipse(origin = {52, 12}, lineColor = {64, 64, 64},
          fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
          lineThickness = 0.6, extent = {{-22, -22}, {22, 22}}),
        Ellipse(origin = {52, 12}, lineColor = {100, 110, 120},
          fillColor = {245, 247, 249}, fillPattern = FillPattern.Solid,
          extent = {{-7, -7}, {7, 7}}),
        Line(points = {{-51.6, 4.25}, {-57.04, 5.62}, {-62.06, 8.13}, {-66.42, 11.65},
            {-69.93, 16.03}, {-72.42, 21.06}, {-73.76, 26.51}, {-73.91, 32.12},
            {-72.85, 37.63}, {-70.64, 42.79}, {-67.37, 47.35}, {-63.19, 51.1},
            {-58.31, 53.87}, {1.26, 79.61}, {2.85, 80.2}, {4.5, 80.64},
            {6.18, 80.9}, {7.88, 81}, {9.59, 80.93}, {11.27, 80.68},
            {12.92, 80.27}, {14.53, 79.7}, {16.07, 78.96}, {17.52, 78.08},
            {18.88, 77.06}, {20.14, 75.9}, {67.71, 27.41}, {70.54, 23.85},
            {72.58, 19.78}, {73.74, 15.38}, {73.97, 10.84}, {73.26, 6.35},
            {71.65, 2.1}, {69.19, -1.73}, {66, -4.97}, {62.21, -7.49},
            {57.98, -9.17}, {53.5, -9.95}, {48.96, -9.79}, {-51.6, 4.25}}, color = {0, 128, 180}, thickness = 2.5),
        Line(origin = {-60, 0}, points = {{0, -34}, {0, -72}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {-60, 0}, points = {{-5, -64}, {0, -72}, {5, -64}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {-30, 0}, points = {{0, -34}, {0, -72}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {-30, 0}, points = {{-5, -64}, {0, -72}, {5, -64}}, color = {100, 110, 120},
          thickness = 0.6),
        Line( points = {{0, -34}, {0, -72}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(points = {{-5, -64}, {0, -72}, {5, -64}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {30, 0}, points = {{0, -34}, {0, -72}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {30, 0}, points = {{-5, -64}, {0, -72}, {5, -64}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {60, 0}, points = {{0, -34}, {0, -72}}, color = {100, 110, 120},
          thickness = 0.6),
        Line(origin = {60, 0}, points = {{-5, -64}, {0, -72}, {5, -64}}, color = {100, 110, 120}, thickness = 0.6)
      }
    ),
    defaultComponentName = "world",
    defaultComponentPrefixes = "inner",
    missingInnerMessage = "No \"world\" component is defined. Drag Roll2RollDynamics.WebWorld into the top level of your model.",
    Documentation(info = "<html>
<h4>Web line world</h4>
<p>The world defines the gravity field, the web and operating-point defaults
and the stress-colour scale shared by every component of a web line. It extends
the MSL <code>World</code>, with gravity along <code>-z</code> rather than
MSL's <code>-y</code>. Declare it
<code>inner Roll2RollDynamics.WebWorld world</code>; this also satisfies MSL
components' <code>outer World world</code>. Figure 1 shows the shared
layout, operating-point and web defaults that components inherit.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/WebWorld/layout-defaults.svg\"
     width=\"800\"
     alt=\"Grey side view of three rolls carrying a web, with core and roll radius, span length, tension, line speed, axes and gravity, beside a cross-machine view of roll width, web width and web thickness\">
<strong>Figure 1:</strong> Shared defaults of the web line.</p>
<p>Components inherit defaults but may override them:</p>
<blockquote><pre>
inner Roll2RollDynamics.WebWorld world(
  rollWidth = 0.8,
  webDampingTime = 0.01,
  web(thickness = 0.0002, width = 0.6));
Roll2RollDynamics.Components.Roller wideRoll(width = 1.2);
Roll2RollDynamics.Components.Belt elasticSpan(dampingTime = 0);
</pre></blockquote>
<p><code>web</code> = Shared thickness, width, density and axial stiffness<br>
<code>tension</code>, <code>lineSpeed</code> = Nominal operating point<br>
<code>rollRadius</code>, <code>rollWidth</code>, <code>coreRadius</code>,
<code>spanLength</code> = Layout defaults<br>
<code>stressLow</code>, <code>stressHigh</code> = Stress range of the colour
scale, defaulting to 0.8 and 1.2 times <code>stressNominal</code><br>
<code>innerTraction</code>, <code>outerTraction</code> = Independent face
friction records selected by each contact's <code>contactFace</code>;
defaults match, but differing faces require both records<br>
<code>bearingDamping</code> = Viscous shaft drag, 0.001 N m s/rad by default;
zero disables it<br>
<code>webDampingTime</code> = Span viscosity/modulus ratio, default 0.01 s;
zero disables material damping</p>
<p>Both damping defaults are illustrative: calibrate them for the bearings
and material. Material damping resists span extension, not uniform transport
or shaft rotation; see
<a href=\"modelica://Roll2RollDynamics.Components.Belt\">Belt</a>.</p>
<p><code>lateralDynamics = true</code> enables cross-machine web motion.
Default false retains yawed tangency geometry but suppresses lateral tracking.
<code>enableAnimation = false</code> disables library and MSL shapes without
changing physics.</p>
<p>Animation uses <code>webColor</code>, <code>nominalColor</code> and
<code>stressColor</code> for low, nominal and high stress (blue, gold, red),
and <code>rollColor</code> for drums. The nominal stress is
<code>stressNominal</code> = <code>tension</code>/(<code>web.thickness</code>
&times; <code>web.width</code>), and it always maps to
<code>nominalColor</code>. <code>stressColorGamma = 1</code> gives proportional
colour differences;
values below one exaggerate small deviations. See
<a href=\"modelica://Roll2RollDynamics.Functions.stressColor\">stressColor</a>.</p>
<p>The fixed cylindrical <code>stressBar</code> uses the same scale between
<code>stressBarMinimum</code> and <code>stressBarMaximum</code>. Conical caps
mark the bounds and a wider ring marks nominal stress; read numerical bounds
in the variable browser.</p>
</html>"
    ));
end WebWorld;
