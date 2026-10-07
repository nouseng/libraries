within Roll2RollDynamics.Utilities.Parts;
model VariableTranslation
  "Variable rigid translation that propagates spatial support loads"
  parameter Boolean transmitAxialMoment=true
    "Transmit the offset-force moment about local y; disable when a separate shaft coupling carries it";
  input Modelica.Units.SI.Position r[3]
    "Vector from frame_a to frame_b resolved in frame_a";
  Modelica.Units.SI.Torque offsetMoment[3] "Moment of the tangency force";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    "Roll-center frame"
    annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}}), iconTransformation(origin = {25, 0}, extent = {{-145, -20}, {-105, 20}}))
    );
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
    "Tangency frame"
    annotation(Placement(transformation(extent = {{84, -16}, {116, 16}}), iconTransformation(origin = {-25, -0}, extent = {{105, -20}, {145, 20}})));
equation
  Connections.branch(frame_a.R, frame_b.R);
  frame_b.r_0 = frame_a.r_0
    + Modelica.Mechanics.MultiBody.Frames.resolve1(frame_a.R, r);
  frame_b.R = frame_a.R;
  zeros(3) = frame_a.f + frame_b.f;
  offsetMoment = cross(r, frame_b.f);
  zeros(3) = frame_a.t + frame_b.t
    + {offsetMoment[1], if transmitAxialMoment then offsetMoment[2] else 0, offsetMoment[3]};
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
        graphics = {
      Rectangle(lineColor = {64, 64, 64}, fillColor = {210, 218, 224},
        fillPattern = FillPattern.Solid, extent = {{-90, 10}, {90, -10}}, radius = 3),
      Line(points = {{-90, 0}, {60, 0}}, color = {0, 128, 180}, thickness = 2),
      Polygon(lineColor = {0, 128, 180}, fillColor = {0, 128, 180},
        fillPattern = FillPattern.Solid, points = {{88, 0}, {58, -12}, {58, 12}, {88, 0}}),
      Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64},
        fillPattern = FillPattern.Solid, extent = {{-98, 8}, {-82, -8}}),
      Text(textColor = {64, 64, 64}, extent = {{-100, -98}, {100, -70}}, textString = "%name")}),
    Documentation(info = "<html>
<h4>Variable translation</h4>
<p>This massless adapter separates <code>frame_a</code> and <code>frame_b</code>
by parent-fed input <code>r</code>, resolved in <code>frame_a</code>.
Figure 1 shows the offset-force moment.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/VariableTranslation/variable-offset.svg\"
     width=\"600\"
     alt=\"Rigid offset r from frame_a to frame_b, with offsetMoment = r cross f\">
<strong>Figure 1:</strong> Rigid offset that may change during operation.</p>
<p>Default <code>transmitAxialMoment=true</code> passes the complete moment.
Setting it to <code>false</code> omits local-y offset torque when a separate
web/shaft coupling carries it; transverse moments remain. Load balance stays
exact for changing <code>r</code>, with velocities obtained by differentiating
<code>frame_b.r_0</code>. Use <code>FixedTranslation</code> for constant offset.</p>
</html>"));
end VariableTranslation;
