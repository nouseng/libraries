within Roll2RollDynamics.Utilities.Parts;
model DrumRunout
  "Radial offset of a drum against its bearing axis, kept as a run-time parameter"
  import Modelica.Mechanics.MultiBody.Frames;
  //Parameters
  parameter Modelica.Units.SI.Length runout = 0
    "Radial runout: drum centre offset from the bearing axis";
  parameter Modelica.Units.SI.Angle runoutPhase = 0
    "Direction of the high point at zero shaft angle, from the machine direction toward up";
  //Variables
  Modelica.Units.SI.Position offset[3]
    "Drum centre offset resolved in frame_a";
  // Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    "Bearing side, turning with the shaft"
    annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
    "Drum side"
    annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
equation
  // The offset stays a variable so that runout survives as a settable
  // parameter in an exported FMU
  offset = runout*{cos(runoutPhase), 0, sin(runoutPhase)};
  Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0,
    "Connect frame_a of the DrumRunout to the shaft");
  frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, offset);
  frame_b.R = frame_a.R;

  // Force and torque balance, resolved in frame_a
  zeros(3) = frame_a.f + frame_b.f;
  zeros(3) = frame_a.t + frame_b.t + cross(offset, frame_b.f);
  annotation(Icon(graphics = {
      Rectangle(extent = {{-90, 20}, {90, -20}}, lineColor = {64, 64, 64},
        fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid, radius = 3),
      Line(points = {{-90, 0}, {-20, 0}, {20, 12}, {90, 12}}, color = {0, 128, 180}, thickness = 2),
      Text(extent = {{-150, 80}, {150, 40}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Drum runout</h4>
<p>This massless adapter offsets <code>frame_b</code> from <code>frame_a</code>
by radial <code>runout</code> at <code>runoutPhase</code>. Placed after the
revolute joint, it makes the drum centre orbit once per shaft turn, as
Figure 1 shows.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/DrumRunout/runout-offset.svg\"
     width=\"600\"
     alt=\"Drum centre offset from the bearing axis by runout at runoutPhase\">
<strong>Figure 1:</strong> Drum centre offset from the bearing axis.</p>
<p>Unlike <code>FixedTranslation</code>, the variable offset keeps both
parameters settable in an exported FMU. Root <code>frame_a</code> through
the shaft.</p>
</html>"));
end DrumRunout;
