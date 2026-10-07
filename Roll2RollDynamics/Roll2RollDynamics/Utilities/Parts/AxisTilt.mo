within Roll2RollDynamics.Utilities.Parts;
model AxisTilt
  "Yaw and tram rotation of a roll axis, kept as run-time parameters"
  import Modelica.Mechanics.MultiBody.Frames;
  //Parameters
  parameter Modelica.Units.SI.Angle yaw = 0
    "In-plane misalignment about the vertical, ends stay level";
  parameter Modelica.Units.SI.Angle tram = 0
    "Out-of-plane misalignment about the machine direction, one end up";
  //Variables
  Frames.Orientation R_rel
    "Rotation from frame_a to frame_b, yaw first and then tram";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    "Housing side"
    annotation(Placement(transformation(extent={{-116,-16},{-84,16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
    "Bearing side"
    annotation(Placement(transformation(extent={{84,-16},{116,16}})));
equation
  // The rotation stays a variable so that yaw and tram survive as
  // run-time parameters in an exported FMU
  R_rel = Frames.absoluteRotation(
    Frames.axisRotation(3, yaw, 0), Frames.axisRotation(1, tram, 0));
  Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0,
    "Connect frame_a of the AxisTilt to the machine frame");
  frame_b.r_0 = frame_a.r_0;
  frame_b.R = Frames.absoluteRotation(frame_a.R, R_rel);
  zeros(3) = frame_a.f + Frames.resolve1(R_rel, frame_b.f);
  zeros(3) = frame_a.t + Frames.resolve1(R_rel, frame_b.t);
  annotation(Icon(graphics={
      Rectangle(extent={{-90,20},{90,-20}}, lineColor={95,95,95},
        fillColor={215,215,215}, fillPattern=FillPattern.Solid),
      Line(points={{-70,40},{70,-40}}, color={0,0,0}, thickness=0.5),
      Text(extent={{-150,80},{150,40}}, textString="%name", textColor={64,64,64})}),
    Documentation(info="<html>
<h4>Axis tilt</h4>
<p>This massless adapter rotates <code>frame_b</code> about the common centre:
<code>yaw</code> about vertical, then <code>tram</code> about the machine
direction. Figure 1 shows the housing-to-bearing misalignment.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/AxisTilt/yaw-tram.svg\"
     width=\"600\"
     alt=\"Yaw about the vertical applied before tram about the machine direction\">
<strong>Figure 1:</strong> Yaw and tram rotations from <code>frame_a</code> to <code>frame_b</code>.</p>
<p>Unlike <code>FixedRotation</code>, the variable rotation keeps both angles
settable in an exported FMU. Root <code>frame_a</code> at the machine frame.</p>
</html>"));
end AxisTilt;
