within Roll2RollDynamics.Utilities.Parts;
model TangentFrame "Tangency position and freely aligned span frame"
  //Parameters
  parameter Boolean transmitAxialMoment=false
    "Transmit the longitudinal force moment about the shaft as well as support moments";
  parameter Integer direction(min=-1,max=1)=1
    "Directed circumferential travel, either -1 or +1";
  //Variables
  Modelica.Units.SI.Angle skew
    "Span angle across the roll axis, computed from the supplied direction";
  Modelica.Units.SI.AngularVelocity angleRate "Rate of the tangency radius angle";
  Modelica.Units.SI.AngularVelocity skewRate "Rate of the span skew angle";
  Modelica.Units.SI.Force forceAtCenter[3]
    "Complete tangency force resolved in the centre frame";
  Modelica.Units.SI.Force longitudinalForceAtCenter[3]
    "Local span-x force resolved in the centre frame";
  Modelica.Units.SI.Torque offsetMoment[3]
    "Moment of the complete tangency force about the centre";
  Modelica.Units.SI.Torque longitudinalOffsetMoment[3]
    "Moment of the local span-x force about the centre";
  //Inputs and Outputs
  input Real spanDirection[3](each unit="1")
    "Unit material-travel direction resolved in world coordinates";
  input Modelica.Units.SI.Position r[3]
    "Centre-to-tangency offset resolved in the centre frame";
  input Modelica.Units.SI.Angle angle
    "Tangency radius angle in the roll plane";
  //Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a
    "Non-spinning roll-centre frame carrying the support reaction"
    annotation(Placement(transformation(extent={{-116,-16},{-84,16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b
    "Tangency frame with local x directed along the span"
    annotation(Placement(transformation(extent={{84,-16},{116,16}})));
protected
  Real localDirection[3](each unit="1")
    "Supplied span direction resolved in the non-spinning roll-centre frame";
equation
  // Invert the width-axis alignment on the forward-facing branch.
  localDirection = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, spanDirection);
  skew = Modelica.Math.atan2(localDirection[2],
    direction*(cos(angle)*localDirection[1] - sin(angle)*localDirection[3]));
  assert(direction == 1 or direction == -1, "TangentFrame.direction must be -1 or +1");
  assert(cos(skew)>0,
    "TangentFrame requires the span to follow the selected circumferential direction; adjust tangency angle start guesses or routing");
  // Explicit rates: the angles are algebraic, and the connected belt
  // differentiates this frame's position twice.
  angleRate = der(angle);
  skewRate = der(skew);
  Connections.branch(frame_a.R, frame_b.R);
  for i in 1:3 loop
    frame_b.r_0[i] = frame_a.r_0[i] + sum(frame_a.R.T[j,i]*r[j] for j in 1:3);
  end for;
  frame_b.R = Modelica.Mechanics.MultiBody.Frames.absoluteRotation(frame_a.R,
    Modelica.Mechanics.MultiBody.Frames.axesRotations({2,3,1},
      {angle + (if direction < 0 then Modelica.Constants.pi else 0),skew,0},
      {angleRate,skewRate,0}));
  for i in 1:3 loop
    forceAtCenter[i] = sum(frame_a.R.T[i,j]*sum(frame_b.R.T[k,j]*frame_b.f[k] for k in 1:3) for j in 1:3);
    longitudinalForceAtCenter[i] = frame_b.f[1]
      *sum(frame_a.R.T[i,j]*frame_b.R.T[1,j] for j in 1:3);
  end for;
  zeros(3) = frame_a.f + forceAtCenter;
  offsetMoment = cross(r,forceAtCenter);
  longitudinalOffsetMoment = cross(r,longitudinalForceAtCenter);
  for i in 1:3 loop
    0 = frame_a.t[i] + sum(frame_a.R.T[i,j]*sum(frame_b.R.T[k,j]*frame_b.t[k] for k in 1:3) for j in 1:3)
      + (if i == 2 and not transmitAxialMoment
          then offsetMoment[i]-longitudinalOffsetMoment[i] else offsetMoment[i]);
  end for;
  annotation(
    Icon(graphics={
      Ellipse(extent={{-82,-12},{-58,12}},lineColor={64,64,64},
        fillColor={160,170,180},fillPattern=FillPattern.Solid),
      Line(points={{-58,0},{30,0},{65,35}},color={0,128,180},thickness=2),
      Line(points={{30,0},{65,-35}},color={0,128,180},thickness=2),
      Text(extent={{-100,80},{100,50}},textString="%name",textColor={64,64,64})}),
    Documentation(info="<html>
<h4>Tangent frame</h4>
<p>This massless adapter places a tangency relative to the non-spinning roll
centre at <code>frame_a</code>; the local x-axis of <code>frame_b</code> follows
the span. Supply offset <code>r</code>, geometric <code>angle</code> and
world-resolved <code>spanDirection</code>. Its axial and circumferential
projections determine <code>skew</code> through <code>atan2</code>.
Positive skew cosine is required to follow the selected travel direction.
Figure 1 shows the geometry.</p>
<p style=\"text-align: center;\">
    <img style=\"display: block; margin-left: auto; margin-right: auto;\"
         src=\"modelica://Roll2RollDynamics/Resources/Images/TangentFrame/tangency-geometry.svg\"
         width=\"600\"
         alt=\"Tangency frame located at radius r and angle from a non-spinning roll centre, with skew to the span direction\">
    <strong>Figure 1:</strong> Tangency location from a non-spinning roll centre.</p>
<p>Forces, boundary moments and offset moments reach the centre. Default
<code>transmitAxialMoment=false</code> omits only the shaft-axis moment of
local span-x force, carried separately by the transport coupling. Transverse
force moments remain, including their axial component under skew.
Setting <code>transmitAxialMoment=true</code> transmits the full rigid-offset
moment. The adapter adds no mass or inertia.</p>
<h4>Limitations</h4>
<p>This load separation does not correct the scalar transport coupling's
circumferential torque approximation for skewed web.</p></html>"));
end TangentFrame;
