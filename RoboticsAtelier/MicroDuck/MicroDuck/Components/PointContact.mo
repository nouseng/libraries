within MicroDuck.Components;
model PointContact "Ground contact of one foot point with the soft pyramidal friction cone of MuJoCo"
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  //Parameters
  parameter Real[3] normal(each unit = "1") = {0, 1, 0} "Plane normal, resolved in frame_a";
  parameter Real mu(unit = "1") = 1 "Friction coefficient";
  parameter Modelica.Units.SI.Time timeConstant = 0.02 "Time constant of the contact, MuJoCo solref[1]";
  parameter Real dampingRatio(unit = "1") = 1 "Damping ratio of the contact, MuJoCo solref[2]";
  parameter Real[5] impedance(each unit = "1") = {0.9, 0.95, 0.001, 0.5, 2} "Impedance at zero and full depth, full depth in m, midpoint and power, MuJoCo solimp";
  parameter Real inverseWeight(unit = "1/kg") = 1.25 "Translational inverse weight of the body carrying frame_b, MuJoCo body_invweight0[1]";
  parameter Boolean accelerationTerm = false "Include the inertial term of the contact law, which sets the force at touchdown but adds stiff filter states";
  parameter Modelica.Units.SI.Time filterTime = 1e-4 "Time constant of the acceleration and force estimates" annotation(Dialog(enable = accelerationTerm));
  parameter Modelica.Units.SI.Length onsetDepth = 1e-5 "Penetration over which the force ramps in on touchdown";
  final parameter Real[3] normalUnit(each unit = "1") = Modelica.Math.Vectors.normalize(normal) "Unit plane normal, resolved in frame_a";
  final parameter Real[3] tangent1(each unit = "1") = Modelica.Math.Vectors.normalize(cross(normalUnit, if abs(normalUnit[1]) < 0.9 then {1, 0, 0} else {0, 0, 1})) "First tangent of the plane, resolved in frame_a";
  final parameter Real[3] tangent2(each unit = "1") = cross(normalUnit, tangent1) "Second tangent of the plane, resolved in frame_a";
  final parameter Real[4, 3] localEdge(each unit = "1") = {normalUnit + mu * tangent1, normalUnit - mu * tangent1, normalUnit + mu * tangent2, normalUnit - mu * tangent2} "Edges n +- mu t of the friction pyramid, resolved in frame_a";
  final parameter Real stiffness(unit = "1/s2") = 1 / (impedance[2] ^ 2 * timeConstant ^ 2 * dampingRatio ^ 2) "Reference stiffness per unit mass";
  final parameter Real damping(unit = "1/s") = 2 / (impedance[2] * timeConstant) "Reference damping per unit mass";
  final parameter Real edgeInverseMass(unit = "1/kg") = 2 * mu ^ 2 * (1 + mu ^ 2) * inverseWeight "Approximate inverse mass of each cone edge, as MuJoCo computes it";
  //Variables
  Real[4, 3] edge(each unit = "1") "Edges n +- mu t of the friction pyramid in global coordinates";
  Modelica.Units.SI.Position[3] rRel0 "Position of frame_b relative to frame_a, in global coordinates";
  Modelica.Units.SI.Velocity[3] vRel0 "Velocity of frame_b relative to frame_a, in global coordinates";
  Modelica.Units.SI.Velocity[3] vFiltered if accelerationTerm "Low-passed vRel0";
  Modelica.Units.SI.Acceleration[3] aRel0 if accelerationTerm "Estimated acceleration of frame_b relative to frame_a";
  Modelica.Units.SI.Position height "Height of frame_b above the plane, negative when penetrating";
  Real depthRatio(unit = "1") "Penetration over the full impedance depth, at most 1";
  Real impedanceShape(unit = "1") "Sigmoid of depthRatio";
  Real d(unit = "1") "Impedance";
  Real regularizer(unit = "1/kg") "Softness R of each edge";
  Modelica.Units.SI.Acceleration[4] referenceAcceleration "Reference acceleration of each edge";
  Modelica.Units.SI.Acceleration[4] edgeAcceleration if accelerationTerm "Acceleration along each edge";
  Modelica.Units.SI.Force[4] edgeForce "Force along each edge, never negative";
  Modelica.Units.SI.Force[4] edgeForceFiltered if accelerationTerm "Low-passed edgeForce";
  Modelica.Units.SI.Force normalForce "Normal force on frame_b";
  Modelica.Units.SI.Force[3] totalForce "Force on frame_b, in global coordinates";
initial equation
  // the estimates start settled on the initial motion
  if accelerationTerm then
    vFiltered = vRel0;
    edgeForceFiltered = edgeForce;
  end if;
equation
  // Geometry
  for e in 1:4 loop
    edge[e, :] = Modelica.Mechanics.MultiBody.Frames.resolve1(frame_a.R, localEdge[e, :]);
  end for;
  rRel0 = frame_b.r_0 - frame_a.r_0;
  vRel0 = der(rRel0);
  if accelerationTerm then
    aRel0 = (vRel0 - vFiltered) / filterTime;
    der(vFiltered) = aRel0;
  end if;
  height = Modelica.Mechanics.MultiBody.Frames.resolve1(frame_a.R, normalUnit) * rRel0;

  // Impedance, MuJoCo solimp
  depthRatio = noEvent(min(1, abs(height) / impedance[3]));
  impedanceShape = noEvent(if depthRatio <= impedance[4] then depthRatio ^ impedance[5] / impedance[4] ^ (impedance[5] - 1) else 1 - (1 - depthRatio) ^ impedance[5] / (1 - impedance[4]) ^ (impedance[5] - 1));
  d = impedance[1] + impedanceShape * (impedance[2] - impedance[1]);
  regularizer = (1 - d) / d * edgeInverseMass;

  // Contact law: each edge force solves (A + R) f = aref - a0, with a0 the acceleration without this force
  for e in 1:4 loop
    referenceAcceleration[e] = -damping * edge[e, :] * vRel0 - stiffness * d * height;
    if accelerationTerm then
      edgeAcceleration[e] = edge[e, :] * aRel0;
      edgeForce[e] = noEvent(max(0, if height < 0 then min(1, -height / onsetDepth) * (referenceAcceleration[e] - edgeAcceleration[e] + edgeInverseMass * edgeForceFiltered[e]) / (edgeInverseMass + regularizer) else 0));
      der(edgeForceFiltered[e]) = (edgeForce[e] - edgeForceFiltered[e]) / filterTime;
    else
      edgeForce[e] = noEvent(max(0, if height < 0 then min(1, -height / onsetDepth) * referenceAcceleration[e] / regularizer else 0));
    end if;
  end for;
  normalForce = sum(edgeForce);
  totalForce = edgeForce * edge;

  // Force balance between the two frames
  frame_a.f = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, totalForce);
  frame_a.t = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R, cross(rRel0, totalForce));
  frame_b.f = -Modelica.Mechanics.MultiBody.Frames.resolve2(frame_b.R, totalForce);
  frame_b.t = zeros(3);

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{-80, -60}, {80, -80}}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Backward),
      Line(points = {{-80, -60}, {80, -60}}, color = {64, 64, 64}, thickness = 0.5),
      Line(points = {{-14, -60}, {-14, -46}, {-24, -42}, {-4, -34}, {-24, -26}, {-4, -18}, {-14, -14}, {-14, 0}}, color = {64, 64, 64}),
      Rectangle(extent = {{6, -46}, {26, -24}}, lineColor = {64, 64, 64}),
      Line(points = {{16, -60}, {16, -46}}, color = {64, 64, 64}),
      Line(points = {{16, -32}, {16, 0}}, color = {64, 64, 64}),
      Line(points = {{10, -32}, {22, -32}}, color = {64, 64, 64}),
      Polygon(points = {{-60, 0}, {60, 0}, {64, 8}, {50, 30}, {10, 40}, {-30, 40}, {-60, 22}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid, smooth = Smooth.None),
      Ellipse(extent = {{-6, -6}, {6, 6}}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid),
      Line(points = {{50, 0}, {50, 60}}, color = {0, 128, 180}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}),
      Line(points = {{-30, -10}, {-80, -10}}, color = {0, 128, 180}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}),
      Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Point contact</h4>
<p>Contact of one foot point, <code>frame_b</code>, with a flat ground plane attached to <code>frame_a</code>, following the soft contact of MuJoCo [1] with its default pyramidal friction cone, so that a robot trained in MuJoCo meets the same ground.</p>
<p>Figure 1 shows the quantities. The force is carried by the four edges <i>n</i> &plusmn; <i>&mu; t</i> of the friction pyramid, each pushing with <i>f</i> &ge; 0 while the height <i>h</i> is negative. Each edge solves (<i>A</i> + <i>R</i>) <i>f</i> = <i>a</i><sub>ref</sub> &minus; <i>a</i><sub>0</sub>, with <i>a</i><sub>ref</sub> = &minus;<i>b v</i> &minus; <i>k d h</i>, where <i>v</i> is the speed along the edge, <i>a</i><sub>0</sub> the acceleration without the force, <i>A</i> = <code>edgeInverseMass</code> and <i>R</i> = (1 &minus; <i>d</i>) / <i>d A</i>. <i>k</i> and <i>b</i> follow from <code>timeConstant</code> and <code>dampingRatio</code>, and the impedance <i>d</i> rises with depth as set by <code>impedance</code>. With <i>a</i> = <i>a</i><sub>0</sub> + <i>A f</i> this is <i>R f</i> = <i>a</i><sub>ref</sub> &minus; <i>a</i>: the point approaches the surface critically damped whatever its load, and slides when the load needs more friction than the pyramid allows.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Components/PointContact/pointContact.svg\"
     width=\"600\"
     alt=\"Orange sole corner penetrating the ground plane, with the normal force along n and the friction force opposing the sliding velocity at the contact point\">
<strong>Figure 1:</strong> Contact point, plane and forces.</p>
<p>The parameters that decide the behaviour are:</p>
<p>
<code>timeConstant</code>, <code>dampingRatio</code>, <code>impedance</code> = MuJoCo <code>solref</code> and <code>solimp</code>, defaults as in MuJoCo<br>
<code>mu</code> = Friction coefficient<br>
<code>accelerationTerm</code> = Include <i>a</i>, which gives the right touchdown force but adds stiff filter states and needs an implicit solver<br>
<code>inverseWeight</code> = MuJoCo <code>body_invweight0</code> of the body carrying the point<br>
<code>normal</code> = Plane normal resolved in <code>frame_a</code>
</p>
<p><code>frame_a</code> is the ground, usually <code>world.frame_b</code>. A foot needs several contacts, each on a <code>FixedTranslation</code> from the foot body, as in <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a>.</p>
<h4>Limitations</h4>
<p>By default <i>a</i> is neglected, so <i>f</i> = <i>a</i><sub>ref</sub> / <i>R</i> and the contact has no states. It slips like MuJoCo and gives the same walking speed, but a touchdown at 0.1 m/s peaks at about 150 N instead of 10 N. With <code>accelerationTerm</code>, <i>a</i><sub>0</sub> is estimated from the low-passed acceleration and force (<code>filterTime</code>), which lags briefly at touchdown. The force ramps in over <code>onsetDepth</code> = 10 &micro;m, or a sliding point would chatter. MuJoCo solves all contacts together; here each point is separate. The plane is flat, with no spin or rolling friction.</p>
<h4>References</h4>
<p>[1] Todorov E., Erez T., Tassa Y., MuJoCo: A physics engine for model-based control, IROS 2012. Computation chapter of the MuJoCo 3 documentation.</p>
</html>"));
end PointContact;
