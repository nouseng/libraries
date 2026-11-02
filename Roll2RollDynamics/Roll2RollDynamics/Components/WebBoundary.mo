within Roll2RollDynamics.Components;
model WebBoundary "Prescribed web end pull or MultiBody belt-clamp boundary"
  //Parameters
  parameter Boolean useSupport = false
    "= true, clamp a connected Belt to frame_support; = false, prescribe force[3] at frame_b"
    annotation(Evaluate = true, Dialog(group = "Connection"));
  parameter Integer direction(min = -1, max = 1) = 1
    "+1 feeds a roller entry; -1 terminates a roller exit"
    annotation(Dialog(group = "Connection", enable = not useSupport), choices(
      choice = 1 "Feed a roller entry or rewinder",
      choice = -1 "Terminate a roller exit or unwinder"));
  parameter Modelica.Units.SI.Angle tangencyAngleStart = 0
    "Initial guess of the clamp angle about the local y axis of frame_support; pi/2 for a span leaving straight down"
    annotation(Dialog(tab = "Initialization", enable = useSupport));

  //Variables
  Modelica.Units.SI.Angle angleResidual(start = 0)
    "Residual tangency angle solved by the connected belt"
    annotation(HideResult = true);
  Modelica.Units.SI.Angle tangencyAngle(start = tangencyAngleStart)
    "In-plane tangency angle of the clamped belt relative to frame_support"
    annotation(HideResult = true);
  Modelica.Units.SI.DimensionlessRatio localDirection[3]
    "Connected belt direction resolved in frame_support"
    annotation(HideResult = true);

  //Inputs and Outputs
  Modelica.Blocks.Interfaces.RealInput force[3](each unit = "N") if not useSupport
    "Applied pull in world coordinates"
    annotation(Placement(transformation(extent = {{-120, -20}, {-80, 20}})));

  //Components
  outer Roll2RollDynamics.WebWorld world "Shared web material";
  Roll2RollDynamics.Utilities.Parts.TangentFrame tangent(
    r = zeros(3),
    transmitAxialMoment = true,
    direction = 1,
    angle = tangencyAngle,
    spanDirection = frame_a.spanDirection) if useSupport
    "Freely aligned tangency adapter at the body clamp"
    annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

  //Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_support if useSupport
    "Body or carriage clamp connection"
    annotation(Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
  Roll2RollDynamics.Utilities.Interfaces.RollPort frame_a if useSupport
    "Clamped belt endpoint"
    annotation(Placement(transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}})));
  Roll2RollDynamics.Utilities.Interfaces.SpanPort frame_b if not useSupport
    "Open web boundary"
    annotation(Placement(transformation(extent = {{90, -10}, {110, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}})));
equation
  connect(frame_support, tangent.frame_a)
    annotation(Line(points = {{-100, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(tangent.frame_b, frame_a.frame)
    annotation(Line(points = {{20, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  if useSupport then
    localDirection = Modelica.Mechanics.MultiBody.Frames.resolve2(
      frame_support.R, frame_a.spanDirection);
    tangencyAngle = Modelica.Math.atan2(
      -localDirection[3],
      if abs(localDirection[1]) + abs(localDirection[3]) > Roll2RollDynamics.Utilities.Types.lengthTol
        then localDirection[1] else 1) + angleResidual;
    frame_a.web.s = 0;
  else
    localDirection = {1, 0, 0};
    angleResidual = 0;
    tangencyAngle = 0;
    frame_b.stretch = 1 + Modelica.Math.Vectors.length(force)/world.web.EA;
    assert(abs(direction) == 1, "WebBoundary.direction must be -1 or +1");
    for i in 1:3 loop
      frame_b.frame.f[i] = -sum(frame_b.frame.R.T[i, j]*force[j] for j in 1:3);
    end for;
    assert(Modelica.Math.Vectors.length(force) > Roll2RollDynamics.Utilities.Types.forceTol,
      "WebBoundary requires a nonzero pull to define the span direction");
    frame_b.spanDirection = -direction*force/max(Modelica.Math.Vectors.length(force),
      Roll2RollDynamics.Utilities.Types.forceTol);
    0 = frame_b.frame.R.T[3, :]*force;
    assert(direction*(frame_b.frame.R.T[1, :]*force) < 0,
      "WebBoundary pull must oppose the directed material axis at an entry and follow it at an exit");
    frame_b.frame.t = zeros(3);
    frame_b.web.f = direction*Modelica.Math.Vectors.length(force);
  end if;
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}},
      preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Line(visible = not useSupport, points = {{-80, 0}, {-64, 0}}, color = {0, 0, 127}, thickness = 0.5),
        Line(points = {{64, 0}, {100, 0}}, color = {0, 128, 180}, thickness = 2.5),
        Rectangle(lineColor = {100, 110, 120}, fillColor = {245, 247, 249}, fillPattern = FillPattern.Solid, lineThickness = 0.6, extent = {{-64, -32}, {64, 32}}, radius = 6),
        Polygon(visible = not useSupport, lineColor = {181, 84, 28}, fillColor = {181, 84, 28}, fillPattern = FillPattern.Solid, points = {{-44, 0}, {-16, 14}, {-16, 5}, {44, 5},
          {44, -5}, {-16, -5}, {-16, -14}, {-44, 0}}),
        Rectangle(visible = useSupport, lineColor = {64, 64, 64}, fillColor = {160, 170, 180}, fillPattern = FillPattern.Solid, lineThickness = 0.6, extent = {{-44, -18}, {44, 18}}),
        Text(origin = {0, -4}, textColor = {64, 64, 64}, extent = {{-158.975, -76}, {158.975, -46}},
          textString = "%name")}),
    Documentation(info="<html>
<h4>Web boundary</h4>
<p>This boundary either prescribes tensile load at an open web end
(<code>useSupport = false</code>) or clamps the end of a <code>Belt</code>
span to a MultiBody frame (<code>useSupport = true</code>). Figure 1 shows the
prescribed-load boundary.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/web-force.png\"
     width=\"150\"
     alt=\"WebBoundary load symbol with force input and web boundary port\">
<strong>Figure 1:</strong> Prescribed-load boundary.</p>
<p><code>useSupport</code> = <code>false</code> (default) exposes
<code>force</code> and <code>frame_b</code> for open-end tension;
<code>true</code> exposes <code>frame_support</code> and <code>frame_a</code>
to clamp a <code>Belt</code> to a MultiBody body<br>
<code>direction</code> = <code>+1</code> (default) feeds a roller entry or
rewinder; <code>-1</code> terminates a roller exit or unwinder (when
<code>useSupport = false</code>)<br>
<code>force</code> = Required three-component pull in newtons, in world
coordinates regardless of icon rotation; its direction must match tangency
(when <code>useSupport = false</code>)<br>
<code>tangencyAngleStart</code> = Initial guess of the clamp angle about the local y axis of <code>frame_support</code>, default 0 (when <code>useSupport = true</code>)</p>
<p>For prescribed-force travel along <code>+x</code>, a 100 N inlet uses
<code>direction = 1</code>, <code>force = {-100, 0, 0}</code>; an outlet uses
<code>direction = -1</code>, <code>force = {100, 0, 0}</code>. Connect
<code>frame_b</code> directly to <code>Roller.frame_a</code>,
<code>Roller.frame_b</code> or <code>Winder.frame_t</code>; no intermediate
<code>Belt</code> is needed.</p>
<p>When <code>useSupport = true</code>, connect <code>frame_support</code> to a
MultiBody body or carriage and <code>frame_a</code> to <code>Belt.frame_a</code>
or <code>Belt.frame_b</code>. The clamp locks material transport relative to the
body (<code>frame_a.web.s = 0</code>) while freely aligning the tangency axis
with the connected belt and transmitting the spatial pull to
<code>frame_support</code>. The clamp angle about the local y axis of
<code>frame_support</code> has two roots half a turn apart, and initialization
takes the one nearest <code>tangencyAngleStart</code> (default 0, a span leaving
along local +x). Set it near the expected span direction, for example
<code>pi/2</code> for a span leaving straight down; a span leaving along local
&minus;x or straight down otherwise starts on the flipped root and stops on the
span-direction assertion.</p>
<h4>Displayed variables</h4>
<p>This boundary has no results record; inspect its active connector:</p>
<ul>
<li><code>frame_b.web.f</code> [N]: Signed transport tension, <code>direction*|force|</code> (when <code>useSupport = false</code>).</li>
<li><code>frame_b.stretch</code> [1]: Supplied elastic stretch (when <code>useSupport = false</code>).</li>
<li><code>frame_b.spanDirection</code> [1]: Unit vector of directed span travel in world coordinates (when <code>useSupport = false</code>).</li>
<li><code>frame_a.web.f</code> [N]: Clamped belt end tension (when <code>useSupport = true</code>).</li>
</ul>
<h4>Limitations</h4>
<p>In prescribed-force mode (<code>useSupport = false</code>), load must remain
nonzero and tensile, and the external web has no stored material, weight,
inertia or viscosity; use <code>Winder</code> for storage and changing
radius.</p>
<p><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>
and <a href=\"modelica://Roll2RollDynamics.Examples.RetractingIdler\">RetractingIdler</a>
use this boundary on open lines; <a href=\"modelica://Roll2RollDynamics.Examples.BeltDrivenCarriage\">BeltDrivenCarriage</a> uses it with <code>useSupport = true</code> to clamp a belt to a moving carriage.</p>
</html>"));
end WebBoundary;
