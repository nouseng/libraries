within Roll2RollDynamics.Components;
model WebForce "Prescribed end tension with no external material inventory"
  //Parameters
  parameter Integer direction(min = -1, max = 1) = 1
    "+1 feeds a roller entry; -1 terminates a roller exit"
    annotation(Dialog(group = "Connection"), choices(
      choice = 1 "Feed a roller entry or rewinder",
      choice = -1 "Terminate a roller exit or unwinder"));
  //Inputs and Outputs
  Modelica.Blocks.Interfaces.RealInput force[3](each unit="N")
    "Applied pull in world coordinates"
    annotation(Placement(transformation(extent={{-120,-20},{-80,20}})));
  //Components
  outer Roll2RollDynamics.WebWorld world "Shared web material";

  //Physical connectors
  Roll2RollDynamics.Utilities.Interfaces.SpanPort frame_b "Open web boundary"
    annotation(Placement(transformation(extent={{90,-10},{110,10}})));
equation
  frame_b.stretch = 1 + Modelica.Math.Vectors.length(force)/world.web.EA;
  assert(abs(direction)==1, "WebForce.direction must be -1 or +1");
  for i in 1:3 loop
    frame_b.frame.f[i] = -sum(frame_b.frame.R.T[i,j]*force[j] for j in 1:3);
  end for;
  assert(Modelica.Math.Vectors.length(force) > Roll2RollDynamics.Utilities.Types.forceTol,
    "WebForce requires a nonzero pull to define the span direction");
  frame_b.spanDirection = -direction*force/max(Modelica.Math.Vectors.length(force),
    Roll2RollDynamics.Utilities.Types.forceTol);
  0 = frame_b.frame.R.T[3,:]*force;
  assert(direction*(frame_b.frame.R.T[1,:]*force) < 0,
    "WebForce pull must oppose the directed material axis at an entry and follow it at an exit");
  frame_b.frame.t = zeros(3);
  frame_b.web.f = direction*Modelica.Math.Vectors.length(force);
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}},
      preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Line(points = {{-80, 0}, {-64, 0}}, color = {0, 0, 127}, thickness = 0.5),
        Line(points = {{64, 0}, {100, 0}}, color = {0, 128, 180}, thickness = 2.5),
        Rectangle(lineColor = {100, 110, 120}, fillColor = {245, 247, 249}, fillPattern = FillPattern.Solid, lineThickness = 0.6, extent = {{-64, -32}, {64, 32}}, radius = 6),
        Polygon(lineColor = {181, 84, 28}, fillColor = {181, 84, 28}, fillPattern = FillPattern.Solid,points = {{-44, 0}, {-16, 14}, {-16, 5}, {44, 5},
          {44, -5}, {-16, -5}, {-16, -14}, {-44, 0}}),
        Text(origin = {0, -4}, textColor = {64, 64, 64}, extent = {{-158.975, -76}, {158.975, -46}},
          textString = "%name")}),
    Documentation(info="<html>
<h4>Web force boundary</h4>
<p>This boundary prescribes tensile load at an open web end. Its port supplies
spatial pull, signed transport tension, travel direction and stretch
<code>1 + |force|/world.web.EA</code>; the connected line determines material
speed and position. Figure 1 shows the force input and web port.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/web-force.png\"
     width=\"150\"
     alt=\"WebForce load symbol with force input and web boundary port\">
<strong>Figure 1:</strong> Prescribed-load boundary.</p>
<p><code>direction</code> = <code>+1</code> (default) feeds a roller entry or
rewinder; <code>-1</code> terminates a roller exit or unwinder<br>
<code>force</code> = Required three-component pull in newtons, in world
coordinates regardless of icon rotation; its direction must match tangency</p>
<p>For travel along <code>+x</code>, a 100 N inlet uses
<code>direction = 1</code>, <code>force = {-100, 0, 0}</code>; an outlet uses
<code>direction = -1</code>, <code>force = {100, 0, 0}</code>.</p>
<p>Connect <code>frame_b</code> directly to <code>Roller.frame_a</code>,
<code>Roller.frame_b</code> or <code>Winder.frame_t</code>; no intermediate
<code>Belt</code> is needed. Initialize the adjoining shaft and web states.</p>
<h4>Displayed variables</h4>
<p>This boundary has no results record; inspect its connector:</p>
<ul>
<li><code>frame_b.web.f</code> [N]: Signed transport tension, <code>direction*|force|</code>.</li>
<li><code>frame_b.stretch</code> [1]: Supplied elastic stretch.</li>
<li><code>frame_b.spanDirection</code> [1]: Unit vector of directed span travel in world coordinates.</li>
<li><code>frame_b.web.s</code> [m]: Material transport coordinate set by the connected line.</li>
</ul>
<h4>Limitations</h4>
<p>Load must remain nonzero and tensile. The external web has no stored
material, weight, inertia or viscosity; use <code>Winder</code> for storage
and changing radius.</p>
<p><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>
and <a href=\"modelica://Roll2RollDynamics.Examples.RetractingIdler\">RetractingIdler</a>
use this boundary on open lines.</p>
</html>"));
end WebForce;
