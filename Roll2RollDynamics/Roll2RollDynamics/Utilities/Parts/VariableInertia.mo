within Roll2RollDynamics.Utilities.Parts;
model VariableInertia
  "Rotational inertia whose moment of inertia changes during operation"
  input Modelica.Units.SI.Inertia J "Instantaneous moment of inertia";
  input Modelica.Units.SI.Torque momentumFlow = 0
    "Angular momentum carried into the control volume per second";
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a "Shaft"
    annotation(Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Units.SI.AngularVelocity w "Shaft angular velocity";
protected
  Modelica.Units.SI.Inertia Jeff
    "Moment of inertia held clear of zero, where the shaft equation is singular";
equation
  Jeff = max(J, Roll2RollDynamics.Utilities.Types.inertiaTol);
  w = der(flange_a.phi);
  flange_a.tau + momentumFlow = der(Jeff*w);
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
        graphics = {
      Line(points = {{-100, 0}, {-60, 0}}, color = {95, 95, 95}, thickness = 0.5),
      Ellipse(lineColor = {64, 64, 64}, fillColor = {210, 218, 224},
        fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-60, 60}, {60, -60}}),
      Ellipse(lineColor = {0, 128, 180}, extent = {{-42, 42}, {42, -42}}),
      Ellipse(lineColor = {0, 128, 180}, extent = {{-24, 24}, {24, -24}}),
      Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64},
        fillPattern = FillPattern.Solid, extent = {{-10, 10}, {10, -10}}),
      Text(textColor = {64, 64, 64}, extent = {{-100, -98}, {100, -70}}, textString = "%name")}),
  Documentation(info = "<html>
<h4>Variable inertia</h4>
<p>This shaft inertia uses parent-fed input <code>J</code> and angular momentum
inflow <code>momentumFlow</code> (default zero). Figure 1 shows both contributions.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/VariableInertia/variable-inertia.svg\"
     width=\"600\"
     alt=\"Shaft with time-varying moment of inertia J and incoming momentumFlow\">
<strong>Figure 1:</strong> Shaft inertia with time-varying moment of inertia.</p>
<p>Connect <code>flange_a</code> to the shaft. The balance is
<code>tau = der(J*w) - momentumFlow</code>, with effective inertia floored
at <code>Types.inertiaTol</code> to prevent singularity at zero.
A winder supplies the momentum of incoming or outgoing material;
use <code>Inertia</code> for constant inertia.</p>
<h4>Limitations</h4>
<p><code>J</code> should follow a state: its derivative is required, so an
algebraically determined inertia brings that system into index reduction.</p>
</html>"));
end VariableInertia;
