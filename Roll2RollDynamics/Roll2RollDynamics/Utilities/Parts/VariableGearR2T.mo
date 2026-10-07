within Roll2RollDynamics.Utilities.Parts;
model VariableGearR2T
  "Power-conserving rotational-to-translational transformer with variable radius"
  input Modelica.Units.SI.Length radius
    "Instantaneous transformation radius, signed by the winding sense";
  input Modelica.Units.SI.Velocity velocityOffset = 0
    "Boundary migration correction; its power belongs to the moving geometry";
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeR
    "Rotational shaft"
    annotation(Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flangeT
    "Tangential web coordinate"
    annotation(Placement(transformation(extent = {{90, -10}, {110, 10}})));
protected
  Modelica.Units.SI.Length radiusEff
    "Transformation radius held clear of zero, where the transformer is singular";
equation
  radiusEff = noEvent(if radius >= 0
    then max(radius, Roll2RollDynamics.Utilities.Types.lengthTol)
    else min(radius, -Roll2RollDynamics.Utilities.Types.lengthTol));
  der(flangeT.s) = radiusEff*der(flangeR.phi) + velocityOffset;
  flangeR.tau + radiusEff*flangeT.f = 0;
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Ellipse(extent = {{-70, 44}, {18, -44}}, lineColor = {64, 64, 64},
        fillColor = {235, 238, 240}, fillPattern = FillPattern.Solid),
      Line(points = {{18, 0}, {100, 0}}, color = {0, 127, 0}),
      Text(extent = {{-100, 86}, {100, 54}}, textString = "%name",
        textColor = {64, 64, 64})}),
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Text(extent = {{-100, 70}, {100, 40}}, textString = "%name",
        textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Variable-radius transformer</h4>
<p>This transformer couples shaft <code>flangeR</code> to web transport at
<code>flangeT</code> using parent-fed input <code>radius</code>. Figure 1
shows the coupling.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/VariableGearR2T/variable-radius-coupling.svg\"
     width=\"600\"
     alt=\"Rotational flangeR coupled to translational flangeT through the instantaneous radius\">
<strong>Figure 1:</strong> Variable-radius rotational to translational coupling.</p>
<p><code>v = radius*w + velocityOffset</code> and
<code>tau = -radius*f</code>. Default zero offset gives lossless conversion.
For moving tangencies, the offset makes web speed relative to the boundary;
<code>f*velocityOffset</code> power belongs to that geometry, not the shaft.
Use <code>IdealGearR2T</code> for constant radius.</p>
</html>"));
end VariableGearR2T;
