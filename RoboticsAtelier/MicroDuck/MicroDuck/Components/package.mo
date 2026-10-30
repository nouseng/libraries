within MicroDuck;
package Components "Robot, servo, battery, joint link, sole contact and walking policy"
  extends Modelica.Icons.Package;

  annotation(
    Icon(graphics = {
      Polygon(points = {{18, -60}, {82, -60}, {86, -52}, {78, -40}, {36, -32}, {18, -42}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Rectangle(origin = {5, 5}, rotation = -38, extent = {{-57, -10}, {57, 10}}, lineColor = {64, 64, 64}, fillColor = {172, 164, 153}, fillPattern = FillPattern.Solid, radius = 4),
      Rectangle(extent = {{-72, 8}, {-8, 72}}, lineColor = {64, 64, 64}, fillColor = {66, 71, 77}, fillPattern = FillPattern.Solid, radius = 6),
      Ellipse(extent = {{-58, 22}, {-22, 58}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-44, 36}, {-36, 44}}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid)}),
    Documentation(info = "<html>
<h4>Components</h4>
<p><a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> is the complete plant. It is built from <a href=\"modelica://MicroDuck.Components.Link\">Link</a>, a rigid body behind a revolute joint, <a href=\"modelica://MicroDuck.Components.PositionServo\">PositionServo</a>, which drives such a joint, <a href=\"modelica://MicroDuck.Components.Battery\">Battery</a>, an optional two-cell battery pack with state of charge and internal impedance, and <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a>, the ground contact of one sole point.
<a href=\"modelica://MicroDuck.Components.WalkingPolicy\">WalkingPolicy</a> turns sensor signals into joint angle targets.</p>
</html>"));
end Components;
