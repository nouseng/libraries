within Roll2RollDynamics.Utilities.Parts;
model NipGeometrySource
  "Publishes the wrapped roller radius and web direction on the nip port"

  // Inputs
  input Modelica.Units.SI.Radius radius(min = Modelica.Constants.small)
    "Wrapped roller mechanical radius";
  input Integer rotationDirection(min = -1, max = 1)
    "Signed web travel direction of the wrapped roller";
  input Modelica.Units.SI.Velocity lateralDrift = 0
    "Axial contact-line speed at which the wrapped web has no axial slip";

  // Physical connectors
  Roll2RollDynamics.Utilities.Interfaces.NipGeometry port
    "Geometry sub-connector of the roller nip port"
    annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));

equation
  port.radius = radius;
  port.rotationDirection = rotationDirection;
  port.lateralDrift = lateralDrift;

  annotation(Icon(graphics = {
      Ellipse(extent = {{-90, 60}, {30, -60}}, lineColor = {64, 64, 64},
        fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid, lineThickness = 0.5),
      Ellipse(extent = {{-38, 8}, {-22, -8}}, lineColor = {64, 64, 64},
        fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid),
      Line(points = {{-30, 0}, {84, 0}}, color = {64, 64, 64}, thickness = 1),
      Text(extent = {{-150, 110}, {150, 70}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Nip geometry source</h4>
<p>This massless adapter writes roller radius, <code>rotationDirection</code>
and normal-entry <code>lateralDrift</code> (default zero) onto
<code>NipPort.geometry</code>, as Figure 1 shows. <code>Roller</code> creates
it only with <code>useNip</code>, referencing the optional port through
<code>connect</code>. The connection set supplies the flow partners on
<code>port</code>; the reader sets them to zero.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/NipGeometrySource/geometry-source.svg\"
     width=\"600\"
     alt=\"radius, rotationDirection and lateralDrift written onto the NipGeometry port\">
<strong>Figure 1:</strong> Writing wrapped-roller geometry onto the nip port.</p></html>"));
end NipGeometrySource;
