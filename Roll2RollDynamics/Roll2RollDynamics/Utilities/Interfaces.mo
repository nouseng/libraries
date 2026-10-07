within Roll2RollDynamics.Utilities;
package Interfaces
  "Combined geometry and material transport interfaces"
  extends Modelica.Icons.InterfacesPackage;

  connector SpanPort "Span end: tangent geometry, spatial load and material transport"
    output Real stretch(unit="1")
      "Actual material stretch supplied by the span for boundary mass flux";
    output Real spanDirection[3](each unit="1")
      "Unit material-travel direction supplied by the span, resolved in world coordinates";
    // Diagram draws the terminals; placements locate their connection endpoints.
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame "Tangency geometry and tension load"
      annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a web "Boundary-relative material transport"
      annotation(Placement(visible = false, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}})));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}),
        graphics = {Rectangle(extent = {{-20, -100}, {20, 100}},
          lineColor = {0, 128, 180}, fillColor = {0, 128, 180},
          fillPattern = FillPattern.Solid)}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}),
        graphics = {
          Text(extent = {{-150, 60}, {150, 100}}, textString = "%name",
            textColor = {64, 64, 64}),
          Rectangle(extent = {{-12, -40}, {12, 40}},
            lineColor = {0, 128, 180}, fillColor = {0, 128, 180},
            fillPattern = FillPattern.Solid),
          Rectangle(extent = {{-10, -70}, {10, -50}},
            lineColor = {0, 127, 0}, fillColor = {0, 127, 0},
            fillPattern = FillPattern.Solid)}));
  end SpanPort;

  connector RollPort "Roll end: tangent geometry, spatial load and material transport"
    input Real stretch(unit="1")
      "Actual material stretch supplied by the connected span";
    input Real spanDirection[3](each unit="1")
      "Unit material-travel direction supplied by the connected span, resolved in world coordinates";
    // Diagram draws the terminals; placements locate their connection endpoints.
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame "Tangency geometry and tension load"
      annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Interfaces.Flange_a web "Boundary-relative material transport"
      annotation(Placement(visible = false, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}})));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}),
        graphics = {Rectangle(extent = {{-20, -100}, {20, 100}},
          lineColor = {0, 128, 180}, fillColor = {255, 255, 255},
          fillPattern = FillPattern.Solid)}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}),
        graphics = {
          Text(extent = {{-150, 60}, {150, 100}}, textString = "%name",
            textColor = {64, 64, 64}),
          Rectangle(extent = {{-12, -40}, {12, 40}},
            lineColor = {0, 128, 180}, fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Rectangle(extent = {{-10, -70}, {10, -50}},
            lineColor = {0, 127, 0}, fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid)}));
  end RollPort;

  connector NipGeometry
    "Wrapped roller radius and web direction carried on the nip port"
    Modelica.Units.SI.Radius radius "Wrapped roller mechanical radius";
    Real rotationDirection(unit = "1")
      "Signed web travel direction of the wrapped roller, +1 or -1";
    Modelica.Units.SI.Velocity lateralDrift
      "Contact-line speed along the roller axis at which the wrapped web has no axial slip";
    // ponytail: zero-valued flow partners keep the connector balanced; the
    // reading side sets them to zero, the wrapped roller sets the values
    flow Real radiusFlow(unit = "1") "Balancing flow partner of radius";
    flow Real rotationDirectionFlow(unit = "1")
      "Balancing flow partner of rotationDirection";
    flow Real lateralDriftFlow(unit = "1") "Balancing flow partner of lateralDrift";
    annotation(Icon(graphics = {
        Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {95, 95, 95},
          fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid)}),
      Documentation(info = "<html><p>Three values the wrapped roller
publishes for the nip contact. <code>lateralDrift</code> is the roller's
normal-entry walk, the axial contact-line speed at which the wrapped web
rolls across the roller without sliding; the nip measures its own lateral
slip against the material, not the contact line. Each carries a flow partner that every
connected side sets to zero, so the port stays balanced while the values
propagate through one connect equation.</p></html>"));
  end NipGeometry;

  connector NipPort
    "Roll-centre frame and web transport in one connection"
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame
      "Wrapped roller centre and normal and tangential reactions";
    Modelica.Mechanics.Translational.Interfaces.Flange_a web
      "Web transport and tangential reaction";
    Modelica.Mechanics.Translational.Interfaces.Flange_a lateral
      "Cross-machine web position relative to the wrapped roller and lateral reaction";
    Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft
      "Wrapped roller shaft angle sensed without applying excitation torque";
    NipGeometry geometry "Wrapped roller radius and web direction";
    annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}),
      graphics = {
        Rectangle(extent = {{-30, -100}, {30, 100}},
          lineColor = {95, 95, 95}, fillColor = {192, 192, 192},
          fillPattern = FillPattern.Solid),
        Rectangle(extent = {{-20, -20}, {20, 20}},
          lineColor = {0, 127, 0}, fillColor = {255, 255, 255},
          fillPattern = FillPattern.Solid)}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}),
        graphics = {
          Text(extent = {{-140, -50}, {140, -90}}, textString = "%name",
            textColor = {64, 64, 64}),
          Rectangle(extent = {{-12, 40}, {12, -40}},
            lineColor = {95, 95, 95}, fillColor = {192, 192, 192},
            fillPattern = FillPattern.Solid),
          Rectangle(extent = {{-10, 10}, {10, -10}},
            lineColor = {0, 127, 0})}),
  Documentation(info = "<html><p>A spatial frame and a translational web
flange preserve the spatial-reaction and relative web-power paths in one port.
The frame carries the normal, tangential and cross-machine contact forces;
the web flange carries the tangential transport effort and the
<code>lateral</code> flange the cross-machine web position along the wrapped
roller axis, the same coordinate as the wrapped roller's
<code>lateralFlange</code>, so a cocked nip can drag the web sideways.
The rotational <code>shaft</code>
flange supplies the wrapped roller's actual shaft angle, including slip
relative to web transport. The nip contact reads this angle and applies zero
torque to the flange. The <code>geometry</code> sub-connector carries the
wrapped roller radius and <code>rotationDirection</code>, published by the
wrapped roller and read by the nip contact. Standalone contact boundaries must connect it to a fixed or driven rotational
source, and <code>lateral</code> to a translational boundary. The port is
symmetric: the same type is used on the wrapped roller, on the nip roller and on
the contact element itself, so one connect equation joins them.</p></html>"));
  end NipPort;

  annotation(Documentation(info="<html><p><code>SpanPort</code> and <code>RollPort</code> combine a
MultiBody frame and a translational material-transport flange. The frame's
local x axis follows directed material travel from <code>Belt</code> entry to exit; its
orientation determines tangency independently of spatial force. The span
supplies the world-resolved unit <code>spanDirection</code> through the same
port, so the roll computes cross-machine skew and constructs the tangency
frame orientation from the span geometry. The web
flange velocity is transport relative to that moving tangency. Spatial forces
carry tension, gravity and support inertia, while the web flange carries
signed longitudinal effort. Each component owns its material inventory;
no inventory is transferred through a signal.</p>
<p>The span supplies both signals automatically. <code>stretch</code> is
current material length divided by unstretched length; 1.01 means 1% extension.
Boundary mass flow is reference mass per unit length times boundary-relative
transport speed divided by <code>stretch</code>. Sharing the actual end-cell
stretch keeps the span and roll mass flows consistent; total tension cannot
determine it when viscous force is present. <code>spanDirection</code> points
from span entry to exit and also supplies the direction needed for lateral
contact calculations. The standard translational flange carries position and
force, so these material and direction signals are supplied separately.</p>
<p>Custom span-side boundaries must supply positive <code>stretch</code> in
addition to <code>spanDirection</code>. WebForce supplies the elastic
stretch of its prescribed pull. This changes the orientation contract
for custom boundaries: span-side boundaries must supply the directed unit
vector and impose <code>frame.R.T[3,:]*spanDirection = 0</code> for the
in-plane tangency. Roll-side boundaries use the vector to compute skew while
leaving that tangency free.
Use WebForce for a prescribed pull. The nip port retains its centre frame
and physical contact transport.</p></html>"));
end Interfaces;
