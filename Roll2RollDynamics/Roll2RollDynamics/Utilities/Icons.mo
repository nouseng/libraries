within Roll2RollDynamics.Utilities;
package Icons
  "Common icon base models for the roll-to-roll library"
  extends Modelica.Icons.IconsPackage;


  partial model Roll
    "Icon for a roller: clean side view with axle, hub and name"
    annotation(
      Documentation(
        info = "<html>
<p>Icon for a driven or idling roll, drawn as a side view with an axle, a hub
and the web passing over the surface. Extended by
<code>Roll2RollDynamics.Components.Roller</code>.</p>
</html>"
      ),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
        graphics = {
          Ellipse(lineColor = {64, 64, 64},
            fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
            lineThickness = 0.6, extent = {{-72, 72}, {72, -72}}),
          Ellipse(lineColor = {95, 95, 95},
            fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-22, 22}, {22, -22}}),
          Ellipse(lineColor = {64, 64, 64},
            fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-9, 9}, {9, -9}}),
          Text(textColor = {64, 64, 64}, extent = {{-100, -150}, {100, -120}},
            textString = "%name")}
      ));
  end Roll;

  partial model Belt
    "Icon for a web/belt span with travel-direction arrows"
    annotation(
      Documentation(
        info = "<html>
<p>Icon for a single web span, drawn as a ribbon with an arrow showing the
direction the web travels. Extended by
<code>Roll2RollDynamics.Components.Belt</code>.</p>
</html>"
      ),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
        graphics = {
          Rectangle(lineColor = {0, 128, 180}, lineThickness = 0.6,
            fillColor = {224, 242, 248}, fillPattern = FillPattern.Solid,
            extent = {{-84, -16}, {84, 16}}, radius = 4),
          Line(points = {{-100, 0}, {100, 0}}, color = {0, 128, 180}, thickness = 2.5),
          Polygon(lineColor = {0, 128, 180}, fillColor = {0, 128, 180},
            fillPattern = FillPattern.Solid,
            points = {{24, 0}, {0, -10}, {0, 10}, {24, 0}}),
          Text(textColor = {64, 64, 64}, extent = {{-100, -66}, {100, -36}}, textString = "%name")}
      ));
  end Belt;

  partial model Winder
    "Icon for a wind/unwind roll with wound web layers"
    annotation(
      Documentation(
        info = "<html>
<p>Icon for a wind or unwind roll, drawn as concentric layers around a core to
show that the wound diameter changes during operation. Extended by
<code>Roll2RollDynamics.Components.Winder</code>.</p>
</html>"
      ),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
        graphics = {
          Line(points = {{0, 100}, {0, 90}}, color = {95, 95, 95}, thickness = 0.5),
          Line(points = {{-100, 0}, {-90, 0}}, color = {0, 128, 180}, thickness = 2),
          Line(points = {{90, 0}, {100, 0}}, color = {0, 128, 180}, thickness = 2),
          Ellipse(lineColor = {64, 64, 64},
            fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-90, 90}, {90, -90}}),
          Ellipse(lineColor = {0, 128, 180}, extent = {{-70, 70}, {70, -70}}),
          Ellipse(lineColor = {0, 128, 180}, extent = {{-50, 50}, {50, -50}}),
          Ellipse(lineColor = {95, 95, 95},
            fillColor = {115, 115, 120}, fillPattern = FillPattern.Solid, extent = {{-30, 30}, {30, -30}}),
          Ellipse(lineColor = {64, 64, 64},
            fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-10, 10}, {10, -10}}),
          Text(origin = {0, -30}, textColor = {64, 64, 64}, extent = {{-200, -98}, {200, -70}}, textString = "%name")}
      ));
  end Winder;
  annotation(
    Documentation(
      info = "<html>
<p>Shared icon bases for Roll to Roll Dynamics. Rollers, belts and
winders extend the matching base. <code>NipRoller</code> uses a smaller
coloured drum and a loading arrow to distinguish its contact role;
its icon is defined in the component.</p>
<h4>Notes</h4>
<p>The icons follow the Modelica Standard Library conventions, except that
<code>%name</code> is drawn in <code>{64, 64, 64}</code> rather than blue.</p>
</html>"
    )
  );
end Icons;
