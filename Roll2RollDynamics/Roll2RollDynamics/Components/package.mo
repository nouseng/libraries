within Roll2RollDynamics;
package Components
  "Rollers, winders, web spans, nip rolls and web force boundaries"
  extends Modelica.Icons.Package;
  annotation(
    Icon(
      coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}),
      graphics = {
        Ellipse(lineColor = {64, 64, 64}, fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
          lineThickness = 0.6, extent = {{-66, 2}, {-26, 42}}),
        Ellipse(lineColor = {95, 95, 95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid,
          lineThickness = 0.5, extent = {{-53, 15}, {-39, 29}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid,
          lineThickness = 0.5, extent = {{-48.5, 19.5}, {-43.5, 24.5}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
          lineThickness = 0.6, extent = {{-20, -46}, {20, -6}}),
        Ellipse(lineColor = {95, 95, 95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid,
          lineThickness = 0.5, extent = {{-7, -33}, {7, -19}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid,
          lineThickness = 0.5, extent = {{-2.5, -28.5}, {2.5, -23.5}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {210, 218, 224}, fillPattern = FillPattern.Solid,
          lineThickness = 0.6, extent = {{26, 2}, {66, 42}}),
        Ellipse(lineColor = {95, 95, 95}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid,
          lineThickness = 0.5, extent = {{39, 15}, {53, 29}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid,
          lineThickness = 0.5, extent = {{43.5, 19.5}, {48.5, 24.5}}),
        Line(points = {{-96, 43.2}, {-46, 43.2}, {-44, 43.1}, {-42.1, 42.8},
          {-40.1, 42.4}, {-38.2, 41.7}, {-36.4, 40.9}, {-34.7, 39.9},
          {-33.1, 38.8}, {-31.6, 37.5}, {-30.2, 36.1}, {-28.9, 34.6},
          {-27.8, 32.9}, {-26.9, 31.2}, {-26.1, 29.3}, {-25.5, 27.4},
          {-25.1, 25.5}, {-24.9, 23.5}, {-21.1, -27.5}, {-20.5, -31.4},
          {-19.1, -35.2}, {-17.1, -38.6}, {-14.4, -41.5}, {-11.3, -43.9},
          {-7.8, -45.7}, {-3.9, -46.8}, {0, -47.2}, {3.9, -46.8},
          {7.8, -45.7}, {11.3, -43.9}, {14.4, -41.5}, {17.1, -38.6},
          {19.1, -35.2}, {20.5, -31.4}, {21.1, -27.5}, {24.9, 23.5},
          {25.1, 25.5}, {25.5, 27.4}, {26.1, 29.3}, {26.9, 31.2},
          {27.8, 32.9}, {28.9, 34.6}, {30.2, 36.1}, {31.6, 37.5},
          {33.1, 38.8}, {34.7, 39.9}, {36.4, 40.9}, {38.2, 41.7},
          {40.1, 42.4}, {42.1, 42.8}, {44, 43.1}, {46, 43.2},
          {96, 43.2}},
          color = {160, 160, 164}, thickness = 2.5)}),
    Documentation(info = "<html>
<p><a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a>
carries wrapped web; <a href=\"modelica://Roll2RollDynamics.Components.Winder\">Winder</a>
stores wound web; <a href=\"modelica://Roll2RollDynamics.Components.Belt\">Belt</a>
connects tangencies; <a href=\"modelica://Roll2RollDynamics.Components.NipRoller\">NipRoller</a>
loads a nip-enabled roller; and
<a href=\"modelica://Roll2RollDynamics.Components.WebForce\">WebForce</a>
sets an open-end tension.</p>
<p>Standard MSL support frames and drive flanges connect mounts and drives.
Dancers and guides use <code>Roller</code> on external joints. Internal parts
are in <a href=\"modelica://Roll2RollDynamics.Utilities.Parts\">Utilities.Parts</a>.</p>
</html>"
    ));
end Components;
