within MicroDuckTest;
model DropTest "Mass dropped on one contact point, compared with MuJoCo"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Modelica.Units.SI.Mass mass = 0.8 "Dropped mass";
  parameter Modelica.Units.SI.Length dropHeight = 0.05 "Initial height above the plane";
  parameter Modelica.Units.SI.Length lowestHeight = -7.358e-3 "Lowest height in MuJoCo";
  parameter Modelica.Units.SI.Length finalHeight = -3.672e-4 "Final height in MuJoCo";
  parameter Modelica.Units.SI.Length tolerance = 2e-6 "Allowed error of the final height";
  parameter Modelica.Units.SI.Length lowestTolerance = 5e-5 "Allowed error of the lowest height";
  //Variables
  discrete Modelica.Units.SI.Length lowest(start = 0, fixed = true) "Lowest height so far, to 1 um";
  //Components
  inner Modelica.Mechanics.MultiBody.World world "World frame with gravity along negative y" annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.PointContact contact(accelerationTerm = true) "Single contact point on the y = 0 ground plane" annotation(Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(
    m = mass,
    r_CM = {0, 0, 0},
    I_11 = 1e-3,
    I_22 = 1e-3,
    I_33 = 1e-3,
    r_0(start = {0, dropHeight, 0}, each fixed = true),
    v_0(each fixed = true),
    angles_fixed = true,
    w_0_fixed = true,
    useQuaternions = false,
    animation = false) "Dropped rigid mass" annotation(Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(world.frame_b, contact.frame_a) annotation(Line(points = {{-60, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(body.frame_a, contact.frame_b) annotation(Line(points = {{40, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));

  when contact.height < pre(lowest) - 1e-6 then
    lowest = contact.height;
  end when;
  when terminal() then
    assert(abs(contact.height - finalHeight) < tolerance, "Final height differs from MuJoCo by more than the tolerance");
    assert(abs(lowest - lowestHeight) < lowestTolerance, "Lowest height differs from MuJoCo by more than lowestTolerance");
  end when;

  annotation(
    Diagram(coordinateSystem(extent = {{-100, -50}, {80, 60}})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.001),
    Documentation(
      info = "<html>
<h4>Drop onto a single contact point</h4>
<p>Drops a <code>mass</code> = 0.8 kg body from <code>dropHeight</code> = 50 mm onto one <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a> with the inertial contact term enabled (<code>accelerationTerm</code> = <code>true</code>). MuJoCo drops the same mass on a sphere of 0.1 mm radius with a 0.1 ms time step [1]; at its default 2 ms step it dips only to &minus;6.04 mm, an integration-step effect.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuckTest/Resources/Images/DropTest/height.png\"
     width=\"600\"
     alt=\"Height of the contact point falling to a small negative value and settling\">
<strong>Figure 1:</strong> Height of the contact point.</p>
<p>Figure 1 shows <code>contact.height</code> dipping to <code>lowest</code> = &minus;7.36 mm and settling at &minus;0.367 mm, matching MuJoCo. The terminal assertions require the peak penetration and settled height to match <code>lowestHeight</code> and <code>finalHeight</code> within <code>lowestTolerance</code> and <code>tolerance</code>.</p>
<h4>References</h4>
<p>[1] Todorov E., Erez T., Tassa Y., MuJoCo: A physics engine for model-based control, IROS 2012. MuJoCo 3.15 Python bindings.</p>
</html>",
      figures = {Figure(title = "Drop onto one contact point", identifier = "dropHeight", preferred = true, plots = {Plot(title = "Height of the contact point", curves = {Curve(y = "contact.height", legend = "Contact point height")}, y = Axis(unit = "m"))})}));
end DropTest;
