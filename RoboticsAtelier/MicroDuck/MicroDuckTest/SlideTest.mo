within MicroDuckTest;
model SlideTest "Mass sliding on one contact point, compared with MuJoCo"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Modelica.Units.SI.Mass mass = 0.8 "Sliding mass";
  parameter Modelica.Units.SI.Velocity initialSpeed = 0.5 "Initial sliding speed";
  parameter Real mu(unit = "1") = 0.5 "Friction coefficient";
  parameter Modelica.Units.SI.Length expectedDistance = 0.025771 "Stopping distance in MuJoCo";
  parameter Modelica.Units.SI.Length tolerance = 1e-3 "Allowed error of the stopping distance";
  //Variables
  Modelica.Units.SI.Length distance = body.r_0[1] "Sliding distance";
  Modelica.Units.SI.Length distanceError = distance - expectedDistance "Sliding distance minus the MuJoCo distance";
  //Components
  inner Modelica.Mechanics.MultiBody.World world "World frame with gravity along negative y" annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.PointContact contact(mu = mu, accelerationTerm = true) "Single contact point on the y = 0 ground plane" annotation(Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(
    m = mass,
    r_CM = {0, 0, 0},
    I_11 = 1e-3,
    I_22 = 1e-3,
    I_33 = 1e-3,
    r_0(start = {0, 0, 0}, each fixed = true),
    v_0(start = {initialSpeed, 0, 0}, each fixed = true),
    angles_fixed = true,
    w_0_fixed = true,
    useQuaternions = false,
    animation = false) "Sliding rigid mass" annotation(Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(world.frame_b, contact.frame_a) annotation(Line(points = {{-60, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(body.frame_a, contact.frame_b) annotation(Line(points = {{40, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));

  when terminal() then
    assert(abs(distanceError) < tolerance, "Sliding distance differs from MuJoCo by more than the tolerance");
  end when;

  annotation(
    Diagram(coordinateSystem(extent = {{-100, -50}, {80, 60}})),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.001),
    Documentation(
      info = "<html>
<h4>Sliding to rest</h4>
<p>A 0.8 kg mass (<code>mass</code>) starts at 0.5 m/s (<code>initialSpeed</code>) on one <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a> with friction <code>mu</code> = 0.5 and <code>accelerationTerm = true</code>, touching the plane. MuJoCo launches the same mass on a sphere of 0.1 mm radius with a 0.1 ms time step [1].</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuckTest/Resources/Images/SlideTest/distance.png\"
     width=\"600\"
     alt=\"Sliding distance approaching the Coulomb stopping distance\">
<strong>Figure 1:</strong> Sliding distance of the mass.</p>
<p>Figure 1 shows <code>distance</code> approaching 26.5 mm against <code>expectedDistance</code> = 25.8 mm in MuJoCo (with difference <code>distanceError</code>), both beyond the Coulomb value <i>v</i><sub>0</sub><sup>2</sup> / (2 <i>&mu; g</i>) = 25.5 mm because the soft cone lets the mass creep. The result depends on how the contact force switches on while the point hovers: MuJoCo gives 25.3 mm at its default 2 ms step, and the 0.7 mm here comes from the ramp over <code>onsetDepth</code>. The assertion requires a match within <code>tolerance</code>.</p>
<h4>References</h4>
<p>[1] Todorov E., Erez T., Tassa Y., MuJoCo: A physics engine for model-based control, IROS 2012. MuJoCo 3.15 Python bindings.</p>
</html>",
      figures = {Figure(title = "Sliding distance", identifier = "slideDistance", preferred = true, plots = {Plot(title = "Distance travelled", curves = {Curve(y = "distance", legend = "Sliding distance")}, y = Axis(unit = "m"))})}));
end SlideTest;
