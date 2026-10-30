within MicroDuck.Examples;
model Standing "Robot dropped from 2.8 mm onto the ground with the servos holding the home pose"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Real positionGain(unit = "1") = 800 "Position P gain register value of the servos";
  parameter Boolean checkResults = true "Assert that the robot stands";
  parameter Modelica.Units.SI.Length initialHeight = 0.12 "Initial height of the trunk frame";
  parameter Real loadTolerance(unit = "1") = 0.02 "Allowed relative error of the total ground force";
  parameter Modelica.Units.SI.Length minHeight = 0.105 "Lowest allowed trunk height";
  parameter Modelica.Units.SI.AngularVelocity restTolerance = 1e-2 "Allowed joint speed at the end of the run";
  //Variables
  Modelica.Units.SI.Length trunkHeight = robot.frame_a.r_0[3] "Height of the trunk frame";
  Modelica.Units.SI.Angle trunkPitch = asin(-robot.frame_a.R.T[1, 3]) "Pitch of the trunk about the world y axis, positive nose down";
  Modelica.Units.SI.Force leftLoad = sum(robot.contactLeft.normalForce) "Normal force on the left sole";
  Modelica.Units.SI.Force rightLoad = sum(robot.contactRight.normalForce) "Normal force on the right sole";
  Modelica.Units.SI.Force totalLoad = leftLoad + rightLoad "Normal force on both soles";
  Modelica.Units.SI.Force weight = robot.totalMass * world.g "Weight of the robot";
  Modelica.Units.SI.AngularVelocity jointSpeed = noEvent(max({abs(robot.w[i]) for i in 1:14})) "Largest joint speed";
  //Components
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion trunkJoint(
    r_rel_a(start = {0, 0, initialHeight}, each fixed = true),
    v_rel_a(each fixed = true),
    angles_fixed = true,
    w_rel_a_fixed = true,
    animation = false) annotation(Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape floor(
    shapeType = "box",
    r_shape = {-0.2, 0, -0.005},
    length = 0.4, width = 0.4, height = 0.01,
    color = {190, 190, 190}) annotation(Placement(transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Robot robot(positionGain = positionGain, contactAcceleration = true) annotation(Placement(transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Sources.RealExpression[14] homePose(y = robot.q0) "Joint angle targets, the home pose" annotation(Placement(transformation(origin = {10, 40}, extent = {{-14, -5}, {14, 5}})));
equation
  connect(world.frame_b, trunkJoint.frame_a) annotation(Line(points = {{-70, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(world.frame_b, floor.frame_a) annotation(Line(points = {{-70, 0}, {-55, 0}, {-55, 40}, {-40, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(trunkJoint.frame_b, robot.frame_a) annotation(Line(points = {{-20, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(homePose.y, robot.qRef) annotation(Line(points = {{25.4, 40}, {40, 40}, {40, 20}}, color = {0, 0, 127}));

  when terminal() and checkResults then
    assert(abs(totalLoad - weight) < loadTolerance * weight, "Ground force differs from the weight");
    assert(trunkHeight > minHeight, "The robot has collapsed");
    assert(jointSpeed < restTolerance, "The joints have not come to rest");
  end when;

  annotation(
    Diagram(coordinateSystem(extent = {{-100, -50}, {80, 60}}), graphics = {Text(extent = {{-98, -22}, {2, -48}}, textString = "Simulation settings:
CVODE (implicit), tolerance 1e-6, 3 s
contactAcceleration = true", textColor = {0, 0, 127}, horizontalAlignment = TextAlignment.Left)}),
    experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-6, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--maxSizeLinearTearing=1000",
    __OpenModelica_simulationFlags(s = "cvode"),
    Documentation(
      info = "<html>
<h4>Standing on the soles</h4>
<p><a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> starts at the home pose with the trunk 120 mm up, which leaves the soles 2.8 mm above the ground. <code>homePose</code> feeds <code>q0</code> to the servo targets <code>qRef</code>, held with <code>positionGain</code> = 800; at the walking gain of 200 the robot tips forward without a policy (set <code>positionGain = 200</code> and <code>checkResults = false</code> to see it).</p>
<p>Figure 1 compares the MicroDuck library with MuJoCo running the same model, servo, gear play and gain [1]. On impact each sole peaks at 8.4 N against 10.5 N and the trunk dips to 115.3 mm against 114.9 mm, both at 0.05 s. The settled loads agree at half of the 7.2 N weight per sole, and the trunk stays within 0.2 mm after 0.2 s.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Examples/Standing/standing.png\"
     width=\"600\"
     alt=\"Sole loads and trunk height of the standing robot in the MicroDuck library and in MuJoCo\">
<strong>Figure 1:</strong> Sole loads and trunk height compared with MuJoCo.</p>
<h4>References</h4>
<p>[1] Todorov E., Erez T., Tassa Y., MuJoCo: A physics engine for model-based control, IROS 2012. MuJoCo 3.15 Python bindings.</p>
</html>",
      figures = {
        Figure(title = "Sole loads", identifier = "soleLoads", preferred = true, plots = {Plot(title = "Normal force on the soles", curves = {Curve(y = "leftLoad", legend = "Left sole"), Curve(y = "rightLoad", legend = "Right sole")}, y = Axis(unit = "N"))}),
        Figure(title = "Trunk height", identifier = "trunkHeight", plots = {Plot(title = "Height of the trunk frame", curves = {Curve(y = "trunkHeight", legend = "Trunk frame height")}, y = Axis(unit = "m"))})}));
end Standing;
