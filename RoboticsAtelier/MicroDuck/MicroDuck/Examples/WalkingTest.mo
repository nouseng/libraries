within MicroDuck.Examples;
model WalkingTest "Robot walking forward under the neural network policy"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Modelica.Units.SI.Velocity forwardSpeed = 0.3 "Commanded forward speed";
  parameter Boolean checkResults = true "Assert that the robot walks";
  parameter Modelica.Units.SI.Length minHeight = 0.1 "Lowest allowed trunk height";
  parameter Modelica.Units.SI.Length minDistance = 0.5 "Shortest allowed walking distance";
  //Variables
  Modelica.Units.SI.Length distance = robot.frame_a.r_0[1] "Forward position of the trunk frame";
  Modelica.Units.SI.Length trunkHeight = robot.frame_a.r_0[3] "Height of the trunk frame";
  Modelica.Units.SI.Angle trunkPitch = asin(-robot.frame_a.R.T[1, 3]) "Pitch of the trunk about the world y axis, positive nose down";
  //Components
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape floor(
    shapeType = "box",
    r_shape = {-0.5, 0, -0.005},
    length = 4, width = 1, height = 0.01,
    color = {190, 190, 190}) annotation(Placement(transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion trunkJoint(
    r_rel_a(start = {0, 0, 0.12}, each fixed = true),
    v_rel_a(each fixed = true),
    angles_fixed = true,
    w_rel_a_fixed = true,
    animation = false) annotation(Placement(transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Robot robot annotation(Placement(transformation(origin = {0, 10}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity gyroSensor(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(Placement(transformation(origin = {40, -6}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Sources.RealExpression[3] gravityDirection(y = {-robot.frame_imu.R.T[1, 3], -robot.frame_imu.R.T[2, 3], -robot.frame_imu.R.T[3, 3]}) "Gravity direction in the IMU frame" annotation(Placement(transformation(origin = {40, -18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[14] jointAngles(y = robot.q) "Joint angles of the robot" annotation(Placement(transformation(origin = {40, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[14] jointSpeeds(y = robot.w) "Joint angular velocities of the robot" annotation(Placement(transformation(origin = {40, -42}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[3] command(y = {forwardSpeed, 0, 0}) "Velocity command" annotation(Placement(transformation(origin = {40, -54}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.WalkingPolicy policy(q0 = robot.q0) annotation(Placement(transformation(origin = {100, -30}, extent = {{-30, -30}, {30, 30}})));
equation
  connect(world.frame_b, trunkJoint.frame_a) annotation(Line(points = {{-80, 40}, {-70, 40}, {-70, 10}, {-60, 10}}, color = {95, 95, 95}, thickness = 0.5));
  connect(world.frame_b, floor.frame_a) annotation(Line(points = {{-80, 40}, {-60, 40}}, color = {95, 95, 95}, thickness = 0.5));
  connect(trunkJoint.frame_b, robot.frame_a) annotation(Line(points = {{-10, 0}, {10, -0}}, color = {95, 95, 95}, thickness = 0.5, origin = {-30, 10}));
  connect(robot.frame_imu, gyroSensor.frame_a) annotation(Line(points = {{20, -6}, {23, -6}, {23, -6}, {34, -6}}, color = {95, 95, 95}, thickness = 0.5));
  connect(gyroSensor.w, policy.gyro) annotation(Line(points = {{46.6, -6}, {67, -6}}, color = {0, 0, 127}));
  connect(gravityDirection.y, policy.gravity) annotation(Line(points = {{-8, 0}, {8, 0}}, color = {0, 0, 127}, origin = {59, -18}));
  connect(jointAngles.y, policy.q) annotation(Line(points = {{-8, 0}, {8, 0}}, color = {0, 0, 127}, origin = {59, -30}));
  connect(jointSpeeds.y, policy.w) annotation(Line(points = {{-8, 0}, {8, 0}}, color = {0, 0, 127}, origin = {59, -42}));
  connect(command.y, policy.twist) annotation(Line(points = {{-8, 0}, {8, 0}}, color = {0, 0, 127}, origin = {59, -54}));
  connect(policy.qRef, robot.qRef) annotation(Line(points = {{133, -40}, {140, -40}, {140, 30}, {0, 30}, {0, 20}}, color = {0, 0, 127}, origin = {0, 10}));

  when terminal() and checkResults then
    assert(trunkHeight > minHeight, "The robot has fallen");
    assert(distance > minDistance, "The robot has not walked far enough");
  end when;

  annotation(
    Diagram(coordinateSystem(extent = {{-110, -70}, {150, 60}})),
    experiment(StartTime = 0, StopTime = 6, Tolerance = 1e-6, Interval = 0.001),
    __OpenModelica_commandLineOptions = "--maxSizeLinearTearing=1000",
    __OpenModelica_simulationFlags(s = "rungekutta", variableFilter = "^(time|distance|trunkHeight|trunkPitch|robot[.](q|w|qRef|motorCurrent|copperLoss|frictionLoss|windingTemperature|housingTemperature)[[].*|robot[.](terminalVoltage|batteryCurrent|soc)|policy[.](action|target)[[].*)$"),
    Documentation(
      info = "<html>
<h4>Walking under the neural network policy</h4>
<p><a href=\"modelica://MicroDuck.Components.WalkingPolicy\">WalkingPolicy</a> reads the IMU (inertial measurement unit) and the joint states of <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> at 50 Hz and drives <code>qRef</code> with a forward command <code>forwardSpeed</code> of 0.3 m/s. The robot starts at the home pose, 2.8 mm above the ground, at the walking servo gain of 200.</p>
<p>Figure 1 compares the MicroDuck library with MuJoCo running the same policy, servo, gear play, command delay and speed lag. Over the last 4 s both step at 2.0 Hz at a mean trunk height of 118 mm and walk at 0.132 m/s against 0.131 m/s, covering 0.77 m against 0.79 m in 6 s. Speed follows stance-sole slip governed by <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a>.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Examples/WalkingTest/walking.png\"
     width=\"600\"
     alt=\"Forward position and trunk height of the walking robot in the MicroDuck library and in MuJoCo\">
<strong>Figure 1:</strong> Forward position and trunk height compared with MuJoCo.</p>
<p>Set <code>policy.commandDelay = 0</code>, <code>policy.delayedSpeed = false</code> and <code>robot.backlash = 0</code> to compare with the plant without delay and gear play, or set <code>robot.contactAcceleration = true</code> with an implicit solver such as CVODE to include the inertial touchdown term.</p>
</html>",
      figures = {Figure(title = "Walking", identifier = "walking", preferred = true, plots = {Plot(title = "Forward position of the trunk", curves = {Curve(y = "distance", legend = "Distance")}, y = Axis(unit = "m")), Plot(title = "Height of the trunk frame", curves = {Curve(y = "trunkHeight", legend = "Trunk height")}, y = Axis(unit = "m"))})}));
end WalkingTest;
