within MicroDuck.Components;
block WalkingPolicy "Walking policy of the Microduck, evaluated at 50 Hz"
  //Parameters
  parameter String weightFile = Modelica.Utilities.Files.loadResource("modelica://MicroDuck/Resources/Data/velstand.mat") "MAT file with the network weights";
  parameter Modelica.Units.SI.Time samplePeriod = 0.02 "Control period";
  parameter Modelica.Units.SI.Time commandDelay = 0.0225 "Delay from a control step to the servos, below two control periods";
  parameter Boolean delayedSpeed = true "Observe the joint speeds one control period late";
  parameter Modelica.Units.SI.Angle[14] q0 "Home pose of the joints";
  parameter Real actionScale(unit = "1") = 0.9 "Scale from the network output to joint angle offsets";
  parameter Real legsFilter(unit = "1") = 0.7 "Weight of the new target in the low-pass filter of the leg joints";
  parameter Real headFilter(unit = "1") = 0.5 "Weight of the new target in the low-pass filter of the head joints";
  final parameter Real[14] filter(each unit = "1") = {if i >= 6 and i <= 9 then headFilter else legsFilter for i in 1:14} "Filter weight of each joint";
  //Variables
  Real[61] observation(each start = 0, each fixed = true) "Observation vector of the network";
  Real[14] action(each start = 0, each fixed = true) "Latest network output";
  Modelica.Units.SI.Angle[14] target(start = q0, each fixed = true) "Latest joint angle targets";
  Modelica.Units.SI.Angle[14] previousTarget(start = q0, each fixed = true) "Joint angle targets of the control step before";
  Modelica.Units.SI.Angle[14] command(start = q0, each fixed = true) "Joint angle targets as they reach the servos";
  Modelica.Units.SI.AngularVelocity[14] lastSpeed(each start = 0, each fixed = true) "Joint speeds read at the latest control step";
  Boolean started(start = false, fixed = true) "True after the first control step";
  //Components
  Modelica.Blocks.Interfaces.RealInput[3] gyro(each unit = "rad/s") "Angular velocity of the trunk in the trunk frame" annotation(Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput[3] gravity(each unit = "1") "Gravity direction in the trunk frame" annotation(Placement(transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput[14] q(each unit = "rad") "Joint angles" annotation(Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput[14] w(each unit = "rad/s") "Joint angular velocities" annotation(Placement(transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput[3] twist "Velocity command: forward and sideways speed in m/s, yaw rate in rad/s" annotation(Placement(transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput[14] qRef(each unit = "rad") "Joint angle targets" annotation(Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
equation
  assert(commandDelay >= 0 and commandDelay < 2 * samplePeriod, "commandDelay must lie in [0, 2 samplePeriod)");

  // Control step
  when sample(0, samplePeriod) then
    observation = cat(1, gyro, gravity, q - q0, if delayedSpeed then pre(lastSpeed) else w, pre(action), twist, zeros(10));
    action = MicroDuck.Utilities.policyNetwork(weightFile, observation);
    target = if pre(started) then filter .* (q0 + actionScale * action) + (1 .- filter) .* pre(target) else q0 + actionScale * action;
    previousTarget = pre(target);
    lastSpeed = w;
    started = true;
  end when;

  // The targets of a step reach the servos commandDelay later; past one period they are previousTarget by then
  when sample(commandDelay, samplePeriod) then
    command = if commandDelay < samplePeriod then target else previousTarget;
  end when;
  qRef = command;

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{-50, 45}, {0, 60}}, color = {160, 160, 160}),
      Line(points = {{-50, 45}, {0, 30}}, color = {160, 160, 160}),
      Line(points = {{-50, 45}, {0, 0}}, color = {160, 160, 160}),
      Line(points = {{-50, 45}, {0, -30}}, color = {160, 160, 160}),
      Line(points = {{-50, 45}, {0, -60}}, color = {160, 160, 160}),
      Line(points = {{-50, 15}, {0, 60}}, color = {160, 160, 160}),
      Line(points = {{-50, 15}, {0, 30}}, color = {160, 160, 160}),
      Line(points = {{-50, 15}, {0, 0}}, color = {160, 160, 160}),
      Line(points = {{-50, 15}, {0, -30}}, color = {160, 160, 160}),
      Line(points = {{-50, 15}, {0, -60}}, color = {160, 160, 160}),
      Line(points = {{-50, -15}, {0, 60}}, color = {160, 160, 160}),
      Line(points = {{-50, -15}, {0, 30}}, color = {160, 160, 160}),
      Line(points = {{-50, -15}, {0, 0}}, color = {160, 160, 160}),
      Line(points = {{-50, -15}, {0, -30}}, color = {160, 160, 160}),
      Line(points = {{-50, -15}, {0, -60}}, color = {160, 160, 160}),
      Line(points = {{-50, -45}, {0, 60}}, color = {160, 160, 160}),
      Line(points = {{-50, -45}, {0, 30}}, color = {160, 160, 160}),
      Line(points = {{-50, -45}, {0, 0}}, color = {160, 160, 160}),
      Line(points = {{-50, -45}, {0, -30}}, color = {160, 160, 160}),
      Line(points = {{-50, -45}, {0, -60}}, color = {160, 160, 160}),
      Line(points = {{0, 60}, {50, 45}}, color = {160, 160, 160}),
      Line(points = {{0, 60}, {50, 15}}, color = {160, 160, 160}),
      Line(points = {{0, 60}, {50, -15}}, color = {160, 160, 160}),
      Line(points = {{0, 60}, {50, -45}}, color = {160, 160, 160}),
      Line(points = {{0, 30}, {50, 45}}, color = {160, 160, 160}),
      Line(points = {{0, 30}, {50, 15}}, color = {160, 160, 160}),
      Line(points = {{0, 30}, {50, -15}}, color = {160, 160, 160}),
      Line(points = {{0, 30}, {50, -45}}, color = {160, 160, 160}),
      Line(points = {{0, 0}, {50, 45}}, color = {160, 160, 160}),
      Line(points = {{0, 0}, {50, 15}}, color = {160, 160, 160}),
      Line(points = {{0, 0}, {50, -15}}, color = {160, 160, 160}),
      Line(points = {{0, 0}, {50, -45}}, color = {160, 160, 160}),
      Line(points = {{0, -30}, {50, 45}}, color = {160, 160, 160}),
      Line(points = {{0, -30}, {50, 15}}, color = {160, 160, 160}),
      Line(points = {{0, -30}, {50, -15}}, color = {160, 160, 160}),
      Line(points = {{0, -30}, {50, -45}}, color = {160, 160, 160}),
      Line(points = {{0, -60}, {50, 45}}, color = {160, 160, 160}),
      Line(points = {{0, -60}, {50, 15}}, color = {160, 160, 160}),
      Line(points = {{0, -60}, {50, -15}}, color = {160, 160, 160}),
      Line(points = {{0, -60}, {50, -45}}, color = {160, 160, 160}),
      Ellipse(extent = {{-59, 36}, {-41, 54}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-59, 6}, {-41, 24}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-59, -24}, {-41, -6}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-59, -54}, {-41, -36}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-9, 51}, {9, 69}}, lineColor = {64, 64, 64}, fillColor = {0, 128, 180}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-9, 21}, {9, 39}}, lineColor = {64, 64, 64}, fillColor = {0, 128, 180}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-9, -9}, {9, 9}}, lineColor = {64, 64, 64}, fillColor = {0, 128, 180}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-9, -39}, {9, -21}}, lineColor = {64, 64, 64}, fillColor = {0, 128, 180}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-9, -69}, {9, -51}}, lineColor = {64, 64, 64}, fillColor = {0, 128, 180}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{41, 36}, {59, 54}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{41, 6}, {59, 24}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{41, -24}, {59, -6}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{41, -54}, {59, -36}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Text(extent = {{-90, -70}, {90, -96}}, textString = "50 Hz", textColor = {64, 64, 64}),
      Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Walking policy</h4>
<p>Runs the neural network of the Microduck walking policy every <code>samplePeriod</code> and turns its output into filtered joint angle targets.</p>
<p>Figure 1 shows the signal flow. The observation has 61 values: <code>gyro</code>, <code>gravity</code>, <code>q</code> minus the home pose <code>q0</code>, <code>w</code>, the previous action and the command, in the order the policy was trained with. The command block holds <code>twist</code>
followed by ten zeros for the head, body pose and unused slots. The targets are <code>q0</code> + <code>actionScale</code> &middot; action, then a first-order low-pass with the weights <code>legsFilter</code> and <code>headFilter</code>, and the first step is not filtered.
As in training, the targets reach <code>qRef</code> <code>commandDelay</code> after the step, 22.5 ms by default, and with <code>delayedSpeed</code> the joint speeds enter the observation one control period late, because the servo firmware averages its speed over the previous samples.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Components/WalkingPolicy/policy.svg\"
     width=\"600\"
     alt=\"Observation vector with the joint speeds of the step before, network, scaling, low-pass filter and command delay to the joint targets, with the action fed back to the next step\">
<strong>Figure 1:</strong> Signal flow of the walking policy.</p>
<p>The parameters that decide the behaviour are:</p>
<p>
<code>samplePeriod</code>, <code>commandDelay</code> = Control period (20 ms) and delay to the servos (22.5 ms, below two periods)<br>
<code>delayedSpeed</code> = Observe <code>w</code> one period late<br>
<code>q0</code>, <code>actionScale</code> = Home pose and scale from network output to joint angle offsets<br>
<code>legsFilter</code>, <code>headFilter</code> = Low-pass filter weights of the leg and head targets<br>
<code>weightFile</code> = MATLAB v4 file of network weights, read by <a href=\"modelica://MicroDuck.Utilities.policyNetwork\">policyNetwork</a>
</p>
<p>The inputs belong to the robot: <code>gyro</code> and <code>gravity</code> come from the IMU (inertial measurement unit, the gyroscope and accelerometer in the trunk) frame, and <code>q</code>, <code>w</code> and <code>qRef</code> are the joint vectors of <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a>. In MuJoCo with this servo the policy stood at a forward command of 0.2 m/s and walked at 0.3 m/s and above.</p>
<h4>Limitations</h4>
<p>Training draws the command delay from 15 to 30 ms anew at every 5 ms physics step, delays the IMU signals by 0 or 20 ms, and adds sensor noise and an encoder offset. Here the delay is the fixed mean <code>commandDelay</code>, and the IMU delay, noise and offset are left out. The policy needs the servo gain of 200 and a plant started at the home pose.</p>
</html>"));
end WalkingPolicy;
