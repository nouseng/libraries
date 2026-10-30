within MicroDuck.Examples;

model NousLogo "Two Microducks walking the circular arc and diagonal chord of the Nous logo"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Modelica.Units.SI.Radius arcRadius = 0.45 "Radius of the circular arc of the logo";
  parameter Modelica.Units.SI.Angle arcStartAngle = 3.29169096921163 "Initial polar angle of the arc robot, 188.6 deg";
  parameter Modelica.Units.SI.Velocity arcSpeed = 0.3 "Commanded forward speed of the arc robot";
  parameter Modelica.Units.SI.AngularVelocity arcYawRate = -0.82 "Nominal clockwise yaw rate of the arc robot";
  parameter Real radialGain(unit = "rad/(m.s)") = 6 "Gain from radial error to yaw rate of the arc robot";
  parameter Real headingGain(unit = "1/s") = 2.5 "Gain from heading error to yaw rate";
  parameter Modelica.Units.SI.Position lineStart[2] = {-0.411, -0.333} "Initial floor coordinates of the diagonal-chord robot";
  parameter Modelica.Units.SI.Angle lineAngle = 0.551524043630208 "Heading of the diagonal chord, 31.6 deg";
  parameter Modelica.Units.SI.Velocity lineSpeed = 0.30 "Commanded forward speed of the diagonal-chord robot";
  parameter Real crossTrackGain(unit = "rad/(m.s)") = 4 "Gain from cross-track error to yaw rate of the diagonal-chord robot";
  parameter Boolean checkResults = true "Assert that both robots stay upright and trace the logo";
  parameter Modelica.Units.SI.Length minHeight = 0.1 "Lowest allowed trunk height";
  parameter Modelica.Units.SI.Length maxTrackError = 0.08 "Largest allowed radial or cross-track error";
  parameter Modelica.Units.SI.Distance minSeparation = 0.14 "Minimum allowed trunk-to-trunk separation between the two robots";
  //Variables
  Modelica.Units.SI.Position arcX = arcRobot.frame_a.r_0[1] "World x position of the arc robot";
  Modelica.Units.SI.Position arcY = arcRobot.frame_a.r_0[2] "World y position of the arc robot";
  Modelica.Units.SI.Position arcHeight = arcRobot.frame_a.r_0[3] "Trunk height of the arc robot";
  Modelica.Units.SI.Angle arcYaw = atan2(arcRobot.frame_a.R.T[1, 2], arcRobot.frame_a.R.T[1, 1]) "Yaw angle of the arc robot";
  Modelica.Units.SI.Angle arcPolarAngle = atan2(arcY, arcX) "Polar angle of the arc robot around the logo center";
  Modelica.Units.SI.Angle arcTangentYaw = arcPolarAngle - Modelica.Constants.pi/2 "Clockwise tangent heading at the arc robot position";
  Modelica.Units.SI.Angle arcHeadingError = atan2(sin(arcTangentYaw - arcYaw), cos(arcTangentYaw - arcYaw)) "Heading error of the arc robot";
  Modelica.Units.SI.Length radialError = sqrt(arcX*arcX + arcY*arcY) - arcRadius "Radial tracking error of the arc robot";
  Modelica.Units.SI.AngularVelocity arcYawCommand = max(-1.2, min(1.2, arcYawRate + headingGain*arcHeadingError - radialGain*radialError)) "Commanded yaw rate of the arc robot";
  Modelica.Units.SI.Position lineX = lineRobot.frame_a.r_0[1] "World x position of the diagonal-chord robot";
  Modelica.Units.SI.Position lineY = lineRobot.frame_a.r_0[2] "World y position of the diagonal-chord robot";
  Modelica.Units.SI.Position lineHeight = lineRobot.frame_a.r_0[3] "Trunk height of the diagonal-chord robot";
  Modelica.Units.SI.Angle lineYaw = atan2(lineRobot.frame_a.R.T[1, 2], lineRobot.frame_a.R.T[1, 1]) "Yaw angle of the diagonal-chord robot";
  Modelica.Units.SI.Length lineDistance = (lineX - lineStart[1])*cos(lineAngle) + (lineY - lineStart[2])*sin(lineAngle) "Distance walked along the diagonal chord";
  Modelica.Units.SI.Length crossTrackError = -(lineX - lineStart[1])*sin(lineAngle) + (lineY - lineStart[2])*cos(lineAngle) "Signed lateral deviation from the diagonal chord";
  Modelica.Units.SI.Angle lineHeadingError = atan2(sin(lineAngle - lineYaw), cos(lineAngle - lineYaw)) "Heading error of the diagonal-chord robot";
  Modelica.Units.SI.AngularVelocity lineYawCommand = max(-1.0, min(1.0, headingGain*lineHeadingError - crossTrackGain*crossTrackError)) "Commanded yaw rate of the diagonal-chord robot";
  Modelica.Units.SI.Distance robotSeparation = sqrt((arcX - lineX)*(arcX - lineX) + (arcY - lineY)*(arcY - lineY)) "Horizontal distance between the two robot trunks";
  Boolean arcWaiting = time >= 6.8 and time < 9.0 "True while the arc robot waits before the chord for the chord robot to pass";
  Boolean lineFinished = lineDistance >= 1.36 "True once the diagonal-chord robot reaches the top-right ring";
  Modelica.Units.SI.Force arcLeftLoad = sum(arcRobot.contactLeft.normalForce) "Normal force on the left sole of the arc robot";
  Modelica.Units.SI.Force arcRightLoad = sum(arcRobot.contactRight.normalForce) "Normal force on the right sole of the arc robot";
  Modelica.Units.SI.Force lineLeftLoad = sum(lineRobot.contactLeft.normalForce) "Normal force on the left sole of the diagonal-chord robot";
  Modelica.Units.SI.Force lineRightLoad = sum(lineRobot.contactRight.normalForce) "Normal force on the right sole of the diagonal-chord robot";
  Modelica.Units.SI.Position arcLeftFootX[65](each start = arcRadius*cos(arcStartAngle), each fixed = true) "Sampled x positions of the arc robot left foot";
  Modelica.Units.SI.Position arcLeftFootY[65](each start = arcRadius*sin(arcStartAngle), each fixed = true) "Sampled y positions of the arc robot left foot";
  Modelica.Units.SI.Position arcRightFootX[65](each start = arcRadius*cos(arcStartAngle), each fixed = true) "Sampled x positions of the arc robot right foot";
  Modelica.Units.SI.Position arcRightFootY[65](each start = arcRadius*sin(arcStartAngle), each fixed = true) "Sampled y positions of the arc robot right foot";
  Modelica.Units.SI.Position lineLeftFootX[65](each start = lineStart[1], each fixed = true) "Sampled x positions of the diagonal-chord robot left foot";
  Modelica.Units.SI.Position lineLeftFootY[65](each start = lineStart[2], each fixed = true) "Sampled y positions of the diagonal-chord robot left foot";
  Modelica.Units.SI.Position lineRightFootX[65](each start = lineStart[1], each fixed = true) "Sampled x positions of the diagonal-chord robot right foot";
  Modelica.Units.SI.Position lineRightFootY[65](each start = lineStart[2], each fixed = true) "Sampled y positions of the diagonal-chord robot right foot";
  //Components
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape floor(shapeType = "box", r_shape = {-0.75, 0.05, -0.005}, length = 1.8, width = 1.4, height = 0.01, color = {235, 238, 242}) annotation(
    Placement(transformation(origin = {-100, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion arcTrunkJoint(r_rel_a(start = {arcRadius*cos(arcStartAngle), arcRadius*sin(arcStartAngle), 0.12}, each fixed = true), v_rel_a(each fixed = true), angles_fixed = true, angles_start = {0, 0, arcStartAngle - Modelica.Constants.pi/2}, w_rel_a_fixed = true, animation = false) annotation(
    Placement(transformation(origin = {-50, 51}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Robot arcRobot(headRollVisual11(color = {38, 148, 82}), headRollVisual12(color = {20, 98, 52}), headRollVisual13(color = {20, 98, 52}), mouthVisual2(color = {20, 98, 52}), leftAnkleVisual1(color = {38, 148, 82}), leftAnkleVisual2(color = {20, 98, 52}), leftAnkleVisual4(color = {20, 98, 52}), rightAnkleVisual1(color = {20, 98, 52}), rightAnkleVisual2(color = {38, 148, 82}), rightAnkleVisual4(color = {20, 98, 52})) annotation(
    Placement(transformation(origin = {0, 51}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity arcGyro(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {40, 35}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Sources.RealExpression[3] arcGravity(y = {-arcRobot.frame_imu.R.T[1, 3], -arcRobot.frame_imu.R.T[2, 3], -arcRobot.frame_imu.R.T[3, 3]}) "Gravity direction in the arc robot IMU frame" annotation(
    Placement(transformation(origin = {40, 23}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[14] arcAngles(y = arcRobot.q) "Joint angles of the arc robot" annotation(
    Placement(transformation(origin = {40, 11}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[14] arcSpeeds(y = arcRobot.w) "Joint angular velocities of the arc robot" annotation(
    Placement(transformation(origin = {40, -1}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[3] arcCommand(y = if arcWaiting then {0, 0, 0} else {arcSpeed, 0, 1*arcYawCommand}) "Velocity command of the arc robot" annotation(
    Placement(transformation(origin = {40, -13}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.WalkingPolicy arcPolicy(q0 = arcRobot.q0) annotation(
    Placement(transformation(origin = {100, 11}, extent = {{-25, -25}, {25, 25}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion lineTrunkJoint(r_rel_a(start = {lineStart[1], lineStart[2], 0.12}, each fixed = true), v_rel_a(each fixed = true), angles_fixed = true, angles_start = {0, 0, lineAngle}, w_rel_a_fixed = true, animation = false) annotation(
    Placement(transformation(origin = {-50, -55}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Robot lineRobot annotation(
    Placement(transformation(origin = {0, -55}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity lineGyro(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(
    Placement(transformation(origin = {40, -71}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Sources.RealExpression[3] lineGravity(y = {-lineRobot.frame_imu.R.T[1, 3], -lineRobot.frame_imu.R.T[2, 3], -lineRobot.frame_imu.R.T[3, 3]}) "Gravity direction in the diagonal-chord robot IMU frame" annotation(
    Placement(transformation(origin = {40, -83}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[14] lineAngles(y = lineRobot.q) "Joint angles of the diagonal-chord robot" annotation(
    Placement(transformation(origin = {40, -95}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[14] lineSpeeds(y = lineRobot.w) "Joint angular velocities of the diagonal-chord robot" annotation(
    Placement(transformation(origin = {40, -107}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression[3] lineCommand(y = if lineFinished then {0, 0, 0} else {lineSpeed, 0, 1*lineYawCommand}) "Velocity command of the diagonal-chord robot" annotation(
    Placement(transformation(origin = {40, -119}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.WalkingPolicy linePolicy(q0 = lineRobot.q0) annotation(
    Placement(transformation(origin = {100, -95}, extent = {{-25, -25}, {25, 25}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape[55] arcGuide(each shapeType = "cylinder", r = {arcRadius*{cos(arcStartAngle - (i - 1)*0.0723), sin(arcStartAngle - (i - 1)*0.0723), 0} for i in 1:55}, each r_shape = {0, 0, -0.001}, each lengthDirection = {0, 0, 1}, each widthDirection = {1, 0, 0}, each length = 0.0025, width = {if i >= 39 and i <= 42 then 0 else 0.018 for i in 1:55}, height = {if i >= 39 and i <= 42 then 0 else 0.018 for i in 1:55}, color = {if i <= 38 then {15, 31, 44} else {160, 172, 184} for i in 1:55});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape chordBar(shapeType = "box", r = {lineStart[1], lineStart[2], 0.0005}, lengthDirection = {cos(lineAngle), sin(lineAngle), 0}, widthDirection = {-sin(lineAngle), cos(lineAngle), 0}, length = 1.372, width = 0.018, height = 0.002, color = {15, 31, 44});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape startDisk(shapeType = "cylinder", r = {lineStart[1], lineStart[2], -0.0005}, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, length = 0.003, width = 0.084, height = 0.084, color = {15, 31, 44});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape endRing(shapeType = "pipe", r = {lineStart[1] + 1.372*cos(lineAngle), lineStart[2] + 1.372*sin(lineAngle), -0.0005}, lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0}, length = 0.003, width = 0.084, height = 0.084, extra = 0.72, color = {15, 31, 44});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape[65] arcLeftTrail(each shapeType = "cylinder", r = {{arcLeftFootX[i], arcLeftFootY[i], -0.0005} for i in 1:65}, each lengthDirection = {0, 0, 1}, each widthDirection = {1, 0, 0}, each length = 0.004, width = {if time >= i*0.2 and (i < 35 or i > 45) and abs(-(arcLeftFootX[i] - lineStart[1])*sin(lineAngle) + (arcLeftFootY[i] - lineStart[2])*cos(lineAngle)) > 0.055 then 0.036 else 0 for i in 1:65}, height = {if time >= i*0.2 and (i < 35 or i > 45) and abs(-(arcLeftFootX[i] - lineStart[1])*sin(lineAngle) + (arcLeftFootY[i] - lineStart[2])*cos(lineAngle)) > 0.055 then 0.036 else 0 for i in 1:65}, color = {if -(arcLeftFootX[i] - lineStart[1])*sin(lineAngle) + (arcLeftFootY[i] - lineStart[2])*cos(lineAngle) >= 0 then {24, 112, 62} else {110, 168, 132} for i in 1:65});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape[65] arcRightTrail(each shapeType = "cylinder", r = {{arcRightFootX[i], arcRightFootY[i], -0.0005} for i in 1:65}, each lengthDirection = {0, 0, 1}, each widthDirection = {1, 0, 0}, each length = 0.004, width = {if time >= i*0.2 and (i < 35 or i > 45) and abs(-(arcRightFootX[i] - lineStart[1])*sin(lineAngle) + (arcRightFootY[i] - lineStart[2])*cos(lineAngle)) > 0.055 then 0.036 else 0 for i in 1:65}, height = {if time >= i*0.2 and (i < 35 or i > 45) and abs(-(arcRightFootX[i] - lineStart[1])*sin(lineAngle) + (arcRightFootY[i] - lineStart[2])*cos(lineAngle)) > 0.055 then 0.036 else 0 for i in 1:65}, color = {if -(arcRightFootX[i] - lineStart[1])*sin(lineAngle) + (arcRightFootY[i] - lineStart[2])*cos(lineAngle) >= 0 then {24, 112, 62} else {110, 168, 132} for i in 1:65});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape[65] lineLeftTrail(each shapeType = "cylinder", r = {{lineLeftFootX[i], lineLeftFootY[i], -0.0005} for i in 1:65}, each lengthDirection = {0, 0, 1}, each widthDirection = {1, 0, 0}, each length = 0.004, width = {if time >= i*0.2 and i <= 50 then 0.036 else 0 for i in 1:65}, height = {if time >= i*0.2 and i <= 50 then 0.036 else 0 for i in 1:65}, each color = {217, 95, 2});
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape[65] lineRightTrail(each shapeType = "cylinder", r = {{lineRightFootX[i], lineRightFootY[i], -0.0005} for i in 1:65}, each lengthDirection = {0, 0, 1}, each widthDirection = {1, 0, 0}, each length = 0.004, width = {if time >= i*0.2 and i <= 50 then 0.036 else 0 for i in 1:65}, height = {if time >= i*0.2 and i <= 50 then 0.036 else 0 for i in 1:65}, each color = {217, 95, 2});
equation
// Arc robot connections
  connect(world.frame_b, floor.frame_a) annotation(
    Line(points = {{-90, 10}, {-80, 10}, {-80, -30}, {-110, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(world.frame_b, arcTrunkJoint.frame_a) annotation(
    Line(points = {{-90, 10}, {-75, 10}, {-75, 51}, {-60, 51}}, color = {95, 95, 95}, thickness = 0.5));
  connect(arcTrunkJoint.frame_b, arcRobot.frame_a) annotation(
    Line(points = {{-40, 51}, {-20, 51}}, color = {95, 95, 95}, thickness = 0.5));
  connect(arcRobot.frame_imu, arcGyro.frame_a) annotation(
    Line(points = {{20, 35}, {34, 35}}, color = {95, 95, 95}, thickness = 0.5));
  connect(arcGyro.w, arcPolicy.gyro) annotation(
    Line(points = {{46.6, 35}, {60, 35}, {60, 31}, {72.5, 31}}, color = {0, 0, 127}, thickness = 1));
  connect(arcGravity.y, arcPolicy.gravity) annotation(
    Line(points = {{51, 23}, {62, 23}, {62, 21}, {72.5, 21}}, color = {0, 0, 127}, thickness = 1));
  connect(arcAngles.y, arcPolicy.q) annotation(
    Line(points = {{51, 11}, {72.5, 11}}, color = {0, 0, 127}, thickness = 1));
  connect(arcSpeeds.y, arcPolicy.w) annotation(
    Line(points = {{51, -1}, {62, -1}, {62, 1}, {72.5, 1}}, color = {0, 0, 127}, thickness = 1));
  connect(arcCommand.y, arcPolicy.twist) annotation(
    Line(points = {{51, -13}, {60, -13}, {60, -9}, {72.5, -9}}, color = {0, 0, 127}, thickness = 1));
  connect(arcPolicy.qRef, arcRobot.qRef) annotation(
    Line(points = {{127.5, 11}, {140, 11}, {140, 81}, {0, 81}, {0, 71}}, color = {0, 0, 127}, thickness = 1));
// Diagonal-chord robot connections
  connect(world.frame_b, lineTrunkJoint.frame_a) annotation(
    Line(points = {{-90, 10}, {-75, 10}, {-75, -55}, {-60, -55}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lineTrunkJoint.frame_b, lineRobot.frame_a) annotation(
    Line(points = {{-40, -55}, {-20, -55}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lineRobot.frame_imu, lineGyro.frame_a) annotation(
    Line(points = {{20, -71}, {34, -71}}, color = {95, 95, 95}, thickness = 0.5));
  connect(lineGyro.w, linePolicy.gyro) annotation(
    Line(points = {{46.6, -71}, {60, -71}, {60, -75}, {72.5, -75}}, color = {0, 0, 127}, thickness = 1));
  connect(lineGravity.y, linePolicy.gravity) annotation(
    Line(points = {{51, -83}, {62, -83}, {62, -85}, {72.5, -85}}, color = {0, 0, 127}, thickness = 1));
  connect(lineAngles.y, linePolicy.q) annotation(
    Line(points = {{51, -95}, {72.5, -95}}, color = {0, 0, 127}, thickness = 1));
  connect(lineSpeeds.y, linePolicy.w) annotation(
    Line(points = {{51, -107}, {62, -107}, {62, -105}, {72.5, -105}}, color = {0, 0, 127}, thickness = 1));
  connect(lineCommand.y, linePolicy.twist) annotation(
    Line(points = {{51, -119}, {60, -119}, {60, -115}, {72.5, -115}}, color = {0, 0, 127}, thickness = 1));
  connect(linePolicy.qRef, lineRobot.qRef) annotation(
    Line(points = {{127.5, -95}, {140, -95}, {140, -25}, {0, -25}, {0, -35}}, color = {0, 0, 127}, thickness = 1));
// Verification assertions
  for i in 1:65 loop
    when time >= i*0.2 then
      arcLeftFootX[i] = arcRobot.frame_leftFoot.r_0[1];
      arcLeftFootY[i] = arcRobot.frame_leftFoot.r_0[2];
      arcRightFootX[i] = arcRobot.frame_rightFoot.r_0[1];
      arcRightFootY[i] = arcRobot.frame_rightFoot.r_0[2];
      lineLeftFootX[i] = lineRobot.frame_leftFoot.r_0[1];
      lineLeftFootY[i] = lineRobot.frame_leftFoot.r_0[2];
      lineRightFootX[i] = lineRobot.frame_rightFoot.r_0[1];
      lineRightFootY[i] = lineRobot.frame_rightFoot.r_0[2];
    end when;
  end for;
  assert(not checkResults or robotSeparation > minSeparation, "The two robots have collided");
  when terminal() and checkResults then
    assert(arcHeight > minHeight, "The arc robot has fallen");
    assert(lineHeight > minHeight, "The diagonal-chord robot has fallen");
    assert(abs(radialError) < maxTrackError, "The arc robot has drifted off the circle");
    assert(abs(crossTrackError) < maxTrackError, "The diagonal-chord robot has drifted off the line");
  end when;
  annotation(
    Diagram(coordinateSystem(extent = {{-120, -125}, {150, 85}})),
    experiment(StartTime = 0, StopTime = 13, Tolerance = 1e-06, Interval = 0.001),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(s = "rungekutta"),
    Documentation(info = "<html>
<h4>Walking the Nous logo with two Microducks</h4>
<p>Two instances of <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> driven by <a href=\"modelica://MicroDuck.Components.WalkingPolicy\">WalkingPolicy</a> walk simultaneously on the same floor to trace the Nous logo from above: the dark-green-trimmed <code>arcRobot</code> walks clockwise along the circular arc of radius <code>arcRadius</code> = 0.45 m from 188.6&deg; down to &minus;35&deg;, while the orange-trimmed <code>lineRobot</code> walks straight along the 31.6&deg; diagonal chord from <code>lineStart</code> = (&minus;0.411 m, &minus;0.333 m) to (+0.758 m, +0.386 m), cutting across the circular arc at 22&deg;. At 6.8 s <code>arcRobot</code> pauses just before the diagonal chord so <code>lineRobot</code> can pass through the crossing first without collision, then resumes at 9.0 s to complete the lower-right arc.</p>
<p>Figure 1 shows the top-down trajectories of both trunks and the stance footprints left on the floor by <code>frame_leftFoot</code> and <code>frame_rightFoot</code>, together with the lateral tracking errors over the 13 s walk.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Examples/NousLogo/nousLogo.png\"
     width=\"600\"
     alt=\"Top-down floor trajectories and stance footprints of the two Microducks tracing the Nous logo\">
<strong>Figure 1:</strong> Top-down floor paths and stance footprints tracing the Nous logo.</p>
<p>Increase <code>arcRadius</code> or reduce <code>radialGain</code> to see how stance-foot slip widens the turn when the closed-loop yaw correction is relaxed.</p>
</html>", figures = {Figure(title = "Top-down floor paths", identifier = "nousLogo", preferred = true, plots = {Plot(title = "Floor trajectories of the two robots", x = Axis(label = "x", unit = "m"), y = Axis(label = "y", unit = "m"), curves = {Curve(x = "arcX", y = "arcY", legend = "Arc robot"), Curve(x = "lineX", y = "lineY", legend = "Diagonal-chord robot")})}), Figure(title = "Tracking errors", identifier = "trackingErrors", plots = {Plot(title = "Lateral tracking error", curves = {Curve(y = "radialError", legend = "Arc radial error"), Curve(y = "crossTrackError", legend = "Chord cross-track error")}, y = Axis(unit = "m"))})}));
end NousLogo;
