within MicroDuckTest;
model ForwardKinematicsCheck "Site poses compared with MuJoCo"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Modelica.Units.SI.Angle[14] q = {0.0164943529102, 0.103779385533, -0.335371663959, 0.422842142149, -0.1773446784, -0.138759082498, 0.308852418393, -0.161646951469, 0.0129835970715, -0.149143177807, 0.0584052274521, 0.035949225778, -0.160474076576, 0.271837648733} "Joint angles of the reference sample";
  parameter Modelica.Units.SI.Length positionTolerance = 1e-6 "Allowed position error";
  parameter Real rotationTolerance(unit = "1") = 1e-6 "Allowed error of the rotation matrix entries";
  parameter Modelica.Units.SI.Position[3] rHeadCamera = {0.103297749789, -0.00880785466174, 0.102163895771} "Reference position of head_camera";
  parameter Real[3, 3] tHeadCamera(each unit = "1") = {{0.889731308495, -0.160943905233, -0.427171228025}, {0.139456718475, 0.986880368312, -0.0813570053092}, {0.434660812998, 0.0128139771016, 0.900503070308}} "Reference transformation of head_camera";
  parameter Modelica.Units.SI.Position[3] rImu = {-0.021, 6.64098e-05, -0.0146984} "Reference position of imu";
  parameter Real[3, 3] tImu(each unit = "1") = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} "Reference transformation of imu";
  parameter Modelica.Units.SI.Position[3] rLeftFoot = {0.0261753049753, 0.0579052370366, -0.0908499238257} "Reference position of left_foot";
  parameter Real[3, 3] tLeftFoot(each unit = "1") = {{0.591912937649, -0.093160785666, 0.800599864013}, {0.0164048652998, 0.99448445413, 0.103593199045}, {-0.805834942552, -0.0481844218389, 0.59017650483}} "Reference transformation of left_foot";
  parameter Modelica.Units.SI.Position[3] rRightFoot = {-0.00767900606463, -0.0477234049469, -0.104990261994} "Reference position of right_foot";
  parameter Real[3, 3] tRightFoot(each unit = "1") = {{0.886362964529, 0.106542861877, 0.450565548721}, {-0.148337514339, 0.987212585099, 0.0583720280826}, {-0.438584857184, -0.118574577401, 0.89083297685}} "Reference transformation of right_foot";
  parameter Modelica.Units.SI.Position[3] rMouthTip = {0.0970042871577, -0.00973431755793, 0.0790146723765} "Reference position of mouth_tip";
  parameter Real[3, 3] tMouthTip(each unit = "1") = {{0.885904754371, -0.161049588109, -0.435012409425}, {0.139456718475, 0.986880368312, -0.0813570053092}, {0.442407719028, 0.0114091546904, 0.896741457352}} "Reference transformation of mouth_tip";
  parameter Modelica.Units.SI.Position[3] rTof = {0.105566201233, 0.0133713037623, 0.0985323588169} "Reference position of tof";
  parameter Real[3, 3] tTof(each unit = "1") = {{0.889731308495, -0.160943905233, -0.427171228025}, {0.139456718475, 0.986880368312, -0.0813570053092}, {0.434660812998, 0.0128139771016, 0.900503070308}} "Reference transformation of tof";
  //Variables
  Modelica.Units.SI.Length positionError "Largest site position error";
  Real rotationError(unit = "1") "Largest site rotation matrix entry error";
  //Components
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Robot robot(q0 = q, ground(r = {0, 0, -1})) annotation(Placement(transformation(origin = {30, 0}, extent = {{-30, -30}, {30, 30}})));
  Modelica.Blocks.Sources.RealExpression[14] referencePose(y = q) "Joint angle targets of the reference sample" annotation(Placement(transformation(origin = {-10, 50}, extent = {{-14, -5}, {14, 5}})));
equation
  connect(world.frame_b, robot.frame_a) annotation(Line(points = {{-40, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(referencePose.y, robot.qRef) annotation(Line(points = {{5.4, 50}, {30, 50}, {30, 30}}, color = {0, 0, 127}));
  positionError = max({
    Modelica.Math.Vectors.norm(robot.frame_headCamera.r_0 - rHeadCamera),
    Modelica.Math.Vectors.norm(robot.frame_imu.r_0 - rImu),
    Modelica.Math.Vectors.norm(robot.frame_leftFoot.r_0 - rLeftFoot),
    Modelica.Math.Vectors.norm(robot.frame_rightFoot.r_0 - rRightFoot),
    Modelica.Math.Vectors.norm(robot.frame_mouthTip.r_0 - rMouthTip),
    Modelica.Math.Vectors.norm(robot.frame_tof.r_0 - rTof)
  });
  rotationError = max({
    max(abs(robot.frame_headCamera.R.T - tHeadCamera)),
    max(abs(robot.frame_imu.R.T - tImu)),
    max(abs(robot.frame_leftFoot.R.T - tLeftFoot)),
    max(abs(robot.frame_rightFoot.R.T - tRightFoot)),
    max(abs(robot.frame_mouthTip.R.T - tMouthTip)),
    max(abs(robot.frame_tof.R.T - tTof))
  });

  when terminal() then
    assert(positionError < positionTolerance, "Site position differs from MuJoCo");
    assert(rotationError < rotationTolerance, "Site orientation differs from MuJoCo");
  end when;

  annotation(
    Diagram(coordinateSystem(extent = {{-80, -50}, {80, 70}})),
    experiment(StartTime = 0, StopTime = 0.1, Tolerance = 1e-6, Interval = 0.001),
    Documentation(info = "<html>
<h4>Forward kinematics against MuJoCo</h4>
<p>Compares the six site frames of <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> with MuJoCo for random joint sample 0 of the same MJCF file (MuJoCo XML model), with the trunk fixed, gravity off and <code>referencePose</code> feeding <code>q</code> to <code>robot.qRef</code>.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuckTest/Resources/Images/ForwardKinematicsCheck/errors.png\"
     width=\"600\"
     alt=\"Position and rotation errors of the sites at numerical noise level\">
<strong>Figure 1:</strong> Largest site position and rotation errors.</p>
<p>Figure 1 shows the largest position and rotation error over the sites. Both stay at numerical noise, and the assertions fail beyond <code>positionTolerance</code> or <code>rotationTolerance</code>.
Regenerate the example with another sample index to check other poses.</p>
</html>",
      figures = {Figure(title = "Forward kinematics errors", identifier = "kinematicsErrors", preferred = true, plots = {Plot(title = "Largest site position error", curves = {Curve(y = "positionError", legend = "Position error")}, y = Axis(unit = "m")), Plot(title = "Largest site rotation error", curves = {Curve(y = "rotationError", legend = "Rotation matrix error")})})}));
end ForwardKinematicsCheck;
