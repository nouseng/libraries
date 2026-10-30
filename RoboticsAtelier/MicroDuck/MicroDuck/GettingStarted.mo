within MicroDuck;
class GettingStarted "Getting started with the MicroDuck library"
  extends Modelica.Icons.Information;

  annotation(Documentation(info = "<html>
<h4>Getting started</h4>
<p>This guide sets up the robot standing on the ground; open <a href=\"modelica://MicroDuck.Examples.Standing\">Standing</a> to follow it.</p>
<h4>World and robot</h4>
<p>The robot has its z axis up, so gravity points along negative z and the ground is the plane z = 0. Connect the trunk to the world through a free joint that starts 120 mm up at rest, and command the home pose:</p>
<pre>
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1});
  Modelica.Mechanics.MultiBody.Joints.FreeMotion trunkJoint(
    r_rel_a(start = {0, 0, 0.12}, each fixed = true),
    v_rel_a(each fixed = true),
    angles_fixed = true,
    w_rel_a_fixed = true);
  MicroDuck.Components.Robot robot(positionGain = 800, contactAcceleration = true);
  Modelica.Blocks.Sources.RealExpression[14] homePose(y = robot.q0);
equation
  connect(world.frame_b, trunkJoint.frame_a);
  connect(trunkJoint.frame_b, robot.frame_a);
  connect(homePose.y, robot.qRef);
</pre>
<h4>Settings</h4>
<p><code>positionGain</code> is the servo firmware gain: the robot runs at 200, but without a policy it tips forward, so this model uses 800. <code>supplyVoltage</code> defaults to 7.4 V. Each servo turns its link through <code>backlash</code> = 2&deg; of gear play and reads the link angle. The soles use the MuJoCo contact law of <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a>.</p>
<h4>Simulating</h4>
<p>Simulate for 3 s with CVODE and a tolerance of 1e-6, as stored in the example. Useful results are <code>leftLoad</code>, <code>rightLoad</code>, <code>trunkHeight</code>, <code>trunkPitch</code> and <code>robot.q</code>. The robot settles with each sole carrying about half of the 7.2 N weight. In OMEdit use the animation button for the 3D view; <code>robot.animation = false</code> hides the meshes.</p>
<h4>Targets and sensors</h4>
<p>Replace <code>homePose</code> by any source of 14 angles in the order of <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a>. Sensor frames such as <code>robot.frame_imu</code> are ordinary MultiBody frames. <code>robot.useMouth = true</code> adds the mouth servo, held at <code>robot.mouthHome</code> or driven from <code>robot.mouthRef</code> with <code>robot.useMouthInput = true</code>.</p>
</html>"));
end GettingStarted;
