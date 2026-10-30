within MicroDuckTest;
model JointLimitTest "Servos driven past the joint limits settle at the limits"
  extends Modelica.Icons.Example;
  //Parameters
  parameter Modelica.Units.SI.Angle overshootTolerance = 0.01 "Allowed distance beyond a joint limit";
  parameter Modelica.Units.SI.AngularVelocity restTolerance = 1e-2 "Allowed joint speed at the end of the run";
  parameter Modelica.Units.SI.Angle[14] qTarget = {if i == 2 then 1.0 elseif i == 4 then 2.2 elseif i == 6 then 1.6 else robot.q0[i] for i in 1:14} "Joint angle targets, three of them beyond the limits";
  //Variables
  Modelica.Units.SI.Angle hipRollOvershoot = robot.leftHipRoll.phiServo - robot.leftHipRoll.phiMax "Left hip roll servo output beyond its upper limit";
  Modelica.Units.SI.Angle kneeOvershoot = robot.leftKnee.phiServo - robot.leftKnee.phiMax "Left knee servo output beyond its upper limit";
  Modelica.Units.SI.Angle neckOvershoot = robot.neckPitch.phiServo - robot.neckPitch.phiMax "Neck pitch servo output beyond its upper limit";
  Modelica.Units.SI.AngularVelocity jointSpeed = max({abs(robot.w[i]) for i in 1:14}) "Largest joint speed";
  //Components
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation hanger(r = {0, 0, 0.5}, animation = false) annotation(Placement(transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Robot robot annotation(Placement(transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Sources.RealExpression[14] targetPose(y = qTarget) "Joint angle targets, three beyond the limits" annotation(Placement(transformation(origin = {10, 40}, extent = {{-14, -5}, {14, 5}})));
equation
  connect(world.frame_b, hanger.frame_a) annotation(Line(points = {{-60, 0}, {-30, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(hanger.frame_b, robot.frame_a) annotation(Line(points = {{-10, 0}, {20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(targetPose.y, robot.qRef) annotation(Line(points = {{25.4, 40}, {40, 40}, {40, 20}}, color = {0, 0, 127}));

  when terminal() then
    assert(hipRollOvershoot > 0 and hipRollOvershoot < overshootTolerance, "Left hip roll is not held at its limit");
    assert(kneeOvershoot > 0 and kneeOvershoot < overshootTolerance, "Left knee is not held at its limit");
    assert(neckOvershoot > 0 and neckOvershoot < overshootTolerance, "Neck pitch is not held at its limit");
    assert(jointSpeed < restTolerance, "The joints have not come to rest");
  end when;

  annotation(
    Diagram(coordinateSystem(extent = {{-100, -50}, {80, 60}})),
    experiment(StartTime = 0, StopTime = 2, Tolerance = 1e-6, Interval = 0.002),
    Documentation(
      info = "<html>
<h4>Servos driven past the joint limits</h4>
<p><a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> hangs 0.5 m above the ground on <code>hanger</code> with the servos at the walking gain. <code>targetPose</code> commands the left hip roll, left knee and neck pitch beyond their upper limits via <code>qTarget</code>, while every other joint holds the home pose.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuckTest/Resources/Images/JointLimitTest/overshoot.png\"
     width=\"600\"
     alt=\"Servo output angle beyond the limit for three joints settling a few milliradians past the limit, compared with MuJoCo\">
<strong>Figure 1:</strong> Servo output angle beyond the limit.</p>
<p>Figure 1 shows how far each servo output ends beyond its limit, with MuJoCo running the same model and servo. The stops act on the servo output, as the joint ranges do in the training model, and the link can sit up to half the gear play further. The servo torque of about 0.35 N&middot;m is balanced by a stop torque, so each servo output settles a few
milliradians past the limit, as in MuJoCo. The assertions require the overshoot to be positive and below <code>overshootTolerance</code>.</p>
</html>",
      figures = {Figure(title = "Joint limit overshoot", identifier = "overshoot", preferred = true, plots = {Plot(title = "Servo output angle beyond the upper limit", curves = {Curve(y = "hipRollOvershoot", legend = "Left hip roll"), Curve(y = "kneeOvershoot", legend = "Left knee"), Curve(y = "neckOvershoot", legend = "Neck pitch")}, y = Axis(unit = "rad"))})}));
end JointLimitTest;
