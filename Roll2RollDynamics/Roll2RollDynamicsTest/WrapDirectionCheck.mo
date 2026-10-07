within Roll2RollDynamicsTest;
package WrapDirectionCheck "Rotating contact and wrap-direction regressions"
  extends Modelica.Icons.ExamplesPackage;

  model RotatingContactProbe
    "Material web-contact point on a rotating roll: multibody ground truth for the wrap sign"
    import Modelica.Units.SI;
    parameter SI.Radius radius = 0.15 "Roll radius";
    parameter SI.Angle entryAngle = -0.6 "Entry tangency angle from the vertical";
    parameter SI.Angle exitAngle = 0.6 "Exit tangency angle from the vertical";
    parameter SI.AngularVelocity wDrive = 2 "Prescribed shaft speed";
    final parameter Real wrapSign = sign(exitAngle - entryAngle)
      "Web winding sense from entry to exit";
    final parameter SI.Angle midAngle = (entryAngle + exitAngle)/2
      "Contact arc midpoint angle";

    output SI.Velocity vWeb "Surface velocity along the web travel direction";
    output SI.Velocity vExpected "Velocity predicted by the wrap-sign convention";

    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Support connection";
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute(
      n = {0, 1, 0},
      useAxisFlange = true,
      phi(start = 0, fixed = true),
      w(start = wDrive, fixed = false)) "Roll rotation";
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(w_fixed = wDrive)
      "Prescribed shaft speed";
    Modelica.Mechanics.MultiBody.Parts.Body body(
      m = 1, r_CM = {0, 0, 0}, I_11 = 0.01, I_22 = 0.01, I_33 = 0.01,
      animation = false) "Roll body";
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation contact(
      r = {radius*sin(midAngle), 0, radius*cos(midAngle)},
      animation = false) "Material point at the contact arc midpoint";
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity sensor(
      resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)
      "True velocity of the contact point";
  protected
    SI.Angle theta "Instantaneous contact angle from the vertical";
    Real tangent[3] "Unit web travel direction at the contact point";
  equation
    connect(frame_a, revolute.frame_a);
    connect(revolute.frame_b, body.frame_a);
    connect(revolute.frame_b, contact.frame_a);
    connect(contact.frame_b, sensor.frame_a);
    connect(drive.flange, revolute.axis);
    theta = midAngle + revolute.phi;
    tangent = wrapSign*{cos(theta), 0, -sin(theta)};
    vWeb = sensor.v*tangent;
    vExpected = wrapSign*radius*revolute.w;
    assert(abs(vWeb - vExpected) < 1e-8,
      "Wrap sign convention violated: multibody surface velocity disagrees with wrapSign*radius*w");
  end RotatingContactProbe;

  model WrapDirectionCheck
    "Validates the wrap sign convention for a forward and a reversed wrap"
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1});
    RotatingContactProbe forward(entryAngle = -0.6, exitAngle = 0.6)
      "Web winding in the +theta direction";
    RotatingContactProbe reversed(entryAngle = 0.6, exitAngle = -0.6)
      "Web winding in the -theta direction";
  equation
    connect(world.frame_b, forward.frame_a);
    connect(world.frame_b, reversed.frame_a);
  end WrapDirectionCheck;

  annotation(Documentation(info = "<html><h4>Wrap direction checks</h4>
<p>Fixtures and scenarios for wrap direction on rotating contact surfaces. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end WrapDirectionCheck;
