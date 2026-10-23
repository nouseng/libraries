within Roll2RollDynamicsTest;
package WrapDirectionCheck "Rotating contact and wrap-direction regressions"
  extends Modelica.Icons.ExamplesPackage;

  model RotatingContactProbe
    "Material web-contact point on a rotating roll: multibody ground truth for the wrap sign"
    extends Modelica.Icons.Example;
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

    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Support connection"
      annotation(Placement(visible = true, transformation(origin = {-100, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute(
      n = {0, 1, 0},
      useAxisFlange = true,
      phi(start = 0, fixed = true),
      w(start = wDrive, fixed = false)) "Roll rotation"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(w_fixed = wDrive)
      "Prescribed shaft speed"
      annotation(Placement(visible = true, transformation(origin = {-120, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Body body(
      m = 1, r_CM = {0, 0, 0}, I_11 = 0.01, I_22 = 0.01, I_33 = 0.01,
      animation = false) "Roll body"
      annotation(Placement(visible = true, transformation(origin = {40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation contact(
      r = {radius*sin(midAngle), 0, radius*cos(midAngle)},
      animation = false) "Material point at the contact arc midpoint"
      annotation(Placement(visible = true, transformation(origin = {40, -40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity sensor(
      resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world)
      "True velocity of the contact point"
      annotation(Placement(visible = true, transformation(origin = {120, 0},
        extent = {{-10, -10}, {10, 10}})));
  protected
    SI.Angle theta "Instantaneous contact angle from the vertical";
    Real tangent[3] "Unit web travel direction at the contact point";
  equation
    connect(frame_a, revolute.frame_a)
      annotation(Line(visible = true, points = {{-100, 0}, {-50, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(revolute.frame_b, body.frame_a)
      annotation(Line(visible = true, points = {{-30, 0}, {30, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(revolute.frame_b, contact.frame_a)
      annotation(Line(visible = true, points = {{-30, 0}, {30, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(contact.frame_b, sensor.frame_a)
      annotation(Line(visible = true, points = {{50, -40}, {110, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(drive.flange, revolute.axis)
      annotation(Line(visible = true, points = {{-110, 0}, {-40, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    theta = midAngle + revolute.phi;
    tangent = wrapSign*{cos(theta), 0, -sin(theta)};
    vWeb = sensor.v*tangent;
    vExpected = wrapSign*radius*revolute.w;
    assert(abs(vWeb - vExpected) < 1e-8,
      "Wrap sign convention violated: multibody surface velocity disagrees with wrapSign*radius*w");
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {160, 100}})));
    annotation(Documentation(info = "<html><h4>Rotating contact probe</h4><p>Helper probe that measures a rotating material point with a multibody velocity sensor and projects that velocity along the directed wrap tangent. The parent supplies its support connection; measured speed must equal <code>wrapSign*radius*w</code> within 1e-8 m/s, and the probe is not a standalone scenario.</p></html>"));
  end RotatingContactProbe;

  model WrapDirectionCheck
    "Validates the wrap sign convention for a forward and a reversed wrap"
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})
      annotation(Placement(visible = true, transformation(origin = {-80, 120},
        extent = {{-10, -10}, {10, 10}})));
    RotatingContactProbe forward(entryAngle = -0.6, exitAngle = 0.6)
      "Web winding in the +theta direction"
      annotation(Placement(visible = true, transformation(origin = {0, 40},
        extent = {{-10, -10}, {10, 10}})));
    RotatingContactProbe reversed(entryAngle = 0.6, exitAngle = -0.6)
      "Web winding in the -theta direction"
      annotation(Placement(visible = true, transformation(origin = {0, -40},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, forward.frame_a)
      annotation(Line(visible = true, points = {{-70, 120}, {-10, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(world.frame_b, reversed.frame_a)
      annotation(Line(visible = true, points = {{-70, 120}, {-10, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -100}, {100, 160}})));
    annotation(Documentation(info = "<html><h4>Wrap direction check</h4><p>Connects forward and reversed wrap probes to the same stationary world. Each sensor-based surface speed must match its wrap-sign prediction, giving opposite directed web speeds for the same positive shaft rotation.</p></html>"));
  end WrapDirectionCheck;

  annotation(Documentation(info = "<html><h4>Wrap direction checks</h4>
<p>Fixtures and scenarios for wrap direction on rotating contact surfaces. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end WrapDirectionCheck;
