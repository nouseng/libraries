within Roll2RollDynamicsTest;
model TripleSFrictionCheck
  "Validation harness for the analytic web-to-roll friction law"
  extends Modelica.Icons.Example;
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false)
    annotation(Placement(visible = true, transformation(origin = {-160, 240},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed surfaceMount(animation = false)
    "Roll surface held still"
    annotation(Placement(visible = true, transformation(origin = {-80, 160},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed vectorSurfaceMount(animation = false)
    "Roll surface held still for the friction-circle case"
    annotation(Placement(visible = true, transformation(origin = {-80, 80},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Speed webSpeed(exact = true)
    annotation(Placement(visible = true, transformation(origin = {0, 80},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Speed vectorWebSpeed(exact = true)
    annotation(Placement(visible = true, transformation(origin = {0, 0},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Speed vectorLateralSpeed(exact = true)
    annotation(Placement(visible = true, transformation(origin = {0, -80},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant webVelocity(k = 0.05)
    annotation(Placement(visible = true, transformation(origin = {-80, 0},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant vectorWebVelocity(k = 0.003)
    annotation(Placement(visible = true, transformation(origin = {-80, -80},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant vectorLateralVelocity(k = 0.004)
    annotation(Placement(visible = true, transformation(origin = {-80, -160},
      extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.WebFriction friction(
    radius = 0.1,
    rotationDirection = 1,
    normalForce = 100,
    meanTension=150,
    traction(
      muAdhesion = 0.35,
      muSliding = 0.22,
      vAdhesion = 0.001,
      vSlide = 0.05,
      viscousSlope = 5))
    annotation(Placement(visible = true, transformation(origin = {80, 40},
      extent = {{-10, -10}, {10, 10}})));
  Roll2RollDynamics.Utilities.Parts.WebFriction vectorFriction(
    radius = 0.1,
    rotationDirection = 1,
    lateral = true,
    normalForce = 100,
    meanTension=150,
    traction(
      muAdhesion = 0.35,
      muSliding = 0.22,
      vAdhesion = 0.001,
      vSlide = 0.003,
      viscousSlope = 5))
    annotation(Placement(visible = true, transformation(origin = {80, -40},
      extent = {{-10, -10}, {10, 10}})));
  output Modelica.Units.SI.Velocity slipVelocity = friction.slipVelocity
    "Descriptive contact slip output";
  output Modelica.Units.SI.Force tractionCapacity = friction.tractionCapacity
    "Peak available adhesion force";
  output Modelica.Units.SI.Force longitudinalForce = friction.longitudinalForce
    "Longitudinal contact force";
  output Modelica.Units.SI.Velocity lateralSlip = friction.lateralSlip
    "Cross-machine contact slip";
  output Modelica.Units.SI.Force lateralForce = friction.lateralForce
    "Cross-machine force reported with Roller sign convention";
  output Modelica.Units.SI.Force vectorForceMagnitude = sqrt(
    vectorFriction.longitudinalForce^2 + vectorFriction.lateralForce^2)
    "Friction-circle force magnitude under combined slip";
initial equation
  friction.flange_b.s=0;
  friction.contactFlange.s=0;
  vectorFriction.flange_b.s=0;
  vectorFriction.contactFlange.s=0;
equation
  connect(webVelocity.y, webSpeed.v_ref)
    annotation(Line(visible = true, points = {{-69, 0}, {-12, 80}},
      color = {0, 0, 127}, smooth = Smooth.None));
  connect(webSpeed.flange, friction.flange_a)
    annotation(Line(visible = true, points = {{10, 80}, {73.5, 50}},
      color = {0, 127, 0}, smooth = Smooth.None));
  connect(surfaceMount.frame_b, friction.frame_drum)
    annotation(Line(visible = true, points = {{-70, 160}, {80, 30}},
      color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
  connect(vectorWebVelocity.y, vectorWebSpeed.v_ref)
    annotation(Line(visible = true, points = {{-69, -80}, {-12, 0}},
      color = {0, 0, 127}, smooth = Smooth.None));
  connect(vectorLateralVelocity.y, vectorLateralSpeed.v_ref)
    annotation(Line(visible = true, points = {{-69, -160}, {-12, -80}},
      color = {0, 0, 127}, smooth = Smooth.None));
  connect(vectorWebSpeed.flange, vectorFriction.flange_a)
    annotation(Line(visible = true, points = {{10, 0}, {73.5, -30}},
      color = {0, 127, 0}, smooth = Smooth.None));
  connect(vectorSurfaceMount.frame_b, vectorFriction.frame_drum)
    annotation(Line(visible = true, points = {{-70, 80}, {80, -50}},
      color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
  connect(vectorLateralSpeed.flange, vectorFriction.lateralFlange)
    annotation(Line(visible = true, points = {{10, -80}, {80, -30}},
      color = {0, 127, 0}, smooth = Smooth.None));
  when terminal() then
    assert(abs(slipVelocity - 0.05) < 1e-12,
      "A stationary roll must see the prescribed web speed as slip");
    assert(abs(tractionCapacity - 35) < 1e-9,
      "Capacity must be normal force times the adhesion coefficient");
    assert(abs(longitudinalForce - 47) < 1e-9,
      "Scalar friction must add the normal-load-normalized viscous term");
    assert(abs(lateralSlip) < 1e-12 and abs(lateralForce) < 1e-12,
      "Disabled lateral motion must have zero lateral slip and force");
    assert(abs(vectorForceMagnitude - 24.5) < 1e-8,
      "Combined-slip force magnitude must include the viscous term once");
    assert(abs(vectorFriction.longitudinalForce - 14.7) < 1e-8,
      "Friction circle must resolve the total coefficient longitudinally");
    assert(abs(vectorFriction.lateralForce + 19.6) < 1e-8,
      "Friction circle must resolve the total coefficient laterally");
  end when;
  annotation(Documentation(info = "<html>
<h4>Triple-S friction check</h4>
<p>Prescribed web motion against stationary roll surfaces checks adhesion
capacity, sliding friction and the viscous contribution. A second contact
combines longitudinal and lateral slip to check the friction-circle magnitude
and force directions. At termination, the scalar capacity and force must be
35 N and 47 N, while the combined-slip force must have magnitude 24.5 N and
components 14.7 N longitudinally and -19.6 N laterally.</p>
</html>"));
  annotation(
    Diagram(coordinateSystem(extent = {{-200, -200}, {120, 280}})));
end TripleSFrictionCheck;
