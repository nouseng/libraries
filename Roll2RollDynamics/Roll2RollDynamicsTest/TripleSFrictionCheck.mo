within Roll2RollDynamicsTest;
model TripleSFrictionCheck
  "Validation harness for the analytic web-to-roll friction law"
  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false);
  Modelica.Mechanics.MultiBody.Parts.Fixed surfaceMount(animation = false)
    "Roll surface held still";
  Modelica.Mechanics.MultiBody.Parts.Fixed vectorSurfaceMount(animation = false)
    "Roll surface held still for the friction-circle case";
  Modelica.Mechanics.Translational.Sources.Speed webSpeed(exact = true);
  Modelica.Mechanics.Translational.Sources.Speed vectorWebSpeed(exact = true);
  Modelica.Mechanics.Translational.Sources.Speed vectorLateralSpeed(exact = true);
  Modelica.Blocks.Sources.Constant webVelocity(k = 0.05);
  Modelica.Blocks.Sources.Constant vectorWebVelocity(k = 0.003);
  Modelica.Blocks.Sources.Constant vectorLateralVelocity(k = 0.004);
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
      viscousSlope = 5));
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
      viscousSlope = 5));
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
    annotation(Line(points = {{-80, 40}, {-60, 40}, {-60, 20}}, color = {0, 0, 127}));
  connect(webSpeed.flange, friction.flange_a)
    annotation(Line(points = {{-60, 0}, {-20, 0}, {-20, -10}}, color = {0, 127, 0}));
  connect(surfaceMount.frame_b, friction.frame_drum)
    annotation(Line(points = {{40, 0}, {20, 0}, {20, -20}, {0, -20}}, color = {95, 95, 95}, thickness = 0.5));
  connect(vectorWebVelocity.y, vectorWebSpeed.v_ref)
    annotation(Line(points = {{-80, -40}, {-60, -40}}, color = {0, 0, 127}));
  connect(vectorLateralVelocity.y, vectorLateralSpeed.v_ref)
    annotation(Line(points = {{60, -40}, {80, -40}}, color = {0, 0, 127}));
  connect(vectorWebSpeed.flange, vectorFriction.flange_a)
    annotation(Line(points = {{-60, -60}, {-20, -60}}, color = {0, 127, 0}));
  connect(vectorSurfaceMount.frame_b, vectorFriction.frame_drum)
    annotation(Line(points = {{40, -60}, {0, -60}}, color = {95, 95, 95}, thickness = 0.5));
  connect(vectorLateralSpeed.flange, vectorFriction.lateralFlange)
    annotation(Line(points = {{80, -60}, {20, -60}}, color = {0, 127, 0}));
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
and force directions.</p>
</html>"));
end TripleSFrictionCheck;
