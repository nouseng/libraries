within Roll2RollDynamicsTest;
package NipContactChecks "Nip contact-law regressions"
  extends Modelica.Icons.ExamplesPackage;

  model NipContactComponentCheck
    "Fixed-geometry harness for the nip contact law"
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shaft";
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary";
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false);
    Modelica.Mechanics.MultiBody.Parts.Fixed rollerCenter(
      animation = false,
      r = {0, 0, 0}) "Fixed roller centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed nipCenter(
      animation = false,
      r = {0, 0, 0.19}) "Fixed nip centre with positive penetration";
    Modelica.Mechanics.Translational.Components.Fixed webBoundary
      "Stationary web-transport boundary";
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      enabled = true,
      nipRadius = 0.1, nipWidth = 0.3,
      webThickness = 0.005,
      coverStiffness = 1e5,
      coverDamping = 0,
      maxNipLoad = 1000,
      traction = world.innerTraction) "Nip contact under validation";
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary";
  equation
    connect(shaft.flange, contact.nipPort.shaft);
    connect(geometry.port, contact.nipPort.geometry);
    connect(rollerCenter.frame_b, contact.nipPort.frame);
    connect(nipCenter.frame_b, contact.frameNip);
    connect(webBoundary.flange, contact.nipPort.web);
    connect(webLateral.flange, contact.nipPort.lateral);
  end NipContactComponentCheck;

  model NipContactBehaviorCheck
    "Analytic contact-law cases with fixed roll centres"
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shafts";
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary";
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false);

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerSeparated(
      animation = false, r = {0, 0, 0}) "Separated-case roller centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed nipSeparated(
      animation = false, r = {0, 0, 0.22}) "Separated-case nip centre";
    Modelica.Mechanics.Translational.Components.Fixed webSeparated
      "Separated-case web boundary";
    Roll2RollDynamics.Utilities.Parts.NipContact separated(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Separated contact";

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerShallow(
      animation = false, r = {0, 0, 0}) "Shallow-case roller centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed nipShallow(
      animation = false, r = {0, 0, 0.204}) "Shallow-case nip centre";
    Modelica.Mechanics.Translational.Components.Fixed webShallow
      "Shallow-case web boundary";
    Roll2RollDynamics.Utilities.Parts.NipContact shallow(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Shallow contact";

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerDeep(
      animation = false, r = {0, 0, 0}) "Deep-case roller centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed nipDeep(
      animation = false, r = {0, 0, 0.105}) "Deep-case nip centre";
    Modelica.Mechanics.Translational.Components.Fixed webDeep
      "Deep-case web boundary";
    Roll2RollDynamics.Utilities.Parts.NipContact deep(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Deep contact exceeding maximum actuator force";

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerForward(
      animation = false, r = {0, 0, 0}) "Forward-slip roller centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed nipForward(
      animation = false, r = {0, 0, 0.204}) "Forward-slip nip centre";
    Modelica.Blocks.Sources.Constant forwardSpeed(k = 1)
      "Positive web speed";
    Modelica.Mechanics.Translational.Sources.Speed webForward(exact = true)
      "Forward web boundary";
    Roll2RollDynamics.Utilities.Parts.NipContact forward(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Forward-slip contact";

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerReverse(
      animation = false, r = {0, 0, 0}) "Reverse-slip roller centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed nipReverse(
      animation = false, r = {0, 0, 0.204}) "Reverse-slip nip centre";
    Modelica.Blocks.Sources.Constant reverseSpeed(k = -1)
      "Negative web speed";
    Modelica.Mechanics.Translational.Sources.Speed webReverse(exact = true)
      "Reverse web boundary";
    Roll2RollDynamics.Utilities.Parts.NipContact reverse(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Reverse-slip contact";
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary";

  equation
    connect(shaft.flange, separated.nipPort.shaft);
    connect(geometry.port, separated.nipPort.geometry);
    connect(geometry.port, shallow.nipPort.geometry);
    connect(geometry.port, deep.nipPort.geometry);
    connect(geometry.port, forward.nipPort.geometry);
    connect(geometry.port, reverse.nipPort.geometry);
    connect(shaft.flange, shallow.nipPort.shaft);
    connect(shaft.flange, deep.nipPort.shaft);
    connect(shaft.flange, forward.nipPort.shaft);
    connect(shaft.flange, reverse.nipPort.shaft);
    connect(rollerSeparated.frame_b, separated.nipPort.frame);
    connect(nipSeparated.frame_b, separated.frameNip);
    connect(webSeparated.flange, separated.nipPort.web);
    connect(webLateral.flange, separated.nipPort.lateral);

    connect(rollerShallow.frame_b, shallow.nipPort.frame);
    connect(nipShallow.frame_b, shallow.frameNip);
    connect(webShallow.flange, shallow.nipPort.web);
    connect(webLateral.flange, shallow.nipPort.lateral);

    connect(rollerDeep.frame_b, deep.nipPort.frame);
    connect(nipDeep.frame_b, deep.frameNip);
    connect(webDeep.flange, deep.nipPort.web);
    connect(webLateral.flange, deep.nipPort.lateral);

    connect(rollerForward.frame_b, forward.nipPort.frame);
    connect(nipForward.frame_b, forward.frameNip);
    connect(forwardSpeed.y, webForward.v_ref);
    connect(webForward.flange, forward.nipPort.web);
    connect(webLateral.flange, forward.nipPort.lateral);

    connect(rollerReverse.frame_b, reverse.nipPort.frame);
    connect(nipReverse.frame_b, reverse.frameNip);
    connect(reverseSpeed.y, webReverse.v_ref);
    connect(webReverse.flange, reverse.nipPort.web);
    connect(webLateral.flange, reverse.nipPort.lateral);
    assert(Modelica.Math.Vectors.length(
      Modelica.Mechanics.MultiBody.Frames.resolve1(forward.frameNip.R, forward.frameNip.f)
      + Modelica.Mechanics.MultiBody.Frames.resolve1(forward.nipPort.frame.R, forward.nipPort.frame.f)) < 1e-8,
      "Forward nip contact must apply equal and opposite spatial forces");
    assert(Modelica.Math.Vectors.length(
      Modelica.Mechanics.MultiBody.Frames.resolve1(reverse.frameNip.R, reverse.frameNip.f)
      + Modelica.Mechanics.MultiBody.Frames.resolve1(reverse.nipPort.frame.R, reverse.nipPort.frame.f)) < 1e-8,
      "Reverse nip contact must apply equal and opposite spatial forces");
  end NipContactBehaviorCheck;

  model DisabledNipContactCheck
    "Disabled contact needs no external boundary at its unused nip frame"
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shaft";
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary";
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false);
    Modelica.Mechanics.MultiBody.Parts.Fixed rollerCenter(
      animation = false, r = {2, 0, 3}) "Displaced roller centre";
    Modelica.Blocks.Sources.Constant speed(k = 1) "Moving web";
    Modelica.Mechanics.Translational.Sources.Speed web(exact = true)
      "Prescribed transport even with the nip disabled";
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      enabled = false, nipRadius = 0.1, nipWidth = 0.3,
      webThickness = 0.005, coverStiffness = 1e5,
      coverDamping = 1000, maxNipLoad = 1000,
      traction = world.innerTraction) "Unloaded contact with no nip attached";
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary";
  equation
    connect(shaft.flange, contact.nipPort.shaft);
    connect(geometry.port, contact.nipPort.geometry);
    connect(rollerCenter.frame_b, contact.nipPort.frame);
    connect(speed.y, web.v_ref);
    connect(web.flange, contact.nipPort.web);
    connect(webLateral.flange, contact.nipPort.lateral);
    assert(abs(contact.normalForce) < 1e-10 and
      abs(contact.tangentialForce) < 1e-10 and
      abs(contact.slipVelocity) < 1e-10 and
      abs(contact.penetration) < 1e-10 and
      abs(contact.frictionLoss) < 1e-10 and
      not contact.engaged,
      "Disabled nip must report zero load, slip and penetration");
    assert(Modelica.Math.Vectors.length(contact.nipPort.frame.f) < 1e-10 and
      Modelica.Math.Vectors.length(contact.nipPort.frame.t) < 1e-10 and
      abs(contact.nipPort.web.f) < 1e-10,
      "Disabled nip must not load the roller or web");
  end DisabledNipContactCheck;

  annotation(Documentation(info = "<html><h4>Nip contact checks</h4>
<p>Fixtures and scenarios for nip contact force and invalid parameter rejection. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end NipContactChecks;
