within Roll2RollDynamicsTest;
package NipContactChecks "Nip contact-law regressions"
  extends Modelica.Icons.ExamplesPackage;

  model NipContactComponentCheck
    "Fixed-geometry harness for the nip contact law"
    extends Modelica.Icons.Example;
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shaft"
      annotation(Placement(visible = true, transformation(origin = {-80, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-160, 240},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed rollerCenter(
      animation = false,
      r = {0, 0, 0}) "Fixed roller centre"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipCenter(
      animation = false,
      r = {0, 0, 0.19}) "Fixed nip centre with positive penetration"
      annotation(Placement(visible = true, transformation(origin = {-80, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webBoundary
      "Stationary web-transport boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, -160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      enabled = true,
      nipRadius = 0.1, nipWidth = 0.3,
      webThickness = 0.005,
      coverStiffness = 1e5,
      coverDamping = 0,
      maxNipLoad = 1000,
      traction = world.innerTraction) "Nip contact under validation"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(shaft.flange, contact.nipPort.shaft)
      annotation(Line(visible = true, points = {{-80, 160}, {70, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometry.port, contact.nipPort.geometry)
      annotation(Line(visible = true, points = {{10, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rollerCenter.frame_b, contact.nipPort.frame)
      annotation(Line(visible = true, points = {{-70, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipCenter.frame_b, contact.frameNip)
      annotation(Line(visible = true, points = {{-70, -80}, {90, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webBoundary.flange, contact.nipPort.web)
      annotation(Line(visible = true, points = {{-80, -160}, {70, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, contact.nipPort.lateral)
      annotation(Line(visible = true, points = {{-80, 80}, {70, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -200}, {120, 280}})));
    annotation(Documentation(info = "<html><h4>Nip contact component check</h4>
<p>Provide a fixed-geometry structural check of the nip contact component with stationary shaft and web boundaries. The 15 mm penetration should produce a positive elastic normal force; this fixture supplies no separate result assertions.</p></html>"));
  end NipContactComponentCheck;

  model NipContactBehaviorCheck
    "Analytic contact-law cases with fixed roll centres"
    extends Modelica.Icons.Example;
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shafts"
      annotation(Placement(visible = true, transformation(origin = {-200, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(visible = true, transformation(origin = {-200, 80},
        extent = {{-10, -10}, {10, 10}})));
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-280, 240},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerSeparated(
      animation = false, r = {0, 0, 0}) "Separated-case roller centre"
      annotation(Placement(visible = true, transformation(origin = {-200, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipSeparated(
      animation = false, r = {0, 0, 0.22}) "Separated-case nip centre"
      annotation(Placement(visible = true, transformation(origin = {-200, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webSeparated
      "Separated-case web boundary"
      annotation(Placement(visible = true, transformation(origin = {-200, -160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact separated(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Separated contact"
      annotation(Placement(visible = true, transformation(origin = {120, 160},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerShallow(
      animation = false, r = {0, 0, 0}) "Shallow-case roller centre"
      annotation(Placement(visible = true, transformation(origin = {-120, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipShallow(
      animation = false, r = {0, 0, 0.204}) "Shallow-case nip centre"
      annotation(Placement(visible = true, transformation(origin = {-120, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webShallow
      "Shallow-case web boundary"
      annotation(Placement(visible = true, transformation(origin = {-120, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact shallow(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Shallow contact"
      annotation(Placement(visible = true, transformation(origin = {120, 80},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerDeep(
      animation = false, r = {0, 0, 0}) "Deep-case roller centre"
      annotation(Placement(visible = true, transformation(origin = {-120, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipDeep(
      animation = false, r = {0, 0, 0.105}) "Deep-case nip centre"
      annotation(Placement(visible = true, transformation(origin = {-120, -160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webDeep
      "Deep-case web boundary"
      annotation(Placement(visible = true, transformation(origin = {-40, 160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact deep(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Deep contact exceeding maximum actuator force"
      annotation(Placement(visible = true, transformation(origin = {120, 0},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerForward(
      animation = false, r = {0, 0, 0}) "Forward-slip roller centre"
      annotation(Placement(visible = true, transformation(origin = {-40, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipForward(
      animation = false, r = {0, 0, 0.204}) "Forward-slip nip centre"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant forwardSpeed(k = 1)
      "Positive web speed"
      annotation(Placement(visible = true, transformation(origin = {-40, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed webForward(exact = true)
      "Forward web boundary"
      annotation(Placement(visible = true, transformation(origin = {120, -80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact forward(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Forward-slip contact"
      annotation(Placement(visible = true, transformation(origin = {200, 40},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerReverse(
      animation = false, r = {0, 0, 0}) "Reverse-slip roller centre"
      annotation(Placement(visible = true, transformation(origin = {-40, -160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipReverse(
      animation = false, r = {0, 0, 0.204}) "Reverse-slip nip centre"
      annotation(Placement(visible = true, transformation(origin = {40, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant reverseSpeed(k = -1)
      "Negative web speed"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed webReverse(exact = true)
      "Reverse web boundary"
      annotation(Placement(visible = true, transformation(origin = {120, -160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact reverse(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Reverse-slip contact"
      annotation(Placement(visible = true, transformation(origin = {200, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary"
      annotation(Placement(visible = true, transformation(origin = {40, -80},
        extent = {{-10, -10}, {10, 10}})));

  equation
    connect(shaft.flange, separated.nipPort.shaft)
      annotation(Line(visible = true, points = {{-200, 160}, {110, 160}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometry.port, separated.nipPort.geometry)
      annotation(Line(visible = true, points = {{50, -80}, {110, 160}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometry.port, shallow.nipPort.geometry)
      annotation(Line(visible = true, points = {{50, -80}, {110, 80}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometry.port, deep.nipPort.geometry)
      annotation(Line(visible = true, points = {{50, -80}, {110, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometry.port, forward.nipPort.geometry)
      annotation(Line(visible = true, points = {{50, -80}, {190, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometry.port, reverse.nipPort.geometry)
      annotation(Line(visible = true, points = {{50, -80}, {190, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(shaft.flange, shallow.nipPort.shaft)
      annotation(Line(visible = true, points = {{-200, 160}, {110, 80}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(shaft.flange, deep.nipPort.shaft)
      annotation(Line(visible = true, points = {{-200, 160}, {110, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(shaft.flange, forward.nipPort.shaft)
      annotation(Line(visible = true, points = {{-200, 160}, {190, 40}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(shaft.flange, reverse.nipPort.shaft)
      annotation(Line(visible = true, points = {{-200, 160}, {190, -40}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(rollerSeparated.frame_b, separated.nipPort.frame)
      annotation(Line(visible = true, points = {{-190, 0}, {110, 160}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipSeparated.frame_b, separated.frameNip)
      annotation(Line(visible = true, points = {{-190, -80}, {130, 160}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webSeparated.flange, separated.nipPort.web)
      annotation(Line(visible = true, points = {{-200, -160}, {110, 160}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, separated.nipPort.lateral)
      annotation(Line(visible = true, points = {{-200, 80}, {110, 160}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerShallow.frame_b, shallow.nipPort.frame)
      annotation(Line(visible = true, points = {{-110, 160}, {110, 80}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipShallow.frame_b, shallow.frameNip)
      annotation(Line(visible = true, points = {{-110, 80}, {130, 80}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webShallow.flange, shallow.nipPort.web)
      annotation(Line(visible = true, points = {{-120, 0}, {110, 80}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, shallow.nipPort.lateral)
      annotation(Line(visible = true, points = {{-200, 80}, {110, 80}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerDeep.frame_b, deep.nipPort.frame)
      annotation(Line(visible = true, points = {{-110, -80}, {110, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipDeep.frame_b, deep.frameNip)
      annotation(Line(visible = true, points = {{-110, -160}, {130, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webDeep.flange, deep.nipPort.web)
      annotation(Line(visible = true, points = {{-40, 160}, {110, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, deep.nipPort.lateral)
      annotation(Line(visible = true, points = {{-200, 80}, {110, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerForward.frame_b, forward.nipPort.frame)
      annotation(Line(visible = true, points = {{-30, 80}, {190, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipForward.frame_b, forward.frameNip)
      annotation(Line(visible = true, points = {{-30, 0}, {210, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(forwardSpeed.y, webForward.v_ref)
      annotation(Line(visible = true, points = {{-29, -80}, {108, -80}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(webForward.flange, forward.nipPort.web)
      annotation(Line(visible = true, points = {{130, -80}, {190, 40}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, forward.nipPort.lateral)
      annotation(Line(visible = true, points = {{-200, 80}, {190, 40}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerReverse.frame_b, reverse.nipPort.frame)
      annotation(Line(visible = true, points = {{-30, -160}, {190, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipReverse.frame_b, reverse.frameNip)
      annotation(Line(visible = true, points = {{50, 80}, {210, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(reverseSpeed.y, webReverse.v_ref)
      annotation(Line(visible = true, points = {{51, 0}, {108, -160}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(webReverse.flange, reverse.nipPort.web)
      annotation(Line(visible = true, points = {{130, -160}, {190, -40}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, reverse.nipPort.lateral)
      annotation(Line(visible = true, points = {{-200, 80}, {190, -40}},
        color = {0, 127, 0}, smooth = Smooth.None));
    assert(Modelica.Math.Vectors.length(
      Modelica.Mechanics.MultiBody.Frames.resolve1(forward.frameNip.R, forward.frameNip.f)
      + Modelica.Mechanics.MultiBody.Frames.resolve1(forward.nipPort.frame.R, forward.nipPort.frame.f)) < 1e-8,
      "Forward nip contact must apply equal and opposite spatial forces");
    assert(Modelica.Math.Vectors.length(
      Modelica.Mechanics.MultiBody.Frames.resolve1(reverse.frameNip.R, reverse.frameNip.f)
      + Modelica.Mechanics.MultiBody.Frames.resolve1(reverse.nipPort.frame.R, reverse.nipPort.frame.f)) < 1e-8,
      "Reverse nip contact must apply equal and opposite spatial forces");
    annotation(
      Diagram(coordinateSystem(extent = {{-320, -200}, {240, 280}})));
    annotation(Documentation(info = "<html><h4>Nip contact behavior check</h4>
<p>Compare separated, shallow and deeply compressed contact with forward and reverse web slip against stationary shafts. The normal forces should be 0, 100 and 10000 N respectively, with deep compression exceeding the actuator rating, while reversing slip reverses traction and retains nonnegative friction loss; assertions require equal and opposite spatial forces.</p></html>"));
  end NipContactBehaviorCheck;

  model DisabledNipContactCheck
    "Disabled contact needs no external boundary at its unused nip frame"
    extends Modelica.Icons.Example;
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shaft"
      annotation(Placement(visible = true, transformation(origin = {-80, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-160, 240},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed rollerCenter(
      animation = false, r = {2, 0, 3}) "Displaced roller centre"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant speed(k = 1) "Moving web"
      annotation(Placement(visible = true, transformation(origin = {-80, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed web(exact = true)
      "Prescribed transport even with the nip disabled"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      enabled = false, nipRadius = 0.1, nipWidth = 0.3,
      webThickness = 0.005, coverStiffness = 1e5,
      coverDamping = 1000, maxNipLoad = 1000,
      traction = world.innerTraction) "Unloaded contact with no nip attached"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, -160},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(shaft.flange, contact.nipPort.shaft)
      annotation(Line(visible = true, points = {{-80, 160}, {70, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometry.port, contact.nipPort.geometry)
      annotation(Line(visible = true, points = {{-70, -160}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rollerCenter.frame_b, contact.nipPort.frame)
      annotation(Line(visible = true, points = {{-70, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(speed.y, web.v_ref)
      annotation(Line(visible = true, points = {{-69, -80}, {-12, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(web.flange, contact.nipPort.web)
      annotation(Line(visible = true, points = {{10, 0}, {70, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, contact.nipPort.lateral)
      annotation(Line(visible = true, points = {{-80, 80}, {70, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
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
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -200}, {120, 280}})));
    annotation(Documentation(info = "<html><h4>Disabled nip contact check</h4>
<p>Move the web past a displaced roller with nip contact disabled and the unused nip frame unconnected. Assertions require zero load, slip, penetration, friction loss and roller or web reactions, with contact reported disengaged.</p></html>"));
  end DisabledNipContactCheck;

  annotation(Documentation(info = "<html><h4>Nip contact checks</h4>
<p>Fixed-boundary checks of nip contact forces, forward and reverse slip, and disabled contact. The cases distinguish separation and elastic compression, check balanced spatial reactions, and require a disabled nip to leave the roller and web unloaded.</p>
</html>"));
end NipContactChecks;
