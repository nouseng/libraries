within Roll2RollDynamicsTest;
package NipContactChecks "Nip contact-law regressions"
  extends Modelica.Icons.ExamplesPackage;

  model NipContactComponentCheck
    "Fixed-geometry harness for the nip contact law"
    extends Modelica.Icons.Example;
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shaft"
      annotation(Placement(transformation(origin = {-140, 120},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(transformation(origin = {-140, -30},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(transformation(origin = {-200, 180},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed rollerCenter(
      animation = false,
      r = {0, 0, 0}) "Fixed roller centre"
      annotation(Placement(transformation(origin = {-140, 20},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipCenter(
      animation = false,
      r = {0, 0, 0.19}) "Fixed nip centre with positive penetration"
      annotation(Placement(transformation(origin = {120, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webBoundary
      "Stationary web-transport boundary"
      annotation(Placement(transformation(origin = {-140, -80},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      enabled = true,
      nipRadius = 0.1, nipWidth = 0.3,
      webThickness = 0.005,
      coverStiffness = 1e5,
      coverDamping = 0,
      maxNipLoad = 1000,
      traction = world.innerTraction) "Nip contact under validation"
      annotation(Placement(transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary"
      annotation(Placement(transformation(origin = {-140, 70},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(shaft.flange, geometry.shaft)
      annotation(Line(points = {{-140, 120}, {-140, 80}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometry.nipPort, contact.nipPort)
      annotation(Line(points = {{-130, 70}, {-10, 70}, {-10, 10}, {20, 10}, {20, 0}, {30, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rollerCenter.frame_b, geometry.frame)
      annotation(Line(points = {{-130, 20}, {-115, 20}, {-115, 70}, {-150, 70}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipCenter.frame_b, contact.frameNip)
      annotation(Line(points = {{110, 0}, {50, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webBoundary.flange, geometry.web)
      annotation(Line(points = {{-140, -80}, {-150, -80}, {-150, 64}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometry.lateral)
      annotation(Line(points = {{-140, -30}, {-140, 60}},
        color = {0, 127, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-260, -140}, {180, 240}})),
  Documentation(info = "<html><h4>Nip contact component check</h4>
<p>Provide a fixed-geometry structural check of the nip contact component with stationary shaft and web boundaries. The 15 mm penetration should produce a positive elastic normal force; this fixture supplies no separate result assertions.</p></html>"));
  end NipContactComponentCheck;

  model NipContactBehaviorCheck
    "Analytic contact-law cases with fixed roll centres"
    extends Modelica.Icons.Example;
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shafts"
      annotation(Placement(transformation(origin = {-220, 160},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(transformation(origin = {-220, -120},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(transformation(origin = {-220, 220},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerSeparated(
      animation = false, r = {0, 0, 0}) "Separated-case roller centre"
      annotation(Placement(transformation(origin = {-80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipSeparated(
      animation = false, r = {0, 0, 0.22}) "Separated-case nip centre"
      annotation(Placement(transformation(origin = {80, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webSeparated
      "Separated-case web boundary"
      annotation(Placement(transformation(origin = {-80, -40},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Roll2RollDynamics.Utilities.Parts.NipContact separated(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Separated contact"
      annotation(Placement(transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerShallow(
      animation = false, r = {0, 0, 0}) "Shallow-case roller centre"
      annotation(Placement(transformation(origin = {160, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipShallow(
      animation = false, r = {0, 0, 0.204}) "Shallow-case nip centre"
      annotation(Placement(transformation(origin = {320, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webShallow
      "Shallow-case web boundary"
      annotation(Placement(transformation(origin = {160, -40},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Roll2RollDynamics.Utilities.Parts.NipContact shallow(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Shallow contact"
      annotation(Placement(transformation(origin = {240, 0},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerDeep(
      animation = false, r = {0, 0, 0}) "Deep-case roller centre"
      annotation(Placement(transformation(origin = {400, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipDeep(
      animation = false, r = {0, 0, 0.105}) "Deep-case nip centre"
      annotation(Placement(transformation(origin = {560, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webDeep
      "Deep-case web boundary"
      annotation(Placement(transformation(origin = {400, -40},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Roll2RollDynamics.Utilities.Parts.NipContact deep(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Deep contact exceeding maximum actuator force"
      annotation(Placement(transformation(origin = {480, 0},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerForward(
      animation = false, r = {0, 0, 0}) "Forward-slip roller centre"
      annotation(Placement(transformation(origin = {640, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipForward(
      animation = false, r = {0, 0, 0.204}) "Forward-slip nip centre"
      annotation(Placement(transformation(origin = {800, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant forwardSpeed(k = 1)
      "Positive web speed"
      annotation(Placement(transformation(origin = {560, -40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed webForward(exact = true)
      "Forward web boundary"
      annotation(Placement(transformation(origin = {640, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact forward(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Forward-slip contact"
      annotation(Placement(transformation(origin = {720, 0},
        extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.MultiBody.Parts.Fixed rollerReverse(
      animation = false, r = {0, 0, 0}) "Reverse-slip roller centre"
      annotation(Placement(transformation(origin = {880, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed nipReverse(
      animation = false, r = {0, 0, 0.204}) "Reverse-slip nip centre"
      annotation(Placement(transformation(origin = {1040, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Blocks.Sources.Constant reverseSpeed(k = -1)
      "Negative web speed"
      annotation(Placement(transformation(origin = {800, -40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed webReverse(exact = true)
      "Reverse web boundary"
      annotation(Placement(transformation(origin = {880, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact reverse(
      enabled = true, nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction) "Reverse-slip contact"
      annotation(Placement(transformation(origin = {960, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometrySeparated(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Separated-case roller geometry boundary"
      annotation(Placement(transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometryShallow(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Shallow-case roller geometry boundary"
      annotation(Placement(transformation(origin = {200, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometryDeep(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Deep-case roller geometry boundary"
      annotation(Placement(transformation(origin = {440, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometryForward(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Forward-slip roller geometry boundary"
      annotation(Placement(transformation(origin = {680, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometryReverse(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Reverse-slip roller geometry boundary"
      annotation(Placement(transformation(origin = {920, 0},
        extent = {{-10, -10}, {10, 10}})));

  equation
    connect(shaft.flange, geometrySeparated.shaft)
      annotation(Line(points = {{-220, 160}, {-40, 160}, {-40, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometrySeparated.nipPort, separated.nipPort)
      annotation(Line(points = {{-30, 0}, {-10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometryShallow.nipPort, shallow.nipPort)
      annotation(Line(points = {{210, 0}, {230, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometryDeep.nipPort, deep.nipPort)
      annotation(Line(points = {{450, 0}, {470, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometryForward.nipPort, forward.nipPort)
      annotation(Line(points = {{690, 0}, {710, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(geometryReverse.nipPort, reverse.nipPort)
      annotation(Line(points = {{930, 0}, {950, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(shaft.flange, geometryShallow.shaft)
      annotation(Line(points = {{-220, 160}, {200, 160}, {200, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(shaft.flange, geometryDeep.shaft)
      annotation(Line(points = {{-220, 160}, {440, 160}, {440, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(shaft.flange, geometryForward.shaft)
      annotation(Line(points = {{-220, 160}, {680, 160}, {680, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(shaft.flange, geometryReverse.shaft)
      annotation(Line(points = {{-220, 160}, {920, 160}, {920, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(rollerSeparated.frame_b, geometrySeparated.frame)
      annotation(Line(points = {{-70, 40}, {-55, 40}, {-55, 0}, {-50, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipSeparated.frame_b, separated.frameNip)
      annotation(Line(points = {{70, 0}, {10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webSeparated.flange, geometrySeparated.web)
      annotation(Line(points = {{-80, -40}, {-50, -40}, {-50, -6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometrySeparated.lateral)
      annotation(Line(points = {{-220, -120}, {-40, -120}, {-40, -10}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerShallow.frame_b, geometryShallow.frame)
      annotation(Line(points = {{170, 40}, {185, 40}, {185, 0}, {190, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipShallow.frame_b, shallow.frameNip)
      annotation(Line(points = {{310, 0}, {250, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webShallow.flange, geometryShallow.web)
      annotation(Line(points = {{160, -40}, {190, -40}, {190, -6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometryShallow.lateral)
      annotation(Line(points = {{-220, -120}, {200, -120}, {200, -10}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerDeep.frame_b, geometryDeep.frame)
      annotation(Line(points = {{410, 40}, {425, 40}, {425, 0}, {430, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipDeep.frame_b, deep.frameNip)
      annotation(Line(points = {{550, 0}, {490, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(webDeep.flange, geometryDeep.web)
      annotation(Line(points = {{400, -40}, {430, -40}, {430, -6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometryDeep.lateral)
      annotation(Line(points = {{-220, -120}, {440, -120}, {440, -10}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerForward.frame_b, geometryForward.frame)
      annotation(Line(points = {{650, 40}, {665, 40}, {665, 0}, {670, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipForward.frame_b, forward.frameNip)
      annotation(Line(points = {{790, 0}, {730, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(forwardSpeed.y, webForward.v_ref)
      annotation(Line(points = {{571, -40}, {628, -40}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(webForward.flange, geometryForward.web)
      annotation(Line(points = {{650, -40}, {670, -40}, {670, -6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometryForward.lateral)
      annotation(Line(points = {{-220, -120}, {680, -120}, {680, -10}},
        color = {0, 127, 0}, smooth = Smooth.None));

    connect(rollerReverse.frame_b, geometryReverse.frame)
      annotation(Line(points = {{890, 40}, {905, 40}, {905, 0}, {910, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipReverse.frame_b, reverse.frameNip)
      annotation(Line(points = {{1030, 0}, {970, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(reverseSpeed.y, webReverse.v_ref)
      annotation(Line(points = {{811, -40}, {868, -40}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(webReverse.flange, geometryReverse.web)
      annotation(Line(points = {{890, -40}, {910, -40}, {910, -6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometryReverse.lateral)
      annotation(Line(points = {{-220, -120}, {920, -120}, {920, -10}},
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
      Diagram(coordinateSystem(extent = {{-280, -180}, {1100, 280}})),
  Documentation(info = "<html><h4>Nip contact behavior check</h4>
<p>Compare separated, shallow and deeply compressed contact with forward and reverse web slip against stationary shafts. The normal forces should be 0, 100 and 10000 N respectively, with deep compression exceeding the actuator rating, while reversing slip reverses traction and retains nonnegative friction loss; assertions require equal and opposite spatial forces.</p></html>"));
  end NipContactBehaviorCheck;

  model DisabledNipContactCheck
    "Disabled contact needs no external boundary at its unused nip frame"
    extends Modelica.Icons.Example;
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Stationary wrapped shaft"
      annotation(Placement(transformation(origin = {-140, 120},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(transformation(origin = {-140, -30},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(transformation(origin = {-200, 180},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed rollerCenter(
      animation = false, r = {2, 0, 3}) "Displaced roller centre"
      annotation(Placement(transformation(origin = {-140, 20},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant speed(k = 1) "Moving web"
      annotation(Placement(transformation(origin = {-200, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed web(exact = true)
      "Prescribed transport even with the nip disabled"
      annotation(Placement(transformation(origin = {-140, -80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      enabled = false, nipRadius = 0.1, nipWidth = 0.3,
      webThickness = 0.005, coverStiffness = 1e5,
      coverDamping = 1000, maxNipLoad = 1000,
      traction = world.innerTraction) "Unloaded contact with no nip attached"
      annotation(Placement(transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      useNipPort = true,
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary"
      annotation(Placement(transformation(origin = {-140, 70},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(shaft.flange, geometry.shaft)
      annotation(Line(points = {{-140, 120}, {-140, 80}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometry.nipPort, contact.nipPort)
      annotation(Line(points = {{-130, 70}, {-10, 70}, {-10, 10}, {20, 10}, {20, 0}, {30, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rollerCenter.frame_b, geometry.frame)
      annotation(Line(points = {{-130, 20}, {-115, 20}, {-115, 70}, {-150, 70}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(speed.y, web.v_ref)
      annotation(Line(points = {{-189, -80}, {-152, -80}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(web.flange, geometry.web)
      annotation(Line(points = {{-130, -80}, {-150, -80}, {-150, 64}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, geometry.lateral)
      annotation(Line(points = {{-140, -30}, {-140, 60}},
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
      Diagram(coordinateSystem(extent = {{-260, -140}, {100, 240}})),
  Documentation(info = "<html><h4>Disabled nip contact check</h4>
<p>Move the web past a displaced roller with nip contact disabled and the unused nip frame unconnected. Assertions require zero load, slip, penetration, friction loss and roller or web reactions, with contact reported disengaged.</p></html>"));
  end DisabledNipContactCheck;

  annotation(Documentation(info = "<html><h4>Nip contact checks</h4>
<p>Fixed-boundary checks of nip contact forces, forward and reverse slip, and disabled contact. The cases distinguish separation and elastic compression, check balanced spatial reactions, and require a disabled nip to leave the roller and web unloaded.</p>
</html>"));
end NipContactChecks;
