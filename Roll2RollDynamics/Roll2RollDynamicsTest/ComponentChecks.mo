within Roll2RollDynamicsTest;
package ComponentChecks "Component interfaces and support reactions"
  extends Modelica.Icons.ExamplesPackage;

  model WebForceComponentCheck "Prescribed pull with shared material properties"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared material"
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce boundary "Boundary under validation"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -100}, {100, 120}})));
    annotation(Documentation(info="<html><h4>Web force component check</h4><p>Provides a structural check of a standalone prescribed-pull boundary using the shared material defaults. The boundary must pass model checking without a connected web assembly.</p></html>"));
  end WebForceComponentCheck;

  model RollerComponentCheck
    "OpenModelica validation harness for a standalone multibody roller"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roller(useSupport = true) "Roller under validation (explicit support)"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-70, 80}, {0, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -100}, {100, 120}})));
    annotation(Documentation(info="<html><h4>Roller component check</h4><p>Provides a structural check of a standalone roller with its explicit support fixed to the world. The component and support connection must form a valid model.</p></html>"));
  end RollerComponentCheck;

  model RollerNipComponentCheck
    "Roller and nip roll joined by one combined mechanical port"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-120, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation nipPosition(
      r = {0, 0, 0.205}, animation = false)
      "Nip housing position above the wrapped roller"
      annotation(Placement(visible = true, transformation(origin = {-40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roller(
      useSupport = true,
      radius = 0.1,
      beltThickness = 0.005) "Wrapped roller with a nip pressed on it"
      annotation(Placement(visible = true, transformation(origin = {-40, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.NipRoller nip(
      useSupport = true,
      radius = 0.1) "Undriven nip roll"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-110, 120}, {-40, -50}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(world.frame_b, nipPosition.frame_a)
      annotation(Line(visible = true, points = {{-110, 120}, {-50, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipPosition.frame_b, nip.frame_support)
      annotation(Line(visible = true, points = {{-30, 40}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(roller.frame_nip, nip.nipLoading)
      annotation(Line(visible = true, points = {{-40, -30}, {50.567, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 160}})));
    annotation(Documentation(info="<html><h4>Roller nip component check</h4><p>Checks the combined mechanical connection between a wrapped roller and an undriven nip roll. Both housings are fixed at the prescribed centre spacing, and the assembled interface must pass model checking.</p></html>"));
  end RollerNipComponentCheck;

  model ExternalDancerSupportCheck
    "Validation harness for a roller mounted on an external vertical support"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-120, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(
      n = {0, 0, 1},
      useAxisFlange = true,
      s(start = 0, fixed = true),
      v(fixed = false)) "External vertical dancer degree of freedom"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.SpringDamper suspension(
      c = 5000,
      d = 100,
      s_rel0 = 0) "External dancer restoring load and damping"
      annotation(Placement(visible = true, transformation(origin = {40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roller(useSupport = true)
      "Roller mounted through its support frame"
      annotation(Placement(visible = true, transformation(origin = {40, -40},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, prismatic.frame_a)
      annotation(Line(visible = true, points = {{-110, 120}, {-50, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(prismatic.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-30, 0}, {40, -50}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(prismatic.axis, suspension.flange_a)
      annotation(Line(visible = true, points = {{-32, 6}, {30, 40}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(prismatic.support, suspension.flange_b)
      annotation(Line(visible = true, points = {{-44, 6}, {50, 40}},
        color = {0, 127, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 160}})));
    annotation(Documentation(info="<html><h4>External dancer support check</h4><p>Checks a roller mounted on an external vertical prismatic joint with a spring-damper suspension. Model checking must accept the external dancer degree of freedom and its support connections.</p></html>"));
  end ExternalDancerSupportCheck;

  model RollerSupportLoadCheck
    "Regression harness for roller tangency-force propagation"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-160, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known force resolved in the world frame"
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce tangencyForce(direction=1)
      "External tangency load"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce exitForce(
      direction=-1, force={0,0,100}) "Taut exit boundary with no x load"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, -80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roller(
      useSupport = true,
      useNip = true,
      useFlange = true,
      entryAngleStart = Modelica.Constants.pi,
      exitAngleStart = 3*Modelica.Constants.pi/2,
      fixedInitialWebState = true) "Roller under support-load validation"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-150, 160}, {80, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedForce.y, tangencyForce.force)
      annotation(Line(visible = true, points = {{-69, 80}, {-10, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(tangencyForce.frame_b, roller.frame_a)
      annotation(Line(visible = true, points = {{10, 0}, {70, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(exitForce.frame_b, roller.frame_b)
      annotation(Line(visible = true, points = {{-70, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, roller.flange_a)
      annotation(Line(visible = true, points = {{-80, -80}, {80, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -120}, {120, 200}})));
    annotation(Documentation(info="<html><h4>Roller support load check</h4><p>Applies a 100 N horizontal tangency load to a supported roller with a fixed shaft and a taut exit boundary. The support-reaction probe should recover a balancing horizontal support force of -100 N.</p></html>"));
  end RollerSupportLoadCheck;

  model WinderComponentCheck
    "OpenModelica validation harness for a standalone wound roll"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-120, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(w_fixed = 0)
      "Stationary drive"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder winder(useSupport = true)
      "Wound roll under validation (explicit support)"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, winder.frame_support)
      annotation(Line(visible = true, points = {{-110, 80}, {40, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(drive.flange, winder.flange_a)
      annotation(Line(visible = true, points = {{-30, 0}, {40, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 120}})));
    annotation(Documentation(info="<html><h4>Winder component check</h4><p>Provides a structural check of a standalone wound roll with explicit world support and a drive held at zero speed. The support and shaft boundaries must form a valid model.</p></html>"));
  end WinderComponentCheck;

  model WinderSupportLoadCheck
    "Regression harness for wound-roll tangency-force propagation"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-160, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known force resolved in the world frame"
      annotation(Placement(visible = true, transformation(origin = {-80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce tangencyForce(direction=1)
      "External tangency load"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder winder(
      useSupport = true, tangencyAngleStart = Modelica.Constants.pi)
      "Wound roll under support-load validation (explicit support)"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, winder.frame_support)
      annotation(Line(visible = true, points = {{-150, 120}, {80, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedForce.y, tangencyForce.force)
      annotation(Line(visible = true, points = {{-69, 40}, {-10, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(tangencyForce.frame_b, winder.frame_t)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, winder.flange_a)
      annotation(Line(visible = true, points = {{-80, -40}, {80, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 160}})));
    annotation(Documentation(info="<html><h4>Winder support load check</h4><p>Applies a 100 N horizontal tangency load to a supported wound roll with its shaft fixed. The support-reaction probe should recover a balancing horizontal support force of -100 N.</p></html>"));
  end WinderSupportLoadCheck;

  model BeltComponentCheck
    "Validation harness for a web span taking its defaults from the line"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt belt "Web span under validation"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -100}, {100, 120}})));
    annotation(Documentation(info="<html><h4>Belt component check</h4><p>Provides a structural check of a standalone free span using the shared line defaults. The component must pass model checking without an assembled transport path.</p></html>"));
  end BeltComponentCheck;

  model RollBodyComponentCheck
    "Standalone extracted roll body with explicit support and shaft boundary"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared defaults"
      annotation(Placement(visible = true, transformation(origin = {-160, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedTorque[3](k = {0, 1, 0})
      "Known torque resolved in the world frame"
      annotation(Placement(visible = true, transformation(origin = {-80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque drumTorque(animation = false)
      "External torque applied through the rotating drum frame"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.RollBody body(
      useSupport = true,
      useFlange = true,
      radius = world.rollRadius,
      width = world.rollWidth,
      beltThickness = world.web.thickness,
      bearingDamping = world.bearingDamping,
      normalForce = 0,
      drumDrawnRadius = world.rollRadius - world.web.thickness/20)
      "Roll body under validation"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(visible = true, points = {{-150, 120}, {80, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedTorque.y, drumTorque.torque)
      annotation(Line(visible = true, points = {{-69, 40}, {-12, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(drumTorque.frame_b, body.frame_drum)
      annotation(Line(visible = true, points = {{10, 0}, {80, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(visible = true, points = {{-80, -40}, {90, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 160}})));
    annotation(Documentation(info="<html><h4>Roll body component check</h4><p>Checks the extracted roll body with an explicit support and fixed shaft while a 1 N m world torque acts on the drum. The support, drum and shaft interfaces must form a valid model.</p></html>"));
  end RollBodyComponentCheck;

  model FixedNipRollerComponentCheck
    "Nip roll using its internal fixed housing with a connected roller/web port"
    extends RollerNipComponentCheck(nip(useSupport = false, zPosition = 0.205));
    annotation(Documentation(info="<html><h4>Fixed nip roller component check</h4><p>Repeats the roller-to-nip interface check with the nip roll using its internal fixed housing. The nip connection must remain valid when the external nip support is disabled.</p></html>"));
  end FixedNipRollerComponentCheck;

  model RollBodySupportLoadCheck
    "Extracted roll body carries a known contact load to its support"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared defaults"
      annotation(Placement(visible = true, transformation(origin = {-160, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known world-frame contact load"
      annotation(Placement(visible = true, transformation(origin = {-80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce contactForce(animation = false)
      "Contact load applied at the roll-centre frame"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.RollBody body(
      useSupport = true,
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialSpeed = true,
      radius = world.rollRadius,
      width = world.rollWidth,
      beltThickness = world.web.thickness,
      bearingDamping = world.bearingDamping,
      normalForce = 100,
      drumDrawnRadius = world.rollRadius - world.web.thickness/20)
      "Roll body under support-load validation"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(visible = true, points = {{-150, 120}, {80, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedForce.y, contactForce.force)
      annotation(Line(visible = true, points = {{-69, 40}, {-12, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(contactForce.frame_b, body.frame_center)
      annotation(Line(visible = true, points = {{10, 0}, {71, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(visible = true, points = {{-80, -40}, {90, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    assert(Modelica.Math.Vectors.length(body.frame_support.f + appliedForce.k
      - {0, 0, body.density*Modelica.Constants.pi
        *(body.radius^2 - (body.radius - body.wallThickness)^2)*body.width*world.g}) < 1e-6,
      "The support must balance applied contact force and shell weight for any yaw");
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 160}})));
    annotation(Documentation(info="<html><h4>Roll body support load check</h4><p>Applies a known world-frame contact force at the roll centre with the shaft fixed. The support force must balance that load and the independently calculated shell weight, including when the housing is yawed.</p></html>"));
  end RollBodySupportLoadCheck;

  model YawedRollBodySupportLoadCheck
    "A quarter-turn yaw makes the horizontal load axial while lift offsets weight"
    extends RollBodySupportLoadCheck(
      body(yaw = Modelica.Constants.pi/2),
      appliedForce(k = {100, 0, 50}));
    annotation(Documentation(info="<html><h4>Yawed roll body support load check</h4><p>Turns the roll housing through a quarter-turn and applies a 100 N horizontal load plus 50 N of lift. The inherited support assertion must still balance the world-frame load and shell weight when the horizontal load is axial to the roll.</p></html>"));
  end YawedRollBodySupportLoadCheck;

  model WebWrapComponentCheck
    "Standalone web wrap connected to a fixed roll centre and contact model"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared defaults"
      annotation(Placement(visible = true, transformation(origin = {-160, 200},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed center
      "Fixed non-spinning roll centre"
      annotation(Placement(visible = true, transformation(origin = {-80, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed surface(animation = false)
      "Stationary roll-surface frame"
      annotation(Placement(visible = true, transformation(origin = {-80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant entryLoad[3](k = {-100, 0, 0})
      "Known entry-span load"
      annotation(Placement(visible = true, transformation(origin = {-80, -40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant exitLoad[3](k = {100, 0, 0})
      "Known exit-span load"
      annotation(Placement(visible = true, transformation(origin = {-80, -120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce entryForce(animation = false)
      "Entry-span force boundary"
      annotation(Placement(visible = true, transformation(origin = {0, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce exitForce(animation = false)
      "Exit-span force boundary"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebWrap wrap(
      spanDirectionA=-entryLoad.k/Modelica.Math.Vectors.length(entryLoad.k),
      spanDirectionB=exitLoad.k/Modelica.Math.Vectors.length(exitLoad.k),
      entryTension=Modelica.Math.Vectors.length(entryLoad.k),
      exitTension=Modelica.Math.Vectors.length(exitLoad.k),
      radius = world.rollRadius,
      webWidth = world.web.width,
      beltThickness = world.web.thickness,
      rotationDirection = 1,
      lateral = false,
      stressLow = world.stressLow,
      stressHigh = world.stressHigh,
      stressNominal = world.stressNominal,
      colorGamma = world.stressColorGamma)
      "Web wrap under validation"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction friction(
      radius = world.rollRadius,
      rotationDirection = 1,
      meanTension = wrap.meanTension,
      wrapAngle = abs(wrap.arcAngle),
      traction = world.innerTraction,
      lateral = false,
      lateralDrift = 0)
      "Contact closure for the web transport coordinates"
      annotation(Placement(visible = true, transformation(origin = {0, -80},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(center.frame_b, wrap.frame_center)
      annotation(Line(visible = true, points = {{-70, 120}, {71, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(entryLoad.y, entryForce.force)
      annotation(Line(visible = true, points = {{-69, -40}, {-12, 80}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(exitLoad.y, exitForce.force)
      annotation(Line(visible = true, points = {{-69, -120}, {-12, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(entryForce.frame_b, wrap.frame_a)
      annotation(Line(visible = true, points = {{10, 80}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(exitForce.frame_b, wrap.frame_b)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(surface.frame_b, friction.frame_drum)
      annotation(Line(visible = true, points = {{-70, 40}, {0, -90}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -160}, {120, 240}})));
    annotation(Documentation(info="<html><h4>Web wrap component check</h4><p>Checks the extracted wrap geometry and contact model against fixed centre and surface frames with equal, opposite 100 N span loads. The geometry, tangent directions and contact declarations must pass structural model checking.</p></html>"));
  end WebWrapComponentCheck;

  annotation(Documentation(info = "<html><h4>Component checks</h4>
<p>Standalone component fixtures and support-load regressions. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end ComponentChecks;
