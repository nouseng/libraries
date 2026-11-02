within Roll2RollDynamicsTest;
package ComponentChecks "Component interfaces and support reactions"
  extends Modelica.Icons.ExamplesPackage;

  model WebBoundaryComponentCheck "Prescribed pull with shared material properties"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared material"
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebBoundary boundary "Boundary under validation"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    annotation(
      Documentation(
        info = "<html><h4>Web boundary component check</h4><p>Provides a structural check of a standalone prescribed-pull boundary using the shared material defaults. The boundary must pass model checking without a connected web assembly.</p></html>"
      ),
      Diagram(coordinateSystem(extent = {{-120, -100}, {100, 120}})
  ));
  end WebBoundaryComponentCheck;

  model SemiClosedSlackBeltCheck
    "Semi-closed belt loop clamped to a moving body with pre-tension, variable-length stiffness, slack and bi-directional drive"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(
      enableAnimation = false,
      allowSlack = true,
      g = 0,
      tension = 20,
      webDampingTime = 0.02,
      web(EA = 1000, density = 1000, thickness = 0.002, width = 0.05))
      "Shared belt properties with slack enabled"
      annotation(Placement(visible = true, transformation(origin = {-190, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed leftMount(r = {-0.6, 0, 0}, animation = false)
      "Left pulley mount"
      annotation(Placement(visible = true, transformation(origin = {-120, 0},
        extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.Fixed rightMount(r = {0.6, 0, 0}, animation = false)
      "Right pulley mount"
      annotation(Placement(visible = true, transformation(origin = {120, 0},
        extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.Fixed railMount(r = {0, 0, 0.152}, animation = false)
      "Linear rail height aligned with the top tangency line"
      annotation(Placement(visible = true, transformation(origin = {-90, -40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic rail(
      n = {1, 0, 0},
      useAxisFlange = true,
      animation = true,
      stateSelect = StateSelect.always,
      s(start = 0, fixed = true),
      v(start = 0, fixed = true))
      "Carriage linear guide along +x"
      annotation(Placement(visible = true, transformation(origin = {-50, -40},
        extent = {{-10, 10}, {10, -10}})));
    Modelica.Mechanics.Translational.Components.Damper railDamper(d = 350)
      "Viscous carriage load that resists motion in both directions and drives the return span slack"
      annotation(Placement(visible = true, transformation(origin = {-48, -70},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Body carriage(
      m = 2,
      r_CM = {0, 0, 0},
      I_11 = 0.01,
      I_22 = 0.01,
      I_33 = 0.01,
      animation = true)
      "Moving carriage body closing the top of the belt loop"
      annotation(Placement(visible = true, transformation(origin = {0, -50},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation leftClampOffset(
      r = {-0.1, 0, 0}, animation = true)
      "Left belt clamp offset on the carriage"
      annotation(Placement(visible = true, transformation(origin = {-20, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rightClampOffset(
      r = {0.1, 0, 0}, animation = true)
      "Right belt clamp offset on the carriage"
      annotation(Placement(visible = true, transformation(origin = {20, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebBoundary leftClamp(useSupport = true)
      "Left belt end clamped to the carriage"
      annotation(Placement(visible = true, transformation(origin = {-40, 40},
        extent = {{10, -10}, {-10, 10}})));
    Roll2RollDynamics.Components.WebBoundary rightClamp(useSupport = true)
      "Right belt end clamped to the carriage"
      annotation(Placement(visible = true, transformation(origin = {40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller drivePulley(
      useSupport = true,
      useFlange = true,
      noSlip = true,
      radius = 0.15,
      fixedInitialAngle = false,
      fixedInitialWebState = true,
      entryAngleStart = Modelica.Constants.pi,
      exitAngleStart = 0)
      "Left driven pulley"
      annotation(Placement(visible = true, transformation(origin = {-120, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller idlerPulley(
      useSupport = true,
      noSlip = true,
      radius = 0.15,
      fixedInitialAngle = true,
      fixedInitialSpeed = true,
      fixedInitialWebState = true,
      entryAngleStart = 0,
      exitAngleStart = Modelica.Constants.pi)
      "Right return pulley"
      annotation(Placement(visible = true, transformation(origin = {120, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt leftTopSpan(
      tensionNominal = 20,
      fixedInitialTension = true,
      fixedInitialMomentum = true)
      "Top span between left pulley and carriage left clamp"
      annotation(Placement(visible = true, transformation(origin = {-80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt rightTopSpan(
      tensionNominal = 20,
      fixedInitialTension = true,
      fixedInitialMomentum = true)
      "Top span between carriage right clamp and right pulley"
      annotation(Placement(visible = true, transformation(origin = {80, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt bottomSpan(
      tensionNominal = 20,
      fixedInitialTension = true,
      fixedInitialMomentum = true)
      "Bottom return span between right and left pulleys"
      annotation(Placement(visible = true, transformation(origin = {0, -100},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.Rotational.Sources.Speed speedSource(
      useSupport = false,
      exact = true)
      "Bi-directional pulley speed drive"
      annotation(Placement(visible = true, transformation(origin = {-150, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.CombiTimeTable angularSpeedCommand(
      table = [0, 0; 0.1, 1.5; 0.35, 1.5; 0.55, -2.0; 0.9, -2.0; 1.0, 0])
      "Forward-then-reverse angular speed profile starting from rest"
      annotation(Placement(visible = true, transformation(origin = {-190, 80},
        extent = {{-10, -10}, {10, 10}})));
    output Modelica.Units.SI.Mass totalBeltMass =
      leftTopSpan.beltVariables.webMass
      + rightTopSpan.beltVariables.webMass
      + bottomSpan.beltVariables.webMass
      + drivePulley.webMass
      + idlerPulley.webMass
      "Total belt mass across all spans and pulleys";
    parameter Modelica.Units.SI.Mass initialBeltMass(fixed = false)
      "Initial total belt mass";
    discrete Modelica.Units.SI.Position maxCarriagePos(start = 0, fixed = true)
      "Peak positive carriage travel";
    discrete Modelica.Units.SI.Position minCarriagePos(start = 0, fixed = true)
      "Peak negative carriage travel";
    discrete Modelica.Units.SI.Force minSpanTension(start = 20, fixed = true)
      "Lowest mean span tension seen during the run";
  initial equation
    initialBeltMass = totalBeltMass;
  equation
    connect(leftMount.frame_b, drivePulley.frame_support)
      annotation(Line(visible = true, points = {{-120, 10}, {-120, 30}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rightMount.frame_b, idlerPulley.frame_support)
      annotation(Line(visible = true, points = {{120, 10}, {120, 30}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(railMount.frame_b, rail.frame_a)
      annotation(Line(visible = true, points = {{-80, -40}, {-60, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rail.frame_b, carriage.frame_a)
      annotation(Line(points = {{-40, -40}, {-25, -40}, {-25, -25}, {0, -25}, {-0, -40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rail.axis, railDamper.flange_a)
      annotation(Line(points = {{-42, -46}, {-23, -46}, {-23, -70}, {-38, -70}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(rail.support, railDamper.flange_b)
      annotation(Line(visible = true, points = {{-54, -46}, {-54, -55}, {-62, -55}, {-62, -70}, {-58, -70}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(carriage.frame_a, leftClampOffset.frame_a)
      annotation(Line(visible = true, points = {{0, -40}, {0, 0}, {-10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(carriage.frame_a, rightClampOffset.frame_a)
      annotation(Line(visible = true, points = {{0, -40}, {0, 0}, {10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(leftClampOffset.frame_b, leftClamp.frame_support)
      annotation(Line(points = {{-30, 0}, {-45, 0}, {-45, 20}, {-15, 20}, {-15, 40}, {-30, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(rightClampOffset.frame_b, rightClamp.frame_support)
      annotation(Line(points = {{30, 0}, {45, 0}, {45, 20}, {15, 20}, {15, 40}, {30, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(angularSpeedCommand.y[1], speedSource.w_ref)
      annotation(Line(visible = true, points = {{-179, 80}, {-162, 80}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(speedSource.flange, drivePulley.flange_a)
      annotation(Line(visible = true, points = {{-140, 80}, {-120, 80}, {-120, 40}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(drivePulley.frame_b, leftTopSpan.frame_a)
      annotation(Line(visible = true, points = {{-110, 40}, {-90, 40}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(leftTopSpan.frame_b, leftClamp.frame_a)
      annotation(Line(visible = true, points = {{-70, 40}, {-50, 40}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(rightClamp.frame_a, rightTopSpan.frame_a)
      annotation(Line(visible = true, points = {{50, 40}, {70, 40}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(rightTopSpan.frame_b, idlerPulley.frame_a)
      annotation(Line(visible = true, points = {{90, 40}, {110, 40}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(idlerPulley.frame_b, bottomSpan.frame_a)
      annotation(Line(visible = true, points = {{130, 40}, {145, 40}, {145, -100}, {10, -100}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(bottomSpan.frame_b, drivePulley.frame_a)
      annotation(Line(visible = true, points = {{-10, -100}, {-145, -100}, {-145, 40}, {-130, 40}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    when sample(0, 0.005) then
      maxCarriagePos = max(pre(maxCarriagePos), rail.s);
      minCarriagePos = min(pre(minCarriagePos), rail.s);
      minSpanTension = min(pre(minSpanTension),
        min(leftTopSpan.beltVariables.tension, rightTopSpan.beltVariables.tension));
    end when;
    assert(leftTopSpan.beltVariables.tension >= -1e-12
      and rightTopSpan.beltVariables.tension >= -1e-12
      and bottomSpan.beltVariables.tension >= -1e-12,
      "Slack spans must never transmit compressive negative tension");
    assert(abs(totalBeltMass - initialBeltMass) < 1e-7,
      "Semi-closed belt loop with slack must conserve total belt mass");
    when terminal() then
      assert(maxCarriagePos > 0.01 and minCarriagePos < -0.002,
        "Carriage must travel in both directions under bi-directional belt drive");
      assert(minSpanTension < 1e-6,
        "Pushed span must enter the zero-tension slack regime during the stroke");
      assert(max(leftTopSpan.beltVariables.tension, rightTopSpan.beltVariables.tension) > 10,
        "Pulling span must recover positive tension after slack");
    end when;
    annotation(experiment(StopTime = 1.0, Tolerance = 1e-7),
      Diagram(coordinateSystem(extent = {{-240, -160}, {200, 180}})),
      Documentation(info = "<html><h4>Semi-closed slack belt check</h4><p>Drives a timing-belt carriage forward and backward through two <code>WebBoundary(useSupport = true)</code> clamps with <code>allowSlack = true</code>. Verifies pre-tension, variable-length span stiffness, zero-tension slack cutoff without negative compression, bi-directional carriage travel, and mass conservation.</p></html>"));
  end SemiClosedSlackBeltCheck;

  model RollerComponentCheck
    "OpenModelica validation harness for a standalone multibody roller"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-60, -20},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roller(useSupport = true) "Roller under validation (explicit support)"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-50, -20}, {0, -20}, {0, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -60}, {60, 60}})),
  Documentation(info="<html><h4>Roller component check</h4><p>Provides a structural check of a standalone roller with its explicit support fixed to the world. The component and support connection must form a valid model.</p></html>"));
  end RollerComponentCheck;

  model RollerNipComponentCheck
    "Roller and nip roll joined by one combined mechanical port"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-80, 40},
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
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.NipRoller nip(
      useSupport = true,
      radius = 0.1) "Undriven nip roll"
      annotation(Placement(visible = true, transformation(origin = {0, 40},
        extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    connect(world.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-70, 40}, {-60, 40}, {-60, -20}, {0, -20}, {0, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(world.frame_b, nipPosition.frame_a)
      annotation(Line(visible = true, points = {{-70, 40}, {-50, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nipPosition.frame_b, nip.frame_support)
      annotation(Line(visible = true, points = {{-30, 40}, {-10, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(roller.frame_nip, nip.nipLoading)
      annotation(Line(visible = true, points = {{0, 10}, {0, 29.433}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -60}, {60, 80}})),
  Documentation(info="<html><h4>Roller nip component check</h4><p>Checks the combined mechanical connection between a wrapped roller and an undriven nip roll. Both housings are fixed at the prescribed centre spacing, and the assembled interface must pass model checking.</p></html>"));
  end RollerNipComponentCheck;

  model ExternalDancerSupportCheck
    "Validation harness for a roller mounted on an external vertical support"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
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
      annotation(Placement(visible = true, transformation(origin = {-38, 30},
        extent = {{10, -10}, {-10, 10}})));
    Roll2RollDynamics.Components.Roller roller(useSupport = true)
      "Roller mounted through its support frame"
      annotation(Placement(visible = true, transformation(origin = {20, 20},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, prismatic.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-50, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(prismatic.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-30, 0}, {20, 0}, {20, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(prismatic.axis, suspension.flange_a)
      annotation(Line(points = {{-32, 6}, {-13, 6}, {-13, 30}, {-28, 30}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(prismatic.support, suspension.flange_b)
      annotation(Line(visible = true, points = {{-44, 6}, {-44, 16}, {-52, 16}, {-52, 30}, {-48, 30}},
        color = {0, 127, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-140, -60}, {80, 80}})),
  Documentation(info="<html><h4>External dancer support check</h4><p>Checks a roller mounted on an external vertical prismatic joint with a spring-damper suspension. Model checking must accept the external dancer degree of freedom and its support connections.</p></html>"));
  end ExternalDancerSupportCheck;

  model RollerSupportLoadCheck
    "Regression harness for roller tangency-force propagation"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-40, -40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known force resolved in the world frame"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebBoundary tangencyForce(direction=1)
      "External tangency load"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebBoundary exitForce(
      direction=-1, force={0,0,100}) "Taut exit boundary with no x load"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {0, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roller(
      useSupport = true,
      useNip = true,
      useFlange = true,
      entryAngleStart = Modelica.Constants.pi,
      exitAngleStart = 3*Modelica.Constants.pi/2,
      fixedInitialWebState = true) "Roller under support-load validation"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, roller.frame_support)
      annotation(Line(visible = true, points = {{-30, -40}, {0, -40}, {0, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedForce.y, tangencyForce.force)
      annotation(Line(visible = true, points = {{-69, 0}, {-50, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(tangencyForce.frame_b, roller.frame_a)
      annotation(Line(visible = true, points = {{-30, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(exitForce.frame_b, roller.frame_b)
      annotation(Line(visible = true, points = {{30, 0}, {10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, roller.flange_a)
      annotation(Line(points = {{0, 40}, {0, 55}, {-30, 55}, {-30, 15}, {0, 15}, {0, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-140, -100}, {100, 100}})),
  Documentation(info="<html><h4>Roller support load check</h4><p>Applies a 100 N horizontal tangency load to a supported roller with a fixed shaft and a taut exit boundary. The support-reaction probe should recover a balancing horizontal support force of -100 N.</p></html>"));
  end RollerSupportLoadCheck;

  model WinderComponentCheck
    "OpenModelica validation harness for a standalone wound roll"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-40, 20},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(w_fixed = 0)
      "Stationary drive"
      annotation(Placement(visible = true, transformation(origin = {-40, -20},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder winder(useSupport = true)
      "Wound roll under validation (explicit support)"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, winder.frame_support)
      annotation(Line(visible = true, points = {{-30, 20}, {40, 20}, {40, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(drive.flange, winder.flange_a)
      annotation(Line(visible = true, points = {{-30, -20}, {40, -20}, {40, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -60}, {80, 60}})),
  Documentation(info="<html><h4>Winder component check</h4><p>Provides a structural check of a standalone wound roll with explicit world support and a drive held at zero speed. The support and shaft boundaries must form a valid model.</p></html>"));
  end WinderComponentCheck;

  model WinderSupportLoadCheck
    "Regression harness for wound-roll tangency-force propagation"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(visible = true, transformation(origin = {-20, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known force resolved in the world frame"
      annotation(Placement(visible = true, transformation(origin = {-60, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebBoundary tangencyForce(direction=1)
      "External tangency load"
      annotation(Placement(visible = true, transformation(origin = {-20, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {20, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder winder(
      useSupport = true, tangencyAngleStart = Modelica.Constants.pi)
      "Wound roll under support-load validation (explicit support)"
      annotation(Placement(visible = true, transformation(origin = {20, 0},
        extent = {{10, -10}, {-10, 10}})));
  equation
    connect(world.frame_b, winder.frame_support)
      annotation(Line(visible = true, points = {{-10, 40}, {20, 40}, {20, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedForce.y, tangencyForce.force)
      annotation(Line(visible = true, points = {{-49, 0}, {-30, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(tangencyForce.frame_b, winder.frame_t)
      annotation(Line(visible = true, points = {{-10, 0}, {10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, winder.flange_a)
      annotation(Line(visible = true, points = {{20, -40}, {20, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -80}, {60, 80}})),
  Documentation(info="<html><h4>Winder support load check</h4><p>Applies a 100 N horizontal tangency load to a supported wound roll with its shaft fixed. The support-reaction probe should recover a balancing horizontal support force of -100 N.</p></html>"));
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
      Documentation(
        info = "<html><h4>Belt component check</h4><p>Provides a structural check of a standalone free span using the shared line defaults. The component must pass model checking without an assembled transport path.</p></html>"
      ),
      Diagram(coordinateSystem(extent = {{-120, -100}, {100, 120}})
  ));
  end BeltComponentCheck;

  model RollBodyComponentCheck
    "Standalone extracted roll body with explicit support and shaft boundary"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared defaults"
      annotation(Placement(visible = true, transformation(origin = {-20, -20},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedTorque[3](k = {0, 1, 0})
      "Known torque resolved in the world frame"
      annotation(Placement(visible = true, transformation(origin = {-60, 20},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldTorque drumTorque(animation = false)
      "External torque applied through the rotating drum frame"
      annotation(Placement(visible = true, transformation(origin = {-20, 20},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}}, rotation = 90)));
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
      annotation(Placement(transformation(origin = {40, 0},
        extent = {{-20, -10}, {20, 10}})));
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(points = {{-10, -20}, {5, -20}, {5, -25}, {40, -25}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(appliedTorque.y, drumTorque.torque)
      annotation(Line(points = {{-49, 20}, {-32, 20}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(drumTorque.frame_b, body.frame_drum)
      annotation(Line(points = {{-10, 20}, {5, 20}, {5, 25}, {40, 25}, {40, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(points = {{80, 0}, {60, 0}},
        smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -80}, {140, 80}})),
  Documentation(info="<html><h4>Roll body component check</h4><p>Checks the extracted roll body with an explicit support and fixed shaft while a 1 N m world torque acts on the drum. The support, drum and shaft interfaces must form a valid model.</p></html>"));
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
      annotation(Placement(transformation(origin = {-30, -30},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known world-frame contact load"
      annotation(Placement(transformation(origin = {-70, 30},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce contactForce(animation = false)
      "Contact load applied at the roll-centre frame"
      annotation(Placement(transformation(origin = {-30, 30},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary"
      annotation(Placement(transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Roll2RollDynamics.Utilities.Parts.RollBody body(
      useSupport = true,
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialSpeed = false,
      radius = world.rollRadius,
      width = world.rollWidth,
      beltThickness = world.web.thickness,
      bearingDamping = world.bearingDamping,
      normalForce = 100,
      drumDrawnRadius = world.rollRadius - world.web.thickness/20)
      "Roll body under support-load validation"
      annotation(Placement(transformation(origin = {40, 0},
        extent = {{-20, -10}, {20, 10}})));
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(points = {{-40, -6.667}, {20, -6.667}, {20, 13.333}},
        color = {95, 95, 95}, thickness = 0.5, origin = {20, -23.333}));
    connect(appliedForce.y, contactForce.force)
      annotation(Line(points = {{-8.5, 0}, {8.5, -0}},
        color = {0, 0, 127}, origin = {-50.5, 30}));
    connect(contactForce.frame_b, body.frame_center)
      annotation(Line(points = {{-13.458, 10}, {28.204, 10}, {28.204, -10}, {28.542, -10}},
        color = {95, 95, 95}, thickness = 0.5, origin = {-6.542, 20}));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(points = {{80, 0}, {60, 0}},
        smooth = Smooth.None));
    assert(Modelica.Math.Vectors.length(body.frame_support.f + appliedForce.k
      - {0, 0, body.density*Modelica.Constants.pi
        *(body.radius^2 - (body.radius - body.wallThickness)^2)*body.width*world.g}) < 1e-6,
      "The support must balance applied contact force and shell weight for any yaw");
    annotation(
      Diagram(coordinateSystem(extent = {{-120, -80}, {140, 80}})),
  Documentation(info="<html><h4>Roll body support load check</h4><p>Applies a known world-frame contact force at the roll centre with the shaft fixed. The support force must balance that load and the independently calculated shell weight, including when the housing is yawed.</p></html>"));
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
      annotation(Placement(transformation(origin = {-140, 100},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed center
      "Fixed non-spinning roll centre"
      annotation(Placement(transformation(origin = {-18, -50},
        extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.Fixed surface(animation = false)
      "Stationary roll-surface frame"
      annotation(Placement(transformation(origin = {0, -160},
        extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Blocks.Sources.Constant entryLoad[3](k = {-100, 0, 0})
      "Known entry-span load"
      annotation(Placement(transformation(origin = {-140, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant exitLoad[3](k = {100, 0, 0})
      "Known exit-span load"
      annotation(Placement(transformation(origin = {140, 0},
        extent = {{10, -10}, {-10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce entryForce(animation = false)
      "Entry-span force boundary"
      annotation(Placement(transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce exitForce(animation = false)
      "Exit-span force boundary"
      annotation(Placement(transformation(origin = {80, 0},
        extent = {{10, -10}, {-10, 10}})));
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
      annotation(Placement(transformation(origin = {0, 0},
        extent = {{-20, -10}, {20, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction friction(
      radius = world.rollRadius,
      rotationDirection = 1,
      meanTension = wrap.meanTension,
      wrapAngle = abs(wrap.arcAngle),
      traction = world.innerTraction,
      lateral = false,
      lateralDrift = 0)
      "Contact closure for the web transport coordinates"
      annotation(Placement(transformation(origin = {0, -100},
        extent = {{-20, -10}, {20, 10}})));
  equation
    connect(center.frame_b, wrap.frame_center)
      annotation(Line(points = {{-18, -40}, {-18, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(entryLoad.y, entryForce.force)
      annotation(Line(points = {{-129, 0}, {-92, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(exitLoad.y, exitForce.force)
      annotation(Line(points = {{129, 0}, {92, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(entryForce.frame_b, wrap.frame_a)
      annotation(Line(points = {{-70, 0}, {-20, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(exitForce.frame_b, wrap.frame_b)
      annotation(Line(points = {{70, 0}, {20, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(surface.frame_b, friction.frame_drum)
      annotation(Line(points = {{0, -150}, {0, -110}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -220}, {200, 160}})),
  Documentation(info="<html><h4>Web wrap component check</h4><p>Checks the extracted wrap geometry and contact model against fixed centre and surface frames with equal, opposite 100 N span loads. The geometry, tangent directions and contact declarations must pass structural model checking.</p></html>"));
  end WebWrapComponentCheck;

  annotation(Documentation(info = "<html><h4>Component checks</h4>
<p>Standalone component fixtures and support-load regressions. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end ComponentChecks;
