within Roll2RollDynamicsTest;
package ComponentChecks "Component interfaces and support reactions"
  extends Modelica.Icons.ExamplesPackage;

  model WebForceComponentCheck "Prescribed pull with shared material properties"
    inner Roll2RollDynamics.WebWorld world "Shared material";
    Roll2RollDynamics.Components.WebForce boundary "Boundary under validation";
  end WebForceComponentCheck;

  model RollerComponentCheck
    "OpenModelica validation harness for a standalone multibody roller"
    inner Roll2RollDynamics.WebWorld world;
    Roll2RollDynamics.Components.Roller roller(useSupport = true) "Roller under validation (explicit support)";
  equation
    connect(world.frame_b, roller.frame_support);
  end RollerComponentCheck;

  model RollerNipComponentCheck
    "Roller and nip roll joined by one combined mechanical port"
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false);
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation nipPosition(
      r = {0, 0, 0.205}, animation = false)
      "Nip housing position above the wrapped roller";
    Roll2RollDynamics.Components.Roller roller(
      useSupport = true,
      radius = 0.1,
      beltThickness = 0.005) "Wrapped roller with a nip pressed on it";
    Roll2RollDynamics.Components.NipRoller nip(
      useSupport = true,
      radius = 0.1) "Undriven nip roll";
  equation
    connect(world.frame_b, roller.frame_support);
    connect(world.frame_b, nipPosition.frame_a);
    connect(nipPosition.frame_b, nip.frame_support);
    connect(roller.frame_nip, nip.nipLoading);
  end RollerNipComponentCheck;

  model ExternalDancerSupportCheck
    "Validation harness for a roller mounted on an external vertical support"
    inner Roll2RollDynamics.WebWorld world;
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(
      n = {0, 0, 1},
      useAxisFlange = true,
      s(start = 0, fixed = true),
      v(fixed = false)) "External vertical dancer degree of freedom";
    Modelica.Mechanics.Translational.Components.SpringDamper suspension(
      c = 5000,
      d = 100,
      s_rel0 = 0) "External dancer restoring load and damping";
    Roll2RollDynamics.Components.Roller roller(useSupport = true)
      "Roller mounted through its support frame";
  equation
    connect(world.frame_b, prismatic.frame_a);
    connect(prismatic.frame_b, roller.frame_support);
    connect(prismatic.axis, suspension.flange_a);
    connect(prismatic.support, suspension.flange_b);
  end ExternalDancerSupportCheck;

  model RollerSupportLoadCheck
    "Regression harness for roller tangency-force propagation"
    inner Roll2RollDynamics.WebWorld world;
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known force resolved in the world frame";
    Roll2RollDynamics.Components.WebForce tangencyForce(direction=1)
      "External tangency load";
    Roll2RollDynamics.Components.WebForce exitForce(
      direction=-1, force={0,0,100}) "Taut exit boundary with no x load";
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary";
    Roll2RollDynamics.Components.Roller roller(
      useSupport = true,
      useNip = true,
      useFlange = true,
      entryAngleStart = Modelica.Constants.pi,
      exitAngleStart = 3*Modelica.Constants.pi/2,
      fixedInitialWebState = true) "Roller under support-load validation";
  equation
    connect(world.frame_b, roller.frame_support);
    connect(appliedForce.y, tangencyForce.force);
    connect(tangencyForce.frame_b, roller.frame_a);
    connect(exitForce.frame_b, roller.frame_b);
    connect(fixedDrive.flange, roller.flange_a);
  end RollerSupportLoadCheck;

  model WinderComponentCheck
    "OpenModelica validation harness for a standalone wound roll"
    inner Roll2RollDynamics.WebWorld world;
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(w_fixed = 0)
      "Stationary drive";
    Roll2RollDynamics.Components.Winder winder(useSupport = true)
      "Wound roll under validation (explicit support)";
  equation
    connect(world.frame_b, winder.frame_support);
    connect(drive.flange, winder.flange_a);
  end WinderComponentCheck;

  model WinderSupportLoadCheck
    "Regression harness for wound-roll tangency-force propagation"
    inner Roll2RollDynamics.WebWorld world;
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known force resolved in the world frame";
    Roll2RollDynamics.Components.WebForce tangencyForce(direction=1)
      "External tangency load";
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary";
    Roll2RollDynamics.Components.Winder winder(
      useSupport = true, tangencyAngleStart = Modelica.Constants.pi)
      "Wound roll under support-load validation (explicit support)";
  equation
    connect(world.frame_b, winder.frame_support);
    connect(appliedForce.y, tangencyForce.force);
    connect(tangencyForce.frame_b, winder.frame_t);
    connect(fixedDrive.flange, winder.flange_a);
  end WinderSupportLoadCheck;

  model BeltComponentCheck
    "Validation harness for a web span taking its defaults from the line"
    inner Roll2RollDynamics.WebWorld world;
    Roll2RollDynamics.Components.Belt belt "Web span under validation";
  end BeltComponentCheck;

  model RollBodyComponentCheck
    "Standalone extracted roll body with explicit support and shaft boundary"
    inner Roll2RollDynamics.WebWorld world "Shared defaults";
    Modelica.Blocks.Sources.Constant appliedTorque[3](k = {0, 1, 0})
      "Known torque resolved in the world frame";
    Modelica.Mechanics.MultiBody.Forces.WorldTorque drumTorque(animation = false)
      "External torque applied through the rotating drum frame";
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary";
    Roll2RollDynamics.Utilities.Parts.RollBody body(
      useSupport = true,
      useFlange = true,
      radius = world.rollRadius,
      width = world.rollWidth,
      beltThickness = world.web.thickness,
      bearingDamping = world.bearingDamping,
      normalForce = 0,
      drumDrawnRadius = world.rollRadius - world.web.thickness/20)
      "Roll body under validation";
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(points = {{-60, 40}, {-20, 40}, {-20, 70}, {0, 70}}, color = {95, 95, 95}, thickness = 0.5));
    connect(appliedTorque.y, drumTorque.torque)
      annotation(Line(points = {{-60, 0}, {-40, 0}}, color = {0, 0, 127}));
    connect(drumTorque.frame_b, body.frame_drum)
      annotation(Line(points = {{-20, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(points = {{-60, -40}, {-20, -40}, {-20, -70}, {0, -70}}, color = {0, 0, 0}));
  end RollBodyComponentCheck;

  model FixedNipRollerComponentCheck
    "Nip roll using its internal fixed housing with a connected roller/web port"
    extends RollerNipComponentCheck(nip(useSupport = false, zPosition = 0.205));
  end FixedNipRollerComponentCheck;

  model RollBodySupportLoadCheck
    "Extracted roll body carries a known contact load to its support"
    inner Roll2RollDynamics.WebWorld world "Shared defaults";
    Modelica.Blocks.Sources.Constant appliedForce[3](k = {100, 0, 0})
      "Known world-frame contact load";
    Modelica.Mechanics.MultiBody.Forces.WorldForce contactForce(animation = false)
      "Contact load applied at the roll-centre frame";
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive
      "Stationary shaft boundary";
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
      "Roll body under support-load validation";
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(points = {{-60, 40}, {-20, 40}, {-20, 70}, {0, 70}}, color = {95, 95, 95}, thickness = 0.5));
    connect(appliedForce.y, contactForce.force)
      annotation(Line(points = {{-60, 30}, {-30, 30}, {-30, 20}, {-10, 20}}, color = {0, 0, 127}));
    connect(contactForce.frame_b, body.frame_center)
      annotation(Line(points = {{10, 20}, {40, 20}, {40, 50}, {70, 50}}, color = {95, 95, 95}, thickness = 0.5));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(points = {{-60, -40}, {-20, -40}, {-20, -70}, {0, -70}}, color = {0, 0, 0}));
    assert(Modelica.Math.Vectors.length(body.frame_support.f + appliedForce.k
      - {0, 0, body.density*Modelica.Constants.pi
        *(body.radius^2 - (body.radius - body.wallThickness)^2)*body.width*world.g}) < 1e-6,
      "The support must balance applied contact force and shell weight for any yaw");
  end RollBodySupportLoadCheck;

  model YawedRollBodySupportLoadCheck
    "A quarter-turn yaw makes the horizontal load axial while lift offsets weight"
    extends RollBodySupportLoadCheck(
      body(yaw = Modelica.Constants.pi/2),
      appliedForce(k = {100, 0, 50}));
  end YawedRollBodySupportLoadCheck;

  model WebWrapComponentCheck
    "Standalone web wrap connected to a fixed roll centre and contact model"
    inner Roll2RollDynamics.WebWorld world "Shared defaults";
    Modelica.Mechanics.MultiBody.Parts.Fixed center
      "Fixed non-spinning roll centre";
    Modelica.Mechanics.MultiBody.Parts.Fixed surface(animation = false)
      "Stationary roll-surface frame";
    Modelica.Blocks.Sources.Constant entryLoad[3](k = {-100, 0, 0})
      "Known entry-span load";
    Modelica.Blocks.Sources.Constant exitLoad[3](k = {100, 0, 0})
      "Known exit-span load";
    Modelica.Mechanics.MultiBody.Forces.WorldForce entryForce(animation = false)
      "Entry-span force boundary";
    Modelica.Mechanics.MultiBody.Forces.WorldForce exitForce(animation = false)
      "Exit-span force boundary";
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
      "Web wrap under validation";
    Roll2RollDynamics.Utilities.Parts.WebFriction friction(
      radius = world.rollRadius,
      rotationDirection = 1,
      meanTension = wrap.meanTension,
      wrapAngle = abs(wrap.arcAngle),
      traction = world.innerTraction,
      lateral = false,
      lateralDrift = 0)
      "Contact closure for the web transport coordinates";
  equation
    connect(center.frame_b, wrap.frame_center)
      annotation(Line(points = {{10, 60}, {40, 60}, {40, 30}, {70, 30}}, color = {95, 95, 95}, thickness = 0.5));
    connect(entryLoad.y, entryForce.force)
      annotation(Line(points = {{-70, 20}, {-40, 20}, {-40, 40}, {-10, 40}}, color = {0, 0, 127}));
    connect(exitLoad.y, exitForce.force)
      annotation(Line(points = {{-70, -20}, {-40, -20}, {-40, -40}, {-10, -40}}, color = {0, 0, 127}));
    connect(entryForce.frame_b, wrap.frame_a)
      annotation(Line(points = {{10, 40}, {30, 40}, {30, 10}, {70, 10}}, color = {95, 95, 95}, thickness = 0.5));
    connect(exitForce.frame_b, wrap.frame_b)
      annotation(Line(points = {{10, -40}, {30, -40}, {30, -10}, {70, -10}}, color = {95, 95, 95}, thickness = 0.5));
    connect(surface.frame_b, friction.frame_drum)
      annotation(Line(points = {{-30, -90}, {20, -90}, {20, -80}}, color = {0, 127, 0}));
  end WebWrapComponentCheck;

  annotation(Documentation(info = "<html><h4>Component checks</h4>
<p>Standalone component fixtures and support-load regressions. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end ComponentChecks;
