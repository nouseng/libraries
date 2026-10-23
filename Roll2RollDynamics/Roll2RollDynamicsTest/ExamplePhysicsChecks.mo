within Roll2RollDynamicsTest;
package ExamplePhysicsChecks "Boundary physics and traction-capacity regressions"
  extends Modelica.Icons.ExamplesPackage;

  model IdlerRegressionFixture
    "Viscous bearing drag makes an idler lag the web it carries"
    extends Modelica.Icons.Example;

    inner Roll2RollDynamics.WebWorld world
      annotation(Placement(transformation(extent = {{-120, 60}, {-100, 80}})));

    Roll2RollDynamics.Components.Roller entryRoll(
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      xPosition = 0,
      zPosition = 0,
      rotationDirection = -1) "Driven roll, wrapped underneath"
      annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller idler(
      fixedInitialSpeed = true,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      xPosition = 1.0,
      zPosition = 0.9,
      rotationDirection = 1,
      bearingDamping = 0.05)
      "Undriven roll with viscous drag, raised to carry a real wrap"
      annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller exitRoll(
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      xPosition = 2.0,
      zPosition = 0,
      rotationDirection = -1) "Speed master, wrapped underneath"
      annotation(Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}})));

    Roll2RollDynamics.Components.Belt spanIn "Span into the idler"
      annotation(Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt spanOut "Span out of the idler"
      annotation(Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));

    Modelica.Mechanics.Rotational.Sources.Speed drive(exact=true) "Line speed master"
      annotation(Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.Speed infeed(exact=true)
      "Entry speed adjusted for elastic draw"
      annotation(Placement(transformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}})));

    Modelica.Blocks.Sources.Ramp startup(height=world.lineSpeed,duration=2)
      "Smooth acceleration of both driven surfaces"
      annotation(Placement(transformation(origin={0,-40},extent={{-10,-10},{10,10}})));
    Modelica.Blocks.Sources.Constant upstreamTension[3](
      k = {-world.tension*cos(0.54), 0, world.tension*sin(0.54)})
      "Known tension terminating the web upstream of the entry roll"
      annotation(Placement(transformation(origin = {-130, 40}, extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce upstreamLoad(direction=1)
      "Terminates the web upstream of the entry roll"
      annotation(Placement(transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant downstreamTension[3](
      k = {world.tension*cos(0.54), 0, world.tension*sin(0.54)})
      "Known tension terminating the web downstream of the exit roll"
      annotation(Placement(transformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}})));
    Roll2RollDynamics.Components.WebForce downstreamLoad(direction=-1)
      "Terminates the web downstream of the exit roll"
      annotation(Placement(transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}})));

    // Variables with Binding Equations
    output Modelica.Units.SI.Force dragForce = idler.rollerVariables.bearingTorque/(idler.radius + idler.beltThickness/2)
      "Bearing drag referred to the roll surface, which the web has to supply";
    output Modelica.Units.SI.Force momentumFlux =
      idler.massFlowOut*idler.webVelocityOut - idler.massFlowIn*idler.webVelocityIn
      "Net longitudinal momentum carried out of the wrap";
    output Modelica.Units.SI.Force tensionRise = idler.rollerVariables.tensionOut - idler.rollerVariables.tensionIn
      "Tension increase from the upstream span to the downstream span";
  equation
    drive.w_ref=-startup.y/(world.rollRadius + world.web.thickness/2);
    infeed.w_ref=-startup.y/(world.rollRadius + world.web.thickness/2)
      *(entryRoll.frame_a.stretch + entryRoll.frame_b.stretch)
      /(exitRoll.frame_a.stretch + exitRoll.frame_b.stretch);
    connect(drive.flange, exitRoll.flange_a)
      annotation(Line(visible = true, points = {{110, -40}, {60, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(infeed.flange, entryRoll.flange_a)
      annotation(Line(visible = true, points = {{-90, -40}, {-60, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(upstreamTension.y, upstreamLoad.force)
      annotation(Line(visible = true, points = {{-119, 40}, {-110, 40}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(upstreamLoad.frame_b, entryRoll.frame_a)
      annotation(Line(visible = true, points = {{-90, 40}, {-70, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(downstreamTension.y, downstreamLoad.force)
      annotation(Line(visible = true, points = {{79, 50}, {80, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(downstreamLoad.frame_b, exitRoll.frame_b)
      annotation(Line(visible = true, points = {{100, 0}, {70, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(entryRoll.frame_b, spanIn.frame_a)
      annotation(Line(visible = true, points = {{-50, 0}, {-40, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(spanIn.frame_b, idler.frame_a)
      annotation(Line(visible = true, points = {{-20, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(idler.frame_b, spanOut.frame_a)
      annotation(Line(visible = true, points = {{10, 0}, {20, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(spanOut.frame_b, exitRoll.frame_a)
      annotation(Line(visible = true, points = {{40, 0}, {50, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));

    // This regularized contact law develops traction at nonzero slip.
    when terminal() then
      assert(idler.rollerVariables.slipVelocity > 1e-5,
        "The regularized contact must develop positive slip to drive this idler");
      assert(abs(idler.rollerVariables.Ft - dragForce) < 0.02*dragForce,
        "The web must supply exactly the drag the bearing takes out of the roll");
      assert(abs(tensionRise - dragForce - momentumFlux) < 0.02*dragForce,
        "The drag an idler takes must appear as a tension rise across it");
    end when;

    annotation(
      Diagram(coordinateSystem(extent = {{-170, -100}, {140, 110}})));
    annotation(Documentation(info = "<html><h4>Idler regression fixture</h4><p>Helper fixture for the idler boundary test: two driven rolls accelerate a web around an undriven roll with viscous bearing drag. At the end of the run, positive slip must provide the bearing drag, and the tension rise must balance drag plus longitudinal momentum flux.</p></html>"));
  end IdlerRegressionFixture;

  model IdlerBoundaryPhysicsCheck
    "End tensions must contribute to the steady web transport balance"
    extends IdlerRegressionFixture;
  equation
    if time >= 110 then
      assert(abs(idler.rollerVariables.Ft-dragForce) < 0.02*dragForce,
        "Idler traction must balance bearing drag throughout the settled window");
      assert(abs(tensionRise-dragForce-momentumFlux) < 0.02*dragForce,
        "Idler tension rise must balance drag and momentum flux throughout the settled window");
    end if;
    when terminal() then
      assert(abs(entryRoll.rollerVariables.Ft + entryRoll.massFlowOut*entryRoll.webVelocityOut - entryRoll.massFlowIn*entryRoll.webVelocityIn - (entryRoll.rollerVariables.tensionOut - entryRoll.rollerVariables.tensionIn)) < 0.01,
        "Entry boundary tension is missing from the transport balance");
      assert(abs(exitRoll.rollerVariables.Ft + exitRoll.massFlowOut*exitRoll.webVelocityOut - exitRoll.massFlowIn*exitRoll.webVelocityIn - (exitRoll.rollerVariables.tensionOut - exitRoll.rollerVariables.tensionIn)) < 0.01,
        "Exit boundary tension is missing from the transport balance");
    end when;
    annotation(Documentation(info = "<html><h4>Idler boundary physics check</h4><p>Runs the idler fixture to steady transport and checks both loaded line boundaries. From 110 s onward, idler traction and tension rise must balance drag and momentum flux within 2%; the final entry and exit transport-force residuals must each be below 0.01 N.</p></html>"));
  end IdlerBoundaryPhysicsCheck;

  model SteeringBoundaryPhysicsCheck
    "Steering example end loads must carry web transport effort"
    extends Roll2RollDynamics.Examples.SheltonSteering;
    Modelica.Units.SI.Momentum entryImpulse(start=0, fixed=true);
    Modelica.Units.SI.Momentum exitImpulse(start=0, fixed=true);
  equation
    der(entryImpulse) = span.beltVariables.tensionIn - world.tension - entryRoll.rollerVariables.Ft
      + entryRoll.massFlowIn*entryRoll.webVelocityIn
      - entryRoll.massFlowOut*entryRoll.webVelocityOut;
    der(exitImpulse) = world.tension - span.beltVariables.tensionOut - exitRoll.rollerVariables.Ft
      + exitRoll.massFlowIn*exitRoll.webVelocityIn
      - exitRoll.massFlowOut*exitRoll.webVelocityOut;
    assert(abs(entryImpulse - entryRoll.webMomentum) < 1e-5,
      "Steering entry boundary force must supply the web momentum change");
    assert(abs(exitImpulse - exitRoll.webMomentum) < 1e-5,
      "Steering exit boundary force must supply the web momentum change");
    annotation(Documentation(info = "<html><h4>Steering boundary physics check</h4><p>Integrates the force and momentum-flux balances at both ends of the steering example. Each accumulated impulse must agree with the corresponding wrapped-web momentum within 1e-5 N s throughout the run.</p></html>"));
  end SteeringBoundaryPhysicsCheck;

  model CapstanCapacityPhysicsCheck
    "Dry wrap capacity must recover the Euler tension ratio"
    extends Roll2RollDynamics.Examples.OpenBeltDrive;
    // Variables with Binding Equations
    output Real predictedRatio(unit = "1") =
      (smallRoll.rollerVariables.tensionIn + smallRoll.rollerVariables.tensionOut - 2*smallRoll.centrifugalTension + smallRoll.rollerVariables.Fcap)/
      (smallRoll.rollerVariables.tensionIn + smallRoll.rollerVariables.tensionOut - 2*smallRoll.centrifugalTension - smallRoll.rollerVariables.Fcap)
      "Tension ratio at peak traction for the current mean tension";
    output Modelica.Units.SI.Momentum accumulatedImpulse(start = 0, fixed = true)
      "Impulse supplied to the initially stationary contact web";
  equation
    der(accumulatedImpulse) = smallRoll.rollerVariables.tensionOut - smallRoll.rollerVariables.tensionIn - smallRoll.rollerVariables.Ft
      +smallRoll.massFlowIn*smallRoll.webVelocityIn-smallRoll.massFlowOut*smallRoll.webVelocityOut;
    assert(abs(accumulatedImpulse - smallRoll.webMomentum) < 1e-5,
      "Contact inertia must account for the transient departure from tension-force balance");
    assert(abs(predictedRatio - capstanLimit) < 1e-8,
      "Wrapped traction capacity does not recover the capstan limit");
    assert(abs(transmissibleForce - smallRoll.rollerVariables.Fcap) < 1e-8,
      "Initial-tension form of the capstan bound must match the roller capacity");
    annotation(Documentation(info = "<html><h4>Capstan capacity physics check</h4><p>Checks the open-belt example against the Euler capstan tension ratio and the equivalent initial-tension capacity formula. Both capacity identities must agree within 1e-8, while integrated net force must reproduce contact-web momentum within 1e-5 N s.</p></html>"));
  end CapstanCapacityPhysicsCheck;

  model CapstanFrictionSweepCheck
    "Traction recovers Euler's ratio over wrap angle and reverses dissipatively"
    extends Modelica.Icons.Example;
    inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-120, 240},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed mount[3](each animation = false)
      "Stationary drum frames"
      annotation(Placement(visible = true, transformation(origin = {-40, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.ConstantSpeed speed[3](
      v_fixed = {-0.001, 0, 0.001}) "Reverse, locked and forward contact motion"
      annotation(Placement(visible = true, transformation(origin = {-40, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction contact[3](
      each radius = 0.1,
      each meanTension = 150,
      each wrapAngle = 4*time,
      each traction(muAdhesion = 0.35, muSliding = 0.22,
        vAdhesion = 0.001, vSlide = 0.003))
      "Contacts sweeping from zero to more than half a turn"
      annotation(Placement(visible = true, transformation(origin = {40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed vectorMount(animation = false)
      "Stationary drum for combined slip"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.ConstantSpeed longitudinal(
      v_fixed = 0.0006) "Longitudinal part of adhesion-peak slip"
      annotation(Placement(visible = true, transformation(origin = {-40, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.ConstantSpeed lateral(
      v_fixed = 0.0008) "Lateral part of adhesion-peak slip"
      annotation(Placement(visible = true, transformation(origin = {-40, -160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction vectorContact(
      radius = 0.1, meanTension = 150, wrapAngle = 4*time, lateral = true,
      traction(muAdhesion = 0.35, muSliding = 0.22,
        vAdhesion = 0.001, vSlide = 0.003)) "One shared vector traction budget"
      annotation(Placement(visible = true, transformation(origin = {40, -40},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    for i in 1:3 loop
      contact[i].flange_a.s = 0;
      contact[i].flange_b.s = 0;
      contact[i].contactFlange.s = 0;
    end for;
    vectorContact.flange_a.s = 0;
    vectorContact.flange_b.s = 0;
    vectorContact.contactFlange.s = 0;
    vectorContact.lateralFlange.s = 0;
  equation
    for i in 1:3 loop
      connect(mount[i].frame_b, contact[i].frame_drum)
      annotation(Line(visible = true, points = {{-30, 160}, {40, 30}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
      connect(speed[i].flange, contact[i].flange_a)
      annotation(Line(visible = true, points = {{-30, 80}, {33.5, 50}},
        color = {0, 127, 0}, smooth = Smooth.None));
      assert(contact[i].longitudinalForce*contact[i].slipVelocity >= -1e-10,
        "Contact friction must dissipate power in both directions");
    end for;
    connect(vectorMount.frame_b, vectorContact.frame_drum)
      annotation(Line(visible = true, points = {{-30, 0}, {40, -50}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(longitudinal.flange, vectorContact.flange_a)
      annotation(Line(visible = true, points = {{-30, -80}, {33.5, -30}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(lateral.flange, vectorContact.lateralFlange)
      annotation(Line(visible = true, points = {{-30, -160}, {40, -30}},
        color = {0, 127, 0}, smooth = Smooth.None));
    assert(abs((300 - 2*contact[3].centrifugalTension + contact[3].longitudinalForce)/
        (300 - 2*contact[3].centrifugalTension - contact[3].longitudinalForce) - exp(0.35*4*time)) < 1e-8,
      "Forward adhesion-peak force must recover Euler's ratio at every wrap angle");
    assert(abs(contact[1].longitudinalForce + contact[3].longitudinalForce) < 1e-8,
      "Reverse slip must reverse traction without changing its magnitude");
    assert(abs(contact[2].longitudinalForce) < 1e-8,
      "The regularized law must produce zero traction at zero slip");
    assert(abs(sqrt(vectorContact.longitudinalForce^2 + vectorContact.lateralForce^2)
        - (300-2*vectorContact.centrifugalTension)*tanh(0.35*4*time/2)) < 1e-7,
      "Combined slip must share the same capstan traction budget");
    assert(abs(vectorContact.longitudinalForce*0.8
        + vectorContact.lateralForce*0.6) < 1e-7,
      "The vector force must oppose the slip direction");
    annotation(experiment(StopTime = 1, Interval = 0.005, Tolerance = 1e-8));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -200}, {100, 280}})));
    annotation(Documentation(info = "<html><h4>Capstan friction sweep check</h4><p>Sweeps wrap angle from zero to 4 rad for forward, reverse, stationary and combined-slip contacts. The adhesion-peak force must recover the Euler ratio, reverse slip must reverse traction, zero slip must give zero force, and combined slip must share one dissipative capstan force budget.</p></html>"));
  end CapstanFrictionSweepCheck;

  annotation(Documentation(info = "<html><h4>Example physics checks</h4>
<p>Regression scenarios for idler boundaries, steering and capstan capacity. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end ExamplePhysicsChecks;
