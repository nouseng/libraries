within Roll2RollDynamicsTest;
package ExamplePhysicsChecks "Boundary physics and traction-capacity regressions"
  extends Modelica.Icons.ExamplesPackage;

  model IdlerRegressionFixture
    "Viscous bearing drag makes an idler lag the web it carries"

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
      annotation(Line(points = {{110, -40}, {60, -40}, {60, -10}}));
    connect(infeed.flange, entryRoll.flange_a)
      annotation(Line(points = {{-90, -40}, {-60, -40}, {-60, -10}}));
    connect(upstreamTension.y, upstreamLoad.force)
      annotation(Line(points = {{-119, 40}, {-112, 40}}, color = {0, 0, 127}));
    connect(upstreamLoad.frame_b, entryRoll.frame_a)
      annotation(Line(points = {{-90, 40}, {-70, 40}, {-70, 0}}, color = {95, 95, 95}, thickness = 0.5));
    connect(downstreamTension.y, downstreamLoad.force)
      annotation(Line(points = {{3.25, 25}, {-2.75, 25}, {-2.75, -25}, {2.25, -25}}, color = {0, 0, 127}, origin = {75.75, 25}));
    connect(downstreamLoad.frame_b, exitRoll.frame_b)
      annotation(Line(points = {{80, 0}, {70, 0}}, color = {95, 95, 95}, thickness = 0.5));
    connect(entryRoll.frame_b, spanIn.frame_a)
      annotation(Line(points = {{-50, 0}, {-40, 0}}, color = {0, 128, 180}, thickness = 2));
    connect(spanIn.frame_b, idler.frame_a)
      annotation(Line(points = {{-20, 0}, {-10, 0}}, color = {0, 128, 180}, thickness = 2));
    connect(idler.frame_b, spanOut.frame_a)
      annotation(Line(points = {{10, 0}, {20, 0}}, color = {0, 128, 180}, thickness = 2));
    connect(spanOut.frame_b, exitRoll.frame_a)
      annotation(Line(points = {{40, 0}, {50, 0}}, color = {0, 128, 180}, thickness = 2));

    // This regularized contact law develops traction at nonzero slip.
    when terminal() then
      assert(idler.rollerVariables.slipVelocity > 1e-5,
        "The regularized contact must develop positive slip to drive this idler");
      assert(abs(idler.rollerVariables.Ft - dragForce) < 0.02*dragForce,
        "The web must supply exactly the drag the bearing takes out of the roll");
      assert(abs(tensionRise - dragForce - momentumFlux) < 0.02*dragForce,
        "The drag an idler takes must appear as a tension rise across it");
    end when;

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
  end CapstanCapacityPhysicsCheck;

  model CapstanFrictionSweepCheck
    "Traction recovers Euler's ratio over wrap angle and reverses dissipatively"
    inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false);
    Modelica.Mechanics.MultiBody.Parts.Fixed mount[3](each animation = false)
      "Stationary drum frames";
    Modelica.Mechanics.Translational.Sources.ConstantSpeed speed[3](
      v_fixed = {-0.001, 0, 0.001}) "Reverse, locked and forward contact motion";
    Roll2RollDynamics.Utilities.Parts.WebFriction contact[3](
      each radius = 0.1,
      each meanTension = 150,
      each wrapAngle = 4*time,
      each traction(muAdhesion = 0.35, muSliding = 0.22,
        vAdhesion = 0.001, vSlide = 0.003))
      "Contacts sweeping from zero to more than half a turn";
    Modelica.Mechanics.MultiBody.Parts.Fixed vectorMount(animation = false)
      "Stationary drum for combined slip";
    Modelica.Mechanics.Translational.Sources.ConstantSpeed longitudinal(
      v_fixed = 0.0006) "Longitudinal part of adhesion-peak slip";
    Modelica.Mechanics.Translational.Sources.ConstantSpeed lateral(
      v_fixed = 0.0008) "Lateral part of adhesion-peak slip";
    Roll2RollDynamics.Utilities.Parts.WebFriction vectorContact(
      radius = 0.1, meanTension = 150, wrapAngle = 4*time, lateral = true,
      traction(muAdhesion = 0.35, muSliding = 0.22,
        vAdhesion = 0.001, vSlide = 0.003)) "One shared vector traction budget";
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
      connect(mount[i].frame_b, contact[i].frame_drum);
      connect(speed[i].flange, contact[i].flange_a);
      assert(contact[i].longitudinalForce*contact[i].slipVelocity >= -1e-10,
        "Contact friction must dissipate power in both directions");
    end for;
    connect(vectorMount.frame_b, vectorContact.frame_drum);
    connect(longitudinal.flange, vectorContact.flange_a);
    connect(lateral.flange, vectorContact.lateralFlange);
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
  end CapstanFrictionSweepCheck;

  annotation(Documentation(info = "<html><h4>Example physics checks</h4>
<p>Regression scenarios for idler boundaries, steering and capstan capacity. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end ExamplePhysicsChecks;
