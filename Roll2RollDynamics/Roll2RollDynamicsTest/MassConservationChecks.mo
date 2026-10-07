within Roll2RollDynamicsTest;
package MassConservationChecks "Material inventory and transport regressions"
  model MovingBoundary "Prescribed geometry and boundary-relative transport"
    input Modelica.Units.SI.Position position[3]={0,0,0};
    input Modelica.Units.SI.Velocity speed=0;
    Modelica.Units.SI.Angle yaw(start=0) "Free span yaw";
    Modelica.Units.SI.Angle skew(start=0) "Free span skew";
    Roll2RollDynamics.Utilities.Interfaces.RollPort frame_b;
  initial equation
    frame_b.web.s=0;
  equation
    der(frame_b.web.s)=speed;
    Connections.root(frame_b.frame.R);
    frame_b.frame.r_0=position;
    skew=Modelica.Math.atan2(frame_b.spanDirection[2],
      cos(yaw)*frame_b.spanDirection[1]-sin(yaw)*frame_b.spanDirection[3]);
    frame_b.frame.R=Modelica.Mechanics.MultiBody.Frames.axesRotations(
      {2,3,1}, {yaw,skew,0}, {der(yaw),der(skew),0});
  end MovingBoundary;

  model MovingDrum "Prescribed yaw, shaft spin and vertical mount acceleration"
    parameter Modelica.Units.SI.Angle yaw = 0 "Fixed housing yaw";
    parameter Modelica.Units.SI.AngularVelocity spin = 0 "Shaft speed";
    parameter Modelica.Units.SI.Acceleration acceleration = 0 "Vertical acceleration";
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Drum centre";
  equation
    Connections.root(frame_b.R);
    frame_b.r_0 = {0, 0, acceleration*time^2/2};
    frame_b.R = Modelica.Mechanics.MultiBody.Frames.axesRotations(
      {3, 2, 1}, {yaw, spin*time, 0}, {0, spin, 0});
  end MovingDrum;

  model ContactSupportLoad "Contact carries weight, acceleration and momentum flux through its drum frame"
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false,
      g=9.81, n={0, 0, -1}, innerTraction(muAdhesion=0, muSliding=0, viscousSlope=0),
      outerTraction(muAdhesion=0, muSliding=0, viscousSlope=0))
      "Gravity and frictionless contact defaults";
    parameter Modelica.Units.SI.Velocity speed = 2 "Constant web speed";
    final parameter Real lineDensity(unit="kg/m") = 1/(1 + 150/8000)
      "Strained line density for the prescribed material";
    final parameter Modelica.Units.SI.Mass mass = lineDensity*Modelica.Constants.pi*0.1
      "Independent half-wrap inventory";
    MovingDrum drum[3](yaw={0, 0.4, 0.4}, spin={0, 7, 7},
      acceleration={0, 0, 2}) "Static, spinning and accelerating mounts";
    Modelica.Mechanics.Translational.Sources.ConstantSpeed drive[3](each v_fixed=speed)
      "Prescribed material transport";
    Roll2RollDynamics.Utilities.Parts.WebFriction contact[3](
      each radius=0.1, each referenceDensity=1, each axialStiffness=8000,
      each wrapLength=Modelica.Constants.pi*0.1, each meanTension=150,
      each wrapAngle=Modelica.Constants.pi, each traction=world.innerTraction,
      each entryTangent={0, 0, 1}, each exitTangent={0, 0, -1},
      meanTangent={2/Modelica.Constants.pi*{cos(drum[i].yaw), sin(drum[i].yaw), 0}
        for i in 1:3}) "Half-wrap material with independent centre-frame loading";
    Modelica.Units.SI.Force forceWorld[3, 3] "Contact frame forces resolved in world coordinates";
  initial equation
    for i in 1:3 loop
      contact[i].flange_a.s = 0;
      contact[i].flange_b.s = 0;
      contact[i].contactFlange.s = 0;
    end for;
  equation
    for i in 1:3 loop
      connect(drum[i].frame_b, contact[i].frame_drum);
      connect(drive[i].flange, contact[i].flange_a);
      forceWorld[i, :] = Modelica.Mechanics.MultiBody.Frames.resolve1(
        contact[i].frame_drum.R, contact[i].frame_drum.f);
      assert(abs(forceWorld[i, 1]) < 1e-8 and abs(forceWorld[i, 2]) < 1e-8
        and abs(forceWorld[i, 3] - (mass*(drum[i].acceleration + 9.81)
          - 2*lineDensity*speed^2)) < 1e-8,
        "Drum frame must carry wrapped weight, mount inertia and centrifugal unloading");
      assert(Modelica.Math.Vectors.length(contact[i].frame_drum.t) < 1e-8,
        "A centre-applied support force must not add shaft torque");
    end for;
    annotation(experiment(StopTime=1, Tolerance=1e-9));
  end ContactSupportLoad;

  model HeldSpan "Stretching a closed span must preserve its material"
    inner Roll2RollDynamics.WebWorld world(
      enableAnimation=false, tension=100, web(EA=10000));
    Roll2RollDynamics.Components.Belt belt(dampingTime=0) "Purely elastic span under test";
    final parameter Modelica.Units.SI.Mass expectedMass =
      world.web.density*world.web.thickness*world.web.width/1.01
      "One metre of web initially stretched by one percent";
    MovingBoundary inlet "Closed inlet";
    MovingBoundary outlet(position={1 + 0.001*time, 0, 0}) "Moving closed outlet";
  equation
    connect(inlet.frame_b, belt.frame_a);
    connect(belt.frame_b, outlet.frame_b);
    assert(abs(belt.beltVariables.webMass-expectedMass) < 1e-7,
      "A closed span must preserve its initial material inventory");
    when terminal() then
      assert(abs(belt.tension - 302) < 1e-5,
        "Closed span must stretch from 100 N to 302 N at fixed material mass");
    end when;
    annotation(experiment(StopTime=20, Tolerance=1e-8));
  end HeldSpan;

  model ClosedLoop "Total material includes both free spans and pulley wraps"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(enableAnimation=false));
    //Parameters
    parameter Modelica.Units.SI.Mass initialMass(fixed=false) "Initial loop inventory";
    //Variables
    Modelica.Units.SI.Mass inventory "Material inferred independently from geometry and strain";
  initial equation
    initialMass = inventory;
  equation
    inventory = world.web.density*world.web.thickness*world.web.width*(
      upperSpan.beltVariables.freeLength/2*(1/upperSpan.frame_a.stretch
        + 1/upperSpan.frame_b.stretch)
      + lowerSpan.beltVariables.freeLength/2*(1/lowerSpan.frame_a.stretch
        + 1/lowerSpan.frame_b.stretch)
      + (bigRoll.radius + bigRoll.beltThickness/2)*abs(bigRoll.rollerVariables.arcAngle)
        /((bigRoll.frame_a.stretch + bigRoll.frame_b.stretch)/2)
      + (smallRoll.radius + smallRoll.beltThickness/2)*abs(smallRoll.rollerVariables.arcAngle)
        /((smallRoll.frame_a.stretch + smallRoll.frame_b.stretch)/2));
    assert(abs(inventory - initialMass) < 1e-7,
      "Closed drive created or destroyed web material");
    annotation(experiment(StopTime=5, Tolerance=1e-8));
  end ClosedLoop;
  model WinderFlow "Strained winding and shaft momentum in both directions"
    //Parameters
    parameter Integer windingSense = 1 "Winding or unwinding geometry";
    parameter Integer rotationSense = 1 "Shaft orientation";
    parameter Real speedSign = 1 "Forward or reverse transport";
    parameter Modelica.Units.SI.AngularVelocity tangencyRate = 0 "Rotation rate of the imposed pull";
    //Components
    inner Roll2RollDynamics.WebWorld world(
      enableAnimation=false, web(EA=10000),
      bearingDamping=0);
    Roll2RollDynamics.Components.Winder winder(
      winding=windingSense, rotationDirection=rotationSense, radius0=0.15, useSupport=true,
      tangencyAngleStart=if windingSense*rotationSense>0 then Modelica.Constants.pi else 0,
      fixedInitialAngle=true, fixedInitialWebState=true);
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed=rotationSense*speedSign*2);
    Roll2RollDynamics.Components.WebForce load(direction=windingSense);
  equation
    connect(world.frame_b,winder.frame_support);
    assert(abs(winder.frame_support.t[2])<1e-7,"Winder housing must not duplicate shaft tension torque");
    connect(drive.flange, winder.flange_a);
    connect(load.frame_b, winder.frame_t);
    load.force = {1000*cos(tangencyRate*time), 0, 1000*sin(tangencyRate*time)};
    assert(abs(winder.flange_a.tau + winder.frame_t.web.f*rotationSense*winder.radius) < 1e-6,
      "Winder added spurious torque for mass already corotating at entry");
    when terminal() then
      assert(abs(winder.radius - (0.15 + windingSense*0.005*(speedSign*2 + rotationSense*tangencyRate)*5
        /(2*Modelica.Constants.pi*1.1))) < 1e-8,
        "Winder radius must follow strained material flow for either rotation direction");
    end when;
    annotation(experiment(StopTime=5, Tolerance=1e-9));
  end WinderFlow;

  model WinderDirections "All winding, shaft orientation and transport sign combinations"
    WinderFlow cases[8](
      windingSense={1, 1, 1, 1, -1, -1, -1, -1},
      rotationSense={1, 1, -1, -1, 1, 1, -1, -1},
      speedSign={1, -1, 1, -1, 1, -1, 1, -1});
    annotation(experiment(StopTime=5, Tolerance=1e-9));
  end WinderDirections;

  model WinderMigration "A moving tangency crossing the horizontal force quadrant"
    extends WinderFlow(tangencyRate=0.1);
  end WinderMigration;

  model ObliqueStretch "Closed-span stretching away from the machine plane"
    extends HeldSpan(outlet(position={0.6 + 0.0006*time, 0.8 + 0.0008*time, 0}));
  end ObliqueStretch;

  model ReverseLoop "Closed-loop inventory under reversed material travel"
    extends ClosedLoop(world(lineSpeed=-2));
  end ReverseLoop;

  model MovingWrap "Tangency migration must not create physical slip"
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false);
    Modelica.Mechanics.Translational.Components.Fixed entry "Stationary entry boundary";
    Roll2RollDynamics.Utilities.Parts.WebFriction contact(
      referenceDensity=2.7, axialStiffness=8000,
      meanTension=150, tensionIn=150, tensionOut=150,
      wrapLength=0.3 + 0.001*time, boundaryVelocityB=0.001,
      radius=0.1525, wrapAngle=2, traction=world.innerTraction);
  initial equation
    contact.flange_b.s = 0;
    contact.contactFlange.s = 0;
  equation
    connect(world.frame_b, contact.frame_drum);
    connect(entry.flange, contact.flange_a);
    assert(abs(contact.slipVelocity) < 1e-9,
      "Moving tangency created slip in physically stationary web");
    when terminal() then
      assert(abs(contact.flange_b.s + 0.01) < 1e-8,
        "Exit crossing coordinate did not account for tangency migration");
      assert(abs(contact.webMass - 2.7*0.31/1.01875) < 1e-8,
        "Growing wrap did not acquire the expected material");
    end when;
    annotation(experiment(StopTime=10, Tolerance=1e-9));
  end MovingWrap;

  model WindingLine "Minimal unwind, idler and rewind inventory fixture"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Modelica.Units.SI.Mass initialMass(fixed=false) "Initial material inventory";
    //Variables
    Modelica.Units.SI.Mass totalMass "Material in wound rolls, spans and wrap";
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, lineSpeed=0.25);
    Roll2RollDynamics.Components.Winder unwind(
      winding=-1, xPosition=0, radius0=0.15,
      fixedInitialAngle=true, fixedInitialWebState=true);
    Roll2RollDynamics.Components.Roller idler(
      xPosition=1, zPosition=-0.3, rotationDirection=-1,
      fixedInitialAngle=true, fixedInitialSpeed=true, fixedInitialWebState=true,
      entryAngleStart=-2.5, exitAngleStart=-3.8);
    Roll2RollDynamics.Components.Winder rewind(
      winding=1, xPosition=2, radius0=0.15,
      fixedInitialAngle=true, fixedInitialWebState=true);
    Roll2RollDynamics.Components.Belt inletSpan "Unwind-to-idler span";
    Roll2RollDynamics.Components.Belt outletSpan "Idler-to-rewind span";
    Modelica.Blocks.Sources.Ramp speed(height=world.lineSpeed, duration=2)
      "Surface speed command";
    Modelica.Mechanics.Rotational.Sources.Speed unwindDrive(exact=true);
    Modelica.Mechanics.Rotational.Sources.Speed rewindDrive(exact=true);
  initial equation
    initialMass = totalMass;
  equation
    connect(unwind.frame_t, inletSpan.frame_a);
    connect(inletSpan.frame_b, idler.frame_a);
    connect(idler.frame_b, outletSpan.frame_a);
    connect(outletSpan.frame_b, rewind.frame_t);
    connect(unwindDrive.flange, unwind.flange_a);
    connect(rewindDrive.flange, rewind.flange_a);
    unwindDrive.w_ref = speed.y/unwind.radius;
    rewindDrive.w_ref = speed.y/rewind.radius;
    totalMass = unwind.winderVariables.webMass + rewind.winderVariables.webMass + inletSpan.beltVariables.webMass
      + outletSpan.beltVariables.webMass + idler.webMass;
    assert(abs(totalMass - initialMass) < 1e-6*initialMass,
      "Winding line lost material");
    annotation(experiment(StopTime=10, Tolerance=1e-8),
      Documentation(info="<html><p>Shared fixture for standstill, reversal and
      translating-support regressions. The complete inventory includes both
      wound rolls, both free spans and the idler wrap.</p></html>"));
  end WindingLine;

  model PausedWinding "Start winding after half a second at rest"
    extends WindingLine(speed(startTime=0.5));
    annotation(Documentation(info="<html><p>Checks material conservation at
      rest and through the delayed drive ramp.</p></html>"));
  end PausedWinding;

  model HeldWinding "Keep both winding drives stopped"
    extends WindingLine(world(lineSpeed=0));
    annotation(Documentation(info="<html><p>Checks initialization and material
      conservation when transport stays zero.</p></html>"));
  end HeldWinding;

  model ReverseWinding "Full winding line with reversed material travel"
    extends WindingLine(
      world(enableAnimation=false, lineSpeed=-0.25));
  end ReverseWinding;

  model TranslatingIdler "Full inventory with a moving dancer support"
    extends WindingLine(
      world(enableAnimation=false), idler(useSupport=true));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation base(r={1, 0, -0.3}, animation=false);
    Modelica.Mechanics.MultiBody.Joints.Prismatic slide(n={0, 0, 1}, useAxisFlange=true,
      animation=false, s(fixed=false), v(fixed=false));
    Modelica.Mechanics.Translational.Sources.Position actuator(exact=true);
  equation
    connect(world.frame_b, base.frame_a);
    connect(base.frame_b, slide.frame_a);
    connect(slide.frame_b, idler.frame_support);
    connect(actuator.flange, slide.axis);
    actuator.s_ref = 0.003*(1 - cos(time));
  end TranslatingIdler;

  model ConservativeNip "Nip actuation preserves material in a closed three-roll loop"
    extends Roll2RollDynamics.Examples.NipLoading(world(enableAnimation=false));
    parameter Modelica.Units.SI.Mass initialMass(fixed=false) "Initial loop material";
    Modelica.Units.SI.Mass totalMass "Stored material in spans and wraps";
  initial equation
    initialMass = totalMass;
  equation
    totalMass = risingSpan.beltVariables.webMass + fallingSpan.beltVariables.webMass + returnSpan.beltVariables.webMass
      + driveRoll.webMass + topRoll.webMass + rightRoll.webMass;
    assert(abs(totalMass - initialMass) < 1e-7, "Nip loading changed closed-loop material inventory");
    assert(abs(totalWebMass - totalMass) < 1e-9
        and abs(massBalanceError - (totalMass - initialMass)) < 1e-9,
      "The public closed-loop diagnostics must include every span and wrap");
  end ConservativeNip;

  model ConservativeSteering "Open material balance during lateral steering"
    extends Roll2RollDynamics.Examples.SheltonSteering(world(enableAnimation=false));
    parameter Modelica.Units.SI.Mass initialMass(fixed=false) "Initial line material";
    Real netInflow(unit="kg", start=0, fixed=true) "Integrated external material inflow";
    Modelica.Units.SI.Mass totalMass "Material in the free span and both wraps";
  initial equation
    initialMass = totalMass;
  equation
    totalMass = span.beltVariables.webMass + entryRoll.webMass + exitRoll.webMass;
    der(netInflow) = entryRoll.massFlowIn - exitRoll.massFlowOut;
    assert(abs(totalMass - initialMass - netInflow) < 1e-6,
      "Steering material inventory disagrees with external mass flow");
    assert(abs(totalWebMass - totalMass) < 1e-9
        and abs(massBalanceError - (totalMass - initialMass - netInflow)) < 1e-8,
      "The public open-path diagnostics must reconcile inventory and signed boundary flow");
  end ConservativeSteering;
  model InitializedRoller "Roller with prescribed initial material speed"
    extends Roll2RollDynamics.Components.Roller(
      fixedInitialAngle=true, fixedInitialWebState=false);
    parameter Modelica.Units.SI.Velocity initialVelocity=0;
    output Modelica.Units.SI.Torque housingTorque=rollBody.frame_support.t[2]
      if useSupport;
  initial equation
    webFriction.flange_a.s=0;
    webFriction.flange_b.s=0;
    webFriction.contactFlange.s=0;
    webFriction.vWeb=initialVelocity;
  end InitializedRoller;

  model WrapBearingLoad "Centrifugal unloading and axial torque routing on a half wrap"
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false,g=0,
      bearingDamping=0);
    parameter Modelica.Units.SI.Velocity speeds[2]={0,4};
    InitializedRoller roll[2](each useSupport=true,each useFlange=true,
      initialVelocity=speeds,each entryAngleStart=3*Modelica.Constants.pi/2,
      each exitAngleStart=5*Modelica.Constants.pi/2);
    Roll2RollDynamics.Components.WebForce inlet[2](each direction=1,each force={0,0,-150});
    Roll2RollDynamics.Components.WebForce outlet[2](each direction=-1,each force={0,0,-150});
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive[2](
      w_fixed={speeds[i]/(world.rollRadius+world.web.thickness/2) for i in 1:2});
    final parameter Real lineDensity(unit="kg/m")=
      world.web.density*world.web.thickness*world.web.width/(1+150/world.web.EA);
  equation
    for i in 1:2 loop
      connect(world.frame_b,roll[i].frame_support);
      connect(drive[i].flange,roll[i].flange_a);
      connect(inlet[i].frame_b,roll[i].frame_a);
      connect(outlet[i].frame_b,roll[i].frame_b);
      assert(Modelica.Math.Vectors.length(roll[i].frame_support.f
        - {0, 0, 2*(150-lineDensity*speeds[i]^2)}) < 1e-5,
        "Half-wrap support must include centrifugal unloading");
      assert(abs(roll[i].housingTorque)<1e-7,"Web torque must not be duplicated on the housing");
      assert(abs(roll[i].rollerVariables.Fcap-2*(150-lineDensity*speeds[i]^2)*
        tanh(world.innerTraction.muAdhesion*Modelica.Constants.pi/2))<1e-6,
        "Centrifugal unloading must reduce available traction");
    end for;
    annotation(experiment(StopTime=0.1,Tolerance=1e-9));
  end WrapBearingLoad;

  model FrictionlessTorqueRoute "Tension accelerates web without an extra housing torque"
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false,g=0,
      bearingDamping=0, innerTraction(muAdhesion=0,muSliding=0,viscousSlope=0),
      outerTraction(muAdhesion=0,muSliding=0,viscousSlope=0));
    InitializedRoller roll(useSupport=true,useFlange=true,
      entryAngleStart=3*Modelica.Constants.pi/2,exitAngleStart=5*Modelica.Constants.pi/2);
    Roll2RollDynamics.Components.WebForce inlet(direction=1,force={0,0,-150});
    Roll2RollDynamics.Components.WebForce outlet(direction=-1,force={0,0,-151});
    Modelica.Mechanics.Rotational.Components.Fixed drive;
  equation
    connect(world.frame_b,roll.frame_support);
    connect(drive.flange,roll.flange_a);
    connect(inlet.frame_b,roll.frame_a);
    connect(outlet.frame_b,roll.frame_b);
    assert(abs(roll.housingTorque)<1e-7 and abs(drive.flange.tau)<1e-7,
      "A frictionless wrap must transfer no axial torque to shaft or housing");
    when terminal() then
      assert(roll.webMomentum>0.05,"The tension difference must accelerate the web");
    end when;
    annotation(experiment(StopTime=0.1,Tolerance=1e-9));
  end FrictionlessTorqueRoute;

  model StaticSpanWeight "Independent static support balance includes neighboring span weight"
    extends ClosedLoop(world(lineSpeed=0), bigRoll(useSupport=true),
      smallRoll(radius=0.15,zPosition=0));
    Modelica.Units.SI.Mass shellMass;
    Modelica.Units.SI.Force expectedForce[3];
    Real upperDirection[3];
    Real lowerDirection[3];
  equation
    connect(world.frame_b, bigRoll.frame_support);
    shellMass=bigRoll.density*Modelica.Constants.pi*
      (bigRoll.radius^2-(bigRoll.radius-bigRoll.wallThickness)^2)*bigRoll.width;
    upperDirection=(upperSpan.frame_b.frame.r_0-upperSpan.frame_a.frame.r_0)/upperSpan.beltVariables.freeLength;
    lowerDirection=(lowerSpan.frame_b.frame.r_0-lowerSpan.frame_a.frame.r_0)/lowerSpan.beltVariables.freeLength;
    expectedForce=upperSpan.beltVariables.tensionIn*upperDirection-lowerSpan.beltVariables.tensionOut*lowerDirection
      + (shellMass+bigRoll.webMass+(upperSpan.beltVariables.webMass+lowerSpan.beltVariables.webMass)/2)*{0,0,-world.g};
    assert(Modelica.Math.Vectors.length(bigRoll.frame_support.f + expectedForce)<1e-5,
      "Bearing must support shell, wrap and half of each adjacent span");
    annotation(experiment(StopTime=0.1,Tolerance=1e-9));
  end StaticSpanWeight;
  annotation(Documentation(info = "<html>
<h4>Mass conservation checks</h4>
<p>Check material inventory and mechanical load transfer in stretching spans,
moving wraps, closed loops and winding lines. Stopped, delayed-start and reverse
transport cases exercise the same balances; support checks include stored web
momentum, shell weight and the load from neighboring spans.</p>
</html>"));
end MassConservationChecks;
