within Roll2RollDynamicsTest;
package MassConservationChecks "Material inventory and transport regressions"
  extends Modelica.Icons.ExamplesPackage;

  model MovingBoundary "Prescribed geometry and boundary-relative transport"
    extends Modelica.Icons.Example;
    input Modelica.Units.SI.Position position[3]={0,0,0};
    input Modelica.Units.SI.Velocity speed=0;
    Modelica.Units.SI.Angle yaw(start=0) "Free span yaw";
    Modelica.Units.SI.Angle skew(start=0) "Free span skew";
    Roll2RollDynamics.Utilities.Interfaces.RollPort frame_b
      annotation(Placement(visible = true, transformation(origin = {100, 0},
        extent = {{-10, -10}, {10, 10}})));
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
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -100}, {140, 100}})));
    annotation(Documentation(info="<html><h4>Moving boundary</h4><p>Helper boundary that prescribes position and boundary-relative material speed while allowing the span to determine tangent orientation. Zero speed closes the boundary to material flow; this component is not a standalone test.</p></html>"));
  end MovingBoundary;

  model MovingDrum "Prescribed yaw, shaft spin and vertical mount acceleration"
    extends Modelica.Icons.Example;
    parameter Modelica.Units.SI.Angle yaw = 0 "Fixed housing yaw";
    parameter Modelica.Units.SI.AngularVelocity spin = 0 "Shaft speed";
    parameter Modelica.Units.SI.Acceleration acceleration = 0 "Vertical acceleration";
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Drum centre"
      annotation(Placement(visible = true, transformation(origin = {100, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    Connections.root(frame_b.R);
    frame_b.r_0 = {0, 0, acceleration*time^2/2};
    frame_b.R = Modelica.Mechanics.MultiBody.Frames.axesRotations(
      {3, 2, 1}, {yaw, spin*time, 0}, {0, spin, 0});
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -100}, {140, 100}})));
    annotation(Documentation(info="<html><h4>Moving drum</h4><p>Helper frame boundary that prescribes housing yaw, shaft spin and vertical mount acceleration for contact-load checks. It supplies controlled drum motion rather than a standalone test.</p></html>"));
  end MovingDrum;

  model ContactSupportLoad "Contact carries weight, acceleration and momentum flux through its drum frame"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false,
      g=9.81, n={0, 0, -1}, innerTraction(muAdhesion=0, muSliding=0, viscousSlope=0),
      outerTraction(muAdhesion=0, muSliding=0, viscousSlope=0))
      "Gravity and frictionless contact defaults"
      annotation(Placement(visible = true, transformation(origin = {-120, 120},
        extent = {{-10, -10}, {10, 10}})));
    parameter Modelica.Units.SI.Velocity speed = 2 "Constant web speed";
    final parameter Real lineDensity(unit="kg/m") = 1/(1 + 150/8000)
      "Strained line density for the prescribed material";
    final parameter Modelica.Units.SI.Mass mass = lineDensity*Modelica.Constants.pi*0.1
      "Independent half-wrap inventory";
    MovingDrum drum[3](yaw={0, 0.4, 0.4}, spin={0, 7, 7},
      acceleration={0, 0, 2}) "Static, spinning and accelerating mounts"
      annotation(Placement(visible = true, transformation(origin = {-40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.ConstantSpeed drive[3](each v_fixed=speed)
      "Prescribed material transport"
      annotation(Placement(visible = true, transformation(origin = {-40, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction contact[3](
      each radius=0.1, each referenceDensity=1, each axialStiffness=8000,
      each wrapLength=Modelica.Constants.pi*0.1, each meanTension=150,
      each wrapAngle=Modelica.Constants.pi, each traction=world.innerTraction,
      each entryTangent={0, 0, 1}, each exitTangent={0, 0, -1},
      meanTangent={2/Modelica.Constants.pi*{cos(drum[i].yaw), sin(drum[i].yaw), 0}
        for i in 1:3}) "Half-wrap material with independent centre-frame loading"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Units.SI.Force forceWorld[3, 3] "Contact frame forces resolved in world coordinates";
  initial equation
    for i in 1:3 loop
      contact[i].flange_a.s = 0;
      contact[i].flange_b.s = 0;
      contact[i].contactFlange.s = 0;
    end for;
  equation
    for i in 1:3 loop
      connect(drum[i].frame_b, contact[i].frame_drum)
      annotation(Line(visible = true, points = {{-30, 40}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
      connect(drive[i].flange, contact[i].flange_a)
      annotation(Line(visible = true, points = {{-30, -40}, {33.5, 10}},
        color = {0, 127, 0}, smooth = Smooth.None));
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
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 160}})));
    annotation(Documentation(info="<html><h4>Contact support load</h4><p>Compares frictionless half-wrap support forces for stationary, yawed spinning and vertically accelerating drum frames at constant web speed. Each world-frame reaction must include wrapped weight, mount inertia and centrifugal unloading without adding shaft torque.</p></html>"));
  end ContactSupportLoad;

  model HeldSpan "Stretching a closed span must preserve its material"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(
      enableAnimation=false, tension=100, web(EA=10000))
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt belt(dampingTime=0) "Purely elastic span under test"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    final parameter Modelica.Units.SI.Mass expectedMass =
      world.web.density*world.web.thickness*world.web.width/1.01
      "One metre of web initially stretched by one percent";
    MovingBoundary inlet "Closed inlet"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    MovingBoundary outlet(position={1 + 0.001*time, 0, 0}) "Moving closed outlet"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(inlet.frame_b, belt.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(belt.frame_b, outlet.frame_b)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(belt.beltVariables.webMass-expectedMass) < 1e-7,
      "A closed span must preserve its initial material inventory");
    when terminal() then
      assert(abs(belt.tension - 302) < 1e-5,
        "Closed span must stretch from 100 N to 302 N at fixed material mass");
    end when;
    annotation(experiment(StopTime=20, Tolerance=1e-8));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
    annotation(Documentation(info="<html><h4>Held span</h4><p>Extends a closed, initially one-percent-strained elastic span at 1 mm/s. Material mass must remain fixed, and tension must rise from 100 N to 302 N after 20 s.</p></html>"));
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
    annotation(Documentation(info="<html><h4>Closed loop</h4><p>Checks the two-roll drive using material inventory computed independently from free-span geometry, endpoint stretch and both roller wraps. The complete closed-loop inventory must remain constant.</p></html>"));
  end ClosedLoop;
  model WinderFlow "Strained winding and shaft momentum in both directions"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Integer windingSense = 1 "Winding or unwinding geometry";
    parameter Integer rotationSense = 1 "Shaft orientation";
    parameter Real speedSign = 1 "Forward or reverse transport";
    parameter Modelica.Units.SI.AngularVelocity tangencyRate = 0 "Rotation rate of the imposed pull";
    //Components
    inner Roll2RollDynamics.WebWorld world(
      enableAnimation=false, web(EA=10000),
      bearingDamping=0)
      annotation(Placement(visible = true, transformation(origin = {-120, 120},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder winder(
      winding=windingSense, rotationDirection=rotationSense, radius0=0.15, useSupport=true,
      tangencyAngleStart=if windingSense*rotationSense>0 then Modelica.Constants.pi else 0,
      fixedInitialAngle=true, fixedInitialWebState=true)
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed=rotationSense*speedSign*2)
      annotation(Placement(visible = true, transformation(origin = {-40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce load(direction=windingSense)
      annotation(Placement(visible = true, transformation(origin = {-40, -40},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, winder.frame_support)
      annotation(Line(visible = true, points = {{-110, 120}, {40, 10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(winder.frame_support.t[2])<1e-7,"Winder housing must not duplicate shaft tension torque");
    connect(drive.flange, winder.flange_a)
      annotation(Line(visible = true, points = {{-30, 40}, {40, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(load.frame_b, winder.frame_t)
      annotation(Line(visible = true, points = {{-30, -40}, {50, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    load.force = {1000*cos(tangencyRate*time), 0, 1000*sin(tangencyRate*time)};
    assert(abs(winder.flange_a.tau + winder.frame_t.web.f*rotationSense*winder.radius) < 1e-6,
      "Winder added spurious torque for mass already corotating at entry");
    when terminal() then
      assert(abs(winder.radius - (0.15 + windingSense*0.005*(speedSign*2 + rotationSense*tangencyRate)*5
        /(2*Modelica.Constants.pi*1.1))) < 1e-8,
        "Winder radius must follow strained material flow for either rotation direction");
    end when;
    annotation(experiment(StopTime=5, Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 160}})));
    annotation(Documentation(info="<html><h4>Winder flow</h4><p>Parameterized fixture for the winding-direction and migrating-tangency tests under a 1000 N pull and constant shaft speed. Radius must follow strained material flow after 5 s, tension torque must reach the shaft without an extra corotation torque, and the housing must carry no duplicate shaft torque.</p></html>"));
  end WinderFlow;

  model WinderDirections "All winding, shaft orientation and transport sign combinations"
    extends Modelica.Icons.Example;
    WinderFlow cases[8](
      windingSense={1, 1, 1, 1, -1, -1, -1, -1},
      rotationSense={1, 1, -1, -1, 1, 1, -1, -1},
      speedSign={1, -1, 1, -1, 1, -1, 1, -1})
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    annotation(experiment(StopTime=5, Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    annotation(Documentation(info="<html><h4>Winder directions</h4><p>Runs all eight combinations of winding sense, shaft orientation and transport direction. Every case must satisfy the strained-radius evolution and shaft-versus-housing torque checks in <code>WinderFlow</code>.</p></html>"));
  end WinderDirections;

  model WinderMigration "A moving tangency crossing the horizontal force quadrant"
    extends WinderFlow(tangencyRate=0.1);
    annotation(Documentation(info="<html><h4>Winder migration</h4><p>Rotates the imposed pull at 0.1 rad/s so the wound-roll tangency migrates across the horizontal force quadrant. The inherited radius and torque checks must include tangency motion without spurious shaft or housing torque.</p></html>"));
  end WinderMigration;

  model ObliqueStretch "Closed-span stretching away from the machine plane"
    extends HeldSpan(outlet(position={0.6 + 0.0006*time, 0.8 + 0.0008*time, 0}));
    annotation(Documentation(info="<html><h4>Oblique stretch</h4><p>Repeats the closed-span extension test along an oblique direction outside the machine plane. Material inventory must remain fixed and tension must reach the same 302 N endpoint as the axis-aligned case.</p></html>"));
  end ObliqueStretch;

  model ReverseLoop "Closed-loop inventory under reversed material travel"
    extends ClosedLoop(world(lineSpeed=-2));
    annotation(Documentation(info="<html><h4>Reverse loop</h4><p>Runs the two-roll closed loop with material travel reversed to -2 m/s. Independently calculated inventory in both free spans and both wraps must remain constant.</p></html>"));
  end ReverseLoop;

  model MovingWrap "Tangency migration must not create physical slip"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false)
      annotation(Placement(visible = true, transformation(origin = {-120, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed entry "Stationary entry boundary"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction contact(
      referenceDensity=2.7, axialStiffness=8000,
      meanTension=150, tensionIn=150, tensionOut=150,
      wrapLength=0.3 + 0.001*time, boundaryVelocityB=0.001,
      radius=0.1525, wrapAngle=2, traction=world.innerTraction)
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    contact.flange_b.s = 0;
    contact.contactFlange.s = 0;
  equation
    connect(world.frame_b, contact.frame_drum)
      annotation(Line(visible = true, points = {{-110, 80}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(entry.flange, contact.flange_a)
      annotation(Line(visible = true, points = {{-40, 0}, {33.5, 10}},
        color = {0, 127, 0}, smooth = Smooth.None));
    assert(abs(contact.slipVelocity) < 1e-9,
      "Moving tangency created slip in physically stationary web");
    when terminal() then
      assert(abs(contact.flange_b.s + 0.01) < 1e-8,
        "Exit crossing coordinate did not account for tangency migration");
      assert(abs(contact.webMass - 2.7*0.31/1.01875) < 1e-8,
        "Growing wrap did not acquire the expected material");
    end when;
    annotation(experiment(StopTime=10, Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 120}})));
    annotation(Documentation(info="<html><h4>Moving wrap</h4><p>Grows a stationary-web contact wrap through tangency migration. Physical slip must remain zero, the exit crossing coordinate must reach -0.01 m after 10 s, and wrap inventory must match its increased length and strain.</p></html>"));
  end MovingWrap;

  model WindingLine "Minimal unwind, idler and rewind inventory fixture"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Modelica.Units.SI.Mass initialMass(fixed=false) "Initial material inventory";
    //Variables
    Modelica.Units.SI.Mass totalMass "Material in wound rolls, spans and wrap";
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, lineSpeed=0.25)
      annotation(Placement(visible = true, transformation(origin = {-280, 160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder unwind(
      winding=-1, xPosition=0, radius0=0.15,
      fixedInitialAngle=true, fixedInitialWebState=true)
      annotation(Placement(visible = true, transformation(origin = {-120, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller idler(
      xPosition=1, zPosition=-0.3, rotationDirection=-1,
      fixedInitialAngle=true, fixedInitialSpeed=true, fixedInitialWebState=true,
      entryAngleStart=-2.5, exitAngleStart=-3.8)
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Winder rewind(
      winding=1, xPosition=2, radius0=0.15,
      fixedInitialAngle=true, fixedInitialWebState=true)
      annotation(Placement(visible = true, transformation(origin = {200, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt inletSpan "Unwind-to-idler span"
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt outletSpan "Idler-to-rewind span"
      annotation(Placement(visible = true, transformation(origin = {120, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Ramp speed(height=world.lineSpeed, duration=2)
      "Surface speed command"
      annotation(Placement(visible = true, transformation(origin = {-200, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.Speed unwindDrive(exact=true)
      annotation(Placement(visible = true, transformation(origin = {-200, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.Speed rewindDrive(exact=true)
      annotation(Placement(visible = true, transformation(origin = {-200, -80},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    initialMass = totalMass;
  equation
    connect(unwind.frame_t, inletSpan.frame_a)
      annotation(Line(visible = true, points = {{-110, 0}, {-50, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(inletSpan.frame_b, idler.frame_a)
      annotation(Line(visible = true, points = {{-30, 0}, {30, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(idler.frame_b, outletSpan.frame_a)
      annotation(Line(visible = true, points = {{50, 0}, {110, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(outletSpan.frame_b, rewind.frame_t)
      annotation(Line(visible = true, points = {{130, 0}, {210, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(unwindDrive.flange, unwind.flange_a)
      annotation(Line(visible = true, points = {{-190, 0}, {-120, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(rewindDrive.flange, rewind.flange_a)
      annotation(Line(visible = true, points = {{-190, -80}, {200, -10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    unwindDrive.w_ref = speed.y/unwind.radius;
    rewindDrive.w_ref = speed.y/rewind.radius;
    totalMass = unwind.winderVariables.webMass + rewind.winderVariables.webMass + inletSpan.beltVariables.webMass
      + outletSpan.beltVariables.webMass + idler.webMass;
    assert(abs(totalMass - initialMass) < 1e-6*initialMass,
      "Winding line lost material");
    annotation(experiment(StopTime=10, Tolerance=1e-8),
      Documentation(info="<html><h4>Winding line</h4><p>Shared fixture for delayed-start, stopped, reverse and moving-idler winding tests. Total material in both wound rolls, both free spans and the idler wrap must remain constant as material transfers through the line.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-320, -120}, {240, 200}})));
  end WindingLine;

  model PausedWinding "Start winding after half a second at rest"
    extends WindingLine(speed(startTime=0.5));
    annotation(Documentation(info="<html><h4>Paused winding</h4><p>Delays the winding speed ramp until 0.5 s after initialization. The complete wound-roll, span and wrap inventory must stay constant both at rest and during startup.</p></html>"));
  end PausedWinding;

  model HeldWinding "Keep both winding drives stopped"
    extends WindingLine(world(lineSpeed=0));
    annotation(Documentation(info="<html><h4>Held winding</h4><p>Keeps both winding drives stopped throughout the run. Initialization and the complete material inventory must remain consistent at zero transport speed.</p></html>"));
  end HeldWinding;

  model ReverseWinding "Full winding line with reversed material travel"
    extends WindingLine(
      world(enableAnimation=false, lineSpeed=-0.25));
    annotation(Documentation(info="<html><h4>Reverse winding</h4><p>Reverses the winding-line speed ramp to -0.25 m/s. Signed material transfer must preserve the complete inventory across both wound rolls, both spans and the idler wrap.</p></html>"));
  end ReverseWinding;

  model TranslatingIdler "Full inventory with a moving dancer support"
    extends WindingLine(
      world(enableAnimation=false), idler(useSupport=true));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation base(r={1, 0, -0.3}, animation=false)
      annotation(Placement(visible = true, transformation(origin = {280, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic slide(n={0, 0, 1}, useAxisFlange=true,
      animation=false, s(fixed=false), v(fixed=false))
      annotation(Placement(visible = true, transformation(origin = {360, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Position actuator(exact=true)
      annotation(Placement(visible = true, transformation(origin = {280, -40},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, base.frame_a)
      annotation(Line(visible = true, points = {{-270, 160}, {270, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(base.frame_b, slide.frame_a)
      annotation(Line(visible = true, points = {{290, 40}, {350, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(slide.frame_b, idler.frame_support)
      annotation(Line(visible = true, points = {{370, 0}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(actuator.flange, slide.axis)
      annotation(Line(visible = true, points = {{290, -40}, {368, 6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    actuator.s_ref = 0.003*(1 - cos(time));
    annotation(
      Diagram(coordinateSystem(extent = {{-320, -120}, {400, 200}})));
    annotation(Documentation(info="<html><h4>Translating idler</h4><p>Moves the winding-line idler vertically with a prescribed smooth stroke while material travels through the line. Changing free-span lengths and wrap geometry must preserve the complete wound-roll, span and wrap inventory.</p></html>"));
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
    annotation(Documentation(info="<html><h4>Conservative nip</h4><p>Runs nip actuation in a closed three-roll loop. Independently summed span and wrap inventory must remain constant, and the public total-mass and balance-error diagnostics must report that same complete inventory.</p></html>"));
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
    annotation(Documentation(info="<html><h4>Conservative steering</h4><p>Runs lateral steering on an open web path and integrates signed external material inflow. Span and wrap inventory changes must match that inflow, and the public mass diagnostics must agree with the independent balance.</p></html>"));
  end ConservativeSteering;
  model InitializedRoller "Roller with prescribed initial material speed"
    extends Modelica.Icons.Example;
    extends Roll2RollDynamics.Components.Roller(
      fixedInitialAngle=true, fixedInitialWebState=false)
      annotation(IconMap(primitivesVisible = false));
    parameter Modelica.Units.SI.Velocity initialVelocity=0;
    output Modelica.Units.SI.Torque housingTorque=rollBody.frame_support.t[2]
      if useSupport;
  initial equation
    webFriction.flange_a.s=0;
    webFriction.flange_b.s=0;
    webFriction.contactFlange.s=0;
    webFriction.vWeb=initialVelocity;
    annotation(Documentation(info="<html><h4>Initialized roller</h4><p>Helper roller that fixes the initial transport coordinates and prescribes the initial material velocity for wrap-load tests. It also exposes housing torque when support is enabled; it is not a standalone test.</p></html>"));
  end InitializedRoller;

  model WrapBearingLoad "Centrifugal unloading and axial torque routing on a half wrap"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false,g=0,
      bearingDamping=0)
      annotation(Placement(visible = true, transformation(origin = {-120, 160},
        extent = {{-10, -10}, {10, 10}})));
    parameter Modelica.Units.SI.Velocity speeds[2]={0,4};
    InitializedRoller roll[2](each useSupport=true,each useFlange=true,
      initialVelocity=speeds,each entryAngleStart=3*Modelica.Constants.pi/2,
      each exitAngleStart=5*Modelica.Constants.pi/2)
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce inlet[2](each direction=1,each force={0,0,-150})
      annotation(Placement(visible = true, transformation(origin = {-40, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce outlet[2](each direction=-1,each force={0,0,-150})
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive[2](
      w_fixed={speeds[i]/(world.rollRadius+world.web.thickness/2) for i in 1:2})
      annotation(Placement(visible = true, transformation(origin = {-40, -80},
        extent = {{-10, -10}, {10, 10}})));
    final parameter Real lineDensity(unit="kg/m")=
      world.web.density*world.web.thickness*world.web.width/(1+150/world.web.EA);
  equation
    for i in 1:2 loop
      connect(world.frame_b, roll[i].frame_support)
      annotation(Line(visible = true, points = {{-110, 160}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
      connect(drive[i].flange, roll[i].flange_a)
      annotation(Line(visible = true, points = {{-30, -80}, {40, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
      connect(inlet[i].frame_b, roll[i].frame_a)
      annotation(Line(visible = true, points = {{-30, 80}, {30, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
      connect(outlet[i].frame_b, roll[i].frame_b)
      annotation(Line(visible = true, points = {{-30, 0}, {50, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
      assert(Modelica.Math.Vectors.length(roll[i].frame_support.f
        - {0, 0, 2*(150-lineDensity*speeds[i]^2)}) < 1e-5,
        "Half-wrap support must include centrifugal unloading");
      assert(abs(roll[i].housingTorque)<1e-7,"Web torque must not be duplicated on the housing");
      assert(abs(roll[i].rollerVariables.Fcap-2*(150-lineDensity*speeds[i]^2)*
        tanh(world.innerTraction.muAdhesion*Modelica.Constants.pi/2))<1e-6,
        "Centrifugal unloading must reduce available traction");
    end for;
    annotation(experiment(StopTime=0.1,Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -120}, {100, 200}})));
    annotation(Documentation(info="<html><h4>Wrap bearing load</h4><p>Compares half-wrap bearing loads at material speeds of 0 and 4 m/s under equal 150 N tensions. Centrifugal unloading must reduce both support load and traction capacity, and web torque must not be duplicated on the housing.</p></html>"));
  end WrapBearingLoad;

  model FrictionlessTorqueRoute "Tension accelerates web without an extra housing torque"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false,g=0,
      bearingDamping=0, innerTraction(muAdhesion=0,muSliding=0,viscousSlope=0),
      outerTraction(muAdhesion=0,muSliding=0,viscousSlope=0))
      annotation(Placement(visible = true, transformation(origin = {-120, 160},
        extent = {{-10, -10}, {10, 10}})));
    InitializedRoller roll(useSupport=true,useFlange=true,
      entryAngleStart=3*Modelica.Constants.pi/2,exitAngleStart=5*Modelica.Constants.pi/2)
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce inlet(direction=1,force={0,0,-150})
      annotation(Placement(visible = true, transformation(origin = {-40, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce outlet(direction=-1,force={0,0,-151})
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed drive
      annotation(Placement(visible = true, transformation(origin = {-40, -80},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(world.frame_b, roll.frame_support)
      annotation(Line(visible = true, points = {{-110, 160}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(drive.flange, roll.flange_a)
      annotation(Line(visible = true, points = {{-40, -80}, {40, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(inlet.frame_b, roll.frame_a)
      annotation(Line(visible = true, points = {{-30, 80}, {30, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(outlet.frame_b, roll.frame_b)
      annotation(Line(visible = true, points = {{-30, 0}, {50, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(roll.housingTorque)<1e-7 and abs(drive.flange.tau)<1e-7,
      "A frictionless wrap must transfer no axial torque to shaft or housing");
    when terminal() then
      assert(roll.webMomentum>0.05,"The tension difference must accelerate the web");
    end when;
    annotation(experiment(StopTime=0.1,Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -120}, {100, 200}})));
    annotation(Documentation(info="<html><h4>Frictionless torque route</h4><p>Applies a 1 N tension difference across a frictionless half wrap with its shaft fixed. The web must gain momentum while neither the shaft nor the housing receives axial torque.</p></html>"));
  end FrictionlessTorqueRoute;

  model StaticSpanWeight "Independent static support balance includes neighboring span weight"
    extends ClosedLoop(world(lineSpeed=0), bigRoll(useSupport=true),
      smallRoll(radius=0.15,zPosition=0));
    Modelica.Units.SI.Mass shellMass;
    Modelica.Units.SI.Force expectedForce[3];
    Real upperDirection[3];
    Real lowerDirection[3];
  equation
    connect(world.frame_b, bigRoll.frame_support)
      annotation(Line(visible = true, points = {{-100, 70}, {-40, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    shellMass=bigRoll.density*Modelica.Constants.pi*
      (bigRoll.radius^2-(bigRoll.radius-bigRoll.wallThickness)^2)*bigRoll.width;
    upperDirection=(upperSpan.frame_b.frame.r_0-upperSpan.frame_a.frame.r_0)/upperSpan.beltVariables.freeLength;
    lowerDirection=(lowerSpan.frame_b.frame.r_0-lowerSpan.frame_a.frame.r_0)/lowerSpan.beltVariables.freeLength;
    expectedForce=upperSpan.beltVariables.tensionIn*upperDirection-lowerSpan.beltVariables.tensionOut*lowerDirection
      + (shellMass+bigRoll.webMass+(upperSpan.beltVariables.webMass+lowerSpan.beltVariables.webMass)/2)*{0,0,-world.g};
    assert(Modelica.Math.Vectors.length(bigRoll.frame_support.f + expectedForce)<1e-5,
      "Bearing must support shell, wrap and half of each adjacent span");
    annotation(experiment(StopTime=0.1,Tolerance=1e-9));
    annotation(Documentation(info="<html><h4>Static span weight</h4><p>Checks the independently calculated static bearing reaction in a stopped two-roll loop. The supported roll must carry shell and wrap weight plus half the weight of each neighboring free span, together with the span tension forces.</p></html>"));
  end StaticSpanWeight;
  annotation(Documentation(info = "<html>
<h4>Mass conservation checks</h4>
<p>Check material inventory and mechanical load transfer in stretching spans,
moving wraps, closed loops and winding lines. Stopped, delayed-start and reverse
transport cases exercise the same balances; support checks include stored web
momentum, shell weight and the load from neighboring spans.</p>
</html>"));
end MassConservationChecks;
