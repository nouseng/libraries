within Roll2RollDynamicsTest;
package LateralSteeringChecks "Independent local-velocity and steering regressions"
  extends Modelica.Icons.ExamplesPackage;

  model MovingMount "Prescribed translation with fixed orientation"
    extends Modelica.Icons.Example;
    parameter Modelica.Units.SI.Angle yaw=0 "Rotation of the whole scene";
    parameter Modelica.Units.SI.Velocity velocity[3]=zeros(3) "Mount velocity at startup";
    parameter Modelica.Units.SI.Acceleration acceleration[3]=zeros(3) "Mount acceleration";
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Prescribed mounting frame"
      annotation(Placement(visible = true, transformation(origin = {100, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    Connections.root(frame_b.R);
    frame_b.r_0 = velocity*time + acceleration*time^2/2;
    frame_b.R = Modelica.Mechanics.MultiBody.Frames.planarRotation({0,0,1}, yaw, 0);
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -100}, {140, 100}})));
    annotation(Documentation(info = "<html><h4>Moving mount</h4><p>Helper mounting boundary that prescribes translation from initial velocity and constant acceleration while holding orientation at <code>yaw</code>. The steering fixture uses it to check that common scene motion does not create relative lateral slip.</p></html>"));
  end MovingMount;

  model Fixture "Compare reported slip with physical velocities at the entering tangency"
    extends Modelica.Icons.Example;
    // Parameters
    parameter Modelica.Units.SI.Angle sceneYaw=0 "Rigid rotation of roller and loads";
    parameter Modelica.Units.SI.Angle rollYaw=0 "Roll misalignment relative to the loads";
    parameter Integer wrapDirection=1 "Over or under routing";
    parameter Modelica.Units.SI.Velocity speed=2 "Signed drive surface speed";
    parameter Real entrySlope=0 "Entry cross-machine slope";
    parameter Real exitSlope=0 "Exit cross-machine slope";
    parameter Modelica.Units.SI.AngularVelocity tangencyRate=0 "Migration of the imposed pull angles";
    parameter Modelica.Units.SI.Velocity mountVelocity[3]=zeros(3) "Common translational velocity";
    parameter Modelica.Units.SI.Acceleration mountAcceleration[3]=zeros(3) "Common translational acceleration";
    // Variables
    Real entryDirection[3](each unit="1") "World-resolved directed entry pull";
    Real exitDirection[3](each unit="1") "World-resolved directed exit pull";
    Real axis[3](each unit="1") "Physical roller axis in world coordinates";
    Modelica.Units.SI.Velocity materialVelocity[3] "Material velocity at the entering tangency";
    Modelica.Units.SI.Velocity surfaceVelocity[3] "Roll-surface velocity at that tangency";
    Modelica.Units.SI.Velocity expectedSlip "Relative velocity projected along the roller axis";
    // Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, lateralDynamics=true, g=0)
      annotation(Placement(visible = true, transformation(origin = {-120, 200},
        extent = {{-10, -10}, {10, 10}})));
    MovingMount mount(yaw=sceneYaw, velocity=mountVelocity, acceleration=mountAcceleration)
      annotation(Placement(visible = true, transformation(origin = {-40, 120},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Roller roll(
      useSupport=true, useFlange=true, yaw=rollYaw,
      rotationDirection=wrapDirection, fixedInitialWebState=true, fixedInitialAngle=true,
      entryAngleStart=if wrapDirection > 0 then -0.6 else Modelica.Constants.pi + 0.6,
      exitAngleStart=if wrapDirection > 0 then 0.4 else Modelica.Constants.pi - 0.4)
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed=wrapDirection*speed/(roll.radius + roll.beltThickness/2))
      annotation(Placement(visible = true, transformation(origin = {-40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce inlet(direction=1, force=-150*entryDirection)
      annotation(Placement(visible = true, transformation(origin = {-40, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.WebForce outlet(direction=-1, force=150*exitDirection)
      annotation(Placement(visible = true, transformation(origin = {-40, -120},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(mount.frame_b, roll.frame_support)
      annotation(Line(visible = true, points = {{-30, 120}, {40, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(drive.flange, roll.flange_a)
      annotation(Line(visible = true, points = {{-30, 40}, {40, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(inlet.frame_b, roll.frame_a)
      annotation(Line(visible = true, points = {{-30, -40}, {30, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(outlet.frame_b, roll.frame_b)
      annotation(Line(visible = true, points = {{-30, -120}, {50, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    entryDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(mount.frame_b.R,
      Modelica.Math.Vectors.normalize({cos(0.6 + tangencyRate*time), entrySlope,
        wrapDirection*sin(0.6 + tangencyRate*time)}));
    exitDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(mount.frame_b.R,
      Modelica.Math.Vectors.normalize({cos(0.4 - tangencyRate*time), exitSlope,
        -wrapDirection*sin(0.4 - tangencyRate*time)}));
    axis = Modelica.Mechanics.MultiBody.Frames.resolve1(mount.frame_b.R,
      {-sin(rollYaw), cos(rollYaw), 0});
    // Independently reconstruct both vector velocities, including boundary migration.
    materialVelocity = if speed >= 0 then
      der(roll.frame_a.frame.r_0) + entryDirection*der(roll.frame_a.web.s)
      else der(roll.frame_b.frame.r_0) + exitDirection*der(roll.frame_b.web.s);
    surfaceVelocity = der(mount.frame_b.r_0) + cross(axis*drive.w_fixed,
      (if speed >= 0 then roll.frame_a.frame.r_0 else roll.frame_b.frame.r_0)
        - mount.frame_b.r_0);
    expectedSlip = axis*(materialVelocity - surfaceVelocity);
    // With migrating tangencies, check after the intended throughflow has started.
    if abs(tangencyRate) < 1e-12 or time > 0.1 then
      assert(abs(roll.lateralSlip - expectedSlip) < 1e-7,
        "Lateral slip must equal the physical web/surface relative velocity along the roll axis");
    end if;
    assert(roll.rollerVariables.Ft*roll.rollerVariables.slipVelocity - roll.lateralForce*roll.lateralSlip >= -1e-9,
      "Combined friction must oppose relative slip");
    annotation(Documentation(info = "<html><h4>Lateral steering fixture</h4><p>Helper fixture that independently reconstructs material velocity at the entering tangency, including boundary migration, and subtracts rigid-body roll-surface velocity. Reported axial slip must match that projection within 1e-7 m/s after startup for migrating tangencies, and combined friction must remain dissipative. Signed web motion selects the entering end; unequal end slopes distinguish forward and reverse transport without using the contact drift formula.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -160}, {100, 240}})));
  end Fixture;

  model AlignedRotation "Rigid scene rotation and translation cannot create steering"
    extends Modelica.Icons.Example;
    Fixture original
      annotation(Placement(visible = true, transformation(origin = {0, 80},
        extent = {{-10, -10}, {10, 10}})));
    Fixture rotated(sceneYaw=0.35)
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Fixture moving(sceneYaw=0.35, mountVelocity={1,2,3}, mountAcceleration={0.2,-0.3,0.1})
      annotation(Placement(visible = true, transformation(origin = {0, -80},
        extent = {{-10, -10}, {10, 10}})));
  equation
    assert(abs(original.roll.rollerVariables.lateralPosition) < 1e-8
      and abs(rotated.roll.rollerVariables.lateralPosition) < 1e-8
      and abs(moving.roll.rollerVariables.lateralPosition) < 1e-8,
      "Aligned rolls must not drift after a rigid rotation or common mount translation");
    annotation(experiment(StopTime=0.2, Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -120}, {100, 120}})));
    annotation(Documentation(info = "<html><h4>Aligned rotation</h4><p>Compares aligned rolls in the original scene, a rigidly rotated scene and a rotated scene with common translation and acceleration. All lateral positions must remain below 1e-8 m, while each fixture checks physical relative slip and friction dissipation.</p></html>"));
  end AlignedRotation;

  model DirectionAndMigration "Misalignment, wrap sense, reverse transport and migrating tangencies"
    extends Modelica.Icons.Example;
    Fixture forward(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02)
      annotation(Placement(visible = true, transformation(origin = {-40, 160},
        extent = {{-10, -10}, {10, 10}})));
    Fixture rotated(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, sceneYaw=0.35)
      annotation(Placement(visible = true, transformation(origin = {-40, 80},
        extent = {{-10, -10}, {10, 10}})));
    Fixture mirrored(rollYaw=-0.03, entrySlope=-0.01, exitSlope=0.02)
      annotation(Placement(visible = true, transformation(origin = {-40, 0},
        extent = {{-10, -10}, {10, 10}})));
    Fixture reverse(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, speed=-2)
      annotation(Placement(visible = true, transformation(origin = {-40, -80},
        extent = {{-10, -10}, {10, 10}})));
    Fixture under(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, wrapDirection=-1)
      annotation(Placement(visible = true, transformation(origin = {-40, -160},
        extent = {{-10, -10}, {10, 10}})));
    Fixture reverseUnder(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, wrapDirection=-1, speed=-2)
      annotation(Placement(visible = true, transformation(origin = {40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Fixture migrating(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, tangencyRate=0.1,
      mountVelocity={1,2,3})
      annotation(Placement(visible = true, transformation(origin = {40, -40},
        extent = {{-10, -10}, {10, 10}})));
  equation
    assert(abs(forward.roll.rollerVariables.lateralPosition - rotated.roll.rollerVariables.lateralPosition) < 1e-7,
      "Rotating a misaligned scene must preserve roller-relative steering");
    assert(abs(forward.roll.rollerVariables.lateralPosition + mirrored.roll.rollerVariables.lateralPosition) < 1e-7,
      "Mirroring misalignment and pulls must mirror steering");
    assert(abs(forward.roll.rollerVariables.lateralPosition - under.roll.rollerVariables.lateralPosition) < 1e-7,
      "Mirroring over/under routing must preserve axial steering signs");
    annotation(experiment(StopTime=0.5, Tolerance=1e-9));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -200}, {100, 200}})));
    annotation(Documentation(info = "<html><h4>Direction and migration</h4><p>Exercises yaw, rigid scene rotation, mirrored pulls, over and under wraps, reverse transport and moving tangencies. Steering must be invariant under scene rotation and over/under mirroring, reverse sign when yaw and pulls are mirrored, and satisfy each fixture's independent relative-velocity and dissipation checks.</p></html>"));
  end DirectionAndMigration;

  model ReversingSlip "A changing transport sign selects the new entering span"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0)
      annotation(Placement(visible = true, transformation(origin = {-160, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Revolute shaft(
      n={0,1,0}, useAxisFlange=true, animation=false, phi(start=0, fixed=true))
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed spin(w_fixed=20)
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Speed transport(exact=true)
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.ConstantSpeed lateral(v_fixed=0.02)
      annotation(Placement(visible = true, transformation(origin = {-80, -80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebFriction contact(
      radius=0.1, wrapLength=0.1, meanTension=150, wrapAngle=1,
      traction=world.innerTraction, lateral=true,
      spanDirectionA={1,0.1,0}/sqrt(1.01),
      spanDirectionB={1,-0.2,0}/sqrt(1.04))
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Units.SI.Velocity expectedSlip "Independent prescribed axial relative velocity";
  initial equation
    contact.flange_b.s = 0;
    contact.contactFlange.s = 0;
    contact.lateralFlange.s = 0;
  equation
    connect(world.frame_b, shaft.frame_a)
      annotation(Line(visible = true, points = {{-150, 160}, {-10, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(shaft.frame_b, contact.frame_drum)
      annotation(Line(visible = true, points = {{10, 0}, {80, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(spin.flange, shaft.axis)
      annotation(Line(visible = true, points = {{-70, 80}, {0, 10}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(transport.flange, contact.flange_a)
      annotation(Line(visible = true, points = {{-70, 0}, {73.5, 10}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(lateral.flange, contact.lateralFlange)
      annotation(Line(visible = true, points = {{-70, -80}, {80, 10}},
        color = {0, 127, 0}, smooth = Smooth.None));
    transport.v_ref = 2*sin(2*Modelica.Constants.pi*time);
    expectedSlip = 0.02 + transport.v_ref*
      (if time < 0.5 then 0.1/sqrt(1.01) else -0.2/sqrt(1.04));
    assert(abs(contact.lateralSlip - expectedSlip) < 1e-8,
      "Reversal must use the new entering end; shaft spin cannot substitute for web transport");
    assert(contact.longitudinalForce*contact.slipVelocity
      - contact.lateralForce*contact.lateralSlip >= -1e-9,
      "Friction must remain dissipative through stop and reversal");
    annotation(experiment(StopTime=1, Tolerance=1e-9),
      Documentation(info = "<html><h4>Reversing slip</h4><p>Reverses prescribed web transport after 0.5 s while shaft spin remains positive, using unequal span-axis projections to distinguish the entering boundary. Lateral slip must follow the newly entering span within 1e-8 m/s and friction must remain dissipative; the skew contribution vanishes at standstill despite continued drum rotation.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -120}, {120, 200}})));
  end ReversingSlip;
  annotation(uses(Modelica(version="4.1.0")), Documentation(info = "<html>
<h4>Lateral steering checks</h4>
<p>Compare contact slip with the physical velocities at the entering tangency.
The fixtures cover rigid scene motion, yaw, winding direction, moving tangencies
and transport reversal, including the change of entering span at zero speed.</p>
</html>"));
end LateralSteeringChecks;
