within Roll2RollDynamicsTest;
package LateralSteeringChecks "Independent local-velocity and steering regressions"
  extends Modelica.Icons.ExamplesPackage;

  model MovingMount "Prescribed translation with fixed orientation"
    parameter Modelica.Units.SI.Angle yaw=0 "Rotation of the whole scene";
    parameter Modelica.Units.SI.Velocity velocity[3]=zeros(3) "Mount velocity at startup";
    parameter Modelica.Units.SI.Acceleration acceleration[3]=zeros(3) "Mount acceleration";
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Prescribed mounting frame";
  equation
    Connections.root(frame_b.R);
    frame_b.r_0 = velocity*time + acceleration*time^2/2;
    frame_b.R = Modelica.Mechanics.MultiBody.Frames.planarRotation({0,0,1}, yaw, 0);
  end MovingMount;

  model Fixture "Compare reported slip with physical velocities at the entering tangency"
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
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, lateralDynamics=true, g=0);
    MovingMount mount(yaw=sceneYaw, velocity=mountVelocity, acceleration=mountAcceleration);
    Roll2RollDynamics.Components.Roller roll(
      useSupport=true, useFlange=true, yaw=rollYaw,
      rotationDirection=wrapDirection, fixedInitialWebState=true, fixedInitialAngle=true,
      entryAngleStart=if wrapDirection > 0 then -0.6 else Modelica.Constants.pi + 0.6,
      exitAngleStart=if wrapDirection > 0 then 0.4 else Modelica.Constants.pi - 0.4);
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed=wrapDirection*speed/(roll.radius + roll.beltThickness/2));
    Roll2RollDynamics.Components.WebForce inlet(direction=1, force=-150*entryDirection);
    Roll2RollDynamics.Components.WebForce outlet(direction=-1, force=150*exitDirection);
  equation
    connect(mount.frame_b, roll.frame_support);
    connect(drive.flange, roll.flange_a);
    connect(inlet.frame_b, roll.frame_a);
    connect(outlet.frame_b, roll.frame_b);
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
    annotation(Documentation(info="<html><p>The expected slip is computed from
    the velocities of the actual tangency frame and material crossing it, minus
    rigid-body surface velocity. The reference does not use the contact drift
    formula. Signed drive motion independently selects the entering end after
    startup; unequal end slopes distinguish forward and reverse selection.</p></html>"));
  end Fixture;

  model AlignedRotation "Rigid scene rotation and translation cannot create steering"
    Fixture original;
    Fixture rotated(sceneYaw=0.35);
    Fixture moving(sceneYaw=0.35, mountVelocity={1,2,3}, mountAcceleration={0.2,-0.3,0.1});
  equation
    assert(abs(original.roll.rollerVariables.lateralPosition) < 1e-8
      and abs(rotated.roll.rollerVariables.lateralPosition) < 1e-8
      and abs(moving.roll.rollerVariables.lateralPosition) < 1e-8,
      "Aligned rolls must not drift after a rigid rotation or common mount translation");
    annotation(experiment(StopTime=0.2, Tolerance=1e-9));
  end AlignedRotation;

  model DirectionAndMigration "Misalignment, wrap sense, reverse transport and migrating tangencies"
    Fixture forward(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02);
    Fixture rotated(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, sceneYaw=0.35);
    Fixture mirrored(rollYaw=-0.03, entrySlope=-0.01, exitSlope=0.02);
    Fixture reverse(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, speed=-2);
    Fixture under(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, wrapDirection=-1);
    Fixture reverseUnder(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, wrapDirection=-1, speed=-2);
    Fixture migrating(rollYaw=0.03, entrySlope=0.01, exitSlope=-0.02, tangencyRate=0.1,
      mountVelocity={1,2,3});
  equation
    assert(abs(forward.roll.rollerVariables.lateralPosition - rotated.roll.rollerVariables.lateralPosition) < 1e-7,
      "Rotating a misaligned scene must preserve roller-relative steering");
    assert(abs(forward.roll.rollerVariables.lateralPosition + mirrored.roll.rollerVariables.lateralPosition) < 1e-7,
      "Mirroring misalignment and pulls must mirror steering");
    assert(abs(forward.roll.rollerVariables.lateralPosition - under.roll.rollerVariables.lateralPosition) < 1e-7,
      "Mirroring over/under routing must preserve axial steering signs");
    annotation(experiment(StopTime=0.5, Tolerance=1e-9));
  end DirectionAndMigration;

  model ReversingSlip "A changing transport sign selects the new entering span"
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0);
    Modelica.Mechanics.MultiBody.Joints.Revolute shaft(
      n={0,1,0}, useAxisFlange=true, animation=false, phi(start=0, fixed=true));
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed spin(w_fixed=20);
    Modelica.Mechanics.Translational.Sources.Speed transport(exact=true);
    Modelica.Mechanics.Translational.Sources.ConstantSpeed lateral(v_fixed=0.02);
    Roll2RollDynamics.Utilities.Parts.WebFriction contact(
      radius=0.1, wrapLength=0.1, meanTension=150, wrapAngle=1,
      traction=world.innerTraction, lateral=true,
      spanDirectionA={1,0.1,0}/sqrt(1.01),
      spanDirectionB={1,-0.2,0}/sqrt(1.04));
    Modelica.Units.SI.Velocity expectedSlip "Independent prescribed axial relative velocity";
  initial equation
    contact.flange_b.s = 0;
    contact.contactFlange.s = 0;
    contact.lateralFlange.s = 0;
  equation
    connect(world.frame_b, shaft.frame_a);
    connect(shaft.frame_b, contact.frame_drum);
    connect(spin.flange, shaft.axis);
    connect(transport.flange, contact.flange_a);
    connect(lateral.flange, contact.lateralFlange);
    transport.v_ref = 2*sin(2*Modelica.Constants.pi*time);
    expectedSlip = 0.02 + transport.v_ref*
      (if time < 0.5 then 0.1/sqrt(1.01) else -0.2/sqrt(1.04));
    assert(abs(contact.lateralSlip - expectedSlip) < 1e-8,
      "Reversal must use the new entering end; shaft spin cannot substitute for web transport");
    assert(contact.longitudinalForce*contact.slipVelocity
      - contact.lateralForce*contact.lateralSlip >= -1e-9,
      "Friction must remain dissipative through stop and reversal");
    annotation(experiment(StopTime=1, Tolerance=1e-9),
      Documentation(info="<html><p>Prescribed web transport reverses after half
      a second while shaft spin remains positive. Unequal span-axis projections
      distinguish which boundary enters; at standstill the skew contribution
      vanishes even though the drum keeps spinning.</p></html>"));
  end ReversingSlip;
  annotation(uses(Modelica(version="4.1.0")), Documentation(info = "<html>
<h4>Lateral steering checks</h4>
<p>Compare contact slip with the physical velocities at the entering tangency.
The fixtures cover rigid scene motion, yaw, winding direction, moving tangencies
and transport reversal, including the change of entering span at zero speed.</p>
</html>"));
end LateralSteeringChecks;
