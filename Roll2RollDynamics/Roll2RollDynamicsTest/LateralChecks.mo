within Roll2RollDynamicsTest;
package LateralChecks "Lateral motion and skewed-contact regressions"
  extends Modelica.Icons.ExamplesPackage;

  model SkewReachesWebCheck
    "Roll yaw reaches the web width while the tangent follows the imposed pull"
    inner Roll2RollDynamics.WebWorld world;
    Modelica.Blocks.Sources.Constant squareEntryForce[3](k = {-100*cos(0.6),0,-100*sin(0.6)})
      "Known force applied to the square roll's entry tangency frame";
    Modelica.Blocks.Sources.Constant squareExitForce[3](k = {100*cos(0.6),0,-100*sin(0.6)})
      "Known force applied to the square roll's exit tangency frame";
    Modelica.Blocks.Sources.Constant skewedEntryForce[3](k = {-100*cos(0.6),0,-100*sin(0.6)})
      "Known force applied to the skewed roll's entry tangency frame";
    Modelica.Blocks.Sources.Constant skewedExitForce[3](k = {100*cos(0.6),0,-100*sin(0.6)})
      "Known force applied to the skewed roll's exit tangency frame";
    Roll2RollDynamics.Components.WebForce squareEntryLoad(direction=1)
      "External load on the square roll's entry frame";
    Roll2RollDynamics.Components.WebForce squareExitLoad(direction=-1)
      "External load on the square roll's exit frame";
    Roll2RollDynamics.Components.WebForce skewedEntryLoad(direction=1)
      "External load on the skewed roll's entry frame";
    Roll2RollDynamics.Components.WebForce skewedExitLoad(direction=-1)
      "External load on the skewed roll's exit frame";
    Modelica.Mechanics.Rotational.Components.Fixed squareDrive
      "Stationary shaft boundary for the square roll";
    Modelica.Mechanics.Rotational.Components.Fixed skewedDrive
      "Stationary shaft boundary for the skewed roll";
    Roll2RollDynamics.Components.Roller square(
      yaw = 0,
      useSupport = true,
      useFlange = true,
      fixedInitialWebState = true) "Roll with no yaw";
    Roll2RollDynamics.Components.Roller skewed(
      yaw = 0.4,
      useSupport = true,
      useFlange = true,
      fixedInitialWebState = true) "Roll yawed by 0.4 rad";
    // The local x-axis follows the imposed pull. The width is the roll axis
    // projected into the plane normal to that directed span.
    final parameter Real rollAxis[3]={-sin(0.4),cos(0.4),0}
      "Yawed roll axis in world coordinates";
    final parameter Real exitDirection[3]={cos(0.6),0,-sin(0.6)}
      "Prescribed exiting web direction";
    final parameter Real expectedWidth[3]=Modelica.Math.Vectors.normalize(
      rollAxis-exitDirection*(rollAxis*exitDirection))
      "Roll axis projected normal to the exiting span";
    Real squareTilt "Cross-machine tilt of the square roll's exit frame";
    Real skewedTilt "Cross-machine tilt of the yawed roll's exit frame";
  protected
    Real squareTiltResolved[3]
      "Square roll's exit-frame cross-machine axis resolved into world coordinates";
    Real skewedTiltResolved[3]
      "Skewed roll's exit-frame cross-machine axis resolved into world coordinates";
  equation
    connect(world.frame_b, square.frame_support);
    connect(world.frame_b, skewed.frame_support);
    connect(squareEntryForce.y, squareEntryLoad.force);
    connect(squareEntryLoad.frame_b, square.frame_a);
    connect(squareExitForce.y, squareExitLoad.force);
    connect(squareExitLoad.frame_b, square.frame_b);
    connect(skewedEntryForce.y, skewedEntryLoad.force);
    connect(skewedEntryLoad.frame_b, skewed.frame_a);
    connect(skewedExitForce.y, skewedExitLoad.force);
    connect(skewedExitLoad.frame_b, skewed.frame_b);
    connect(squareDrive.flange, square.flange_a);
    connect(skewedDrive.flange, skewed.flange_a);
    squareTiltResolved = Modelica.Mechanics.MultiBody.Frames.resolve1(
      square.frame_b.frame.R, {0, 1, 0});
    skewedTiltResolved = Modelica.Mechanics.MultiBody.Frames.resolve1(
      skewed.frame_b.frame.R, {0, 1, 0});
    squareTilt = squareTiltResolved*squareExitForce.k/100;
    skewedTilt = skewedTiltResolved*skewedExitForce.k/100;
    assert(abs(squareTilt) < 1e-9,
      "A square roll must present an untilted tangency frame");
    assert(abs(skewedTilt) < 1e-9,
      "The yawed roll's transverse frame axis must remain normal to the pull");
    assert(abs(square.frame_a.frame.R.T[1,2]) < 1e-9
        and abs(skewed.frame_a.frame.R.T[1,2]) < 1e-9,
      "The imposed in-plane web direction must have zero world cross-machine skew");
    assert(Modelica.Math.Vectors.length(skewedTiltResolved-expectedWidth)<1e-9,
      "Roll yaw must reach the transverse web width direction");
  end SkewReachesWebCheck;

  model SpanStringForceCheck
    "A laterally displaced span pulls its ends together through its frames"
    extends Modelica.Icons.Example;

    inner Roll2RollDynamics.WebWorld world(lateralDynamics = true)
      "Line defaults, with out-of-plane dynamics on";

    SpanForceBoundary anchorA(position = {0, 0, 0})
      "Upstream span end, on centre";
    SpanForceBoundary anchorB(position = {1, 0.01, 0})
      "Downstream span end, offset 0.01 m across the machine";

    Roll2RollDynamics.Components.Belt span(tensionNominal = 150)
      "Span under test";

    final parameter Modelica.Units.SI.Length spanLength =
      sqrt(1^2 + 0.01^2 + Roll2RollDynamics.Utilities.Types.lengthTol^2)
      "Span length with the lateral offset, in the softened form Belt uses";

    output Real skew(unit = "1") = span.beltVariables.spanSkewSin
      "Sine of the span skew from geometry";
    Modelica.Units.SI.Force forceA[3] "Entry force resolved in world coordinates";
    Modelica.Units.SI.Force forceB[3] "Exit force resolved in world coordinates";

  equation
    connect(anchorA.port, span.frame_a);
    connect(anchorB.port, span.frame_b);
    forceA = Modelica.Mechanics.MultiBody.Frames.resolve1(
      span.frame_a.frame.R, span.frame_a.frame.f);
    forceB = Modelica.Mechanics.MultiBody.Frames.resolve1(
      span.frame_b.frame.R, span.frame_b.frame.f);

    // The span is 1 m long with a 0.01 m offset, so e[2] is 0.01/L. The frames
    // must carry that as a sideways pull of T*e[2], and as an exact reaction
    // pair, without anything having written a lateral force by hand.
    when terminal() then
      assert(abs(skew - 0.01/spanLength) < 1e-9,
        "Span skew must be the lateral offset over the span length");
      assert(abs(forceA[2] + forceB[2]) < 1e-9,
        "A stationary span must carry no net lateral force");
      assert(abs(forceA[2] + span.beltVariables.tensionIn*skew) < 1e-6,
        "The span must pull its upstream end toward the other by T times the skew");
    end when;

    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8),
      Documentation(info = "<html>
  <p>Two anchors hold a span with one end offset across the machine. The string
  force is not written anywhere: it is what the tension along the three
  dimensional span direction already does.</p>
  </html>"));
  end SpanStringForceCheck;

  model SpanForceBoundary
    "Closed stationary endpoint with free in-plane tangency"
    parameter Modelica.Units.SI.Position position[3]=zeros(3) "Endpoint position";
    Modelica.Units.SI.Angle yaw(start=0) "Free tangent yaw";
    Modelica.Units.SI.Angle skew(start=0) "Free tangent skew";
    Roll2RollDynamics.Utilities.Interfaces.RollPort port "Closed span endpoint";
  equation
    Connections.root(port.frame.R);
    port.frame.r_0=position;
    port.web.s=0;
    skew=Modelica.Math.atan2(port.spanDirection[2],
      cos(yaw)*port.spanDirection[1]-sin(yaw)*port.spanDirection[3]);
    port.frame.R=Modelica.Mechanics.MultiBody.Frames.axesRotations(
      {2,3,1},{yaw,skew,0},{der(yaw),der(skew),0});
  end SpanForceBoundary;

  model RollLateralFrictionCheck
    "A yawed roll drags the web across the machine against a restraint"
    inner Roll2RollDynamics.WebWorld world(lateralDynamics = true)
      "Line defaults, with out-of-plane dynamics on";
    parameter Modelica.Units.SI.TranslationalSpringConstant spanStiffness = 2000
      "String stiffness of the span standing in for whatever holds the web";
    Roll2RollDynamics.Components.Roller roll(
      useSupport = true,
      useFlange = true,
      yaw = 0.02,
      fixedInitialWebState = true,
      fixedInitialAngle = true) "Roll yawed by 0.02 rad";
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed = world.lineSpeed/world.rollRadius) "Surface speed master";
    Modelica.Blocks.Sources.Constant entryLoad[3](
      k = {-150*cos(0.6), 0, -150*sin(0.6)}) "Entry tension, angled below the roll";
    Modelica.Blocks.Sources.RealExpression exitLoad[3](
      y = {150*cos(0.6), -spanStiffness*roll.rollerVariables.lateralPosition, -150*sin(0.6)})
      "Exit tension, angled below the roll, pulling the web back the way a skewed span would";
    Roll2RollDynamics.Components.WebForce entryForce(direction=1)
      "Applies the entry tension to the roll";
    Roll2RollDynamics.Components.WebForce exitForce(direction=-1)
      "Applies the exit tension to the roll";
    output Modelica.Units.SI.Position settledLateral = roll.rollerVariables.lateralPosition
      "Lateral position, read from the script once settled";
  equation
    connect(world.frame_b, roll.frame_support);
    connect(drive.flange, roll.flange_a);
    connect(entryLoad.y, entryForce.force);
    connect(exitLoad.y, exitForce.force);
    connect(entryForce.frame_b, roll.frame_a);
    connect(exitForce.frame_b, roll.frame_b);
    // The web is dragged toward the side the roll is yawed to and settles where
    // the exit load's lateral pull balances the friction. Read settledLateral
    // once settled; it cannot be asserted, because at t = 0 the web has not
    // moved yet.
    when terminal() then
      assert(roll.rollerVariables.lateralPosition > 0.005,
        "A positively yawed roll must have dragged the web to positive lateral position");
      // The restraint reaches the roll through its exit frame, so the joint
      // carries it to the support and nothing puts it there by hand. Once the
      // web has settled the restraint equals the friction, so the support load
      // must match the friction reaction to within 1% and is nowhere near zero.
      assert(abs(roll.frame_support.f[2] - roll.lateralForce) <
          0.01*max(abs(roll.lateralForce), 1e-6),
        "Roll support must carry the lateral friction reaction");
    end when;
    annotation(
      experiment(StartTime = 0, StopTime = 10, Interval = 0.01),
      Documentation(info = "<html>
  <p>A yawed roll drags the web across the machine by lateral friction, and the
  exit load carries a lateral term standing in for the spans that would otherwise
  hold it. The web settles where that pull balances the friction force.</p>
  <p>The restraint is applied to <code>frame_b</code>, not to a lateral
  connector, because the roll has none. It therefore loads the cross-machine
  joint the way a span would, and the support reaction is whatever the joint
  carries rather than anything written by hand.</p>
  <p>A restraining pull is not a span. It resists displacement without changing
  the angle the web arrives at, so this roll never reaches normal entry and stays
  on the sliding branch, with the settled slip equal to minus the surface drift.
  A roll fed by a real span settles differently, at the offset that skews the
  span to match the yaw; <code>Roll2RollDynamics.Examples.SheltonSteering</code> shows that
  case.</p>
  <p>The settled-position assertion needs the full 10 s run declared by this
  experiment annotation. A wrong friction sign does not show up as a mirrored
  settle to the opposite side; it is positive feedback, so it shows up as a
  growing oscillation instead of a steady offset, and at 5 s the sign of that
  oscillation is not reliable. 10 s is long enough for the correct
  implementation to be settled and for a wrong sign to have diverged
  negative.</p>
  </html>"));
  end RollLateralFrictionCheck;

  model WebFrictionCircleCheck
    "Cross-machine grip falls off when the machine direction saturates"
    extends Modelica.Icons.Example;

    parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters traction(
      vAdhesion = 0.01, vSlide = 0.1, muAdhesion = 0.4, muSliding = 0.3)
      "Traction characteristic under test";

    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      "Shared defaults and multibody root";
    Modelica.Mechanics.MultiBody.Parts.Fixed surfaceMount(animation = false)
      "Roll surface held still";

    Roll2RollDynamics.Utilities.Parts.WebFriction contact(
      radius = 0.1,
      rotationDirection = 1,
      traction = traction,
      normalForce = 100,
      lateral = true,
      referenceDensity=0.01, meanTension=150,
      lateralDrift = 0)
      "Contact patch resolving both channels";

    Modelica.Mechanics.Translational.Sources.ConstantSpeed webDrive(v_fixed = 0.5)
      "Web driven fast enough to saturate the machine direction";
    Modelica.Mechanics.Translational.Sources.ConstantSpeed lateralDrive(v_fixed = 0.5)
      "Cross-machine web motion, at the same slip speed as the machine direction";

    output Real muMachine = contact.mu "Signed machine-direction coefficient";
    output Modelica.Units.SI.Force lateralForce = -contact.lateralForce
      "Cross-machine friction force";

  equation
    connect(webDrive.flange, contact.flange_a);
    connect(surfaceMount.frame_b, contact.frame_drum);
    connect(lateralDrive.flange, contact.lateralFlange);

    // Both channels slip at 0.5 m/s, well past vSlide, so the sliding
    // coefficient 0.3 is shared between them at 45 degrees: each channel sees
    // 0.3/sqrt(2) = 0.2121. Without a friction circle the machine direction
    // would still show the full 0.3.
    when terminal() then
      assert(abs(muMachine - 0.3/sqrt(2)) < 0.01,
        "Machine-direction grip must be reduced by the cross-machine slip sharing the circle");
      assert(abs(lateralForce - 100*0.3/sqrt(2)) < 1.0,
        "Cross-machine friction must carry the other half of the circle");
    end when;

    annotation(experiment(StopTime = 2, Tolerance = 1e-8),
      Documentation(info = "<html>
  <p>Drives the machine direction and the cross-machine direction at equal slip
  speeds, both well into sliding. A friction circle shares one saturated
  coefficient between the two channels rather than giving each the full value.</p>
  </html>"));
  end WebFrictionCircleCheck;

  annotation(Documentation(info = "<html><h4>Lateral checks</h4>
<p>Regression scenarios for lateral force, roller yaw and skewed nip contact. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end LateralChecks;
