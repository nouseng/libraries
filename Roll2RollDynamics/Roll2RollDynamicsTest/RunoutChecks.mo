within Roll2RollDynamicsTest;
package RunoutChecks "Roller drum runout regressions"
  extends Modelica.Icons.ExamplesPackage;

  partial model RunoutIdlerBase
    "Eccentric idler swings the entry span by the runout once per turn and does not steer"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      idler(yaw = 0, runout = runoutMagnitude,
        runoutPhase = runoutAngle, variableRunout = runtimeRunout),
      world(enableAnimation = false));
    //Parameters
    parameter Modelica.Units.SI.Length runoutMagnitude = 0.3e-3
      "Radial runout of the idler drum against its bearing axis";
    parameter Modelica.Units.SI.Angle runoutAngle = 0
      "Direction of the runout high point at zero shaft angle";
    parameter Boolean runtimeRunout = false
      "= true, keep runout settable in an exported FMU";
    parameter Modelica.Units.SI.Force minimumTensionRipple = 2
      "Entry-span ripple a runout of this size must produce";
    //Variables
    // The tangency geometry fixes the swing: the drum carries its tangency
    // line, so displacing its centre by the runout changes the free length of
    // the adjacent span by the same amount, once per turn.
    final parameter Modelica.Units.SI.Length expectedSpanSwing = 2*runoutMagnitude
      "Peak-to-peak free-span swing a first-order tangency geometry predicts";
    Real orbitCosine(unit = "1") = Modelica.Math.cos(idler.rollerVariables.angle)
      "Once-per-turn reference aligned with the runout high point";
    Real orbitSine(unit = "1") = Modelica.Math.sin(idler.rollerVariables.angle)
      "Once-per-turn reference a quarter turn behind the high point";
    discrete Modelica.Units.SI.Force tensionMax(start = -1e9, fixed = true)
      "Highest entry-span tension after the line has settled";
    discrete Modelica.Units.SI.Force tensionMin(start = 1e9, fixed = true)
      "Lowest entry-span tension after the line has settled";
    discrete Modelica.Units.SI.Length spanLengthMax(start = -1e9, fixed = true)
      "Longest entry span after the line has settled";
    discrete Modelica.Units.SI.Length spanLengthMin(start = 1e9, fixed = true)
      "Shortest entry span after the line has settled";
    discrete Modelica.Units.SI.Position lateralMax(start = -1e9, fixed = true)
      "Largest web offset of the settled window";
    discrete Modelica.Units.SI.Position lateralMin(start = 1e9, fixed = true)
      "Smallest web offset of the settled window";
    discrete Modelica.Units.SI.Position lateralSum(start = 0, fixed = true)
      "Accumulated web offset of the settled window";
    // Single-pass accumulators of the settled window. They project the span
    // swing onto the two once-per-turn references, so the check can tell a
    // clean orbit harmonic from a ripple carrying other content.
    discrete Modelica.Units.SI.Length lengthSum(start = 0, fixed = true)
      "Sum of sampled span lengths in the settled window";
    discrete Real cosineSum(unit = "1", start = 0, fixed = true)
      "Sum of sampled in-phase orbit references";
    discrete Real sineSum(unit = "1", start = 0, fixed = true)
      "Sum of sampled quadrature orbit references";
    discrete Modelica.Units.SI.Length lengthCosineSum(start = 0, fixed = true)
      "Sum of sampled span lengths weighted by the in-phase orbit reference";
    discrete Modelica.Units.SI.Length lengthSineSum(start = 0, fixed = true)
      "Sum of sampled span lengths weighted by the quadrature orbit reference";
    discrete Integer sampleCount(start = 0, fixed = true)
      "Number of samples accumulated in the settled window";
    Modelica.Units.SI.Length spanHarmonicAmplitude
      "Amplitude of the once-per-turn component of the span swing";
    Modelica.Units.SI.Length spanHarmonicCosine "Half the in-phase span-swing harmonic";
    Modelica.Units.SI.Length spanHarmonicSine "Half the quadrature span-swing harmonic";
  equation
    spanHarmonicCosine = lengthCosineSum/max(sampleCount, 1)
      - (lengthSum/max(sampleCount, 1))*(cosineSum/max(sampleCount, 1));
    spanHarmonicSine = lengthSineSum/max(sampleCount, 1)
      - (lengthSum/max(sampleCount, 1))*(sineSum/max(sampleCount, 1));
    // A sinusoid projects half its amplitude onto its own unit basis, so the
    // once-per-turn amplitude is twice the projection below.
    spanHarmonicAmplitude = 2*sqrt(max(spanHarmonicCosine^2
      + spanHarmonicSine^2, 0));
    when sample(15, 1e-3) then
      tensionMax = max(pre(tensionMax), idlerSpanTension);
      tensionMin = min(pre(tensionMin), idlerSpanTension);
      spanLengthMax = max(pre(spanLengthMax), idlerSpanLength);
      spanLengthMin = min(pre(spanLengthMin), idlerSpanLength);
      lateralMax = max(pre(lateralMax), idlerLateralOffset);
      lateralMin = min(pre(lateralMin), idlerLateralOffset);
      lateralSum = pre(lateralSum) + idlerLateralOffset;
      lengthSum = pre(lengthSum) + idlerSpanLength;
      cosineSum = pre(cosineSum) + orbitCosine;
      sineSum = pre(sineSum) + orbitSine;
      lengthCosineSum = pre(lengthCosineSum) + idlerSpanLength*orbitCosine;
      lengthSineSum = pre(lengthSineSum) + idlerSpanLength*orbitSine;
      sampleCount = pre(sampleCount) + 1;
    end when;
    when terminal() then
      // The free span swings by twice the runout, and by nothing at all for a
      // true drum, which leaves the drum centre fixed.
      assert(abs(spanLengthMax - spanLengthMin - expectedSpanSwing)
        < 0.15*expectedSpanSwing + 1e-6,
        "An eccentric idler must swing the entry span by twice the runout");
      // The tension ripple is the elastic response of that swing.
      assert(runoutMagnitude <= 0
        or tensionMax - tensionMin > minimumTensionRipple,
        "An eccentric idler must ripple the entry span tension");
      assert(runoutMagnitude > 0 or tensionMax - tensionMin < 0.05,
        "A true drum must not ripple the span tension");
      // The swing is the drum orbit, so it is a single harmonic at the shaft
      // turning rate: its once-per-turn projection must account for the whole
      // peak-to-peak swing, with no other content in it.
      assert(runoutMagnitude <= 0 or abs(spanHarmonicAmplitude
        - 0.5*(spanLengthMax - spanLengthMin))
        < 0.05*(spanLengthMax - spanLengthMin) + 1e-9,
        "The span swing must be one clean once-per-turn harmonic");
      // The orbit lies in the plane of the spans, so it carries no
      // cross-machine component: it neither walks the web nor rocks it.
      assert(abs(lateralSum/sampleCount) < 1e-4,
        "Runout must not walk the web");
      assert(runoutMagnitude <= 0 or lateralMax - lateralMin < 1e-4,
        "An in-plane drum orbit must not move the web across the machine at all");
      // Normal entry is measured against the drum axis. An in-plane orbit
      // leaves it untouched, and so does a true drum.
      assert(abs(idlerSpanAxisSkew) < 2e-4,
        "An in-plane drum orbit must keep normal entry at the idler");
    end when;
  end RunoutIdlerBase;

  model RunoutIdler "Runout folded in at translation"
    extends RunoutIdlerBase;
  end RunoutIdler;

  model RunoutIdlerVariable "Runout kept as a run-time parameter"
    extends RunoutIdlerBase(runtimeRunout = true);
  end RunoutIdlerVariable;

  model RunoutIdlerOverride
    "Translated with zero runout; the runout is set with -override at run time"
    extends RunoutIdlerBase(runtimeRunout = true, runoutMagnitude = 0);
  end RunoutIdlerOverride;

  model RunoutIdlerHalf "Half the runout, half the swing and half the ripple"
    extends RunoutIdlerBase(runoutMagnitude = 0.15e-3, minimumTensionRipple = 1.2);
  end RunoutIdlerHalf;

  model RunoutIdlerQuarter "A quarter of the runout, a quarter of the swing"
    extends RunoutIdlerBase(runoutMagnitude = 0.075e-3, minimumTensionRipple = 0.8);
  end RunoutIdlerQuarter;

  model RunoutIdlerPhase "The runout high point turned by 0.8 rad"
    extends RunoutIdlerBase(runoutAngle = 0.8);
  end RunoutIdlerPhase;

  model AlignedIdler "A true drum swings and ripples nothing"
    extends RunoutIdlerBase(runoutMagnitude = 0, minimumTensionRipple = 0);
  end AlignedIdler;

  partial model LoopRunoutBase
    "An eccentric loop idler ripples every span and conserves the web"
    extends Roll2RollDynamics.Examples.ThreeRollWebLoop(
      world(enableAnimation = false));
    //Parameters
    final parameter Modelica.Units.SI.Mass initialWebMass(fixed = false)
      "Loop web inventory at initialization";
    //Variables
    Modelica.Units.SI.Mass totalWebMass =
      risingSpan.beltVariables.webMass + fallingSpan.beltVariables.webMass
      + returnSpan.beltVariables.webMass + driveRoll.webMass
      + topRoll.webMass + rightRoll.webMass
      "Complete closed-loop web inventory";
    Real massBalanceError(unit = "kg") = totalWebMass - initialWebMass
      "Closed-loop conservation residual, expected to stay zero";
    Modelica.Units.SI.Force spanTension[3] =
      {risingSpan.tension, fallingSpan.tension, returnSpan.tension}
      "Tensions of the rising, falling and return spans";
    discrete Modelica.Units.SI.Force tensionMax[3](
      start = {-1e9, -1e9, -1e9}, each fixed = true)
      "Highest settled tension of each span";
    discrete Modelica.Units.SI.Force tensionMin[3](
      start = {1e9, 1e9, 1e9}, each fixed = true)
      "Lowest settled tension of each span";
  initial equation
    initialWebMass = totalWebMass;
  equation
    when sample(23, 1e-3) then
      for i in 1:3 loop
        tensionMax[i] = max(pre(tensionMax[i]), spanTension[i]);
        tensionMin[i] = min(pre(tensionMin[i]), spanTension[i]);
      end for;
    end when;
    when terminal() then
      // The loop is closed and carries no winder, so an eccentric idler may
      // only redistribute web between the spans, never create or lose it. The
      // loop holds 9.7 kg, and the bound is one part per million of it, above
      // the residual the 1e-6 solver tolerance itself leaves behind.
      assert(abs(massBalanceError) < 1e-6*totalWebMass,
        "An eccentric loop idler must preserve the closed-loop web mass");
      // Every span ripples, including the return span, which never touches the
      // eccentric idler: the loop is speed-locked, so the ripple of the two
      // touched spans is shared round the whole path.
      for i in 1:3 loop
        assert(tensionMax[i] - tensionMin[i] > 0.5,
          "Every span of the closed loop must ripple when an idler is eccentric");
      end for;
    end when;
  end LoopRunoutBase;

  model LoopRunoutTop "Eccentric top idler in the closed loop, for the ThreeRollWebLoop figure"
    extends LoopRunoutBase(topRoll(runout = 0.3e-3));
  end LoopRunoutTop;

  model LoopRunoutBoth "Both idlers of the closed loop eccentric, at two turning rates"
    extends LoopRunoutBase(topRoll(runout = 0.3e-3),
      rightRoll(runout = 0.2e-3, runoutPhase = 1));
  end LoopRunoutBoth;

  model DrivenRunout
    "The eccentric roll is the driven one, held at speed by a PI loop, as on Branca's S-wrap"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false);
    parameter Modelica.Units.SI.Length runoutMagnitude = 0.3e-3
      "Radial runout of the driven roll";
    parameter Modelica.Units.SI.AngularVelocity speedSetpoint =
      world.lineSpeed/(driveRoll.radius + driveRoll.beltThickness/2)
      "Shaft speed the loop asks for";
    Roll2RollDynamics.Components.Roller driveRoll(radius = 0.15, useFlange = true,
      fixedInitialAngle = true, fixedInitialWebState = true, fixedInitialSpeed = true,
      xPosition = 0, zPosition = 0, rotationDirection = 1,
      entryAngleStart = -3.14, exitAngleStart = -1.01, runout = runoutMagnitude)
      "Largest roll, driven and eccentric";
    Roll2RollDynamics.Components.Roller topRoll(radius = 0.12, fixedInitialAngle = true,
      fixedInitialWebState = true, fixedInitialSpeed = true, xPosition = 0.5, zPosition = 0.8,
      rotationDirection = 1, entryAngleStart = -1.01, exitAngleStart = 1.01);
    Roll2RollDynamics.Components.Roller rightRoll(radius = 0.09, fixedInitialAngle = true,
      fixedInitialWebState = true, fixedInitialSpeed = true, xPosition = 1.0, zPosition = 0,
      rotationDirection = 1, entryAngleStart = 1.01, exitAngleStart = 3.14);
    Roll2RollDynamics.Components.Belt risingSpan;
    Roll2RollDynamics.Components.Belt fallingSpan;
    Roll2RollDynamics.Components.Belt returnSpan;
    // Speed loop: the shaft keeps its degree of freedom, so the orbit's
    // torque and the span tensions feed back into the roll speed.
    Modelica.Mechanics.Rotational.Sources.Torque drive;
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor;
    Modelica.Blocks.Sources.Constant setpoint(k = speedSetpoint);
    Modelica.Blocks.Math.Feedback speedError;
    Modelica.Blocks.Continuous.PI speedController(k = 20, T = 0.1, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 0);
    Modelica.Units.SI.Angle driveAngle = driveRoll.rollerVariables.angle
      "Shaft angle of the eccentric driven roll";
    Modelica.Units.SI.Torque driveTorque = drive.tau "Torque the speed loop applies";
  equation
    connect(setpoint.y, speedError.u1);
    connect(speedSensor.w, speedError.u2);
    connect(speedError.y, speedController.u);
    connect(speedController.y, drive.tau);
    connect(drive.flange, driveRoll.flange_a);
    connect(speedSensor.flange, driveRoll.flange_a);
    connect(driveRoll.frame_b, risingSpan.frame_a);
    connect(risingSpan.frame_b, topRoll.frame_a);
    connect(topRoll.frame_b, fallingSpan.frame_a);
    connect(fallingSpan.frame_b, rightRoll.frame_a);
    connect(rightRoll.frame_b, returnSpan.frame_a);
    connect(returnSpan.frame_b, driveRoll.frame_a);
  end DrivenRunout;

  partial model NipRunoutBase
    "A nip drum with radial runout, held on the loaded roller"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(runout = nipRunout),
      world(enableAnimation = false));
    //Parameters
    parameter Modelica.Units.SI.Length nipRunout = 0.3e-3
      "Radial runout of the nip drum against its bearing axis";
    //Variables
    final parameter Modelica.Units.SI.Length expectedSwing = 2*nipRunout
      "Peak-to-peak middle-penetration swing the drum orbit predicts";
    Modelica.Units.SI.Force coverResidual = nip.nipContact.normalForce
      - nip.coverStiffness*nip.nipContact.penetration
      "Held load minus the linear cover law";
    Modelica.Units.SI.Force expectedLoadSwing = nip.coverStiffness*(
      penetrationMax - penetrationMin)
      "Swing the cover law gives over the penetration range it saw";
    discrete Modelica.Units.SI.Force loadMax(start = -1e9, fixed = true)
      "Highest held load of the dwell";
    discrete Modelica.Units.SI.Force loadMin(start = 1e9, fixed = true)
      "Lowest held load of the dwell";
    discrete Modelica.Units.SI.Length penetrationMax(start = -1e9, fixed = true)
      "Deepest middle penetration of the dwell";
    discrete Modelica.Units.SI.Length penetrationMin(start = 1e9, fixed = true)
      "Shallowest middle penetration of the dwell";
    discrete Modelica.Units.SI.Length centreMax(start = -1e9, fixed = true)
      "Furthest load centre of the dwell, toward the nip side";
    discrete Modelica.Units.SI.Length centreMin(start = 1e9, fixed = true)
      "Furthest load centre of the dwell, toward the other side";
  equation
    // The stroke holds the middle penetration between 2 s and 5 s and the face
    // stays fully in contact, so the cover law alone sets the load: the
    // footprint integral is linear in penetration,
    // with the damping term a percent of it.
    assert(not (time > 3 and time < 5) or abs(coverResidual)
      < 0.02*nip.nipContact.normalForce,
      "A held nip must load its cover by the linear cover law");
    // The stroke holds the nip from 2 s to 5 s; outside that dwell the drum is
    // either closing or releasing, so only the dwell is measured here.
    when sample(3, 1e-4) and time < 5 then
      loadMax = max(pre(loadMax), nip.nipContact.normalForce);
      loadMin = min(pre(loadMin), nip.nipContact.normalForce);
      penetrationMax = max(pre(penetrationMax), nip.nipContact.penetration);
      penetrationMin = min(pre(penetrationMin), nip.nipContact.penetration);
      centreMax = max(pre(centreMax), nip.nipContact.loadCentre);
      centreMin = min(pre(centreMin), nip.nipContact.loadCentre);
    end when;
  end NipRunoutBase;

  model NipRunoutLoad "An eccentric nip drum swings the held load with its penetration"
    extends NipRunoutBase;
  equation
    when terminal() then
      // The drum orbit carries the nip centre across the contact normal, so the
      // middle penetration swings by the runout diameter once per nip turn.
      assert(abs(penetrationMax - penetrationMin - expectedSwing)
        < 0.15*expectedSwing,
        "An eccentric nip drum must swing the middle penetration by twice the runout");
      // The held load follows the cover law, so its swing is the law's own
      // secant over the penetration range the orbit drove it through.
      assert(abs(loadMax - loadMin - expectedLoadSwing)
        < 0.05*(loadMax - loadMin),
        "The held load must follow its cover law over the penetration swing");
      assert(loadMax - loadMin > 50,
        "A 0.3 mm eccentric nip drum must swing the held load");
      assert(abs(centreMax) < 0.02 and abs(centreMin) < 0.02,
        "Radial runout swings the load but must not offset its centre");
    end when;
  end NipRunoutLoad;
  annotation(Documentation(info = "<html>
<h4>Runout checks</h4>
<p>Check radial drum runout through the amplitude and phase of span motion,
tension ripple and held nip load. Idler cases compare runout magnitudes,
phase angles and parameter evaluation paths; closed-loop and driven-roll cases
check how the disturbance propagates while material remains conserved.</p>
</html>"));
end RunoutChecks;
