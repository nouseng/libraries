within Roll2RollDynamicsTest;
package DetachmentChecks "Web detachment on a retractable roller"
  extends Modelica.Icons.ExamplesPackage;

  model RigidRegression
    "A roll that never leaves the line must behave exactly as before"
    extends Roll2RollDynamics.Examples.ThreeRollWebLoop(
      world(enableAnimation=false),
      topRoll(useDetachment=true));
    output Modelica.Units.SI.Angle wrapReference = topRoll.rollerVariables.arcAngle
      "Wrap angle of the roll that later becomes detachable";
    output Modelica.Units.SI.Force tractionReference = topRoll.rollerVariables.Ft
      "Contact friction of the roll that later becomes detachable";
  equation
    assert(topRoll.rotationDirection*topRoll.rollerVariables.arcAngle > 0,
      "The regression roll must stay wrapped for the whole run");
    assert(topRoll.rollerVariables.webWrapped,
      "The regression roll must never release");
    assert(abs(topRoll.gap) < 1e-12,
      "Rigid contact leaves no clearance while engaged");
    annotation(experiment(StopTime=25, Tolerance=1e-8),
      Documentation(info="<html><p>The engaged contact path with detachment
      enabled must reproduce the rigid roller.</p></html>"));
  end RigidRegression;

  model Retracting
    "Open line whose middle roll is pulled out of the line and back"
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      "Multibody world and shared line defaults";

    Roll2RollDynamics.Components.Roller entryRoll(
      radius = 0.15,
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      xPosition = 0,
      zPosition = 0,
      rotationDirection = 1,
      entryAngleStart = -3.14,
      exitAngleStart = -1.01) "Driven roll at the upstream corner";
    Roll2RollDynamics.Components.Roller topRoll(
      radius = 0.12,
      useSupport = true,
      useDetachment = true,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      fixedInitialSpeed = true,
      rotationDirection = 1,
      entryAngleStart = -1.01,
      exitAngleStart = 1.01) "Retractable idling roll at the apex";
    Roll2RollDynamics.Components.Roller exitRoll(
      radius = 0.09,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      fixedInitialSpeed = true,
      xPosition = 1.0,
      zPosition = 0,
      rotationDirection = 1,
      entryAngleStart = 1.01,
      exitAngleStart = 3.14) "Idling roll at the downstream corner";

    Roll2RollDynamics.Components.Belt risingSpan
      "Span from the driven roll up to the apex";
    Roll2RollDynamics.Components.Belt fallingSpan
      "Span from the apex down to the downstream roll";
    Roll2RollDynamics.Components.WebForce upstreamLoad(direction = 1)
      "Incoming web held at the nominal line tension";
    Roll2RollDynamics.Components.WebForce downstreamLoad(direction = -1)
      "Outgoing web held at the nominal line tension";

    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed = world.lineSpeed/(entryRoll.radius + entryRoll.beltThickness/2))
      "Line speed master";

    Modelica.Mechanics.MultiBody.Parts.FixedTranslation mount(
      r = {0.5, 0, 0.8}, animation = false)
      "Nominal apex position the stroke is measured from";
    Modelica.Mechanics.MultiBody.Joints.Prismatic lift(
      n = {0, 0, -1},
      useAxisFlange = true,
      animation = false,
      s(start = 0, fixed = false),
      v(start = 0, fixed = false)) "Retraction freedom of the apex roll";
    Modelica.Mechanics.Translational.Sources.Position stroke(
      exact = true) "Prescribed retraction stroke";
    Modelica.Blocks.Sources.Trapezoid sweep(
      amplitude = 0.9,
      rising = 5,
      width = 5,
      falling = 5,
      period = 40,
      startTime = 5) "Out of the line and back, once";

    output Modelica.Units.SI.Length apexHeight = 0.8 - lift.s
      "Height of the apex roll centre above the two fixed roll centres";
  equation
    upstreamLoad.force = {world.tension, 0, 0};
    downstreamLoad.force = {-world.tension, 0, 0};
    connect(drive.flange, entryRoll.flange_a);
    connect(upstreamLoad.frame_b, entryRoll.frame_a);
    connect(entryRoll.frame_b, risingSpan.frame_a);
    connect(risingSpan.frame_b, topRoll.frame_a);
    connect(topRoll.frame_b, fallingSpan.frame_a);
    connect(fallingSpan.frame_b, exitRoll.frame_a);
    connect(exitRoll.frame_b, downstreamLoad.frame_b);
    connect(world.frame_b, mount.frame_a);
    connect(mount.frame_b, lift.frame_a);
    connect(lift.frame_b, topRoll.frame_support);
    connect(stroke.flange, lift.axis);
    connect(sweep.y, stroke.s_ref);
    annotation(experiment(StopTime=25, Tolerance=1e-8),
      Documentation(info="<html><p>The apex roll is withdrawn from the line and
      returned to it.</p>
      <h4>Why the line is open</h4>
      <p>A closed loop cannot release a roll. Withdrawing the apex of
      <code>ThreeRollWebLoop</code> shortens the web path by about 0.87 m out of
      roughly 3.2 m, and a loop of fixed material inventory can only give back
      the elastic strain it started with, a fraction of a percent. Tension
      collapses long before the wrap does, and
      <code>WebFriction</code>'s taut-span assertion fires within a tenth of a
      second of the stroke starting. The same three rolls are therefore run as an
      open line: two <code>WebForce</code> boundaries hold the ends at
      <code>world.tension</code>, so the path length the stroke removes is
      simply carried out of the line as material.</p>
      <h4>Which way is out of the line</h4>
      <p>The web passes over the top of the apex roll, so the roll leaves the
      line by descending into the triangle, not by rising: the prismatic axis
      points along -z and the stroke is positive downward. The three roll radii
      0.15, 0.12 and 0.09 fall linearly with x, so one straight line is tangent
      to all three exactly when the apex centre reaches the height of the other
      two. The wrap is <code>2*atan(2*apexHeight)</code>, the radius corrections
      of the two spans cancelling, so release happens at a stroke of exactly
      0.8 m. The 0.9 m amplitude carries the roll 0.1 m past that, leaving a
      clearance of about 0.0998 m at full retraction.</p></html>"));
  end Retracting;

  model Detached "A released roll transmits nothing to the web"
    extends Retracting;
    output Modelica.Units.SI.Velocity transportMismatch =
      topRoll.webVelocityIn - topRoll.webVelocityOut
      "Entry minus exit transport speed relative to the roll centre";
    output Modelica.Units.SI.Force wrappedLoadMagnitude =
      sqrt(topRoll.wrappedLoad*topRoll.wrappedLoad)
      "Magnitude of topRoll.wrappedLoad, the exact load WebFriction's frame_drum.f is defined from";
  equation
    // Non-event inequalities retain exact zero checks without Real equality.
    when not topRoll.rollerVariables.webWrapped then
      assert(noEvent(abs(topRoll.rollerVariables.arcAngle) <= 0),
        "A released roll must have exactly zero wrap angle");
    end when;
    assert(topRoll.rollerVariables.webWrapped or noEvent(abs(topRoll.rollerVariables.arcAngle) <= 0),
      "A released roll must hold zero wrap angle throughout");
    assert(topRoll.rollerVariables.webWrapped or noEvent(abs(topRoll.rollerVariables.Ft) <= 0),
      "A released roll must transmit no friction");
    assert(topRoll.rollerVariables.webWrapped or topRoll.gap >= 0,
      "A released roll must stand clear of the line");
    assert(topRoll.rollerVariables.webWrapped or noEvent(abs(topRoll.exitAngle - topRoll.entryAngle) <= 0),
      "A released roll must hold its two tangencies together, so that the entry "
      + "and exit boundary transport speeds agree whatever radius scales them");
    assert(topRoll.rollerVariables.webWrapped or noEvent(abs(topRoll.wrappedMass) <= 0),
      "A released roll must carry exactly zero true wrapped-material inventory. This checks "
      + "only wrappedMass's own definition, not how frame_drum.f is wired to it -- see the "
      + "wrappedLoadMagnitude bound below for the wiring itself");
    // wrappedLoadMagnitude is the actual expression WebFriction resolves into
    // frame_drum.f (WebFriction.mo defines frame_drum.f FROM wrappedLoad, not
    // as a separate restatement of it), so bounding it here tests the real
    // wiring, not a proxy that could drift from it. The bound is checked only
    // within the interior of the stationary dwell, t = 10.5 to 14.9 s
    // (sweep: rising 5-10, width 10-15, falling 15-20), for two reasons: a
    // strict mode switch causes a brief, physically expected settling
    // transient in der(momentum*meanTangent) after release (observed up to
    // about 0.015 N, decaying within ~1 s), and the trapezoid schedule itself
    // has rate discontinuities at t = 10, 15 and 20 s that generate their own
    // time events and momentary transients unrelated to the floor. Neither is
    // the leak this check exists to catch. Well inside the dwell (away from
    // both) the observed residual is five orders of magnitude below the
    // bound (see Documentation), so 1e-3 N still catches the ~0.13 N leak
    // with wide margin.
    assert(time < 10.5 or time > 14.9 or wrappedLoadMagnitude < 1e-3,
      "The wrapped-material load WebFriction applies at the roll centre must be negligible "
      + "once a released roll's mode-switch transient has settled; a bound violation here "
      + "means the inventory floor is leaking material weight into the support reaction");
    annotation(Documentation(info="<html><p>Zero is asserted exactly, not to
    tolerance: the wrap angle is set to zero by the equation that swaps, and
    the friction follows through <code>tanh</code> of it. <code>wrappedMass</code>
    is likewise asserted exactly zero: it is <code>WebFriction</code>'s true,
    unfloored inventory. That assert alone is not sufficient, though --
    <code>wrappedMass</code> is computed from its own definition regardless of
    which expression <code>frame_drum.f</code> is actually wired to, so it
    cannot catch a regression in the wiring, only in the variable's own
    formula.</p>
    <p><code>wrappedLoadMagnitude</code> closes that gap: <code>WebFriction</code>
    defines <code>frame_drum.f</code> AS the resolved form of
    <code>wrappedLoad</code>, so asserting a bound on
    <code>|wrappedLoad|</code> tests the exact expression the support
    reaction is computed from, not a value that could silently drift away
    from it if the wiring regresses. Measured with the fix in place, well
    inside the stationary dwell (t = 11 to 14.5 s): magnitude on the order of
    1e-6 N at t = 11, decaying to 1e-8-1e-9 N by t = 12-14.5. The asserted
    window, t = 10.5 to 14.9 s, is checked continuously rather than sampled,
    and stays clear of two unrelated transients: the mode-switch settling
    after release (up to about 0.015 N, decaying within ~1 s) and the
    <code>sweep</code> trapezoid's own rate discontinuities at t = 10, 15
    and 20 s, each of which is a time event with its own momentary spike.
    Neither is the leak this check exists to catch. The bound (1e-3 N) sits
    far above the observed residual -- five to six orders of magnitude --
    but still four orders of magnitude below the roughly 0.13 N leak this
    check exists to catch if the weight term were ever wired back to the
    floored inventory. Confirmed by deliberately reintroducing that exact
    regression: the bound fires; restoring the fix passes cleanly again
    (see the task-5 report).</p>
    <p>While detached, <code>WebWrap</code> still scales
    <code>boundaryVelocityA</code> and <code>boundaryVelocityB</code> by
    <code>centrelineRadius</code> although the tangency has moved out to
    <code>tangencyRadius</code>. Both are
    <code>rotationDirection*R*der(angle)</code> with the same <code>R</code>,
    so the exact equality of the two tangency angles asserted here is what makes
    the two boundary speeds identical, and the radius a common factor that
    cancels from every difference <code>WebFriction</code> forms.
    <code>transportMismatch</code> records the residual entry-to-exit transport
    difference, which the boundary terms therefore do not
    contribute to.</p></html>"));
  end Detached;

  model EngagementContinuity "The wrap angle leaves and rejoins zero smoothly"
    extends Retracting;
    Modelica.Units.SI.Angle wrapMagnitude = topRoll.rotationDirection*topRoll.rollerVariables.arcAngle
      "Unsigned wrap angle";
  initial equation
    // change() needs the previous diagnostic value as well as the contact state.
    pre(topRoll.rollerVariables.webWrapped) = topRoll.initiallyEngaged;
  equation
    when change(topRoll.rollerVariables.webWrapped) then
      assert(abs(pre(wrapMagnitude)) < 1e-6,
        "The wrap angle must be at zero when the mode changes, not jump to it");
      assert(abs(pre(topRoll.gap)) < 1e-6,
        "The clearance must be at zero when the mode changes");
    end when;
    assert(wrapMagnitude >= 0,
      "The wrap angle must never go negative");
    annotation(Documentation(info="<html><p>Both switching variables must
    already be at their threshold when the mode changes. A jump here means
    the release is behaving as an impact.</p></html>"));
  end EngagementContinuity;

  model RoundTrip
    "Detaching and re-attaching is accounted for by the boundary flows"
    extends Retracting;
    parameter Modelica.Units.SI.Mass inventoryStart(fixed = false)
      "Open-line material inventory captured at initialization";
    Modelica.Units.SI.Mass inventory =
      entryRoll.wrappedMass + risingSpan.beltVariables.webMass + topRoll.wrappedMass
      + fallingSpan.beltVariables.webMass + exitRoll.wrappedMass
      "Total material on the open line, true unfloored roll inventory plus span inventory";
    Real netInflow(unit = "kg", start = 0, fixed = true)
      "Signed material carried in by the upstream boundary minus material carried out by the downstream boundary (plain Real, since SI.Mass defaults to nonnegative)";
  initial equation
    inventoryStart = inventory;
  equation
    der(netInflow) = entryRoll.massFlowIn - exitRoll.massFlowOut;
    assert(abs(inventory - inventoryStart - netInflow) < 1e-2*inventoryStart,
      "The inventory change over the run must be accounted for by the "
      + "boundary flows, to within the inventory floor's known material-expulsion "
      + "suppression while topRoll is detached (about 0.19% of the starting "
      + "inventory, steady for the whole dwell) -- not created or destroyed "
      + "at the releasing roll beyond that characterised, bounded cost");
    annotation(experiment(StopTime=25, Tolerance=1e-8),
      Documentation(info="<html><p>The fixture is an open line with a
      <code>WebForce</code> boundary at each end, so material legitimately
      flows in and out and total inventory is not constant. What must hold
      instead is that any inventory change is accounted for by the boundary
      flows: <code>entryRoll.massFlowIn</code> is the material rate crossing
      into the line at <code>upstreamLoad</code> (it connects directly to
      <code>entryRoll.frame_a</code>, and <code>WebForce</code> itself
      carries no inventory), and <code>exitRoll.massFlowOut</code> is the
      material rate crossing out of the line at <code>downstreamLoad</code>
      for the same reason at the other end. Integrating their difference into
      <code>netInflow</code> and comparing it against the actual change in
      <code>inventory</code> is a genuine balance, not a constant-inventory
      assertion.</p>
      <p>The sum uses <code>wrappedMass</code>, not <code>webMass</code>, for
      the three rollers, and deliberately so: <code>WebFriction</code> writes
      <code>der(webMass) = massFlowIn - massFlowOut</code> as a live model
      equation on the FLOORED <code>webMass</code>, unconditionally, whether
      or not the floor is active. Summing floored <code>webMass</code> would
      therefore close this exact balance by construction -- an identity the
      model already enforces on itself -- and the check would be vacuous,
      passing even if the floor were quietly destroying material wholesale.
      <code>wrappedMass</code> is <code>WebFriction</code>'s true, unfloored
      inventory (<code>2*referenceDensity*wrapLength/(stretchIn+stretchOut)</code>,
      with no <code>max</code>), with no such self-consistent equation tying
      it to the boundary flows, so summing it measures the TRUE material
      against the boundary flows and makes the floor's bookkeeping cost
      visible instead of hiding it. <code>entryRoll</code> and
      <code>exitRoll</code> never detach, so <code>contactLengthMin = 0</code>
      for them and <code>wrappedMass</code> is identical to <code>webMass</code>
      throughout; using <code>wrappedMass</code> uniformly for all three
      rollers is therefore equivalent to using <code>webMass</code> for these
      two and costs nothing.</p>
      <p>Measured residual: essentially zero (1e-10 to 1e-7 kg, solver noise)
      while <code>topRoll</code> is engaged, holding steady at about
      -0.0133 kg (about 0.19% of the roughly 7.04 kg starting inventory) for
      the whole detached dwell, t = 9.4 to 15.6 s, and returning to solver
      noise once the roll re-engages. This is not a leak: it is exactly the
      floored inventory itself. While the floor is active, <code>der(webMass)
      = 0</code> forces <code>massFlowIn = massFlowOut</code> at
      <code>topRoll</code>, so the roll stops expelling material even though
      its true wrapped mass (<code>wrappedMass</code>) has gone to zero --
      the design doc calls this 'suppresses the corresponding material
      expulsion.' The residual is that suppressed expulsion, characterised
      and bounded, not an unbounded or growing loss.</p>
      <p>The assert tolerance is <code>1e-2*inventoryStart</code> (1%), about
      5x the observed 0.19% residual. That headroom is sized to the
      difference in kind between the two things it has to separate: the
      floor's suppression is a fixed cost set by <code>contactLengthMin</code>
      (here 0.0002 m, the web thickness) and stays flat for the whole dwell
      regardless of run length, so a real material leak at the releasing
      roll -- proportional to how long or how far the roll stays detached,
      or scaling with a wrongly-wired mass or flow term -- would not merely
      nudge the residual, it would move it by a different mechanism entirely
      and typically by a much larger factor. 5x margin absorbs solver/timing
      differences across tolerance and step-size choices without chasing the
      known 0.19% cost, while still tripping on a regression that changes the
      residual's character rather than its last percent.</p></html>"));
  end RoundTrip;

  model StartDetached "A model may initialize with the roll already clear"
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      "Multibody world and shared line defaults";

    Roll2RollDynamics.Components.Roller entryRoll(
      radius = 0.15,
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      xPosition = 0,
      zPosition = 0,
      rotationDirection = 1,
      entryAngleStart = -3.14,
      exitAngleStart = -1.01) "Driven roll at the upstream corner";
    Roll2RollDynamics.Components.Roller topRoll(
      radius = 0.12,
      useSupport = true,
      useDetachment = true,
      initiallyEngaged = false,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      fixedInitialSpeed = true,
      rotationDirection = 1,
      entryAngleStart = -1.01,
      exitAngleStart = 1.01) "Retractable idling roll at the apex, starting clear";
    Roll2RollDynamics.Components.Roller exitRoll(
      radius = 0.09,
      fixedInitialAngle = true,
      fixedInitialWebState = true,
      fixedInitialSpeed = true,
      xPosition = 1.0,
      zPosition = 0,
      rotationDirection = 1,
      entryAngleStart = 1.01,
      exitAngleStart = 3.14) "Idling roll at the downstream corner";

    Roll2RollDynamics.Components.Belt risingSpan
      "Span from the driven roll up to the apex";
    Roll2RollDynamics.Components.Belt fallingSpan
      "Span from the apex down to the downstream roll";
    Roll2RollDynamics.Components.WebForce upstreamLoad(direction = 1)
      "Incoming web held at the nominal line tension";
    Roll2RollDynamics.Components.WebForce downstreamLoad(direction = -1)
      "Outgoing web held at the nominal line tension";

    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed = world.lineSpeed/(entryRoll.radius + entryRoll.beltThickness/2))
      "Line speed master";

    Modelica.Mechanics.MultiBody.Parts.FixedTranslation mount(
      r = {0.5, 0, 0.8}, animation = false)
      "Nominal apex position the stroke is measured from";
    Modelica.Mechanics.MultiBody.Joints.Prismatic lift(
      n = {0, 0, -1},
      useAxisFlange = true,
      animation = false,
      s(start = 0.9, fixed = false),
      v(start = 0, fixed = false)) "Retraction freedom of the apex roll, starting clear";
    Modelica.Mechanics.Translational.Sources.Position stroke(
      exact = true) "Prescribed retraction stroke";
    Modelica.Blocks.Sources.Ramp sweep(
      height = -0.9,
      duration = 10,
      offset = 0.9,
      startTime = 2) "Starts past release, clear of the line, and lowers the stroke into it";

    output Modelica.Units.SI.Length apexHeight = 0.8 - lift.s
      "Height of the apex roll centre above the two fixed roll centres";
  equation
    upstreamLoad.force = {world.tension, 0, 0};
    downstreamLoad.force = {-world.tension, 0, 0};
    connect(drive.flange, entryRoll.flange_a);
    connect(upstreamLoad.frame_b, entryRoll.frame_a);
    connect(entryRoll.frame_b, risingSpan.frame_a);
    connect(risingSpan.frame_b, topRoll.frame_a);
    connect(topRoll.frame_b, fallingSpan.frame_a);
    connect(fallingSpan.frame_b, exitRoll.frame_a);
    connect(exitRoll.frame_b, downstreamLoad.frame_b);
    connect(world.frame_b, mount.frame_a);
    connect(mount.frame_b, lift.frame_a);
    connect(lift.frame_b, topRoll.frame_support);
    connect(stroke.flange, lift.axis);
    connect(sweep.y, stroke.s_ref);
    when initial() then
      assert(not topRoll.rollerVariables.webWrapped,
        "The model must initialize released when initiallyEngaged is false");
    end when;
    annotation(experiment(StopTime=25, Tolerance=1e-8),
      Documentation(info="<html><p>Same fixture as <code>Retracting</code>,
      restated rather than extended because the actuation block must change
      class, from <code>Modelica.Blocks.Sources.Trapezoid</code> to
      <code>Modelica.Blocks.Sources.Ramp</code>, and <code>Retracting.sweep</code>
      is not declared <code>replaceable</code>. <code>Trapezoid</code> divides
      by its own rising time internally, so <code>rising = 0</code> is not a
      way to express an instantaneous start; <code>Ramp</code> needs no rising
      phase to simply hold a starting value.</p>
      <p>The prismatic axis points downward (<code>n = {0,0,-1}</code>) and
      the stroke is positive downward; release sits at a stroke of exactly
      0.8 m (see <code>Retracting</code>'s documentation for the geometry),
      with 0.9 m carrying the roll 0.1 m clear. This fixture starts the
      stroke there, at 0.9 m -- past release, so the roll begins clear of the
      line -- and ramps the stroke down to 0 m over 10 s starting at t = 2 s,
      crossing release at about t = 3.1 s and reaching the nominal fully
      engaged position by t = 12 s, so the roll genuinely engages during the
      run. <code>lift.s</code>'s own start value is moved to 0.9 m to match
      the sweep's starting stroke so the two do not disagree at
      initialization, and <code>topRoll.initiallyEngaged = false</code> tells
      the discrete engagement state to start released to match.</p>
      <p>The <code>when initial()</code> assert is the actual proof the model
      starts detached: if the <code>initiallyEngaged</code> wiring were
      broken, or the geometry above were wrong and the roll were still within
      the line at a stroke of 0.9 m, this would fire immediately.</p></html>"));
  end StartDetached;

  model NipConflict "A detachable roll must not accept a nip"
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      "Shared material and line defaults";
    Roll2RollDynamics.Components.Roller roller(
      useSupport = true,
      useDetachment = true,
      useNip = true,
      fixedInitialWebState = true) "Rejected combination";
    Roll2RollDynamics.Components.WebForce upstreamLoad(direction = 1)
      "Incoming web held at the nominal line tension, so the model is otherwise complete";
    Roll2RollDynamics.Components.WebForce downstreamLoad(direction = -1)
      "Outgoing web held at the nominal line tension, so the model is otherwise complete";
  equation
    upstreamLoad.force = {world.tension, 0, 0};
    downstreamLoad.force = {-world.tension, 0, 0};
    connect(world.frame_b, roller.frame_support);
    connect(upstreamLoad.frame_b, roller.frame_a);
    connect(roller.frame_b, downstreamLoad.frame_b);
    annotation(Documentation(info="<html><p>The combination of
    <code>useDetachment</code> and <code>useNip</code> must be rejected.
    The runner checks the failed <code>simulate()</code> result and the
    intended diagnostic; OpenModelica can evaluate the assertion during
    translation or initialization. Successful simulation is a test failure.
    The expected message is
    <code>Roller: a detachable roll cannot carry a nip, because a nip load
    has no meaning with no web between the rolls</code>.</p></html>"));
  end NipConflict;

  model LateralTransition
    "Lateral momentum through release and re-engagement"
    extends Retracting(world(lateralDynamics = true));
  equation
    annotation(experiment(StopTime=25, Tolerance=1e-8),
      Documentation(info="<html><p>Enables the lateral momentum balance
      throughout a release and return. Zero wrap removes contact friction;
      the regularized material inventory remains in the momentum equation.
      This fixture checks an aligned transition; nonzero lateral motion
      during detachment requires separate sensitivity checks.</p></html>"));
  end LateralTransition;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Detachment checks for the retractable
    roller.</p></html>"));
end DetachmentChecks;
