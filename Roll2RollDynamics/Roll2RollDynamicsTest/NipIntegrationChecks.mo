within Roll2RollDynamicsTest;
package NipIntegrationChecks "Nip assembly and friction-ownership regressions"
  extends Modelica.Icons.ExamplesPackage;

  model NipIntegrationCheck
    "Running loop that closes and releases a nip against one roller"
    extends Roll2RollDynamics.Examples.ThreeRollWebLoop(
      topRoll(useNip = true),
      world(enableAnimation = false));

    Modelica.Mechanics.MultiBody.Parts.FixedTranslation loadingBase(
      r = {0.5, 0, 1.026}, animation = false)
      "Base above the top roller with one millimetre initial clearance"
      annotation(Placement(visible = true, transformation(origin = {120, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic loadingJoint(
      n = {0, 0, -1},
      useAxisFlange = true,
      animation = false,
      s(start = 0, fixed = false),
      v(start = 0, fixed = false)) "Prescribed nip loading motion"
      annotation(Placement(visible = true, transformation(origin = {280, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Sources.Position loadingMotion(
      exact = true) "Closes and releases the nip"
      annotation(Placement(visible = true, transformation(origin = {200, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.TimeTable loadingPosition(table = [
      0, 0;
      1, 0;
      2, 0.002;
      5, 0.002;
      6, 0;
      8, 0]) "Two millimetre close-and-release stroke"
      annotation(Placement(visible = true, transformation(origin = {120, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.NipRoller nip(
      radius = 0.1,
      coverStiffness = 1e6,
      coverDamping = 1000,
      maxNipLoad = 1000,
      useSupport = true,
      fixedInitialAngle = true,
      fixedInitialSpeed = true) "Free nip drum driven only by web contact"
      annotation(Placement(visible = true, transformation(origin = {360, 0},
        extent = {{-10, -10}, {10, 10}})));

    output Modelica.Units.SI.Force capacityIncrement =
      topRoll.rollerVariables.Fcap - topRoll.wrapCapacity
      "Traction capacity supplied by nip loading";
    output Modelica.Units.SI.Force expectedIncrement =
      topRoll.rollerVariables.nipLoad*topRoll.traction.muAdhesion
      "Analytic nip contribution to traction capacity";

  equation
    connect(world.frame_b, loadingBase.frame_a)
      annotation(Line(visible = true, points = {{-100, 70}, {110, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(loadingBase.frame_b, loadingJoint.frame_a)
      annotation(Line(visible = true, points = {{130, 40}, {270, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(loadingJoint.frame_b, nip.frame_support)
      annotation(Line(visible = true, points = {{290, 0}, {360, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(loadingPosition.y, loadingMotion.s_ref)
      annotation(Line(visible = true, points = {{131, -40}, {188, 0}},
        color = {0, 0, 127}, smooth = Smooth.None));
    connect(loadingMotion.flange, loadingJoint.axis)
      annotation(Line(visible = true, points = {{210, 0}, {288, 6}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(topRoll.frame_nip, nip.nipLoading)
      annotation(Line(visible = true, points = {{0, 60}, {370.567, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));

    annotation(experiment(StartTime = 0, StopTime = 8, Interval = 0.005,
      Tolerance = 1e-8));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {400, 110}})));
    annotation(Documentation(info = "<html><h4>Nip integration check</h4>
<p>Close a free nip drum onto the top roller of a running closed web loop, hold it and release it. The drum should spin up through contact, the sensed nip load should raise roller traction capacity by <code>expectedIncrement</code>, and release should remove contact traction while the drum coasts.</p></html>"));
  end NipIntegrationCheck;

  model PrescribedNipLoadCheck
    "A nip without a support carries loadFraction times maxNipLoad"
    extends Roll2RollDynamics.Examples.ThreeRollWebLoop(
      topRoll(useNip = true),
      world(enableAnimation = false));
    Roll2RollDynamics.Components.NipRoller nip(
      radius = 0.1,
      coverStiffness = 1e6,
      coverDamping = 1000,
      maxNipLoad = 1000,
      loadFraction = 0.4,
      xPosition = 0.5,
      zPosition = 1.025,
      contactFace = if topRoll.contactFace ==
        Roll2RollDynamics.Utilities.Types.WebFace.Inner then
        Roll2RollDynamics.Utilities.Types.WebFace.Outer else
        Roll2RollDynamics.Utilities.Types.WebFace.Inner)
      "Fixed nip pressing on the top roller"
      annotation(Placement(visible = true, transformation(origin = {240, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(topRoll.frame_nip, nip.nipLoading)
      annotation(Line(visible = true, points = {{0, 60}, {250.567, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    assert(time < 0.5 or abs(topRoll.rollerVariables.nipLoad
      - nip.loadFraction*nip.maxNipLoad) < 1e-6,
      "A fixed nip must carry loadFraction*maxNipLoad");
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {280, 110}})));
    annotation(Documentation(info = "<html><h4>Prescribed nip load check</h4>
<p>Attach a nip without an exposed support to the top roller of the running loop with <code>loadFraction = 0.4</code>. After 0.5 s the roller must sense the prescribed 400 N normal load.</p></html>"));
  end PrescribedNipLoadCheck;

  model FullPrescribedNipLoadCheck
    "A full actuator command gives the maximum actuator force at finite penetration"
    extends PrescribedNipLoadCheck(nip(loadFraction = 1));
  equation
    assert(abs(nip.nipVariables.penetration - 0.001) < 1e-8,
      "Full actuator command must give the finite elastic penetration F/k");
    annotation(Documentation(info = "<html><h4>Full prescribed nip load check</h4>
<p>Apply the full actuator command to the fixed nip in the running loop. The inherited load check must reach 1000 N, and the elastic cover must attain a finite penetration of 1 mm.</p></html>"));
  end FullPrescribedNipLoadCheck;

  model DisabledNipRegressionCheck
    "A disabled nip port leaves the roller nip load at zero"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      world(enableAnimation = false));
    annotation(Documentation(info = "<html><h4>Disabled nip regression check</h4>
<p>Run the converting line with its default disabled nip port as the reference for the enabled but unconnected case. The idler should report zero nip load and retain its wrap-only traction capacity.</p></html>"));
  end DisabledNipRegressionCheck;

  model OpenNipRegressionCheck
    "An enabled but unconnected nip port adds no load or traction capacity"
    extends DisabledNipRegressionCheck(idler(useNip = true));
  equation
    assert(abs(idler.rollerVariables.nipLoad) < 1e-12 and abs(idler.rollerVariables.Fcap - idler.wrapCapacity) < 1e-12,
      "An enabled unconnected nip must carry no load or additional traction");
    annotation(Documentation(info = "<html><h4>Open nip regression check</h4>
<p>Enable the idler nip port without connecting a nip. Assertions require zero nip load and traction capacity equal to the wrap-only value.</p></html>"));
  end OpenNipRegressionCheck;

  model NipFrictionOwnershipCheck
    "Nip surface friction is independent of the wrapped roller surface"
    extends NipIntegrationCheck(
      nip(traction(muAdhesion = 0.6, muSliding = 0.4)));
  equation
    assert(abs(nip.nipVariables.nipTraction - nip.nipVariables.nipLoad*
      Roll2RollDynamics.Functions.tripleS(
        nip.traction.vAdhesion, nip.traction.vSlide,
        nip.traction.muAdhesion, nip.traction.muSliding,
        nip.nipVariables.nipSlip, nip.traction.viscousSlope)) < 1e-7,
      "Nip friction must use the characteristic configured on NipRoller");
    assert(abs(topRoll.rollerVariables.nipLoad - nip.nipVariables.nipLoad) < 1e-7,
      "Combined port must carry the nip load through to Roller");
    assert(abs(capacityIncrement - expectedIncrement) < 1e-7,
      "Main roller traction capacity must retain its own friction coefficient");
    when time >= 3 then
      assert(nip.nipVariables.nipLoad > 500 and abs(nip.nipVariables.penetration - 0.001) < 1e-7,
        "Connected roller geometry and nip radius must determine penetration");
    end when;
    annotation(Documentation(info = "<html><h4>Nip friction ownership check</h4>
<p>Give the closing nip its own adhesion and sliding coefficients of 0.6 and 0.4. Assertions require nip traction to follow that characteristic, the wrapped roller capacity to retain its own coefficient, and both components to report the same normal load with 1 mm penetration during the hold.</p></html>"));
  end NipFrictionOwnershipCheck;

  model NipMomentumCheck
    "Whole-loop horizontal momentum balance during nip engagement and release"
    extends NipIntegrationCheck(
      driveRoll(useSupport=true), topRoll(useSupport=true), rightRoll(useSupport=true));
    // Parameters
    parameter Modelica.Units.SI.Momentum initialMomentumX(fixed=false)
      "Initial horizontal momentum of all circulating material";
    // Variables
    Modelica.Units.SI.Momentum momentumX "Horizontal momentum of free and wrapped web";
    Modelica.Units.SI.Force externalForceX "Horizontal force from all four mounts";
    Modelica.Units.SI.Momentum impulseX(start=0, fixed=true)
      "Integrated external horizontal force";
    // Inputs and Outputs
    output Modelica.Units.SI.Momentum momentumResidual
      "Stored momentum change minus external impulse";
    output Modelica.Units.SI.Force forceResidual
      "Momentum rate minus external force";
    // Components
    Modelica.Mechanics.MultiBody.Parts.Fixed mounts[3](
      each animation=false, r={{0,0,0}, {0.5,0,0.8}, {1,0,0}})
      "Stationary roll mounts with exposed support reactions"
      annotation(Placement(visible = true, transformation(origin = {480, 0},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    initialMomentumX = momentumX;
  equation
    connect(mounts[1].frame_b, driveRoll.frame_support)
      annotation(Line(visible = true, points = {{490, 0}, {-60, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(mounts[2].frame_b, topRoll.frame_support)
      annotation(Line(visible = true, points = {{490, 0}, {0, 40}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(mounts[3].frame_b, rightRoll.frame_support)
      annotation(Line(visible = true, points = {{490, 0}, {60, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    // The arc-average tangent is its endpoint displacement divided by arc length.
    // Shell centres are fixed and the nip moves only vertically, so their x momentum is zero.
    momentumX = risingSpan.beltVariables.webMass*risingSpan.physicalVelocity[1]
      + fallingSpan.beltVariables.webMass*fallingSpan.physicalVelocity[1]
      + returnSpan.beltVariables.webMass*returnSpan.physicalVelocity[1]
      + driveRoll.webMomentum*(driveRoll.frame_b.frame.r_0[1] - driveRoll.frame_a.frame.r_0[1])
        /((driveRoll.radius + driveRoll.beltThickness/2)*driveRoll.rotationDirection*driveRoll.rollerVariables.arcAngle)
      + topRoll.webMomentum*(topRoll.frame_b.frame.r_0[1] - topRoll.frame_a.frame.r_0[1])
        /((topRoll.radius + topRoll.beltThickness/2)*topRoll.rotationDirection*topRoll.rollerVariables.arcAngle)
      + rightRoll.webMomentum*(rightRoll.frame_b.frame.r_0[1] - rightRoll.frame_a.frame.r_0[1])
        /((rightRoll.radius + rightRoll.beltThickness/2)*rightRoll.rotationDirection*rightRoll.rollerVariables.arcAngle);
    // All support frames share the world orientation; gravity has no x component.
    externalForceX = driveRoll.frame_support.f[1] + topRoll.frame_support.f[1]
      + rightRoll.frame_support.f[1] + nip.frame_support.f[1];
    der(impulseX) = externalForceX;
    momentumResidual = momentumX - initialMomentumX - impulseX;
    forceResidual = der(momentumX) - externalForceX;
    assert(abs(forceResidual) < 1e-6,
      "Closed nip loop must balance horizontal external force and momentum rate");
    assert(abs(momentumResidual) < 1e-5,
      "Nip engagement must not create unaccounted horizontal impulse");
    assert(abs(topRoll.rollerVariables.nipLoad - nip.nipVariables.nipLoad) < 1e-7,
      "Tangential nip reaction must not increase the sensed normal load");
    when time >= 2 then
      assert(nip.nipVariables.nipLoad > 500 and abs(nip.nipVariables.nipTraction) > 1,
        "Momentum regression must exercise a loaded nip with nonzero traction");
    end when;
    annotation(Documentation(info = "<html><h4>Nip momentum check</h4>
<p>Compare total horizontal momentum of free and wrapped web with the independent sum of all four mount reactions while the nip engages and releases. The drive supplies torque, gravity is vertical, the mounts remain fixed horizontally and no material crosses the closed-loop boundary, so assertions require internal contact forces to cancel, momentum change to equal external impulse and tangential traction to leave the sensed normal load unchanged.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {520, 110}})));
  end NipMomentumCheck;

  model ReverseNipMomentumCheck
    "Whole-loop nip momentum balance with reversed material travel"
    extends NipMomentumCheck(world(lineSpeed=-2));
    annotation(Documentation(info = "<html><h4>Reverse nip momentum check</h4>
<p>Repeat the whole-loop horizontal momentum check with <code>world.lineSpeed = -2</code> m/s. Nip traction should reverse while the normal load remains the same, and the inherited force and impulse balances must still hold.</p></html>"));
  end ReverseNipMomentumCheck;

  annotation(Documentation(info = "<html><h4>Nip integration checks</h4>
<p>Regression scenarios for assembled nips, friction ownership and contact constraints. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end NipIntegrationChecks;
