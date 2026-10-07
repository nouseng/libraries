within Roll2RollDynamicsTest;
package SkewNipChecks "Skewed, wedged and eccentric nip regressions"
  extends Modelica.Icons.ExamplesPackage;

  model SkewNipGeometryCheck
    "Roller yawed +3 degrees and nip -3 degrees: relative skew and the integrated load"
    parameter Boolean checkFootprint = true
      "Check the integrated load against the closed-form crossed-axis result";
    Modelica.Units.SI.Force expectedFootprint =
      contact.coverStiffness*(contact.penetration - contact.edgeLift/3)
      "Closed-form integral of the crossed-axis gap over the full face";
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false);
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Wrapped roller shaft boundary";
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary";
    Modelica.Mechanics.MultiBody.Parts.FixedRotation roller(
      n = {0, 0, 1}, angle = 3, animation = false);
    Modelica.Mechanics.MultiBody.Parts.FixedRotation nip(
      r = {0, 0, 0.204}, n = {0, 0, 1}, angle = -3, animation = false);
    Modelica.Mechanics.Translational.Components.Fixed web;
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction);
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary";
  equation
    connect(shaft.flange, contact.nipPort.shaft);
    connect(geometry.port, contact.nipPort.geometry);
    connect(world.frame_b, roller.frame_a);
    connect(world.frame_b, nip.frame_a);
    connect(roller.frame_b, contact.nipPort.frame);
    connect(nip.frame_b, contact.frameNip);
    connect(web.flange, contact.nipPort.web);
    connect(webLateral.flange, contact.nipPort.lateral);
    assert(not checkFootprint or
      abs(contact.elasticFootprint - expectedFootprint) < 1e-9,
      "The cover law must integrate the crossed-axis gap over the face");
  end SkewNipGeometryCheck;

  model WedgedNipCheck
    "Nip tilted about the machine direction against an aligned roller"
    extends SkewNipGeometryCheck(checkFootprint = false,
      roller(angle = 0), nip(n = {1, 0, 0}, angle = wedge*180/Modelica.Constants.pi));
    parameter Modelica.Units.SI.Angle wedge "Tilt of the nip axis about the machine direction";
    Modelica.Units.SI.Length liftedEnd = if contact.wedgeSlope > 0 then 0.15 else -0.15
      "Face end the wedge opens";
  equation
    assert(abs(contact.wedgeAngle - wedge) < 1e-9 and abs(contact.edgeLift) < 1e-12
      and abs(contact.endOpening - 0.15*tan(wedge)) < 1e-12,
      "A pure wedge must report its tilt and end opening and lift neither edge");
  end WedgedNipCheck;

  model SkewNipStaticChecks
    "Crossed, parallel, released and wedged nips against the closed-form gap integrals"
    SkewNipGeometryCheck crossed;
    SkewNipGeometryCheck parallel(nip(angle = 3));
    SkewNipGeometryCheck released(checkFootprint = false, nip(r = {0, 0, 0.206}));
    WedgedNipCheck smallWedge(wedge = 0.004);
    WedgedNipCheck largeWedge(wedge = 0.01);
    Modelica.Units.SI.Position largeContactEnd = 0.001/tan(0.01)
      "Where the 10 mrad wedge gap closes to zero at 1 mm middle penetration";
    Modelica.Units.SI.Force expectedLargeFootprint =
      1e5/0.3*(0.001*(largeContactEnd + 0.15) - tan(0.01)*(largeContactEnd^2 - 0.15^2)/2)
      "Integral of the wedged gap from the closed end to the contact boundary";
  equation
    // Crossed axes: six degrees of relative skew, both edges lifted, load lower and centred
    assert(abs(crossed.contact.relativeMisalignment - Modelica.Constants.pi/30) < 1e-10,
      "Opposite three-degree yaw must give six-degree relative skew");
    assert(abs(crossed.contact.edgeLift - (0.15*tan(Modelica.Constants.pi/30))^2/0.4) < 1e-10,
      "The edge lift must not average opposite edge offsets to zero");
    assert(crossed.contact.normalForce < parallel.contact.normalForce - 15
      and abs(crossed.contact.loadCentre) < 1e-9,
      "Crossed axes must lower the steady load and keep it centred");
    // Parallel axes: the integral reduces to coverStiffness times penetration
    assert(abs(parallel.contact.elasticFootprint - 100) < 1e-6
      and abs(parallel.contact.edgeLift) < 1e-12,
      "With parallel axes the load must equal coverStiffness times penetration");
    // Open contact
    assert(not released.contact.engaged and abs(released.contact.normalForce) < 1e-12
      and abs(released.contact.tangentialForce) < 1e-12,
      "Open contact must have zero normal and tangential force");
    // Small wedge: whole face touching, same load, resultant moved toward the closed end
    assert(abs(smallWedge.contact.elasticFootprint - 100) < 1e-6
      and abs(smallWedge.contact.contactWidth - 0.3) < 1e-12,
      "A wedge that keeps the whole face in contact must not change the load");
    assert(abs(abs(smallWedge.contact.loadCentre) - tan(0.004)*0.3^2/(12*0.001)) < 1e-9
      and smallWedge.contact.loadCentre*smallWedge.liftedEnd < 0,
      "The resultant must move toward the closed end by b w^2 / (12 delta)");
    // Large wedge: one end clear, load from the contacting part, moment on both housings
    assert(abs(largeWedge.contact.contactWidth - (largeContactEnd + 0.15)) < 1e-9
      and abs(largeWedge.contact.elasticFootprint - expectedLargeFootprint) < 1e-6,
      "A wedge opening beyond the penetration must lift one end clear");
    assert(abs(Modelica.Math.Vectors.length(largeWedge.contact.frameNip.t)
        - largeWedge.contact.normalForce*abs(largeWedge.contact.loadCentre)) < 1e-6
      and abs(Modelica.Math.Vectors.length(largeWedge.contact.nipPort.frame.t)
        - largeWedge.contact.normalForce*abs(largeWedge.contact.loadCentre)) < 1e-6,
      "Both housings must receive the moment of the off-centre normal load");
  end SkewNipStaticChecks;

  model SkewNipIntegrationCheck
    "A crossed nip on the loaded roller reports its edge lift"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(yaw = -0.06), world(enableAnimation = false));
  equation
    assert(abs(nip.nipContact.edgeLift
      - (0.5*nip.width*tan(0.06))^2/(2*(topRoll.radius + nip.radius))) < 1e-9,
      "The default nip must report the crossed-axis edge lift");
  end SkewNipIntegrationCheck;

  model WedgedNipIntegrationCheck
    "A nip wedged by a tram difference on the loaded roller, for the NipLoading figure"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(tram = 0.003), world(enableAnimation = false));
  end WedgedNipIntegrationCheck;

  model NipRunoutCheck
    "An eccentric nip drum on the loaded roller, for the NipLoading figure"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(runout = 0.3e-3), world(enableAnimation = false));
  end NipRunoutCheck;

  model NipSteerCheck
    "A crossed nip drags the tracking web sideways, for the NipLoading figure"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(yaw = -0.06), world(enableAnimation = false, lateralDynamics = true),
      loadingPosition(table = [0, 0; 0.5, 0; 1, 0.002; 3, 0.002; 3.5, 0; 4, 0]));
  equation
    // Nip surface lateral speed = -R_n*omega_n*crossedSkewSin; the web is
    // dragged its way while the wrapped roller (aligned) holds it back.
    assert(not (time > 1.5 and time < 2.5) or nip.nipContact.lateralForce
      *nip.nipContact.lateralSlip >= -1e-9,
      "Nip lateral friction must oppose nip-to-web lateral slip");
  end NipSteerCheck;

  model NipStraightSteerCheck
    "The same tracking web with a parallel nip stays put"
    extends Roll2RollDynamics.Examples.NipLoading(
      world(enableAnimation = false, lateralDynamics = true),
      loadingPosition(table = [0, 0; 0.5, 0; 1, 0.002; 3, 0.002; 3.5, 0; 4, 0]));
  end NipStraightSteerCheck;

  model YawedRollerDriftCheck
    "The loaded roller yawed 0.06 rad steers the tracking web; the nip stays clear"
    extends Roll2RollDynamics.Examples.NipLoading(
      topRoll(yaw = 0.06), world(enableAnimation = false, lateralDynamics = true),
      loadingPosition(table = [0, 0; 4, 0]));
  end YawedRollerDriftCheck;

  model YawedRollerStraightNipCheck
    "The yawed roller with a nip parallel to the machine, crossed with the roller"
    extends YawedRollerDriftCheck(
      loadingPosition(table = [0, 0; 0.5, 0; 1, 0.002; 3, 0.002; 3.5, 0; 4, 0]));
  end YawedRollerStraightNipCheck;

  model YawedNipStandCheck
    "The yawed roller with the nip yawed along with it: a cocked stand"
    extends YawedRollerStraightNipCheck(nip(yaw = 0.06));
  end YawedNipStandCheck;

  model YawedRollerRubberNipCheck
    "The yawed roller with a crossed nip whose cover out-grips the roller: the nip wins"
    extends YawedRollerStraightNipCheck(
      nip(traction = Roll2RollDynamics.Utilities.Types.TractionFrictionParameters(
        muAdhesion = 0.8, muSliding = 0.6)));
  end YawedRollerRubberNipCheck;

  annotation(Documentation(info = "<html><h4>Skew nip checks</h4>
<p>Regression scenarios for skew, wedge, runout and steering in assembled nips. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end SkewNipChecks;
