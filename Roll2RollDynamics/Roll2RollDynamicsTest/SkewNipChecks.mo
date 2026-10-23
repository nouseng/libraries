within Roll2RollDynamicsTest;
package SkewNipChecks "Skewed, wedged and eccentric nip regressions"
  extends Modelica.Icons.ExamplesPackage;

  model SkewNipGeometryCheck
    "Roller yawed +3 degrees and nip -3 degrees: relative skew and the integrated load"
    extends Modelica.Icons.Example;
    parameter Boolean checkFootprint = true
      "Check the integrated load against the closed-form crossed-axis result";
    Modelica.Units.SI.Force expectedFootprint =
      contact.coverStiffness*(contact.penetration - contact.edgeLift/3)
      "Closed-form integral of the crossed-axis gap over the full face";
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      annotation(Placement(visible = true, transformation(origin = {-160, 240},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Rotational.Components.Fixed shaft "Wrapped roller shaft boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, 160},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed webLateral
      "Stationary cross-machine web boundary"
      annotation(Placement(visible = true, transformation(origin = {-80, 80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation roller(
      n = {0, 0, 1}, angle = 3, animation = false)
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation nip(
      r = {0, 0, 0.204}, n = {0, 0, 1}, angle = -3, animation = false)
      annotation(Placement(visible = true, transformation(origin = {-80, -80},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.Translational.Components.Fixed web
      annotation(Placement(visible = true, transformation(origin = {-80, -160},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipContact contact(
      nipRadius = 0.1, nipWidth = 0.3, webThickness = 0.005,
      coverStiffness = 1e5, coverDamping = 0, maxNipLoad = 1000,
      traction = world.innerTraction)
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.NipGeometrySource geometry(
      radius = 0.1, rotationDirection = 1) "Wrapped roller geometry boundary"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(shaft.flange, contact.nipPort.shaft)
      annotation(Line(visible = true, points = {{-80, 160}, {70, 0}},
        color = {0, 0, 0}, smooth = Smooth.None));
    connect(geometry.port, contact.nipPort.geometry)
      annotation(Line(visible = true, points = {{10, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(world.frame_b, roller.frame_a)
      annotation(Line(visible = true, points = {{-150, 240}, {-90, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(world.frame_b, nip.frame_a)
      annotation(Line(visible = true, points = {{-150, 240}, {-90, -80}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(roller.frame_b, contact.nipPort.frame)
      annotation(Line(visible = true, points = {{-70, 0}, {70, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(nip.frame_b, contact.frameNip)
      annotation(Line(visible = true, points = {{-70, -80}, {90, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(web.flange, contact.nipPort.web)
      annotation(Line(visible = true, points = {{-80, -160}, {70, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
    connect(webLateral.flange, contact.nipPort.lateral)
      annotation(Line(visible = true, points = {{-80, 80}, {70, 0}},
        color = {0, 127, 0}, smooth = Smooth.None));
    assert(not checkFootprint or
      abs(contact.elasticFootprint - expectedFootprint) < 1e-9,
      "The cover law must integrate the crossed-axis gap over the face");
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -200}, {120, 280}})));
    annotation(Documentation(info = "<html><h4>Skew nip geometry check</h4>
<p>Provide a fixed-centre contact fixture with roller yaw of +3 degrees and nip yaw of -3 degrees. The six-degree crossed-axis contact must give an elastic load equal to the closed-form full-face gap integral when <code>checkFootprint</code> is enabled; the static composite case supplies further comparisons.</p></html>"));
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

    annotation(Documentation(info = "<html><h4>Wedged nip check</h4>
<p>Provide a reusable pure-wedge fixture by replacing crossed yaw with nip tilt about the machine direction. A parent must supply <code>wedge</code>; assertions require the reported wedge angle and half-face end opening to match that tilt with zero crossed-axis edge lift.</p></html>"));
  end WedgedNipCheck;

  model SkewNipStaticChecks
    "Crossed, parallel, released and wedged nips against the closed-form gap integrals"
    extends Modelica.Icons.Example;
    SkewNipGeometryCheck crossed
      annotation(Placement(visible = true, transformation(origin = {0, 160},
        extent = {{-10, -10}, {10, 10}})));
    SkewNipGeometryCheck parallel(nip(angle = 3))
      annotation(Placement(visible = true, transformation(origin = {0, 80},
        extent = {{-10, -10}, {10, 10}})));
    SkewNipGeometryCheck released(checkFootprint = false, nip(r = {0, 0, 0.206}))
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    WedgedNipCheck smallWedge(wedge = 0.004)
      annotation(Placement(visible = true, transformation(origin = {0, -80},
        extent = {{-10, -10}, {10, 10}})));
    WedgedNipCheck largeWedge(wedge = 0.01)
      annotation(Placement(visible = true, transformation(origin = {0, -160},
        extent = {{-10, -10}, {10, 10}})));
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
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -200}, {100, 200}})));
    annotation(Documentation(info = "<html><h4>Skew nip static checks</h4>
<p>Compare crossed, parallel, released and purely wedged contacts against closed-form gap integrals. Assertions require crossed axes to lower a centred load, release to remove both forces, and wedge tilt to shift load toward the closed end, reduce contact width when an end lifts and apply the resulting moments to both housings.</p></html>"));
  end SkewNipStaticChecks;

  model SkewNipIntegrationCheck
    "A crossed nip on the loaded roller reports its edge lift"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(yaw = -0.06), world(enableAnimation = false));
  equation
    assert(abs(nip.nipContact.edgeLift
      - (0.5*nip.width*tan(0.06))^2/(2*(topRoll.radius + nip.radius))) < 1e-9,
      "The default nip must report the crossed-axis edge lift");
    annotation(Documentation(info = "<html><h4>Skew nip integration check</h4>
<p>Close a nip yawed by -0.06 rad onto the top roller of the running loop. The assembled contact must report crossed-axis edge lift equal to the analytical expression based on both radii and half the nip width.</p></html>"));
  end SkewNipIntegrationCheck;

  model WedgedNipIntegrationCheck
    "A nip wedged by a tram difference on the loaded roller, for the NipLoading figure"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(tram = 0.003), world(enableAnimation = false));
    annotation(Documentation(info = "<html><h4>Wedged nip integration check</h4>
<p>Close a nip tilted out of plane by <code>tram = 0.003</code> rad onto the running top roller. The resulting wedge should move the normal-load centre toward the closed end and alter the contact width and load during the stroke.</p></html>"));
  end WedgedNipIntegrationCheck;

  model NipRunoutCheck
    "An eccentric nip drum on the loaded roller, for the NipLoading figure"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(runout = 0.3e-3), world(enableAnimation = false));
    annotation(Documentation(info = "<html><h4>Nip runout check</h4>
<p>Run the nip loading stroke with 0.3 mm radial eccentricity in the nip drum. Its orbit should modulate penetration and the held normal load once per nip revolution.</p></html>"));
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
    annotation(Documentation(info = "<html><h4>Nip steer check</h4>
<p>Engage a nip yawed by -0.06 rad on a web with lateral dynamics enabled. The crossed nip should drag the web laterally against the aligned supporting roller, and the dwell assertion requires the lateral traction and nip-to-web slip to have a dissipative sign.</p></html>"));
  end NipSteerCheck;

  model NipStraightSteerCheck
    "The same tracking web with a parallel nip stays put"
    extends Roll2RollDynamics.Examples.NipLoading(
      world(enableAnimation = false, lateralDynamics = true),
      loadingPosition(table = [0, 0; 0.5, 0; 1, 0.002; 3, 0.002; 3.5, 0; 4, 0]));
    annotation(Documentation(info = "<html><h4>Nip straight steer check</h4>
<p>Engage a parallel nip using the same loading stroke and lateral web dynamics as the crossed-nip case. With no yaw disturbance, the tracking web should remain at its initial lateral position.</p></html>"));
  end NipStraightSteerCheck;

  model YawedRollerDriftCheck
    "The loaded roller yawed 0.06 rad steers the tracking web; the nip stays clear"
    extends Roll2RollDynamics.Examples.NipLoading(
      topRoll(yaw = 0.06), world(enableAnimation = false, lateralDynamics = true),
      loadingPosition(table = [0, 0; 4, 0]));
    annotation(Documentation(info = "<html><h4>Yawed roller drift check</h4>
<p>Yaw the top roller by 0.06 rad with lateral web dynamics enabled and keep the nip clear throughout the run. The roller should steer the web laterally, providing the unloaded reference for the engaged-nip comparisons.</p></html>"));
  end YawedRollerDriftCheck;

  model YawedRollerStraightNipCheck
    "The yawed roller with a nip parallel to the machine, crossed with the roller"
    extends YawedRollerDriftCheck(
      loadingPosition(table = [0, 0; 0.5, 0; 1, 0.002; 3, 0.002; 3.5, 0; 4, 0]));
    annotation(Documentation(info = "<html><h4>Yawed roller straight nip check</h4>
<p>Engage a nip aligned with the machine onto the roller yawed by 0.06 rad. The crossed contact should apply opposing lateral sliding friction while the stronger supporting roller keeps the web drift close to the clear-nip reference.</p></html>"));
  end YawedRollerStraightNipCheck;

  model YawedNipStandCheck
    "The yawed roller with the nip yawed along with it: a cocked stand"
    extends YawedRollerStraightNipCheck(nip(yaw = 0.06));
    annotation(Documentation(info = "<html><h4>Yawed nip stand check</h4>
<p>Yaw the nip by the same 0.06 rad as the supporting roller to form a cocked but mutually parallel stand. Engagement should preserve steering from the yawed stand while removing the relative crossed-axis skew of the machine-aligned nip case.</p></html>"));
  end YawedNipStandCheck;

  model YawedRollerRubberNipCheck
    "The yawed roller with a crossed nip whose cover out-grips the roller: the nip wins"
    extends YawedRollerStraightNipCheck(
      nip(traction = Roll2RollDynamics.Utilities.Types.TractionFrictionParameters(
        muAdhesion = 0.8, muSliding = 0.6)));
    annotation(Documentation(info = "<html><h4>Yawed roller rubber nip check</h4>
<p>Engage a machine-aligned nip with adhesion and sliding coefficients of 0.8 and 0.6 against the yawed supporting roller. Its stronger lateral grip should dominate the crossed-contact competition and reverse the drift tendency of the weaker-nip case.</p></html>"));
  end YawedRollerRubberNipCheck;

  annotation(Documentation(info = "<html><h4>Skew nip checks</h4>
<p>Regression scenarios for skew, wedge, runout and steering in assembled nips. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end SkewNipChecks;
