within Roll2RollDynamicsTest;
package TramChecks "Out-of-plane roll tilt regressions"
  extends Modelica.Icons.ExamplesPackage;

  model TrammedIdler "Tilted idler steers through its inclined spans, not its yaw"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      idler(yaw = 0, tram = 0.004), world(enableAnimation = false));
    // The entry span, inclined 0.6 rad, must skew across the machine by the
    // tilt seen along that inclination before it enters the tilted axis normally
    Real expectedSpanSkew(unit = "1") = -sin(WebWorldAngle)*sin(idler.tram)
      "Cross-machine sine the entry span settles to for normal entry";
  equation
    when terminal() then
      assert(abs(idlerSpanAxisSkew) < 2e-4,
        "Normal entry must still hold at the tilted idler");
      assert(abs(idlerSpanSkewSin - expectedSpanSkew) < 2e-4,
        "The inclined entry span must skew by the tilt seen along its inclination");
      assert(abs(idlerLateralOffset) > 1e-3,
        "Tram must walk the web through the inclined spans");
    end when;
  end TrammedIdler;

  model TrammedNip "Tilted nip reports a wedge and no crossed-axis edge lift"
    // 2.5 mrad opens one end by 0.83 mm, inside the 1 mm clearance, so the
    // face still clears the web at both ends of the stroke
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(yaw = 0, tram = 0.0025), world(enableAnimation = false));
  equation
    assert(abs(nip.nipContact.edgeLift) < 1e-12,
      "A wedge-only tilt must not lift both edges");
    assert(abs(nip.nipContact.endOpening - 0.5*nip.width*tan(0.0025)) < 1e-9,
      "The end opening must follow the wedge over half the face");
    assert(abs(nip.nipContact.relativeMisalignment - 0.0025) < 1e-9,
      "Relative misalignment must report the tilt");
    assert(abs(nip.nipContact.wedgeAngle - 0.0025) < 1e-9,
      "The wedge angle must report the out-of-plane tilt");
    assert(nip.nipContact.loadCentre*nip.nipContact.normalForce <= 0,
      "The load resultant must sit toward the closed end of the wedge");
  end TrammedNip;

  model CrossedNip "Yawed nip still reports crossed axes and no wedge"
    extends Roll2RollDynamics.Examples.NipLoading(
      nip(yaw = -0.06), world(enableAnimation = false));
  equation
    assert(abs(nip.nipContact.wedgeAngle) < 1e-9,
      "In-plane yaw above the roller must give no wedge");
    assert(abs(nip.nipContact.endOpening) < 1e-9,
      "In-plane yaw above the roller must open neither end");
  end CrossedNip;
  annotation(Documentation(info = "<html>
<h4>Tram checks</h4>
<p>Check the effect of out-of-plane roll tilt on lateral steering and nip
contact. The idler case relates the span skew to the tilted axis; the nip
cases distinguish the one-sided opening caused by tilt from the symmetric
edge lift caused by crossed axes.</p>
</html>"));
end TramChecks;
