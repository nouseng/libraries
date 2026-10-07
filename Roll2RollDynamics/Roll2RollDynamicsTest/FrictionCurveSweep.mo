within Roll2RollDynamicsTest;
model FrictionCurveSweep
  "Signed slip sweep behind the tripleS documentation figure"
  parameter Roll2RollDynamics.Utilities.Types.TractionFrictionParameters traction(
    viscousSlope = 5)
    "Friction parameters shared by both curves";
  parameter Modelica.Units.SI.Velocity maximumSlipVelocity = 0.02
    "Slip velocity reached at each end of the sweep";

  final constant Modelica.Units.SI.Time sweepDuration = 1
    "Fixed duration of the sweep";

  output Modelica.Units.SI.Velocity slipVelocity =
    maximumSlipVelocity*(2*time/sweepDuration - 1)
    "Signed web-to-roll slip velocity, swept from -max to +max";
  output Real tripleSCoefficient(unit = "1") = Roll2RollDynamics.Functions.tripleS(
    traction.vAdhesion,
    traction.vSlide,
    traction.muAdhesion,
    traction.muSliding,
    slipVelocity,
    traction.viscousSlope)
    "Signed Triple-S coefficient";
  output Real stribeckCoefficient(unit = "1") =
    sign(slipVelocity)*(traction.muSliding +
      (traction.muAdhesion - traction.muSliding)*exp(
        -(slipVelocity/traction.vAdhesion)^2)) +
    traction.viscousSlope*slipVelocity
    "Classical Stribeck coefficient, signed by the slip direction";
  annotation(
    experiment(StartTime = 0, StopTime = 1, Interval = 0.0005),
    Documentation(info = "<html>
<p>Sweeps the slip velocity from <code>-maximumSlipVelocity</code> to
<code>+maximumSlipVelocity</code> and evaluates both friction laws on it, for
the friction-curve documentation figure. It is a reference probe rather
than a library class, since it models no machine.</p>
</html>"));
end FrictionCurveSweep;
