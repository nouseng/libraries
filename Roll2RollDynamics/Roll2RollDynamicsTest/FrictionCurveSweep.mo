within Roll2RollDynamicsTest;
model FrictionCurveSweep
  "Signed slip sweep behind the tripleS documentation figure"
  extends Modelica.Icons.Example;
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
    Documentation(info = "<html><h4>Friction curve sweep</h4><p>Reference helper that sweeps <code>slipVelocity</code> from <code>-maximumSlipVelocity</code> to <code>+maximumSlipVelocity</code> and evaluates Triple-S and classical Stribeck coefficients with the same viscous slope. Its outputs supply the friction-curve documentation figure; it contains no acceptance assertions and models no machine.</p></html>"));
end FrictionCurveSweep;
