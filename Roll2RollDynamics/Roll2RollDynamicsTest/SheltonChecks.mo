within Roll2RollDynamicsTest;
package SheltonChecks "Normal-entry benchmark checks against idealized steering"
  extends Modelica.Icons.ExamplesPackage;

  model PositiveYaw "Positive yaw follows the independent steering prediction"
    extends Roll2RollDynamics.Examples.SheltonSteering(world(enableAnimation=false));
  equation
    assert(abs(trackingError) < 0.0025*spanLength*abs(yaw) + 1e-7,
      "Physical steering must remain within 0.25% of the ideal offset scale");
    assert(abs(massBalanceError) < 1e-8,
      "Steering must conserve the open web inventory");
    when terminal() then
      assert(abs(relativeOffset - spanLength*yaw)
          < 0.02*spanLength*abs(yaw),
        "The settled relative offset must agree with normal-entry geometry");
      assert(abs(exitRoll.entrySkewSin) < 0.02*abs(yaw),
        "The web must approach the yawed roller normally");
      assert(abs(span.beltVariables.webVelocity - lineSpeed) < 0.002,
        "The web must run at the requested benchmark speed");
    end when;
    annotation(experiment(StopTime=8, Tolerance=1e-9),
      Documentation(info = "<html><h4>Positive yaw</h4><p>Compares the steering example with the independent idealized normal-entry prediction. Tracking error must stay within 0.25% of the offset scale and material balance within 1e-8; the final offset and entry skew must satisfy their 2% limits, and web speed must be within 0.002 m/s of the requested speed.</p></html>"));
  end PositiveYaw;

  model NegativeYaw "Reversing yaw reverses the steering response"
    extends PositiveYaw(yaw=-0.004);
    annotation(Documentation(info = "<html><h4>Negative yaw</h4><p>Reverses the benchmark yaw to -0.004 rad. The steering response must reverse while satisfying the inherited analytical tracking, material-balance, settled normal-entry and transport-speed limits.</p></html>"));
  end NegativeYaw;

  model HalfSpeed "Slower transport follows the longer steering time scale"
    extends PositiveYaw(lineSpeed=1);
    annotation(Documentation(info = "<html><h4>Half speed</h4><p>Runs the normal-entry benchmark at <code>lineSpeed=1</code> m/s to exercise the longer span-length/transport-speed response time. The inherited analytical tracking, material-balance, final offset, entry skew and web-speed checks must still pass over the 8 s run.</p></html>"));
  end HalfSpeed;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Analytical verification of idealized
    normal-entry steering; not experimental friction validation.</p></html>"));
end SheltonChecks;
