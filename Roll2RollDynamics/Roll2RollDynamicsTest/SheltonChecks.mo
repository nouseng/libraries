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
      Documentation(info="<html><p>Checks measured motion against the
      analytical prediction, normal-entry geometry and material balance.</p></html>"));
  end PositiveYaw;

  model NegativeYaw "Reversing yaw reverses the steering response"
    extends PositiveYaw(yaw=-0.004);
    annotation(Documentation(info="<html><p>Exercises the opposite
      steering direction with the same analytical error limits.</p></html>"));
  end NegativeYaw;

  model HalfSpeed "Slower transport follows the longer steering time scale"
    extends PositiveYaw(lineSpeed=1);
    annotation(Documentation(info="<html><p>Checks the L/V scaling
      independently of the default speed.</p></html>"));
  end HalfSpeed;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Analytical verification of idealized
    normal-entry steering; not experimental friction validation.</p></html>"));
end SheltonChecks;
