within Roll2RollDynamicsTest;
package InvalidRotationDirectionCheck "Invalid rotation-direction regressions"
  extends Modelica.Icons.ExamplesPackage;

  model InvalidRotationDirectionCheck
    "Validation harness for a forbidden zero roller direction"
    extends Roll2RollDynamics.Examples.ThreeRollWebLoop(topRoll(rotationDirection = 0));
  end InvalidRotationDirectionCheck;

  model InvalidWinderDirectionCheck
    "Validation harness for a forbidden zero winder direction"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(unwindDrum(rotationDirection = 0));
  end InvalidWinderDirectionCheck;

  annotation(Documentation(info = "<html><h4>Invalid rotation direction</h4>
<p>Negative scenarios that verify rejection of invalid rotation directions. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end InvalidRotationDirectionCheck;
