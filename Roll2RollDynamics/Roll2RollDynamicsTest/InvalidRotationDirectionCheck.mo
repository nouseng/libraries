within Roll2RollDynamicsTest;
package InvalidRotationDirectionCheck "Invalid rotation-direction regressions"
  extends Modelica.Icons.ExamplesPackage;

  model InvalidRotationDirectionCheck
    "Validation harness for a forbidden zero roller direction"
    extends Roll2RollDynamics.Examples.ThreeRollWebLoop(topRoll(rotationDirection = 0));
    annotation(Documentation(info = "<html><h4>Invalid roller rotation direction</h4><p>Sets the top roller direction to zero in the three-roll web loop. Initialization is expected to fail because <code>rotationDirection</code> must be -1 or +1.</p></html>"));
  end InvalidRotationDirectionCheck;

  model InvalidWinderDirectionCheck
    "Validation harness for a forbidden zero winder direction"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(unwindDrum(rotationDirection = 0));
    annotation(Documentation(info = "<html><h4>Invalid winder rotation direction</h4><p>Sets the unwind drum direction to zero in the misaligned idler line. Initialization is expected to fail because the winder requires <code>rotationDirection</code> to be -1 or +1.</p></html>"));
  end InvalidWinderDirectionCheck;

  annotation(Documentation(info = "<html><h4>Invalid rotation direction</h4>
<p>Negative scenarios that verify rejection of invalid rotation directions. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end InvalidRotationDirectionCheck;
