within Roll2RollDynamicsTest;
package OpenBeltSweep
  "Initial tension, damping and bearing drag cases behind the OpenBeltDrive startup figures"
  model LowTension "Spans preloaded to half the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(tension = 75));
  end LowTension;

  model HighTension "Spans preloaded to twice the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(tension = 300));
  end HighTension;

  model LowDamping "Material damping time one tenth of the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(webDampingTime = 0.001));
  end LowDamping;

  model HighDamping "Material damping time ten times the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(webDampingTime = 0.1));
  end HighDamping;

  model HighBearing "Bearing drag one hundred times the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(bearingDamping = 0.1));
  end HighBearing;
  annotation(Documentation(info = "<html>
<h4>Open belt sweep</h4>
<p>Compare startup of <code>OpenBeltDrive</code> with lower and higher initial
tension, lower and higher material damping, and increased bearing drag.
Each case changes one setting so its effect on speed and tension can be
compared with the base example.</p>
</html>"));
end OpenBeltSweep;
