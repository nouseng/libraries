within Roll2RollDynamicsTest;
package OpenBeltSweep
  "Initial tension, damping and bearing drag cases behind the OpenBeltDrive startup figures"
  extends Modelica.Icons.ExamplesPackage;

  model LowTension "Spans preloaded to half the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(tension = 75));
    annotation(Documentation(info = "<html><h4>Low tension</h4>
<p>Start the open belt drive with <code>world.tension = 75</code> N, half the base preload. The reduced traction capacity should slow the free pulley acceleration compared with the base case.</p></html>"));
  end LowTension;

  model HighTension "Spans preloaded to twice the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(tension = 300));
    annotation(Documentation(info = "<html><h4>High tension</h4>
<p>Start the open belt drive with <code>world.tension = 300</code> N, twice the base preload. The increased traction capacity should accelerate the free pulley faster and increase startup overshoot.</p></html>"));
  end HighTension;

  model LowDamping "Material damping time one tenth of the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(webDampingTime = 0.001));
    annotation(Documentation(info = "<html><h4>Low damping</h4>
<p>Start the open belt drive with <code>world.webDampingTime = 0.001</code> s. Compared with the base case, reduced material damping should prolong the free pulley speed and span tension oscillations.</p></html>"));
  end LowDamping;

  model HighDamping "Material damping time ten times the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(webDampingTime = 0.1));
    annotation(Documentation(info = "<html><h4>High damping</h4>
<p>Start the open belt drive with <code>world.webDampingTime = 0.1</code> s. Increased material damping should suppress startup ringing in the free pulley speed and span tensions.</p></html>"));
  end HighDamping;

  model HighBearing "Bearing drag one hundred times the line default"
    extends Roll2RollDynamics.Examples.OpenBeltDrive(world(bearingDamping = 0.1));
    annotation(Documentation(info = "<html><h4>High bearing</h4>
<p>Run the open belt drive with <code>world.bearingDamping = 0.1</code> N&middot;m&middot;s/rad. The greater drag should increase the steady tight-side and slack-side tension difference needed to turn the free pulley.</p></html>"));
  end HighBearing;
  annotation(Documentation(info = "<html>
<h4>Open belt sweep</h4>
<p>Compare startup of <code>OpenBeltDrive</code> with lower and higher initial
tension, lower and higher material damping, and increased bearing drag.
Each case changes one setting so its effect on speed and tension can be
compared with the base example.</p>
</html>"));
end OpenBeltSweep;
