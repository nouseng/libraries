within ;
package Roll2RollDynamicsTest "Regression scenarios for Roll to Roll Dynamics"
  extends Modelica.Icons.ExamplesPackage;

  annotation(
    version = "1.0.0",
    uses(Modelica(version = "4.1.0"), Roll2RollDynamics(version = "1.0.0")),
    Documentation(info = "<html>
<h4>Roll to Roll Dynamics tests</h4>
<p>Regression scenarios for components, belt dynamics, material conservation,
friction, nip contact, steering, drum runout and geometry. Scenario families
share fixtures inside their packages; the sweep packages provide documentation
figure inputs.</p>
<p>Load Roll2RollDynamics before this package, then open a scenario to inspect
or simulate it. Partial models are fixtures. Some scenarios deliberately reject
invalid parameters; their assertion failures are expected.</p>
<p>The reference runners select simulation settings, check expected failures
and perform numerical post-processing. Use those runners for regression
verification; simulating an individual model does not perform every check.</p>
</html>"));
end Roll2RollDynamicsTest;
