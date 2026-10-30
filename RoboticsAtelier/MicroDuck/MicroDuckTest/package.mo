within;
package MicroDuckTest "Checks of the MicroDuck library components"
  extends Modelica.Icons.ExamplesPackage;

  annotation(
    version = "0.1.0",
    uses(Modelica(version = "4.1.0"), MicroDuck(version = "0.1.0")),
    Documentation(info = "<html>
<h4>MicroDuck tests</h4>
<p>Verification models for the <a href=\"modelica://MicroDuck\">MicroDuck</a> library against the MuJoCo reference simulator. Each model checks its result with <code>assert</code> statements at the end of the simulation:</p>
<ul>
<li><a href=\"modelica://MicroDuckTest.ForwardKinematicsCheck\">ForwardKinematicsCheck</a> sets the 14 joints to a random reference sample with the trunk fixed and checks the position and rotation matrix of all six sensor site frames against MuJoCo.</li>
<li><a href=\"modelica://MicroDuckTest.JointLimitTest\">JointLimitTest</a> drives three joints (<code>leftHipYaw</code>, <code>leftHipRoll</code> and <code>neckPitch</code>) past their upper mechanical limits and checks that each settles at its stop against MuJoCo.</li>
<li><a href=\"modelica://MicroDuckTest.DropTest\">DropTest</a> drops a 0.8 kg mass from 50 mm onto one <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a> and checks the peak penetration and settled resting depth against MuJoCo.</li>
<li><a href=\"modelica://MicroDuckTest.SlideTest\">SlideTest</a> launches a 0.8 kg mass at 0.5 m/s across one <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a> with friction and checks the stopping distance against MuJoCo.</li>
</ul>
</html>"));
end MicroDuckTest;
