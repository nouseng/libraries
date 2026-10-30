within MicroDuck;
package Examples "The robot standing and walking, compared with MuJoCo"
  extends Modelica.Icons.ExamplesPackage;

  annotation(Documentation(info = "<html>
<h4>Examples</h4>
<p><a href=\"modelica://MicroDuck.Examples.Standing\">Standing</a> drops the robot onto the ground with the servos holding the home pose, <a href=\"modelica://MicroDuck.Examples.WalkingTest\">WalkingTest</a> lets the neural network policy walk it in a straight line, and <a href=\"modelica://MicroDuck.Examples.NousLogo\">NousLogo</a> steers two robots simultaneously along the circular arc and diagonal chord of the Nous logo. The baseline standing and walking examples are compared with MuJoCo, and the checks of single components are in the <code>MicroDuckTest</code> library.</p>
</html>"));
end Examples;
