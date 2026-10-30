within MicroDuck;
class Introduction "Introduction to the MicroDuck library"
  extends Modelica.Icons.Information;

  annotation(Documentation(info = "<html>
<h4>Introduction</h4>
<p>The MicroDuck library, developed by <a href=\"https://www.linkedin.com/in/senolvedat\">Vedat Senol</a> and <a href=\"https://www.linkedin.com/in/ogulcanisitman\">Ogulcan Isitman</a> at <a href=\"https://nouseng.co\">Nous Engineering &amp; Research</a>, models the Microduck, a 25 cm biped with 14 servo-driven joints, as a MultiBody plant. Joint angle targets go in; joint angles, sensor frames, motor currents, temperatures and contact forces come out. The deployed walking policy is included as <a href=\"modelica://MicroDuck.Components.WalkingPolicy\">WalkingPolicy</a>.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Examples/Standing/standing.png\"
     width=\"600\"
     alt=\"Sole loads and trunk height of the standing robot\">
<strong>Figure 1:</strong> Robot standing on its soles, compared with MuJoCo.</p>
<h4>Components</h4>
<p><a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> is the complete model: the trunk, fourteen <a href=\"modelica://MicroDuck.Components.Link\">Link</a> parts for legs and head, each driven by a <a href=\"modelica://MicroDuck.Components.PositionServo\">PositionServo</a> through 2&deg; of gear play, an optional <a href=\"modelica://MicroDuck.Components.Battery\">Battery</a> pack and eight <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a> elements under each sole. It is generated from the MJCF file (the MuJoCo XML model) used to train the policies, so masses, inertias, joint offsets and site frames match it. The beak is rigid, as in training; <code>useMouth</code> adds a fifteenth link that opens it.</p>
<h4>Examples</h4>
<ul>
<li><a href=\"modelica://MicroDuck.Examples.Standing\">Standing</a>: the robot on the ground, against MuJoCo.</li>
<li><a href=\"modelica://MicroDuck.Examples.WalkingTest\">WalkingTest</a>: the robot walking under the policy, against MuJoCo.</li>
</ul>
<p>The <a href=\"modelica://MicroDuckTest\">MicroDuckTest</a> library checks the kinematics, joint limits and sole contact:</p>
<ul>
<li><a href=\"modelica://MicroDuckTest.ForwardKinematicsCheck\">ForwardKinematicsCheck</a>: the six sensor site frames at a random pose, against MuJoCo.</li>
<li><a href=\"modelica://MicroDuckTest.JointLimitTest\">JointLimitTest</a>: three servos driven past their upper limits, against MuJoCo.</li>
<li><a href=\"modelica://MicroDuckTest.DropTest\">DropTest</a>: a 0.8 kg mass dropped 50 mm onto one <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a>, against MuJoCo.</li>
<li><a href=\"modelica://MicroDuckTest.SlideTest\">SlideTest</a>: a 0.8 kg mass sliding to rest on one <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a>, against MuJoCo.</li>
</ul>
<h4>Why Modelica</h4>
<p>Modelica suits the physical plant; it is a poor choice for fast walking experiments.</p>
<ul>
<li><b>Multibody mechanics:</b> MultiBody builds the equations of motion from the tree, masses and inertias. Forward kinematics agree with MuJoCo to 4e-13 m.</li>
<li><b>Physical servo:</b> motor equations with identified back-EMF (electromotive force), resistance, rotor inertia and friction; change the supply voltage and the physics follows.</li>
<li><b>Acausal modeling:</b> equations are written once, and a single <code>connect</code> gives force balance, so a joint can be driven by a servo torque or a prescribed motion without rewriting.</li>
<li><b>Mixed domains and reuse:</b> mechanics, motor, 50 Hz controller and contact in one model, which other controllers or payloads can be connected to.</li>
<li><b>Speed:</b> a 6 s walk takes about 6 s with fixed-step Runge-Kutta, against 0.5 s in MuJoCo. The contact has no states by default, which is what allows a fixed step.</li>
<li><b>Contact:</b> MuJoCo solves all contacts together; <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a> reimplements its law point by point.</li>
</ul>
<p>A sensible split is to train in a fast contact engine such as MuJoCo and verify and integrate the plant in Modelica, which is how this library was built.</p>
<h4>Limitations</h4>
<p>The optional mouth is estimated from the meshes, and only the soles collide with the ground. Standing needs a stiffer servo gain than walking, because standing relies on the policy.</p>
</html>"));
end Introduction;
