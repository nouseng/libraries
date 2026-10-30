within;
package MicroDuck "Rigid-body model of the Pollen Robotics Microduck"
  extends Modelica.Icons.Package;

  annotation(
    version = "0.1.0",
    uses(Modelica(version = "4.1.0")),
    Icon(graphics = {
      Polygon(points = {{2, -70}, {11, -65}, {4, -48}, {1, -47}, {3, -44}, {14, -26}, {15, -25}, {16, -18}, {13, -17}, {13, -15}, {9, -14}, {9, -11}, {-4, -12}, {-4, -13}, {-8, -13}, {-9, -14}, {-9, -20}, {-17, -24}, {-19, -28}, {-18, -35}, {-15, -40}, {-8, -57}, {-5, -61}, {-6, -62}, {-3, -68}}, lineColor = {64, 64, 64}, fillColor = {140, 140, 140}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-7, -75}, {18, -75}, {22, -72}, {22, -68}, {22, -67}, {12, -65}, {9, -59}, {7, -58}, {3, -58}, {1, -56}, {-2, -56}, {-5, -58}, {-9, -65}, {-11, -66}, {-11, -72}}, lineColor = {64, 64, 64}, fillColor = {204, 82, 25}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-18, -35}, {-16, -35}, {-16, -33}, {-13, -32}, {-12, -13}, {7, -13}, {7, -11}, {8, -11}, {8, -10}, {9, -7}, {10, 2}, {14, 4}, {15, 6}, {15, 8}, {14, 11}, {11, 12}, {-1, 12}, {-18, 13}, {-29, 12}, {-34, 8}, {-35, 1}, {-34, -4}, {-32, -8}, {-33, -32}, {-22, -32}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Polygon(points = {{2, 1}, {7, 1}, {16, 5}, {7, 33}, {7, 41}, {6, 42}, {4, 42}, {5, 43}, {5, 45}, {3, 45}, {9, 46}, {10, 50}, {10, 54}, {9, 57}, {7, 59}, {-12, 59}, {-14, 54}, {-14, 49}, {-12, 46}, {-6, 45}, {-7, 45}, {-7, 43}, {-6, 42}, {-9, 42}, {-9, 37}, {-10, 36}, {-9, 31}, {-5, 28}, {-4, 25}, {-6, 24}, {-5, 20}, {-2, 13}, {-1, 13}, {1, 10}, {-0, 7}}, lineColor = {64, 64, 64}, fillColor = {80, 85, 90}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-19, -70}, {-14, -69}, {-5, -64}, {-8, -57}, {-9, -57}, {-10, -51}, {-17, -37}, {-14, -32}, {-14, -34}, {3, -33}, {3, -26}, {6, -22}, {7, -18}, {5, -15}, {3, -14}, {2, -11}, {-10, -12}, {-11, -13}, {-15, -13}, {-16, -16}, {-19, -16}, {-36, -25}, {-39, -29}, {-38, -34}, {-32, -46}, {-30, -49}, {-24, -48}, {-23, -51}, {-27, -53}}, lineColor = {64, 64, 64}, fillColor = {172, 164, 153}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-23, -75}, {2, -75}, {6, -72}, {6, -66}, {5, -65}, {-3, -64}, {-7, -58}, {-10, -56}, {-12, -56}, {-15, -58}, {-19, -58}, {-21, -59}, {-25, -66}, {-27, -67}, {-27, -72}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Polygon(points = {{13, 42}, {35, 42}, {38, 44}, {38, 49}, {35, 49}, {34, 51}, {30, 51}, {30, 50}, {15, 51}, {14, 50}, {11, 57}, {8, 58}, {5, 58}, {3, 57}, {2, 55}, {3, 50}, {-2, 50}, {-6, 57}, {-10, 58}, {-14, 56}, {-15, 50}, {-29, 51}, {-29, 50}, {-32, 50}, {-33, 49}, {-32, 44}, {-30, 43}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Polygon(points = {{18, 49}, {38, 49}, {39, 69}, {37, 73}, {33, 75}, {-3, 72}, {-20, 69}, {-25, 66}, {-29, 62}, {-33, 51}, {-33, 49}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Polygon(points = {{27, 52}, {32, 52}, {34, 55}, {34, 63}, {32, 69}, {27, 68}, {25, 65}, {17, 65}, {17, 56}, {25, 56}}, lineColor = {64, 64, 64}, fillColor = {66, 71, 77}, fillPattern = FillPattern.Solid)}),
    Documentation(info = "<html>
<h4>MicroDuck</h4>
<h5>What is the MicroDuck library?</h5>
<p>MultiBody plant model of the Microduck, a 25 cm biped from Pollen Robotics, developed by <a href=\"https://nouseng.co\">Nous Engineering &amp; Research</a>: a trunk, two five-joint legs, a four-joint head with a moving beak, a position servo with gear play at every joint, a two-cell battery pack, servo thermal networks and ground contact under both soles.</p>
<h5>How does it work?</h5>
<p><a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> is generated from the training MJCF file (MuJoCo XML model) of the robot by <code>tools/gen_duck.py</code>. Its joints use <a href=\"modelica://MicroDuck.Components.PositionServo\">PositionServo</a> on the identified XL330 (Dynamixel servo) motor, behind the 2&deg; gear play of the training model, and its soles use <a href=\"modelica://MicroDuck.Components.PointContact\">PointContact</a>. <a href=\"modelica://MicroDuck.Components.WalkingPolicy\">WalkingPolicy</a> runs the neural network policy of the robot with the command delay and late speed reading it was trained with.
See <a href=\"modelica://MicroDuck.Introduction\">Introduction</a>, <a href=\"modelica://MicroDuck.GettingStarted\">Getting Started</a> and <a href=\"modelica://MicroDuck.Examples\">Examples</a>.</p>
<h5>Limitations</h5>
<p>No training model has the mouth, so the optional mouth servo has its pivot and mass estimated from the meshes. Only the soles collide with the ground. The randomized parts of training, such as the varying delay, sensor noise and supply voltage, are replaced by fixed mean values or left out.</p>
<h5>Licence</h5>
<p>Copyright &copy; 2026 <a href=\"https://www.linkedin.com/in/senolvedat\">Vedat Senol</a> and <a href=\"https://www.linkedin.com/in/ogulcanisitman\">Ogulcan Isitman</a> (<a href=\"https://nouseng.co\">Nous Engineering &amp; Research</a>). Released under the BSD 3-Clause Licence.</p>
<p>Contact: <a href=\"mailto:vedat@nouseng.co\">vedat@nouseng.co</a>, <a href=\"mailto:ogulcan@nouseng.co\">ogulcan@nouseng.co</a>.</p>
</html>"));
end MicroDuck;
