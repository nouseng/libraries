within Roll2RollDynamics;
class Introduction
  "Overview of the library contents"
  extends Modelica.Icons.Information;
  annotation(Documentation(info = "<html>
<p>Roll to Roll Dynamics simulates the transport of a continuous web between
rolls in converting lines, printing presses, coaters, calenders and belt
drives, including tension control, traction, winding and lateral tracking.
Figure 1 shows a film line with a translating dancer and a yawed idler.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/MisalignedIdlerLine/MisalignmentGif3D.gif\"
     width=\"600\"
     alt=\"Animation of a seven-roll film line running from unwind to rewind\">
<strong>Figure 1:</strong> A film line running, from unwind to rewind.</p>
<p>The library defines its own
<a href=\"modelica://Roll2RollDynamics.Utilities.Interfaces\">web and nip connectors</a>
for geometry, loads and material transport. It uses MSL components for rigid-body
mechanics, joints, drives and controllers. Follow
<a href=\"modelica://Roll2RollDynamics.GettingStarted\">Getting Started</a>
to configure the line.</p>
<h4>Components</h4>
<h5>WebWorld</h5>
<p><a href=\"modelica://Roll2RollDynamics.WebWorld\">WebWorld</a> supplies shared
material, tension, speed, geometry and friction defaults. Components can
override these locally. Coordinates use <code>x</code> along the machine and
<code>z</code> upward, with gravity along <code>-z</code>.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/web-line.png\"
     width=\"140\"
     alt=\"WebWorld icon: three rolls carrying a web over a gravity field\">
<strong>Figure 2:</strong> The <code>WebWorld</code> icon.</p>
<h5>Roller</h5>
<p><a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a>
is an idler, driven roll or translating dancer, with optional nip loading,
drum runout and lateral tracking under fixed yaw.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/roller-over.png\"
     width=\"150\"
     alt=\"Roller icon with the web passing over the top of the roll\">
<strong>Figure 3:</strong> The <code>Roller</code> icon.</p>
<h5>Winder</h5>
<p><a href=\"modelica://Roll2RollDynamics.Components.Winder\">Winder</a>
is an unwind or rewind whose radius and inertia change as material transfers.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/winder.png\"
     width=\"140\"
     alt=\"Winder icon: concentric wound layers around a core\">
<strong>Figure 4:</strong> The <code>Winder</code> icon.</p>
<h5>Belt</h5>
<p><a href=\"modelica://Roll2RollDynamics.Components.Belt\">Belt</a>
is a free span with elasticity, Kelvin-Voigt damping and longitudinal inertia.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/belt.png\"
     width=\"140\"
     alt=\"Belt icon: a web ribbon with a travel-direction arrow\">
<strong>Figure 5:</strong> The <code>Belt</code> icon.</p>
<h5>NipRoller</h5>
<p><a href=\"modelica://Roll2RollDynamics.Components.NipRoller\">NipRoller</a>
is an undriven drum pressed against a roller to increase traction capacity.
Its misalignment and runout shape the load, and a crossed nip steers the
tracking web.</p>
<h5>WebForce</h5>
<p><a href=\"modelica://Roll2RollDynamics.Components.WebForce\">WebForce</a>
is a prescribed tensile load at an open web boundary.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/web-force.png\"
     width=\"140\"
     alt=\"WebForce icon: a force arrow acting on a web connection\">
<strong>Figure 6:</strong> The <code>WebForce</code> icon.</p>
<h4>Functions, utilities and resources</h4>
<p><a href=\"modelica://Roll2RollDynamics.Functions\">Functions</a> provides
geometry, friction and animation helpers.
<a href=\"modelica://Roll2RollDynamics.Utilities\">Utilities</a> contains
material records, connectors and internal parts.
<a href=\"modelica://Roll2RollDynamics.Resources\">Resources</a> holds bundled
images and data.</p>
<h4>How it works</h4>
<p>Roll positions, radii and <code>rotationDirection</code> define the web route;
the connected geometry determines tangency points, span lengths and wraps.
Each span and wrap conserves its own material inventory. Elastic stretch,
damping and longitudinal inertia determine tension, while friction transfers
force between the web and rolls. A translating dancer changes both geometry
and stored material.</p>
<h4>Examples</h4>
<p>The <a href=\"modelica://Roll2RollDynamics.Examples\">Examples</a> package
starts with assembled lines in
<a href=\"modelica://Roll2RollDynamics.Examples.WebLines\">WebLines</a>,
followed by focused demonstrations of individual effects:</p>
<ul>
<li><a href=\"modelica://Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine\">MisalignedIdlerLine</a>:
winding, tension control, a dancer and fixed-yaw tracking.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.WebLines.KimTwoActuatorLine\">KimTwoActuatorLine</a>:
winding-line speed, tension and radius compared with published measurements.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.OpenBeltDrive\">OpenBeltDrive</a>:
unequal pulleys, common-tangent geometry and the capstan tension limit.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.ThreeRollWebLoop\">ThreeRollWebLoop</a>:
a closed path whose wrap angles sum to a full turn.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.SlipAndTractionCapacity\">SlipAndTractionCapacity</a>:
signed slip and traction capacity with and without nip loading.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.NipLoading\">NipLoading</a>:
nip engagement, spin-up and release, then misalignment, runout and
steering against a yawed roller.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>:
fixed-yaw tracking compared with Shelton's first-order response.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.RetractingIdler\">RetractingIdler</a>:
loss and recovery of wrap as an idler withdraws and returns.</li>
</ul>
<h4>Limitations</h4>
<p>The web must remain taut and roller mount orientations fixed. Spans remain
straight: bending, wrinkling, buckling and distributed wave propagation are
outside the model. Lateral motion has one degree of freedom per roll, with
the same offset at entry and exit; the span's lateral shape is not resolved.</p>
</html>"
    ));
end Introduction;
