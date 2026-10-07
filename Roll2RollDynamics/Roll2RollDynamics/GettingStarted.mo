within Roll2RollDynamics;
class GettingStarted
  "Guided walkthrough of a complete web line, built one component at a time"
  extends Modelica.Icons.Information;
  annotation(Documentation(info = "<html>
<h4>Introduction</h4>
<p>This guide shows how to set up a web line with the Roll to Roll Dynamics
library, one component type at a time. It assumes basic knowledge of Modelica
and of the Modelica Standard Library (MSL) rotational and multibody parts used
for drives and supports. Component help pages carry the equations and full
parameter descriptions; this page keeps to the settings a new model needs.</p>
<h4>Model description</h4>
<p>This guide explains the setup of
<a href=\"modelica://Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine\">MisalignedIdlerLine</a>.
Open a copy of that model to follow along.</p>

<p>The line transports 125 &micro;m polyester film, 600 mm wide, at 2 m/s and
400 N nominal tension. Six spans connect an unwind, infeed, dancer, yawed
idler, guide, outfeed and rewind. The infeed controls tension, while a 4 mrad
idler yaw produces lateral tracking. Figure 1 shows the assembled line.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/MisalignedIdlerLine/MisalignmentGif3D.gif\"
     width=\"600\"
     alt=\"Animation of the assembled seven-roll line running\">
<strong>Figure 1:</strong> The finished line, running.</p>
<p>The sections below follow the order in which the line is built: the world
component, the rolls, the spans between them, the dancer, the winders, and
finally the drives and tension control. Check the model after each stage
before adding the next.</p>
<h4>The WebWorld component</h4>
<p>Place one <a href=\"modelica://Roll2RollDynamics.WebWorld\">WebWorld</a>
at the top level, declared <code>inner</code> and named <code>world</code>.
It supplies gravity and shared component defaults. Figure 2 shows its icon.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/web-line.png\"
     width=\"140\"
     alt=\"WebWorld icon: three rolls carrying a web over a gravity field\">
<strong>Figure 2:</strong> The <code>WebWorld</code> icon.</p>
<blockquote><pre>
inner Roll2RollDynamics.WebWorld world(
  web(thickness = 125e-6, width = 0.6, density = 1390, EA = 270e3),
  tension = 400,
  lineSpeed = 2.0,
  rollRadius = 0.075,
  coreRadius = 0.076,
  lateralDynamics = true);
</pre></blockquote>
<p>Components inherit these values unless overridden locally.
<code>innerTraction</code> and <code>outerTraction</code> describe the two web
faces; <code>contactFace</code> selects the roller's default friction record.
Coordinates use <code>x</code> along the machine and <code>z</code> upward,
with gravity along <code>-z</code>.</p>
<h4>General settings</h4>
<h5>Web route</h5>
<p>Set <code>rotationDirection</code> in the Geometry dialog group:
<code>+1</code> when the roll touches the web bottom face (web passes over)
and <code>-1</code> when it touches the top face (web passes under),
Figure 3. The dancer and guide use <code>-1</code>; the other rolls use
<code>+1</code>. Tangency is solved from the connected geometry.
<code>entryAngleStart</code> and <code>exitAngleStart</code> provide initial
guesses where several geometric solutions are possible.</p>
<p style=\"text-align: center;\">
<img style=\"display: inline-block; margin-right: 12px;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/roller-over.png\"
     width=\"150\"
     alt=\"Roller icon with the web passing over the top of the roll\">
<img style=\"display: inline-block;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/roller-under.png\"
     width=\"150\"
     alt=\"Roller icon with the web passing underneath the roll\">
<br><strong>Figure 3:</strong> A roll touching the web bottom face,
<code>rotationDirection = +1</code>, left, and the top face, <code>-1</code>, right.</p>
<h5>Lateral dynamics</h5>
<p>Set <code>world.lateralDynamics = true</code> to let the web move across
the rolls. The default is false, which retains yawed tangency geometry but
disables lateral tracking. The steering theory follows Shelton [1, 2].</p>
<h5>Initialization</h5>
<p>On the Initialization tab, set <code>fixedInitialAngle = true</code>
unless a position source already fixes the shaft angle. Fix the initial speed
of undriven rollers at rest; leave <code>fixedInitialSpeed = false</code>
where an exact speed source prescribes it.</p>
<p>For this startup from rest, set <code>fixedInitialWebState = true</code>
on rollers and winders. On <code>Roller</code> this also fixes initial web speed to zero.
Belts default to <code>fixedInitialTension = true</code> and
<code>fixedInitialMomentum = true</code>, setting both elastic cell tensions
to <code>tensionNominal</code> and projected momentum to zero. The driven
surfaces share a two-second speed ramp. Starting at speed requires consistent
shaft and material initial states; see the component help.</p>
<h4>Rollers</h4>
<p>Use <a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a>
for idlers, driven rolls and dancers. Its <code>RollPort</code> connectors
<code>frame_a</code> and <code>frame_b</code> receive the incoming and outgoing
spans. Enable <code>useFlange</code> for a drive or brake.</p>
<blockquote><pre>
Roll2RollDynamics.Components.Roller infeed(
  useFlange = true,
  fixedInitialAngle = true,
  fixedInitialWebState = true,
  fixedInitialSpeed = true,
  xPosition = 0,
  zPosition = 0);
</pre></blockquote>
<p>Place fixed rolls with <code>xPosition</code> and <code>zPosition</code>.
The shell dimensions and density determine inertia.</p>
<h5>Misalignment and runout</h5>
<p>Each roll carries its own alignment faults, all zero by default and set
in the Misalignment and Runout dialog groups. <code>yaw</code> turns the axis
in plan view and steers the web, as measured by Yun et al. [3]; the idler here
uses <code>0.004</code>.
<code>tram</code> tilts one end of the roll up and the other down, which
matters for inclined spans and for a nip on that roll. <code>runout</code>
with <code>runoutPhase</code> makes the drum eccentric on its bearing axis,
giving a once-per-turn tension ripple in the neighbouring spans [4]; an
eccentric roll must then be driven by a torque or a speed controller, not
a kinematic speed source. A <code>NipRoller</code> takes the same three
settings. See <a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a>
for what each fault does and
<a href=\"modelica://Roll2RollDynamics.Validation\">Validation</a> for the
published data each has been checked against.</p>
<h4>Spans</h4>
<p>Combine adjacent rolls with
<a href=\"modelica://Roll2RollDynamics.Components.Belt\">Belt</a>. Connect
its <code>frame_a</code> to the upstream roll's <code>frame_b</code>, and
its <code>frame_b</code> to the downstream roll's <code>frame_a</code>.
The arrow in Figure 4 indicates positive transport; negative speed reverses
material flow. Span length follows from the tangency points.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/belt.png\"
     width=\"140\"
     alt=\"Belt icon: a web ribbon with a travel-direction arrow\">
<strong>Figure 4:</strong> The <code>Belt</code> icon.</p>
<p>Axial stiffness comes from <code>world.web.EA</code>.
<code>dampingTime</code> inherits <code>world.webDampingTime</code>, defaulting
to 0.01 s; set it to zero for purely elastic material response. The
<code>tension</code> output gives mean boundary tension for a controller.
For individual boundary tensions and transport speed, plot
<code>belt3.beltVariables.tensionIn</code>,
<code>belt3.beltVariables.tensionOut</code> and
<code>belt3.beltVariables.webVelocity</code>.</p>
<h4>Dancers and moving rolls</h4>
<p>Set <code>useSupport = true</code> on the dancer and connect
<code>frame_support</code> to an MSL <code>Prismatic</code> joint.
The joint and its base then set the roll position; <code>xPosition</code>
and <code>zPosition</code> are ignored.</p>
<blockquote><pre>
Roll2RollDynamics.Components.Roller dancer(
  useSupport = true, rotationDirection = -1,
  fixedInitialAngle = true, fixedInitialSpeed = true,
  fixedInitialWebState = true);
Modelica.Mechanics.MultiBody.Parts.Fixed dancerBase(r = {xMid, 0, zDeflected});
Modelica.Mechanics.MultiBody.Joints.Prismatic dancerPrismatic(
  n = {0, 0, 1}, useAxisFlange = true,
  s(start = 0, fixed = true), v(start = 0, fixed = true));
Modelica.Mechanics.Translational.Components.SpringDamper dancerSuspension(
  c = 15000, d = 400,
  s_rel0 = (2*world.tension*sin(WebWorldAngle)
    - world.g*(dancer.density*Modelica.Constants.pi*
      (dancer.radius^2 - (dancer.radius - dancer.wallThickness)^2)*dancer.width
      + world.web.density*world.web.thickness*world.web.width*
        (world.spanLength/2 + dancer.radius*2*WebWorldAngle)))/dancerSuspension.c);
</pre></blockquote>
<p>Connect the joint's axis and support flanges to the suspension flanges.
Keep the model's <code>s_rel0</code> expression: it preloads the spring against
nominal web tension and supported weight. Mounts may translate but must keep
a fixed orientation. For geometric release from a taut web, see
<a href=\"modelica://Roll2RollDynamics.Examples.RetractingIdler\">RetractingIdler</a>.</p>
<h4>Wound rolls and boundaries</h4>
<p>Use <a href=\"modelica://Roll2RollDynamics.Components.Winder\">Winder</a>
at each end of this line. Its radius and inertia change as material transfers
(Figure 5). The web connects at <code>frame_t</code>; the shaft flange
connects to a drive or brake.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Icons/winder.png\"
     width=\"140\"
     alt=\"Winder icon: concentric wound layers around a core\">
<strong>Figure 5:</strong> The <code>Winder</code> icon.</p>
<blockquote><pre>
Roll2RollDynamics.Components.Winder unwindDrum(
  winding = -1, radius0 = unwindRadius,
  fixedInitialWebState = true,
  xPosition = xStart, zPosition = zUnwindCenter, rotationDirection = 1);
</pre></blockquote>
<p>Set <code>winding = -1</code> for unwinding and <code>+1</code> for rewinding.
This is independent of the geometric <code>rotationDirection</code>.
<code>radius0</code> sets the initial wound radius and <code>coreRadius</code>
its lower limit. Here <code>unwindRadius</code> is a layout constant in the
complete model.</p>
<p>For an open end with prescribed tension, use
<a href=\"modelica://Roll2RollDynamics.Components.WebForce\">WebForce</a>, as in
<a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>.</p>
<h4>Drives and tension control</h4>
<p>The outfeed and both winders use MSL rotational <code>Speed</code> sources
with <code>exact = true</code>. Divide the shared <code>startup.y</code>
surface speed by each current wound radius, or by
<code>world.rollRadius + world.web.thickness/2</code> for the outfeed.
The infeed uses a <code>Torque</code> source.</p>
<p>A <code>LimPID</code> configured as PI regulates <code>belt2.tension</code>
to <code>world.tension</code> through the infeed torque, using
<code>k = 0.0016</code>, <code>Ti = 0.5</code> s and torque limits of
&plusmn;100 N&middot;m.
Recheck the gain and sign if the driven roll or measured span changes.</p>
<h4>Improving model fidelity</h4>
<p>Adjust <code>bearingDamping</code> for viscous shaft drag and
<code>traction</code> for web-to-roll friction. To add nip loading, enable
<code>useNip</code> and connect <code>frame_nip</code> to a
<a href=\"modelica://Roll2RollDynamics.Components.NipRoller\">NipRoller</a>.
See <a href=\"modelica://Roll2RollDynamics.Examples.SlipAndTractionCapacity\">SlipAndTractionCapacity</a>
for slip behaviour and
<a href=\"modelica://Roll2RollDynamics.Examples.NipLoading\">NipLoading</a>
for nip engagement, release and steering.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/30409\">1</a>] Shelton, J. J.,
<em>Lateral Dynamics of a Moving Web</em>, PhD dissertation, Oklahoma State
University, 1968, Chapter III.</li>
<li>[<a href=\"https://doi.org/10.1115/1.3426495\">2</a>] Shelton, J. J. and
Reid, K. N., &quot;Lateral Dynamics of an Idealized Moving Web&quot;,
<em>Journal of Dynamic Systems, Measurement, and Control</em>, 1971.</li>
<li>[<a href=\"https://doi.org/10.3390/polym17212907\">3</a>] Yun, Lee, Jang, Kim,
Kim and Lee, &quot;Sensor-Efficient Estimation of Roll Misalignment via
Side-to-Side Tension Differences in Roll-to-Roll Polymer Film Processing&quot;,
<em>Polymers</em> 17(21):2907, 2025, Table 4.</li>
<li>[<a href=\"https://hdl.handle.net/20.500.14446/321969\">4</a>] Branca, C.,
Pagilla, P. R. and Reid, K. N., &quot;Web Tension Behavior in the Presence of
Eccentric Rollers: Modeling and Validation&quot;, Proc. International Conference
on Web Handling, Oklahoma State University, 2011.</li>
</ul>
</html>"
    ));
end GettingStarted;
