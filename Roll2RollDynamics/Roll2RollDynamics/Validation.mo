within Roll2RollDynamics;
class Validation
  "What the library has been checked against, and what it has not"
  extends Modelica.Icons.Information;
  annotation(Documentation(info = "<html>
<p>This page lists, for every physical effect the library claims, the
evidence behind it: a published measurement, a published theory, or only the
library's own consistency checks. The sources are cited in the
tables below and the transcribed data is in <code>Resources/Data</code>. A row marked
<em>consistency</em> means no external number exists to compare with; the
effect is then kinematic or follows from validated parts, and its docs say so.</p>
<h4>Checked against measurement</h4>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Effect</th><th>Source</th><th>Measured</th><th>Library</th><th>Where</th></tr>
<tr><td>Roll yaw steers the web, offset linear in angle</td>
<td>Yun et al. (2025), Table 4, PET 150 mm, 0.01 to 0.03 deg</td>
<td>nine points within &plusmn;10 % of one line through the origin</td>
<td>offset = yaw &times; span, tension-independent; means 0.90, 0.98, 1.01 of library at 13, 27, 37 N</td>
<td><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a> Fig. 3</td></tr>
<tr><td>Lateral response to an upstream position input, amplitude and phase</td>
<td>Shelton (1968), Fig. 4.7.4, polystyrene webs at KL = 2 and 10</td>
<td>points follow his second-order theory; phase lag reaches 150 deg</td>
<td>first-order law: within 10 % and 10 deg up to &omega;T<sub>1</sub> = 0.56, phase capped at 90 deg above</td>
<td><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a> Fig. 4</td></tr>
<tr><td>Eccentric roll ripples span tension at its turning rate</td>
<td>Branca, Pagilla and Reid (2011), Figs. 8 to 10, Euclid Web Line</td>
<td>fundamental at the roller rate; 2nd harmonic 0.17, 3rd 0.21 of it at 250 FPM</td>
<td>fundamental at 4.20 Hz against 4.24 Hz; 2nd harmonic 0.003 dragged, 0.0006 driven under a PI speed loop: the paper's harmonics are not reproduced</td>
<td><a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a> Fig. 6</td></tr>
<tr><td>Closed-loop tension tracking of a two-actuator line</td>
<td>Kim et al. (2022), Figs. 9, 11, 13</td>
<td>rewinder tracking error within &plusmn;0.3 N at 0.1 to 0.3 m/s; wound radius 78.4 to 79.3 mm</td>
<td>+0.08 to +0.24 N; 77.9 mm wound radius; unwinder held at 38 N</td>
<td><a href=\"modelica://Roll2RollDynamics.Examples.WebLines.KimTwoActuatorLine\">KimTwoActuatorLine</a></td></tr>
</table>
<h4>Checked against published theory</h4>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Effect</th><th>Source</th><th>Agreement</th><th>Where</th></tr>
<tr><td>Yaw steering transient, L/V time constant</td>
<td>Shelton (1968), Ch. III, first-order normal-entry law</td>
<td>0.25 % of the ideal offset over the transient, both signs, half speed</td>
<td><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a> Fig. 1</td></tr>
<tr><td>Span length beside an eccentric roll, its rate and phase</td>
<td>Branca et al. (2011), Eqs. (10), (11)</td>
<td>swing 0.283 mm for e = 0.300 mm; rate 7.57 of 7.99 mm/s; phase 0.799 of 0.800 rad; linear to 0.02 %</td>
<td><a href=\"modelica://Roll2RollDynamics.Components.Roller\">Roller</a> Fig. 5</td></tr>
<tr><td>Traction capacity and slip on a wrapped roll</td>
<td>capstan law; Triple-S friction curve</td>
<td>capacity and slip onset follow the capstan limit; the friction curve reproduces its defining points</td>
<td><a href=\"modelica://Roll2RollDynamics.Examples.SlipAndTractionCapacity\">SlipAndTractionCapacity</a> Fig. 1</td></tr>
</table>
<h4>Consistency only</h4>
<table border=\"1\" cellspacing=\"0\" cellpadding=\"4\">
<tr><th>Effect</th><th>What is checked</th><th>Why nothing more</th></tr>
<tr><td>Runout tension amplitude in newtons</td>
<td>proportional to runout; equals the span's elastic response to the geometric swing</td>
<td>no published experiment states the roller eccentricity it measured with; Branca's line never had its e quantified</td></tr>
<tr><td>Out-of-plane tilt (<code>tram</code>)</td>
<td>inclined spans skew by the tilt seen along them; a tilted nip reports a wedge and no crossed-axis lift</td>
<td>Yang et al. (2015) measured tilted-roller tracking, but reproducing it needs lateral bending the axial <code>Belt</code> does not carry</td></tr>
<tr><td>Nip load under crossed or wedged axes</td>
<td>closed-form face integral of the cover law; load centre moves toward the closed end</td>
<td>Good (2001) gives a beam-on-foundation model, no measurement; nothing measured for the load ripple of an eccentric nip drum</td></tr>
<tr><td>Crossed-nip steering</td>
<td>direction: web goes with the larger friction budget</td>
<td>WHRC results reported by Roisum are qualitative, web tracks to the high-load side; no magnitude published</td></tr>
<tr><td>Material inventory, wrap geometry, detachment, support reactions</td>
<td>mass balance to solver tolerance; tangency and wrap angle against constructions; force sums</td>
<td>bookkeeping, no external number applies</td></tr>
</table>
</html>"));
end Validation;
