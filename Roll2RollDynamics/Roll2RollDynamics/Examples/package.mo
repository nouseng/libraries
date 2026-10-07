within Roll2RollDynamics;
package Examples
  "Worked examples of web mechanics and complete winding lines"
  extends Modelica.Icons.ExamplesPackage;
  annotation(Documentation(info = "<html>
<p>Six focused examples demonstrate web geometry, traction, steering and nip
loading. Two assembled examples in
<a href=\"modelica://Roll2RollDynamics.Examples.WebLines\">WebLines</a>
combine transport and winding-line dynamics.
Material conservation is built into every span, wrap and winder.</p>
<ul>
<li><a href=\"modelica://Roll2RollDynamics.Examples.OpenBeltDrive\">OpenBeltDrive</a>
is a flat-belt drive between unequal pulleys, checked against both the
common-tangent wrap angles and the capstan tension limit.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.ThreeRollWebLoop\">ThreeRollWebLoop</a>
closes a web path round three rolls of different sizes and checks that the
wraps add to a full turn.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.SlipAndTractionCapacity\">SlipAndTractionCapacity</a>
sweeps signed slip and compares wrap-only and nip-loaded traction with their
total dry capacity limits.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.NipLoading\">NipLoading</a>
closes and releases an undriven nip against a running web through one
composite nip connection, then repeats the stroke with crossed or wedged
axes and with an eccentric drum.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.SheltonSteering\">SheltonSteering</a>
compares small-yaw steering with an independent first-order analytical
prediction for a horizontal span.</li>
<li><a href=\"modelica://Roll2RollDynamics.Examples.RetractingIdler\">RetractingIdler</a> releases and re-engages a movable roll while the web follows the remaining path.</li>
</ul>
<p>For a complete line, start with
<a href=\"modelica://Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine\">MisalignedIdlerLine</a>.</p>
</html>"));
end Examples;
