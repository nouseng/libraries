within Roll2RollDynamics;
package Functions
  "Friction, tangent geometry and animation helper functions"
  extends Modelica.Icons.FunctionsPackage;
  annotation(Documentation(info = "<html>
<p>The analytic pieces the components are built from.
<a href=\"modelica://Roll2RollDynamics.Functions.tripleS\">tripleS</a> is the
regularized dry-friction characteristic used between web and roll, assembled
from three cubic
<a href=\"modelica://Roll2RollDynamics.Functions.sFunction\">sFunction</a> transitions,
and <a href=\"modelica://Roll2RollDynamics.Functions.stressColor\">stressColor</a> maps
a web stress onto the diverging colour ramp the spans and wraps are drawn
in.</p>
</html>"));
end Functions;
