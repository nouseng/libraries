within Roll2RollDynamics.Functions;
function tripleS
  "Point-symmetric regularized dry-friction characteristic"
  extends Modelica.Icons.Function;
  input Modelica.Units.SI.Velocity xMax
    "Velocity of the adhesion peak";
  input Modelica.Units.SI.Velocity xSat
    "Velocity at which sliding friction saturates";
  input Real yMax(unit = "1") "Peak adhesion friction coefficient";
  input Real ySat(unit = "1") "Saturated sliding friction coefficient";
  input Modelica.Units.SI.Velocity x "Relative sliding velocity";
  input Real viscousSlope(min = 0, unit = "s/m") = 0
    "Slope of the linear viscous term the characteristic rises on";
  output Real y(unit = "1") "Signed regularized friction coefficient";
algorithm
  assert(xMax > 0, "tripleS requires xMax > 0");
  assert(xSat > xMax, "tripleS requires xSat > xMax");
  assert(yMax >= ySat, "tripleS requires yMax >= ySat");
  if x > xMax then
    y := sFunction(xMax, xSat, yMax, ySat, x);
  elseif x < -xMax then
    y := sFunction(-xSat, -xMax, -ySat, -yMax, x);
  else
    y := sFunction(-xMax, xMax, -yMax, yMax, x);
  end if;
  y := y + viscousSlope*x;
  annotation(
    smoothOrder = 1,
    Documentation(info = "<html>
<h4>Triple-S friction characteristic</h4>
<p>This function contains the point-symmetric dry-friction regularization the
web-to-roll contact runs on, composed from three cubic S-functions.</p>
<p>Starting at zero relative velocity, the magnitude rises smoothly to
<code>yMax</code> at <code>xMax</code>, then falls smoothly to
<code>ySat</code> at <code>xSat</code> and rises again on
<code>viscousSlope</code>. That is the whole Stribeck characteristic, static
drop and viscous rise together, returned as one coefficient: nothing that uses
the curve adds a term of its own. <code>viscousSlope</code> defaults to zero,
which leaves the dry law.</p>
<p>Figure 1 shows it against the classical steady-state Stribeck curve on the
same axes and the same parameters. The classical curve is a magnitude that a
separate law has to give a sign, so it jumps the whole way from
<code>-yMax</code> to <code>+yMax</code> across zero slip and is undefined
there. This function is signed and passes through zero, which is what lets a
roll reverse, stand still or creep without an event or a separate sticking
state. Away from the adhesion region the two agree, so the regularization
costs nothing where the contact is actually sliding.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Functions/triple-s-curve.png\"
     width=\"600\"
     alt=\"Signed Triple-S curve passing through zero beside the classical Stribeck magnitude\">
<strong>Figure 1:</strong> Triple-S and classical Stribeck coefficients across
zero slip.</p>
<p>This regularization deliberately replaces an exact locked/static-friction
state with a small adhesion-slip region. It is therefore continuously
differentiable but permits small relative motion near zero speed.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://portal.research.lu.se/en/publications/friction-models-and-friction-compensation\">1</a>]
H. Olsson, K. J. Astrom, C. Canudas de Wit, M. Gafvert and P. Lischinsky,
&quot;Friction Models and Friction Compensation&quot;, European Journal of
Control, Vol. 4, No. 3, 1998, pp. 176&ndash;195. The Stribeck curve this
function regularizes. Accessed 2026-09-04.</li>
<li>[<a href=\"https://dl.icdst.org/pdfs/files3/ad7608c18e740b0e402c025fa3187de8.pdf\">2</a>]
R. G. Budynas and J. K. Nisbett, &quot;Shigley&apos;s Mechanical Engineering
Design&quot;, 10th ed. in SI units, McGraw-Hill, 2015, sec. 12&ndash;1 (page
numbers are those of the SI printing linked here). The lubrication regimes
named on p. 610 are what the three branches of this function stand for. The
curve through them is not in the text. Accessed 2026-09-04.</li>
</ul>
</html>"));
end tripleS;
