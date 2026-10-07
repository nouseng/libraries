within Roll2RollDynamics.Functions;
function stressColor
  "Diverging animation color for a web stress about its nominal"
  extends Modelica.Icons.Function;
  input Modelica.Units.SI.Stress stress "Axial web stress to be drawn";
  input Modelica.Units.SI.Stress stressLow "Stress drawn in lowColor";
  input Modelica.Units.SI.Stress stressHigh "Stress drawn in highColor";
  input Modelica.Units.SI.Stress stressNominal "Stress drawn in midColor";
  input Real gamma(min = Modelica.Constants.small)
    "Contrast exponent; below one spreads deviations near nominal";
  input Real lowColor[3] "Color at and below stressLow";
  input Real midColor[3] "Color midway between the two limits";
  input Real highColor[3] "Color at and above stressHigh";
  output Real color[3] "Color drawn for this stress";
protected
  Real deviation "Signed deviation from the nominal stress, as a fraction of its side of the window";
  Real graded "Deviation after the contrast exponent";
algorithm
  deviation := if stress < stressNominal then
    max(-1, (stress - stressNominal)/
      max(stressNominal - stressLow, Roll2RollDynamics.Utilities.Types.lengthTol))
  else
    min(1, (stress - stressNominal)/
      max(stressHigh - stressNominal, Roll2RollDynamics.Utilities.Types.lengthTol));
  graded := sign(deviation)*abs(deviation)^gamma;
  if graded < 0 then
    color := midColor + (lowColor - midColor)*(-graded);
  else
    color := midColor + (highColor - midColor)*graded;
  end if;
  annotation(Documentation(info = "<html>
<p>Maps a web stress onto a diverging color ramp: <code>lowColor</code> at
<code>stressLow</code>, <code>midColor</code> at <code>stressNominal</code>, and
<code>highColor</code> at <code>stressHigh</code>. Each side of nominal ramps
over its own distance, so an asymmetric window still draws the nominal
stress in <code>midColor</code>. Stresses outside the window are clamped.</p>
<h4>Notes</h4>
<p>With <code>gamma = 1</code> the color moves in proportion to the stress
deviation, so a span drawn halfway to <code>highColor</code> really is halfway
to <code>stressHigh</code>. <code>gamma</code> below one exaggerates small
deviations: the deviation is raised to <code>gamma</code> before it is used,
which keeps a converged line readable when every span sits within a percent of
nominal, at the price of a two-percent excursion already looking hot. Use it
deliberately, not as a default.</p>
<p>Three stops rather than two matter for the same reason. A two-stop ramp
puts its least saturated color exactly where a healthy line sits, which is
the one place the animation has to stay readable. Here nominal is a definite
color of its own, and the sign of the deviation is read from the hue.</p>
</html>"));
end stressColor;
