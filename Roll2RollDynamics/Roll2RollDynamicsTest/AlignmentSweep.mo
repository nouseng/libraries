within Roll2RollDynamicsTest;
package AlignmentSweep
  "Idler yaw cases behind the MisalignedIdlerLine alignment figure"
  extends Modelica.Icons.ExamplesPackage;

  model Aligned "Idler in tram"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(idler(yaw = 0));
    annotation(Documentation(info = "<html><h4>Aligned</h4>
<p>Run the converting line with <code>idler.yaw = 0</code> as the alignment reference. The web should settle without yaw-induced lateral walk while retaining the base tension response.</p></html>"));
  end Aligned;

  model AtGuideline "Idler yawed by the 0.01 degree converting alignment guideline"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      idler(yaw = 0.01*Modelica.Constants.pi/180));
    annotation(Documentation(info = "<html><h4>At guideline</h4>
<p>Run the converting line with idler yaw of 0.01 degrees. Compared with the aligned case, a small lateral offset should develop as the entry span approaches normal entry at the idler.</p></html>"));
  end AtGuideline;

  model TenTimesGuideline "Idler yawed by ten times the alignment guideline"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      idler(yaw = 0.1*Modelica.Constants.pi/180));
    annotation(Documentation(info = "<html><h4>Ten times guideline</h4>
<p>Run the converting line with idler yaw of 0.1 degrees to amplify the alignment disturbance. Its settled lateral offset should exceed that of the 0.01-degree case while the entry span approaches normal entry.</p></html>"));
  end TenTimesGuideline;
  annotation(Documentation(info = "<html>
<h4>Alignment sweep</h4>
<p>Compare the same web line with an aligned idler and idler yaw of 0.01 and
0.1 degrees. The cases isolate how yaw changes lateral tracking while the
transport speed, material and support settings remain those of
<code>MisalignedIdlerLine</code>.</p>
</html>"));
end AlignmentSweep;
