within Roll2RollDynamicsTest;
package AlignmentSweep
  "Idler yaw cases behind the MisalignedIdlerLine alignment figure"
  model Aligned "Idler in tram"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(idler(yaw = 0));
  end Aligned;

  model AtGuideline "Idler yawed by the 0.01 degree converting alignment guideline"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      idler(yaw = 0.01*Modelica.Constants.pi/180));
  end AtGuideline;

  model TenTimesGuideline "Idler yawed by ten times the alignment guideline"
    extends Roll2RollDynamics.Examples.WebLines.MisalignedIdlerLine(
      idler(yaw = 0.1*Modelica.Constants.pi/180));
  end TenTimesGuideline;
  annotation(Documentation(info = "<html>
<h4>Alignment sweep</h4>
<p>Compare the same web line with an aligned idler and idler yaw of 0.01 and
0.1 degrees. The cases isolate how yaw changes lateral tracking while the
transport speed, material and support settings remain those of
<code>MisalignedIdlerLine</code>.</p>
</html>"));
end AlignmentSweep;
