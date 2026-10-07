within Roll2RollDynamics.Functions;
function spanDirection "Span direction with regular forward derivatives"
  extends Modelica.Icons.Function;
  input Modelica.Units.SI.Position x "Machine-direction displacement";
  input Modelica.Units.SI.Position y "Cross-machine displacement";
  input Modelica.Units.SI.Position z "Vertical displacement";
  input Integer axis "Direction component";
  output Real direction(unit="1") "Span direction component";
protected
  Modelica.Units.SI.Length length "Regularized span length";
  Modelica.Units.SI.Position displacement[3] "End-to-end displacement";
algorithm
  displacement := {x, y, z};
  length := sqrt(displacement*displacement + Roll2RollDynamics.Utilities.Types.lengthTol^2);
  direction := displacement[axis]/length;
  annotation(Inline=false, derivative=Roll2RollDynamics.Functions.spanDirectionRate,
    Documentation(info="<html><p>Forward geometric derivatives avoid recovering a length derivative by dividing by a zero component of a planar span direction.</p></html>"));
end spanDirection;
