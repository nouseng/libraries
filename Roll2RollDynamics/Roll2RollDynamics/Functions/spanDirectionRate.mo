within Roll2RollDynamics.Functions;
function spanDirectionRate "First derivative of span direction"
  extends Modelica.Icons.Function;
  input Modelica.Units.SI.Position x "Machine-direction displacement";
  input Modelica.Units.SI.Position y "Cross-machine displacement";
  input Modelica.Units.SI.Position z "Vertical displacement";
  input Integer axis "Direction component";
  input Modelica.Units.SI.Velocity vx "Machine-direction displacement rate";
  input Modelica.Units.SI.Velocity vy "Cross-machine displacement rate";
  input Modelica.Units.SI.Velocity vz "Vertical displacement rate";
  output Real directionRate(unit="1/s") "Rate of span direction component";
protected
  Modelica.Units.SI.Length length "Regularized span length";
  Modelica.Units.SI.Position displacement[3] "End-to-end displacement";
  Modelica.Units.SI.Velocity velocity[3] "End-to-end displacement rate";
algorithm
  displacement := {x, y, z};
  velocity := {vx, vy, vz};
  length := sqrt(displacement*displacement + Roll2RollDynamics.Utilities.Types.lengthTol^2);
  directionRate := velocity[axis]/length - displacement[axis]*(displacement*velocity)/length^3;
  annotation(Inline=false, derivative(order=2)=Roll2RollDynamics.Functions.spanDirectionAcceleration,
    Documentation(info="<html><p>Analytic derivative of the regularized direction vector.</p></html>"));
end spanDirectionRate;
