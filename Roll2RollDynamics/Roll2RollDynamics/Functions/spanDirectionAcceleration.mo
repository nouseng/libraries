within Roll2RollDynamics.Functions;
function spanDirectionAcceleration "Derivative of the span direction rate"
  extends Modelica.Icons.Function;
  input Modelica.Units.SI.Position x "Machine-direction displacement";
  input Modelica.Units.SI.Position y "Cross-machine displacement";
  input Modelica.Units.SI.Position z "Vertical displacement";
  input Integer axis "Direction component";
  input Modelica.Units.SI.Velocity vx "Machine-direction rate argument";
  input Modelica.Units.SI.Velocity vy "Cross-machine rate argument";
  input Modelica.Units.SI.Velocity vz "Vertical rate argument";
  input Modelica.Units.SI.Acceleration ax "Derivative of vx";
  input Modelica.Units.SI.Acceleration ay "Derivative of vy";
  input Modelica.Units.SI.Acceleration az "Derivative of vz";
  output Real directionAcceleration(unit="1/s2") "Derivative of direction component rate";
protected
  Modelica.Units.SI.Length length "Regularized span length";
  Modelica.Units.SI.Position displacement[3] "End-to-end displacement";
  Modelica.Units.SI.Velocity velocity[3] "Rate arguments";
  Modelica.Units.SI.Acceleration velocityRate[3] "Derivative of velocity";
algorithm
  displacement := {x, y, z};
  velocity := {vx, vy, vz};
  velocityRate := {ax, ay, az};
  length := sqrt(displacement*displacement + Roll2RollDynamics.Utilities.Types.lengthTol^2);
  directionAcceleration := velocityRate[axis]/length
    - 2*velocity[axis]*(displacement*velocity)/length^3
    - displacement[axis]*(velocity*velocity + displacement*velocityRate)/length^3
    + 3*displacement[axis]*(displacement*velocity)^2/length^5;
  annotation(Inline=false, smoothOrder=2,
    Documentation(info="<html><p>Analytic derivative used when a moving wrap requires acceleration-level geometric constraints.</p></html>"));
end spanDirectionAcceleration;
