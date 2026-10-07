within Roll2RollDynamics.Functions;
function tangentAngleRate "Rate of a nonzero force direction"
  extends Modelica.Icons.Function;
  //Inputs and Outputs
  input Modelica.Units.SI.Force x "Machine-direction force component";
  input Modelica.Units.SI.Force z "Vertical force component";
  input Real dx(unit = "N/s") "Rate of machine-direction force";
  input Real dz(unit = "N/s") "Rate of vertical force";
  output Modelica.Units.SI.AngularVelocity rate "Angular rate";
algorithm
  rate := (z*dx-x*dz)/(x*x+z*z);
  annotation(Inline=true,smoothOrder=2,
    Documentation(info="<html><p>Time derivative of tangentAngle for a
nonzero force vector, away from its angle branch cut.</p></html>"));
end tangentAngleRate;
