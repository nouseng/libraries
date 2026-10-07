within Roll2RollDynamics.Functions;
function tangentAngle "Quadrant-preserving angle of a nonzero tangency force"
  extends Modelica.Icons.Function;
  //Inputs and Outputs
  input Modelica.Units.SI.Force x "Machine-direction force component";
  input Modelica.Units.SI.Force z "Vertical force component";
  output Modelica.Units.SI.Angle angle "Angle measured from the vertical";
algorithm
  if z < 0 then
    angle := (if x >= 0 then Modelica.Constants.pi else -Modelica.Constants.pi)
      - Modelica.Math.atan(x/(-z));
  else
    angle := Modelica.Math.atan2(x,z);
  end if;
  annotation(Inline=false,derivative=tangentAngleRate,
    Documentation(info="<html><p>Returns the angle atan2(x,z), with the
negative-z half-plane evaluated explicitly to preserve its quadrant when
x is identically zero. The force vector must be nonzero. Its derivative is
smooth away from the angle branch cut.</p></html>"));
end tangentAngle;
