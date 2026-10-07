within Roll2RollDynamics.Functions;
function sFunction
  "Continuously differentiable cubic transition between two points"
  extends Modelica.Icons.Function;
  input Real xMin "Lower transition abscissa";
  input Real xMax "Upper transition abscissa";
  input Real yMin "Value at and below xMin";
  input Real yMax "Value at and above xMax";
  input Real x "Function argument";
  output Real y "Interpolated value";
protected
  Real xScaled "Argument scaled to -1 .. +1";
  Real yScaled "Cubic S-function value";
algorithm
  assert(xMax > xMin, "sFunction requires xMax > xMin");
  xScaled := 2*(x - (xMin + xMax)/2)/(xMax - xMin);
  if xScaled > 1 then
    yScaled := 1;
  elseif xScaled < -1 then
    yScaled := -1;
  else
    yScaled := -0.5*xScaled^3 + 1.5*xScaled;
  end if;
  y := yScaled*(yMax - yMin)/2 + (yMax + yMin)/2;
  annotation(
    smoothOrder = 1,
    Documentation(info = "<html>
<p>Cubic S-function used by <code>tripleS</code>. The value and first
derivative are continuous at both ends of the transition interval.</p>
</html>"));
end sFunction;
