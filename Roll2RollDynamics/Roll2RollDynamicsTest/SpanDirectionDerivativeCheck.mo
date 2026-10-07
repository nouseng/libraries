within Roll2RollDynamicsTest;
model SpanDirectionDerivativeCheck
  "First and second direction derivatives along analytic trajectories"
  Real direction[3,3] "Direction components for three trajectories";
  Real rate[3,3] "First derivatives supplied through annotations";
  Real acceleration[3,3] "Second derivatives supplied through annotations";
  Real position[3,3] = {{cos(time),sin(time),0},
    {1,0,time*time}, {1+time,2-time,time*time}}
    "Circular, planar accelerating and spatial trajectories";
  Real expected[3,3] "Directions computed directly from the trajectories";
  Real expectedRate[3,3] "First derivatives of the direct expressions";
  Real expectedAcceleration[3,3] "Second derivatives of the direct expressions";
equation
  for trajectory in 1:3 loop
    for axis in 1:3 loop
      direction[trajectory,axis] = Roll2RollDynamics.Functions.spanDirection(
        position[trajectory,1], position[trajectory,2], position[trajectory,3], axis);
      expected[trajectory,axis] = position[trajectory,axis]/sqrt(
        position[trajectory,:]*position[trajectory,:]
        + Roll2RollDynamics.Utilities.Types.lengthTol^2);
      rate[trajectory,axis] = der(direction[trajectory,axis]);
      acceleration[trajectory,axis] = der(rate[trajectory,axis]);
      expectedRate[trajectory,axis] = der(expected[trajectory,axis]);
      expectedAcceleration[trajectory,axis] = der(expectedRate[trajectory,axis]);
      assert(abs(rate[trajectory,axis]-expectedRate[trajectory,axis]) < 1e-9,
        "The first direction derivative disagrees with the analytic trajectory");
      assert(abs(acceleration[trajectory,axis]-expectedAcceleration[trajectory,axis]) < 1e-9,
        "The second direction derivative disagrees with the analytic trajectory");
    end for;
  end for;
  annotation(experiment(StopTime=2,Tolerance=1e-9), Documentation(info = "<html>
<h4>Span direction derivative check</h4>
<p>Compare the first and second derivatives supplied by the span-direction
functions with derivatives of their defining expressions. Circular, accelerating
planar and spatial trajectories exercise all three direction components.</p>
</html>"));
end SpanDirectionDerivativeCheck;
