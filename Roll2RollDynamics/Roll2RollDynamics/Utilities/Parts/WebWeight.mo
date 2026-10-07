within Roll2RollDynamics.Utilities.Parts;
model WebWeight "Variable web mass supported at a roll centre"
  //Variables
  Modelica.Units.SI.Velocity velocity[3] "Support velocity in world coordinates";

  //Inputs and Outputs
  input Modelica.Units.SI.Mass mass "Supported material mass";
  input Modelica.Units.SI.Force transportForce[3]=zeros(3)
    "Stored and transported relative web momentum reaction in world coordinates";
  //Components
  outer Modelica.Mechanics.MultiBody.World world "Gravity and animation defaults";

  //Physical connectors
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Support load"
    annotation(Placement(transformation(extent={{-110,-10},{-90,10}})));
equation
  velocity = der(frame_a.r_0);
  frame_a.f = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_a.R,
    mass*(der(velocity) - world.gravityAcceleration(frame_a.r_0)) + transportForce);
  frame_a.t = zeros(3);
  annotation(Icon(graphics={Ellipse(extent={{-60,-60},{60,60}},
    fillColor={0,128,180},fillPattern=FillPattern.Solid),
    Text(extent={{-100,100},{100,60}},textString="%name", textColor = {64, 64, 64})}),
    Documentation(info="<html>
<h4>Web weight</h4>
<p>This component applies gravity and translational acceleration load from
parent-fed <code>mass</code> at <code>frame_a</code>, plus
<code>transportForce</code> (default zero), as Figure 1 shows.
<code>WebFriction</code> supplies longitudinal material momentum;
<code>Belt</code> supplies free-span loads through its own connections.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/WebWeight/support-load.svg\"
     width=\"600\"
     alt=\"Gravity, acceleration and transportForce combined into the frame_a support load\">
<strong>Figure 1:</strong> Supported mass load at a roll centre.</p>
<p>The lumped support approximation omits sag, distributed gravity moments
and moments from an accelerating offset wrap centroid. Asymmetric wraps on
accelerating mounts therefore lack a full spatial momentum balance.</p></html>"));
end WebWeight;
