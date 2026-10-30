within MicroDuck.Components;
model Link "Rigid link behind a revolute joint about its local z axis"
  //Parameters
  parameter Modelica.Units.SI.Position[3] r "Joint frame origin relative to the parent frame, resolved in the parent frame";
  parameter Real[3] nx(each unit = "1") "Joint frame x axis, resolved in the parent frame";
  parameter Real[3] ny(each unit = "1") "Joint frame y axis, resolved in the parent frame";
  parameter Modelica.Units.SI.Angle phi0 = 0 "Initial joint angle";
  parameter Modelica.Units.SI.Angle phiMin = -Modelica.Constants.inf "Lower joint limit";
  parameter Modelica.Units.SI.Angle phiMax = Modelica.Constants.inf "Upper joint limit";
  parameter Real stopStiffness(unit = "N.m/rad") = 100 "Stiffness of the joint limit stops";
  parameter Real stopDamping(unit = "N.m.s/rad") = 0.9 "Damping of the joint limit stops";
  parameter Real positionGain(unit = "1") = 200 "Position P gain register value of the servo";
  parameter Modelica.Units.SI.Voltage supplyVoltage = 7.4 "Servo supply voltage";
  parameter Boolean useSupplyInput = false "Take the servo supply voltage from the input vSupply, else use supplyVoltage";
  parameter Boolean useThermal = false "Enable winding and housing thermal network in the servo";
  parameter Modelica.Units.SI.Angle backlash = 0.0349065850398866 "Total play between the servo output and the link, 2 deg";
  parameter Real backlashStiffness(unit = "N.m/rad") = 2000 "Stiffness of the gear teeth once the play is taken up";
  parameter Real backlashDamping(unit = "N.m.s/rad") = 1 "Damping of the gear teeth once the play is taken up";
  parameter Real playDamping(unit = "N.m.s/rad") = 0.01 "Viscous damping across the play";
  parameter Modelica.Units.SI.Inertia playArmature = 0.001 "Inertia on the play angle, as on the backlash hinge of the training model";
  parameter Modelica.Units.SI.Angle contactWidth = 0.001 "Angle over which the tooth contact is smoothed";
  parameter Modelica.Units.SI.Mass m "Link mass";
  parameter Modelica.Units.SI.Position[3] rCM "Center of mass relative to the link frame, resolved in the link frame";
  parameter Modelica.Units.SI.Inertia i11 "Moment of inertia about the center of mass, xx";
  parameter Modelica.Units.SI.Inertia i22 "Moment of inertia about the center of mass, yy";
  parameter Modelica.Units.SI.Inertia i33 "Moment of inertia about the center of mass, zz";
  parameter Modelica.Units.SI.Inertia i21 "Inertia tensor entry yx";
  parameter Modelica.Units.SI.Inertia i31 "Inertia tensor entry zx";
  parameter Modelica.Units.SI.Inertia i32 "Inertia tensor entry zy";
  //Variables
  Modelica.Units.SI.Angle phi = joint.phi "Joint angle";
  Modelica.Units.SI.AngularVelocity w = joint.w "Joint angular velocity";
  Modelica.Units.SI.Angle phiServo = phi - play.phi_rel "Servo output angle, on the motor side of the play";
  Modelica.Units.SI.AngularVelocity wServo = w - play.w_rel "Servo output angular velocity";
  Modelica.Units.SI.Torque tauTeeth "Torque transmitted by the gear teeth";
  Modelica.Units.SI.Torque tauStop "Torque of the joint limit stops";
  Modelica.Units.SI.Current motorCurrent = servo.motorCurrent "Motor current of the servo";
  Modelica.Units.SI.Current supplyCurrent = servo.supplyCurrent "Supply current of the servo";
  Modelica.Units.SI.Power copperLoss = servo.copperLoss "Electrical Joule heating power in the servo winding";
  Modelica.Units.SI.Power frictionLoss = servo.frictionLoss "Mechanical friction heating power in the servo gearbox and bearings";
  Modelica.Units.SI.Temperature windingTemperature = servo.windingTemperature "Motor winding temperature of the servo";
  Modelica.Units.SI.Temperature housingTemperature = servo.housingTemperature "Housing and gearbox temperature of the servo";
  //Components
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Parent link frame" annotation(Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Link frame, rotated by the joint angle" annotation(Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Blocks.Interfaces.RealInput phiRef(unit = "rad") "Joint angle target" annotation(Placement(transformation(origin = {-110, 75}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-50, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput vSupply(unit = "V") if useSupplyInput "Servo supply voltage from a shared battery pack" annotation(Placement(transformation(origin = {-110, 95}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {50, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation mount(
    r = r,
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = nx,
    n_y = ny,
    animation = false) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute joint(
    n = {0, 0, 1},
    useAxisFlange = true,
    phi(start = phi0, fixed = true),
    w(start = 0, fixed = true),
    animation = false) annotation(Placement(transformation(origin = {0, 0}, extent = {{-15, -15}, {15, 15}})));
  MicroDuck.Components.PositionServo servo(
    positionGain = positionGain,
    supplyVoltage = supplyVoltage,
    useSupplyInput = useSupplyInput,
    useThermal = useThermal) annotation(Placement(transformation(origin = {0, 75}, extent = {{-15, -15}, {15, 15}})));
  Modelica.Mechanics.Rotational.Sources.Torque2 teeth "Gear teeth, in contact once the play is taken up" annotation(Placement(transformation(origin = {0, 35}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Components.Damper play(
    d = playDamping,
    phi_nominal = 0.01,
    phi_rel(start = 0, fixed = true),
    w_rel(start = 0, fixed = true)) "Gear play between the servo output and the link, with its viscous damping" annotation(Placement(transformation(origin = {25, 35}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sensors.RelAngleSensor encoder "Encoder on the link side of the play" annotation(Placement(transformation(origin = {-25, 35}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Torque stop(useSupport = true) annotation(Placement(transformation(origin = {40, 65}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body body(
    m = m,
    r_CM = rCM,
    I_11 = i11,
    I_22 = i22,
    I_33 = i33,
    I_21 = i21,
    I_31 = i31,
    I_32 = i32,
    animation = false) annotation(Placement(transformation(origin = {60, -35}, extent = {{-10, -10}, {10, 10}})));
equation
  // Frames
  connect(frame_a, mount.frame_a) annotation(Line(points = {{-100, 0}, {-70, 0}}, color = {95, 95, 95}));
  connect(mount.frame_b, joint.frame_a) annotation(Line(points = {{-50, 0}, {-15, 0}}, color = {95, 95, 95}));
  connect(joint.frame_b, frame_b) annotation(Line(points = {{15, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(joint.frame_b, body.frame_a) annotation(Line(points = {{15, 0}, {30, 0}, {30, -35}, {50, -35}}, color = {95, 95, 95}));

  // Servo drive through the gear play, position read on the link side
  connect(phiRef, servo.phiRef) annotation(Line(points = {{-110, 75}, {-16.5, 75}}, color = {0, 0, 127}));
  connect(vSupply, servo.vSupply) annotation(Line(points = {{-110, 95}, {-30, 95}, {-30, 84}, {-16.5, 84}}, color = {0, 0, 127}));
  connect(servo.flange, teeth.flange_a) annotation(Line(points = {{0, 60}, {0, 45}}));
  connect(teeth.flange_b, joint.axis) annotation(Line(points = {{0, 25}, {0, 15}}));
  connect(play.flange_a, teeth.flange_a) annotation(Line(points = {{25, 45}, {25, 50}, {0, 50}, {0, 45}}));
  connect(play.flange_b, teeth.flange_b) annotation(Line(points = {{25, 25}, {25, 20}, {0, 20}, {0, 25}}));
  connect(servo.support, joint.support) annotation(Line(points = {{0, 90}, {0, 95}, {-60, 95}, {-60, 20}, {-9, 20}, {-9, 15}}));
  connect(encoder.flange_a, joint.support) annotation(Line(points = {{-25, 45}, {-25, 50}, {-60, 50}, {-60, 20}, {-9, 20}, {-9, 15}}));
  connect(encoder.flange_b, teeth.flange_b) annotation(Line(points = {{-25, 25}, {0, 25}}));

  // Teeth: smoothed dead zone of width backlash, a spring-damper beyond it, and the inertia on the play angle
  tauTeeth = backlashStiffness * 0.5 * (play.phi_rel - backlash / 2 + sqrt((play.phi_rel - backlash / 2) ^ 2 + contactWidth ^ 2)
      + play.phi_rel + backlash / 2 - sqrt((play.phi_rel + backlash / 2) ^ 2 + contactWidth ^ 2))
    + backlashDamping * 0.5 * (2 + (play.phi_rel - backlash / 2) / sqrt((play.phi_rel - backlash / 2) ^ 2 + contactWidth ^ 2)
      - (play.phi_rel + backlash / 2) / sqrt((play.phi_rel + backlash / 2) ^ 2 + contactWidth ^ 2)) * play.w_rel
    + playArmature * der(play.w_rel);
  teeth.tau = -tauTeeth;
  connect(encoder.phi_rel, servo.phiEncoder) annotation(Line(points = {{-14, 35}, {-14, 52}, {-22, 52}, {-22, 66}, {-16.5, 66}}, color = {0, 0, 127}));

  // The stops act on the servo output and only push it back into its range
  tauStop = noEvent(if phiServo > phiMax then -max(0, stopStiffness * (phiServo - phiMax) + stopDamping * wServo)
    elseif phiServo < phiMin then max(0, stopStiffness * (phiMin - phiServo) - stopDamping * wServo)
    else 0);
  stop.tau = tauStop;
  connect(stop.flange, servo.flange) annotation(Line(points = {{30, 65}, {20, 65}, {20, 55}, {0, 55}, {0, 60}}));
  connect(stop.support, joint.support) annotation(Line(points = {{40, 75}, {40, 95}, {-60, 95}, {-60, 20}, {-9, 20}, {-9, 15}}));

  annotation(
    Diagram(coordinateSystem(extent = {{-120, -80}, {120, 110}})),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{-100, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 0.5),
      Line(points = {{80, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 0.5),
      Rectangle(extent = {{-50, -16}, {80, 16}}, lineColor = {64, 64, 64}, fillColor = {172, 164, 153}, fillPattern = FillPattern.Solid, radius = 6),
      Ellipse(extent = {{34, -8}, {50, 8}}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{36, -6}, {48, 6}}, color = {64, 64, 64}),
      Line(points = {{36, 6}, {48, -6}}, color = {64, 64, 64}),
      Rectangle(extent = {{-80, -40}, {-20, 40}}, lineColor = {64, 64, 64}, fillColor = {66, 71, 77}, fillPattern = FillPattern.Solid, radius = 6),
      Ellipse(extent = {{-68, -18}, {-32, 18}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-54, -4}, {-46, 4}}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid),
      Text(extent = {{-150, -100}, {150, -140}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Link</h4>
<p>Rigid body behind a revolute joint about its local z axis, as every joint of the robot description.</p>
<p>Figure 1 shows a link turned by the joint angle <code>phi</code>. The joint frame is offset by <code>r</code> from <code>frame_a</code>, the parent frame, and oriented by <code>nx</code> and <code>ny</code>, its x and y axes in the parent frame. <code>frame_b</code> is the joint frame rotated by <code>phi</code> about z, so it sits on the joint axis, and the body, child links and sites all attach to it.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Components/Link/link.svg\"
     width=\"600\"
     alt=\"Parent frame, offset r, joint frame axes nx and ny, link body turned by phi with frame_b at the joint axis, and the XL330 servo driven by phiRef\">
<strong>Figure 1:</strong> Frames, body and servo of a link.</p>
<p>The body has the mass <code>m</code>, the center of mass <code>rCM</code> and the inertia entries <code>i11</code> to <code>i32</code>, all resolved in <code>frame_b</code>. A <a href=\"modelica://MicroDuck.Components.PositionServo\">PositionServo</a> inside drives the joint to the target <code>phiRef</code>, with <code>positionGain</code> and <code>supplyVoltage</code> as parameters. The joint starts at <code>phi0</code> at rest with the play centred.</p>
<p>The servo turns the link through a gear play that transmits almost no torque within &plusmn;<code>backlash</code>/2 and a stiff tooth contact beyond, as the passive backlash hinge of the training model does. The dead zone is smoothed over <code>contactWidth</code>, so the contact needs no events, and <code>playArmature</code> puts on the play angle the small inertia that hinge carries. The encoder reads the link angle <code>phi</code>, on the output side of the play, so the servo holds the link and its own output <code>phiServo</code> rests against a tooth flank under load.
Between <code>phiMin</code> and <code>phiMax</code> the servo output moves freely. Outside them the stops push it back with a spring-damper torque, as the joint ranges do in the training model, so the link can go up to <code>backlash</code>/2 further. The defaults are infinite, so a link has no limits unless you set them.</p>
<p>The parameters that decide the behaviour of the play are:</p>
<p>
<code>backlash</code> = Total play, 2&deg; as in training; 0 leaves a stiff spring<br>
<code>backlashStiffness</code>, <code>backlashDamping</code> = Tooth contact once the play is taken up<br>
<code>playDamping</code>, <code>playArmature</code> = Viscous damping of 0.01 N&middot;m&middot;s/rad and inertia of 0.001 kg&middot;m&sup2; on the play angle, as in training
</p>
<p><code>phiRef</code> is an input that <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a> feeds from its <code>qRef</code> vector; it has to be connected. Setting <code>useSupplyInput</code> to <code>true</code> exposes <code>vSupply</code> and passes it to the servo; setting <code>useThermal</code> to <code>true</code> enables the servo's winding and housing thermal network.</p>
</html>"));
end Link;
