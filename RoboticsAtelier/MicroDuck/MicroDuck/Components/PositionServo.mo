within MicroDuck.Components;
model PositionServo "Dynamixel XL330 position servo with an identified DC motor, friction and optional thermal model"
  //Parameters
  parameter Real positionGain(unit = "1") = 200 "Position P gain register value of the servo";
  parameter Modelica.Units.SI.Voltage supplyVoltage = 7.4 "Supply voltage";
  parameter Real torqueConstant(unit = "N.m/A") = 0.3459739511711113 "Motor torque constant at the output shaft";
  parameter Modelica.Units.SI.Resistance resistance = 2.501880848390462 "Motor terminal resistance at ambientTemperature";
  parameter Modelica.Units.SI.Inertia armature = 0.001573222677933975 "Rotor inertia reflected to the output shaft";
  parameter Modelica.Units.SI.Torque frictionBase = 0.011919956825702578 "Coulomb friction torque";
  parameter Real frictionViscous(unit = "N.m.s/rad") = 0.005788445053875673 "Viscous friction coefficient";
  parameter Modelica.Units.SI.AngularVelocity frictionSpeed = 0.005 "Speed scale that regularizes the Coulomb friction";
  parameter Real gainDivisor(unit = "1") = 256 "Position P gain divisor of the XL330 firmware";
  parameter Real pwmLimit(unit = "1") = 885 "Pulse-width-modulation (PWM) count at full drive";
  parameter Real countsPerRevolution(unit = "1") = 4096 "Encoder counts per revolution";
  parameter Boolean useSupplyInput = false "Take the supply voltage from the input vSupply, else use supplyVoltage";
  parameter Boolean useThermal = false "Enable winding and housing thermal network with temperature-dependent copper resistance";
  parameter Modelica.Units.SI.Temperature ambientTemperature = 293.15 "Ambient and initial servo temperature, 20 degC";
  parameter Modelica.Units.SI.LinearTemperatureCoefficient alphaCopper = 0.00393 "Linear temperature coefficient of copper resistance at ambientTemperature";
  parameter Modelica.Units.SI.HeatCapacity windingHeatCapacity = 1.0 "Heat capacity of the motor winding";
  parameter Modelica.Units.SI.HeatCapacity housingHeatCapacity = 12.0 "Heat capacity of the servo housing and gearbox";
  parameter Modelica.Units.SI.ThermalConductance windingToHousingConductance = 0.15 "Thermal conductance from the winding to the housing";
  parameter Modelica.Units.SI.ThermalConductance housingToAmbientConductance = 0.08 "Thermal conductance from the housing to ambient air";
  final parameter Real errorGain(unit = "1/rad") = positionGain * (countsPerRevolution / (2 * Modelica.Constants.pi)) / (gainDivisor * pwmLimit) "Drive fraction per radian of position error";
  //Variables
  Real drive(unit = "1") "Drive fraction, limited to -1 to 1";
  Modelica.Units.SI.Voltage voltage "Voltage across the motor";
  Modelica.Units.SI.Resistance windingResistance "Instantaneous motor winding resistance";
  Modelica.Units.SI.Current motorCurrent "Current through the motor winding";
  Modelica.Units.SI.Current supplyCurrent "Current drawn from the supply by an ideal H-bridge";
  Modelica.Units.SI.Torque frictionTorque "Friction torque opposing motion in the gearbox and bearings";
  Modelica.Units.SI.Power copperLoss "Electrical Joule heating power in the motor winding";
  Modelica.Units.SI.Power frictionLoss "Mechanical friction heating power in the gearbox and bearings";
  Modelica.Units.SI.Temperature windingTemperature "Motor winding temperature";
  Modelica.Units.SI.Temperature housingTemperature "Servo housing and gearbox temperature";
  Modelica.Units.SI.Torque tau "Torque on the joint";
  //Components
  Modelica.Blocks.Interfaces.RealInput phiRef(unit = "rad") "Joint angle target" annotation(Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput phiEncoder(unit = "rad") "Joint angle read by the encoder on the output side of the gear play" annotation(Placement(transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealInput vSupply(unit = "V") if useSupplyInput "Supply voltage from a shared battery pack" annotation(Placement(transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange "Joint drive flange" annotation(Placement(transformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a support "Joint support flange" annotation(Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport = true) annotation(Placement(transformation(origin = {0, -20}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Components.Inertia rotor(J = armature) annotation(Placement(transformation(origin = {-40, -60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.Rotational.Sensors.RelSpeedSensor speedSensor annotation(Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow copperHeat if useThermal "Joule heating from the motor winding" annotation(Placement(transformation(origin = {-90, 90}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor winding(
    C = windingHeatCapacity,
    T(start = ambientTemperature, fixed = true)) if useThermal "Motor winding thermal capacitance" annotation(Placement(transformation(origin = {-60, 100}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor windingToHousing(G = windingToHousingConductance) if useThermal "Conduction from winding to housing" annotation(Placement(transformation(origin = {-35, 90}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow frictionHeat if useThermal "Mechanical friction heating in the gearbox and bearings" annotation(Placement(transformation(origin = {35, 90}, extent = {{8, -8}, {-8, 8}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor housing(
    C = housingHeatCapacity,
    T(start = ambientTemperature, fixed = true)) if useThermal "Housing and gearbox thermal capacitance" annotation(Placement(transformation(origin = {-10, 100}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor housingToAmbient(G = housingToAmbientConductance) if useThermal "Convection from housing to ambient air" annotation(Placement(transformation(origin = {65, 90}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature ambient(T = ambientTemperature) if useThermal "Ambient air temperature" annotation(Placement(transformation(origin = {95, 90}, extent = {{8, -8}, {-8, 8}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor windingSensor if useThermal "Winding temperature sensor" annotation(Placement(transformation(origin = {-60, 72}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor housingSensor if useThermal "Housing temperature sensor" annotation(Placement(transformation(origin = {-10, 72}, extent = {{-8, -8}, {8, 8}})));
protected
  Modelica.Blocks.Interfaces.RealInput vIn(unit = "V") "Supply voltage in use" annotation(Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput qCopperOut(unit = "W") "Copper heating power signal" annotation(Placement(transformation(origin = {-110, 90}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Interfaces.RealOutput qFrictionOut(unit = "W") "Friction heating power signal" annotation(Placement(transformation(origin = {55, 72}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Interfaces.RealInput tWinding(unit = "K") "Winding temperature in use" annotation(Placement(transformation(origin = {-40, 72}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Blocks.Interfaces.RealInput tHousing(unit = "K") "Housing temperature in use" annotation(Placement(transformation(origin = {10, 72}, extent = {{-6, -6}, {6, 6}})));
equation
  // Supply and thermal state selection
  connect(vSupply, vIn) annotation(Line(points = {{-120, 60}, {-70, 60}}, color = {0, 0, 127}));
  if not useSupplyInput then
    vIn = supplyVoltage;
  end if;
  if not useThermal then
    tWinding = ambientTemperature;
    tHousing = ambientTemperature;
  end if;

  // Servo control law, electrical current, friction and losses
  windingTemperature = tWinding;
  housingTemperature = tHousing;
  windingResistance = resistance * (1 + alphaCopper * (windingTemperature - ambientTemperature));
  drive = noEvent(max(-1, min(1, errorGain * (phiRef - phiEncoder))));
  voltage = vIn * drive;
  motorCurrent = (voltage - torqueConstant * speedSensor.w_rel) / windingResistance;
  supplyCurrent = drive * motorCurrent;
  frictionTorque = frictionBase * tanh(speedSensor.w_rel / frictionSpeed)
    + frictionViscous * speedSensor.w_rel;
  copperLoss = windingResistance * motorCurrent ^ 2;
  frictionLoss = frictionTorque * speedSensor.w_rel;
  qCopperOut = copperLoss;
  qFrictionOut = frictionLoss;
  tau = torqueConstant * motorCurrent - frictionTorque;
  torque.tau = tau;

  // Two-node thermal network (winding and housing)
  connect(qCopperOut, copperHeat.Q_flow) annotation(Line(points = {{-110, 90}, {-98, 90}}, color = {0, 0, 127}));
  connect(copperHeat.port, winding.port) annotation(Line(points = {{-82, 90}, {-60, 90}, {-60, 92}}, color = {191, 0, 0}));
  connect(winding.port, windingToHousing.port_a) annotation(Line(points = {{-60, 92}, {-60, 90}, {-43, 90}}, color = {191, 0, 0}));
  connect(winding.port, windingSensor.port) annotation(Line(points = {{-60, 92}, {-60, 72}, {-68, 72}}, color = {191, 0, 0}));
  connect(windingSensor.T, tWinding) annotation(Line(points = {{-52, 72}, {-40, 72}}, color = {0, 0, 127}));
  connect(windingToHousing.port_b, housing.port) annotation(Line(points = {{-27, 90}, {-10, 90}, {-10, 92}}, color = {191, 0, 0}));
  connect(qFrictionOut, frictionHeat.Q_flow) annotation(Line(points = {{55, 72}, {43, 72}, {43, 90}}, color = {0, 0, 127}));
  connect(frictionHeat.port, housing.port) annotation(Line(points = {{27, 90}, {-10, 90}, {-10, 92}}, color = {191, 0, 0}));
  connect(housing.port, housingToAmbient.port_a) annotation(Line(points = {{-10, 92}, {-10, 90}, {57, 90}}, color = {191, 0, 0}));
  connect(housing.port, housingSensor.port) annotation(Line(points = {{-10, 92}, {-10, 72}, {-18, 72}}, color = {191, 0, 0}));
  connect(housingSensor.T, tHousing) annotation(Line(points = {{-2, 72}, {10, 72}}, color = {0, 0, 127}));
  connect(housingToAmbient.port_b, ambient.port) annotation(Line(points = {{73, 90}, {87, 90}}, color = {191, 0, 0}));

  // Mechanical rotational connections
  connect(torque.flange, flange) annotation(Line(points = {{0, -30}, {0, -100}}));
  connect(torque.support, support) annotation(Line(points = {{10, -20}, {20, -20}, {20, 80}, {0, 80}, {0, 100}}));
  connect(rotor.flange_a, flange) annotation(Line(points = {{-30, -60}, {0, -60}}));
  connect(support, speedSensor.flange_a) annotation(Line(points = {{0, 100}, {0, 80}, {80, 80}, {80, 10}}));
  connect(flange, speedSensor.flange_b) annotation(Line(points = {{0, -100}, {0, -70}, {80, -70}, {80, -10}}));

  annotation(
    Diagram(coordinateSystem(extent = {{-130, -110}, {110, 110}})),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{0, 70}, {0, 100}}, color = {64, 64, 64}),
      Line(points = {{0, -70}, {0, -100}}, color = {64, 64, 64}),
      Rectangle(extent = {{-70, -40}, {70, 70}}, lineColor = {64, 64, 64}, fillColor = {66, 71, 77}, fillPattern = FillPattern.Solid, radius = 8),
      Text(extent = {{-60, 60}, {60, 30}}, textString = "XL330", textColor = {255, 255, 255}),
      Ellipse(extent = {{-34, -74}, {34, -6}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-6, -46}, {6, -34}}, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-25, -44}, {-17, -36}}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{17, -44}, {25, -36}}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-4, -65}, {4, -57}}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-4, -23}, {4, -15}}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{-100, 0}, {-70, 0}}, color = {0, 0, 127}),
      Line(points = {{-100, -60}, {-80, -60}, {-80, -20}, {-70, -20}}, color = {0, 0, 127}),
      Text(extent = {{-150, -105}, {150, -145}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Position servo</h4>
<p>Drives a joint to a target angle like a Dynamixel XL330 (the 18 g servo of the robot [2]), with the motor and friction parameters identified for this servo in the training simulation [1] and an optional two-node thermal network [3].</p>
<p>Figure 1 shows the signal flow. The position error gives a drive fraction <i>u</i> = clip(<code>errorGain</code> (<code>phiRef</code> &minus; <code>phiEncoder</code>), &minus;1, 1) of the supply voltage, with <code>errorGain</code> = <code>positionGain</code> (4096 / 2&pi;) / (256 &middot; 885).
The motor torque is <i>&tau;</i> = <i>k</i><sub>t</sub> (<i>u V</i> &minus; <i>k</i><sub>t</sub> <i>&omega;</i>) / <i>R</i>, so the back-EMF damps the motion, and Coulomb and viscous friction oppose it. <i>&omega;</i> is the speed of <code>flange</code>, the servo output, and <code>armature</code> is the rotor inertia reflected to it.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Components/PositionServo/servo.svg\"
     width=\"600\"
     alt=\"Block diagram from the position error through the clipped drive fraction, supply voltage and DC motor torque to the joint, with angle and speed fed back\">
<strong>Figure 1:</strong> Signal flow of the position servo.</p>
<p>The parameters that decide the behaviour are:</p>
<p>
<code>positionGain</code> = Register value, 200 while the robot runs<br>
<code>supplyVoltage</code> = Battery voltage; training samples 6.5 to 8.2 V<br>
<code>torqueConstant</code>, <code>resistance</code>, <code>armature</code>, <code>frictionBase</code>, <code>frictionViscous</code> = Identified motor values [1]<br>
<code>useThermal</code> = Enables the winding and housing thermal network when <code>true</code> (default <code>false</code>)<br>
<code>windingHeatCapacity</code>, <code>housingHeatCapacity</code>, <code>windingToHousingConductance</code>, <code>housingToAmbientConductance</code> = Two-node thermal network parameters [3]
</p>
<p>The input <code>phiEncoder</code> is the angle the encoder reads. It has to be connected: the real encoder sits on the output side of the gear play, so <a href=\"modelica://MicroDuck.Components.Link\">Link</a> feeds it the link angle, not the angle of <code>flange</code>. <code>flange</code> drives the gear and <code>support</code> holds the housing. Setting <code>useSupplyInput</code> to <code>true</code> exposes <code>vSupply</code>, the instantaneous supply voltage from a shared battery pack; the default is <code>false</code>, which uses <code>supplyVoltage</code>. <code>motorCurrent</code> is the winding current and <code>supplyCurrent</code> = <code>drive</code> &middot; <code>motorCurrent</code> is the current drawn from the supply by an ideal H-bridge.</p>
<p>Setting <code>useThermal</code> to <code>true</code> activates a two-node thermal network built from <code>Modelica.Thermal.HeatTransfer</code> components [3]: electrical Joule loss <code>copperLoss</code> = <code>windingResistance</code> &middot; <code>motorCurrent</code><sup>2</sup> heats the winding capacitance <code>windingHeatCapacity</code> (<code>windingTemperature</code>), while mechanical friction dissipation <code>frictionLoss</code> = <code>frictionTorque</code> &middot; <i>&omega;</i> heats the housing and gearbox capacitance <code>housingHeatCapacity</code> (<code>housingTemperature</code>) before convecting to <code>ambientTemperature</code>. Winding temperature feeds back into <code>windingResistance</code> = <code>resistance</code> (1 + <code>alphaCopper</code> (<code>windingTemperature</code> &minus; <code>ambientTemperature</code>)). When <code>useThermal</code> is <code>false</code>, <code>windingTemperature</code> and <code>housingTemperature</code> stay at <code>ambientTemperature</code> and <code>windingResistance</code> equals <code>resistance</code>.</p>
<h4>Limitations</h4>
<p>Load-dependent and Stribeck friction and encoder offset are not modelled, and Coulomb friction is regularized, so a held joint creeps slowly. The thermal capacitances and conductances are representative values for an 18 g micro-servo rather than calorimetric measurements of the XL330.</p>
<h4>References</h4>
<p>[1] Duclusaud M., Passault G., BAM: Better Actuator Models, Apache-2.0, <a href=\"https://github.com/Rhoban/bam\">github.com/Rhoban/bam</a>, <code>bam/params/xl330/m6.json</code>. Accessed 2026-10-08.<br>
[2] ROBOTIS Co., Ltd., Dynamixel XL330-M288-T e-Manual, <a href=\"https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/\">emanual.robotis.com/docs/en/dxl/x/xl330-m288/</a>. Accessed 2026-10-23.<br>
[3] Maxon Group, \"Thermal Equivalent Circuits of DC Motors\", Maxon Academy Technical Notes (two-node winding and housing thermal RC network with copper temperature coefficient &alpha;<sub>Cu</sub> = 0.00393 K<sup>&minus;1</sup>).</p>
</html>"));
end PositionServo;
