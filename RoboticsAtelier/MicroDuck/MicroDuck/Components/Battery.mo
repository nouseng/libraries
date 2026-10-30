within MicroDuck.Components;
model Battery "Two-cell NP-F550 lithium-ion battery pack built from Modelica.Electrical.Batteries.BatteryStacks.CellRCStack"
  //Parameters
  parameter Real soc0(unit = "1", min = 0.01, max = 0.99) = 0.68 "Initial state of charge of the 2S pack, 0.68 gives 7.4 V open-circuit";
  parameter Modelica.Units.SI.ElectricCharge cellCapacity(displayUnit = "A.h") = 9360 "Nominal cell charge capacity, 2.6 Ah (9360 C) for an NP-F550 18650 cell";
  parameter Modelica.Units.SI.Resistance cellResistance = 0.05 "Total internal resistance per cell, 0.1 Ohm across the 2S pack";
  parameter Modelica.Electrical.Batteries.ParameterRecords.TransientData.ExampleData cellData(
    Qnom = cellCapacity,
    Ri = cellResistance,
    Idis = 0) "Transient Li-ion cell parameters with nonlinear OCV(SOC) curve and two RC branches" annotation(Placement(transformation(origin = {-60, 60}, extent = {{-10, -10}, {10, 10}})));
  //Variables
  Modelica.Units.SI.Voltage vOC = pack.ocv.v "Open-circuit voltage of the 2S pack";
  //Components
  Modelica.Blocks.Interfaces.RealInput iLoad(unit = "A") "Current drawn from the pack" annotation(Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput vTerminal(unit = "V") "Terminal voltage of the pack" annotation(Placement(transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput soc(unit = "1") "State of charge, 0 empty to 1 full" annotation(Placement(transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Batteries.BatteryStacks.CellRCStack pack(
    Ns = 2,
    Np = 1,
    cellData = cellData,
    useHeatPort = false,
    SOC(start = soc0, fixed = true)) "2S1P lithium-ion cell stack with two RC branches" annotation(Placement(transformation(origin = {-10, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(transformation(origin = {-10, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Sources.SignalCurrent load annotation(Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(Placement(transformation(origin = {40, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 270)));
equation
  connect(pack.n, ground.p) annotation(Line(points = {{-10, -10}, {-10, -40}}, color = {0, 0, 255}));
  connect(load.n, ground.p) annotation(Line(points = {{-50, -10}, {-50, -30}, {-10, -30}, {-10, -40}}, color = {0, 0, 255}));
  connect(pack.p, load.p) annotation(Line(points = {{-10, 10}, {-10, 30}, {-50, 30}, {-50, 10}}, color = {0, 0, 255}));
  connect(pack.p, voltageSensor.p) annotation(Line(points = {{-10, 10}, {-10, 30}, {40, 30}, {40, 10}}, color = {0, 0, 255}));
  connect(ground.p, voltageSensor.n) annotation(Line(points = {{-10, -40}, {-10, -30}, {40, -30}, {40, -10}}, color = {0, 0, 255}));
  connect(iLoad, load.i) annotation(Line(points = {{-110, 0}, {-62, 0}}, color = {0, 0, 127}));
  connect(voltageSensor.v, vTerminal) annotation(Line(points = {{51, 0}, {80, 0}, {80, 40}, {110, 40}}, color = {0, 0, 127}));
  soc = pack.SOC;

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{-66, -50}, {66, 44}}, lineColor = {64, 64, 64}, fillColor = {66, 71, 77}, fillPattern = FillPattern.Solid, radius = 8),
      Rectangle(extent = {{-44, 44}, {-20, 58}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{20, 44}, {44, 58}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{-52, -36}, {20, 30}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid, radius = 4),
      Text(extent = {{-46, 20}, {46, -22}}, textString = "2S", textColor = {255, 255, 255}),
      Text(extent = {{-150, -105}, {150, -145}}, textString = "%name", textColor = {64, 64, 64})}),
    Diagram(coordinateSystem(extent = {{-120, -100}, {120, 100}})),
    Documentation(info = "<html>
<h4>Battery</h4>
<p>Two-cell (2S1P) NP-F550 lithium-ion battery pack [3] feeding the servos of <a href=\"modelica://MicroDuck.Components.Robot\">Robot</a>, built on <a href=\"modelica://Modelica.Electrical.Batteries.BatteryStacks.CellRCStack\">Modelica.Electrical.Batteries.BatteryStacks.CellRCStack</a> and <a href=\"modelica://Modelica.Electrical.Batteries.ParameterRecords.TransientData.ExampleData\">TransientData.ExampleData</a> from the Modelica Standard Library [1, 2].</p>
<p>The stack connects <code>Ns = 2</code> cells in series (<code>Np = 1</code>) with the 29-point nonlinear Li-ion open-circuit voltage table <code>OCV_SOC</code> (<code>OCVmax = 4.2</code> V per cell, 8.4 V full pack), series ohmic resistance <code>R0 = 0.7 Ri</code>, and two RC polarization branches (<code>0.2 Ri</code> at a 60 s time constant and <code>0.1 Ri</code> at a 10 s time constant) [1]. A <code>SignalCurrent</code> load draws the total servo H-bridge current <code>iLoad</code> from the stack and a <code>VoltageSensor</code> feeds the loaded terminal voltage <code>vTerminal</code> back to the servos.</p>
<p>The parameters that decide the behaviour are:</p>
<p>
<code>soc0</code> = Initial state of charge (0.68 gives 7.4 V open-circuit; 0.92 gives 8.2 V; 0.18 gives 6.6 V, the shutdown threshold of the robot daemon [3])<br>
<code>cellCapacity</code> = Nominal charge capacity per 18650 cell (2.6 Ah = 9360 C for an NP-F550 pack)<br>
<code>cellResistance</code> = Total internal resistance per cell (0.05 &Omega;, giving 0.1 &Omega; for the 2S pack)
</p>
<h4>Limitations</h4>
<p><code>cellResistance</code> and the RC time constants of <code>ExampleData</code> are representative 18650 values [1] rather than impedance measurements of the Microduck pack and wiring harness.</p>
<h4>References</h4>
<p>[1] Grimm A., Haumer A., Parameterization of a Simplified Physical Battery Model, 13th International Modelica Conference, Regensburg, Germany, 2019, <a href=\"https://2019.international.conference.modelica.org/proceedings/html/papers/Modelica2019paper2D1.pdf\">Modelica2019paper2D1.pdf</a>.<br>
[2] Einhorn M., Conte V., Kral C., Niklas C., Popp H., Fleig J., A Modelica Library for Simulation of Electric Energy Storages, 8th International Modelica Conference, Dresden, Germany, 2011, <a href=\"https://2011.international.conference.modelica.org/proceedings/pages/papers/17_4_ID_105_a_fv.pdf\">17_4_ID_105_a_fv.pdf</a>.<br>
[3] Pollen Robotics, Microduck, Apache-2.0, <a href=\"https://github.com/pollen-robotics/microduck\">github.com/pollen-robotics/microduck</a>, <code>duck-control/src/model.rs</code>. Accessed 2026-10-22.</p>
</html>"));
end Battery;

