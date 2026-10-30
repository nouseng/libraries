within MicroDuck.Components;
model Robot "Microduck trunk, legs and head with servo-driven joints and sole contacts"
  //Parameters
  parameter Modelica.Units.SI.Angle[14] q0 = {0, -0.0873, -0.4579, -0.0049, 0.453, 0.3491, 0.3491, 0, 0, 0, 0.0873, 0.4579, 0.0049, -0.453} "Initial joint angles, the home pose";
  parameter Real positionGain(unit = "1") = 200 "Position P gain register value of the servos";
  parameter Modelica.Units.SI.Voltage supplyVoltage = 7.4 "Servo supply voltage when useBattery is false";
  parameter Boolean useBattery = false "Model the supply as a 2S NP-F550 Battery component with nonlinear OCV(SOC) and two RC branches";
  parameter Real soc0(unit = "1", min = 0.01, max = 0.99) = 0.68 "Initial state of charge of the battery pack, 0.68 gives 7.4 V open-circuit" annotation(Dialog(enable = useBattery));
  parameter Modelica.Units.SI.ElectricCharge batteryCapacity(displayUnit = "A.h") = 9360 "Charge capacity per cell of the 2S NP-F550 battery pack, 2.6 Ah = 9360 C" annotation(Dialog(enable = useBattery));
  parameter Modelica.Units.SI.Resistance batteryResistance = 0.1 "Total internal resistance of the 2S battery pack and wiring" annotation(Dialog(enable = useBattery));
  parameter Boolean useThermal = false "Enable winding and housing thermal network in all servos";
  parameter Real mu(unit = "1") = 1 "Friction coefficient of the sole";
  parameter Modelica.Units.SI.Time contactTimeConstant = 0.02 "Time constant of the sole contact, MuJoCo solref[1]";
  parameter Real contactDampingRatio(unit = "1") = 1 "Damping ratio of the sole contact, MuJoCo solref[2]";
  parameter Boolean contactAcceleration = false "Include the inertial term in the sole contacts, see PointContact";
  final parameter Real footInverseWeight(unit = "1/kg") = 7.85437513124 "MuJoCo body_invweight0 of the ankle body carrying the sole";
  parameter Boolean animation = true "Show the meshes of the robot in animations";
  parameter Real stopStiffness(unit = "N.m/rad") = 100 "Stiffness of the joint limit stops";
  parameter Real stopDamping(unit = "N.m.s/rad") = 0.9 "Damping of the joint limit stops";
  parameter Modelica.Units.SI.Angle backlash = 0.0349065850398866 "Total gear play of each servo, 2 deg";
  parameter Real backlashStiffness(unit = "N.m/rad") = 2000 "Stiffness of the gear teeth once the play is taken up";
  parameter Real backlashDamping(unit = "N.m.s/rad") = 1 "Damping of the gear teeth once the play is taken up";
  parameter Boolean useMouth = false "Model the mouth as a fifteenth servo-driven link, else the beak is rigid in the head as in training";
  parameter Boolean useMouthInput = false "Take the mouth angle target from the input mouthRef" annotation(Dialog(enable = useMouth));
  parameter Modelica.Units.SI.Angle mouthHome = 0 "Mouth angle target without mouthRef" annotation(Dialog(enable = useMouth));
  final parameter Modelica.Units.SI.Mass totalMass = 0.73724318 "Mass of the modelled bodies";
  //Variables
  Modelica.Units.SI.Angle[14] q "Joint angles in the order of q0";
  Modelica.Units.SI.AngularVelocity[14] w "Joint angular velocities in the order of q0";
  Modelica.Units.SI.Current[14] motorCurrent "Motor currents of the 14 servos";
  Modelica.Units.SI.Power[14] copperLoss "Electrical Joule heating power in the 14 servo windings";
  Modelica.Units.SI.Power[14] frictionLoss "Mechanical friction heating power in the 14 servo gearboxes";
  Modelica.Units.SI.Temperature[14] windingTemperature "Motor winding temperatures of the 14 servos";
  Modelica.Units.SI.Temperature[14] housingTemperature "Housing and gearbox temperatures of the 14 servos";
  Modelica.Units.SI.Current batteryCurrent "Total current drawn from the supply by the servos";
  Modelica.Units.SI.Voltage terminalVoltage "Terminal voltage at the servos";
  Real soc(unit = "1") "State of charge of the battery pack, 0 empty to 1 full";
  //Components
  Modelica.Mechanics.MultiBody.Parts.Fixed ground(animation = false) "Ground plane frame, the world frame" annotation(Placement(transformation(origin = {170, 95}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Trunk frame" annotation(Placement(transformation(origin = {-200, -125}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_imu "Site imu" annotation(Placement(transformation(origin = {205, -125}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {100, -80}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_leftFoot "Site left_foot" annotation(Placement(transformation(origin = {205, 165}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {100, 80}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_headCamera "Site head_camera" annotation(Placement(transformation(origin = {205, -30}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {100, 16}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_tof "Site tof" annotation(Placement(transformation(origin = {205, -60}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {100, -16}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_mouthTip "Site mouth_tip" annotation(Placement(transformation(origin = {205, -90}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {100, -48}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rightFoot "Site right_foot" annotation(Placement(transformation(origin = {205, 15}, extent = {{-8, -8}, {8, 8}}), iconTransformation(origin = {100, 48}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Interfaces.RealInput[14] qRef(each unit = "rad") "Joint angle targets in the order of q0" annotation(Placement(transformation(origin = {-200, 170}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput mouthRef(unit = "rad") if useMouthInput "Mouth angle target" annotation(Placement(transformation(origin = {-200, -155}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {50, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.Body trunk(
    m = 0.199224,
    r_CM = {-0.0226332, -2.70828e-05, 0.00280064},
    I_11 = 0.000123975, I_22 = 0.000145931, I_33 = 0.000115351,
    I_21 = 7.1651e-08, I_31 = -2.06915e-05, I_32 = 1.32429e-07,
    animation = false) annotation(Placement(transformation(origin = {-160, -135}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  MicroDuck.Components.Link leftHipYaw(
    r = {0.006, 0.0175, -0.005},
    nx = {2.22044604925e-16, 1, 0}, ny = {1, 2.22044604925e-16, 0},
    phi0 = q0[1],
    phiMin = -0.436332312999, phiMax = 0.523598775598, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0230406, rCM = {-8.8e-11, 0.00099414, 0.0168917},
    i11 = 4.1964e-06, i22 = 3.41752e-06, i33 = 2.3265e-06,
    i21 = -0, i31 = -0, i32 = -5.0881e-08) annotation(Placement(transformation(origin = {-120, 140}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link leftHipRoll(
    r = {0, 0.0165, 0.0125},
    nx = {1, -0, 0}, ny = {0, 2.22044604925e-16, -1},
    phi0 = q0[2],
    phiMin = -0.383972435439, phiMax = 0.383972435439, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.00618934, rCM = {0.014815, 1.6179e-08, -0.00829992},
    i11 = 8.60626e-07, i22 = 1.15753e-06, i33 = 6.51588e-07,
    i21 = 1e-12, i31 = 3.89882e-07, i32 = -1e-12) annotation(Placement(transformation(origin = {-75, 140}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link leftHipPitch(
    r = {0.025, 0, -0.0185},
    nx = {0, -1, 0}, ny = {0, 0, -1},
    phi0 = q0[3],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0482067, rCM = {0.006766, 0.0228838, 0.0134531},
    i11 = 1.68753e-05, i22 = 1.03691e-05, i33 = 1.98269e-05,
    i21 = -4.41852e-06, i31 = 2.27982e-07, i32 = 3.89298e-07) annotation(Placement(transformation(origin = {-30, 140}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link leftKnee(
    r = {0.022, 0.0357771, -0.004},
    nx = {2.22044604925e-16, 1, 0}, ny = {1, 2.22044604925e-16, 0},
    phi0 = q0[4],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0215844, rCM = {-1.52e-10, 0.0318937, -0.00953505},
    i11 = 5.25056e-06, i22 = 2.15507e-06, i33 = 4.33925e-06,
    i21 = -1e-12, i31 = 0, i32 = 7.65005e-07) annotation(Placement(transformation(origin = {15, 140}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link leftAnkle(
    r = {-0, 0.042, -0.026},
    nx = {1, 0, 0}, ny = {0, -1, -0},
    phi0 = q0[5],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0300246, rCM = {-0.00615742, -0.0137483, -0.0156585},
    i11 = 5.91525e-06, i22 = 1.11534e-05, i33 = 7.74016e-06,
    i21 = -2.05354e-07, i31 = 8.218e-08, i32 = 5.6722e-08) annotation(Placement(transformation(origin = {60, 140}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link neckPitch(
    r = {0.026, 0.0145, 0.0324215},
    nx = {-1, 0, -0}, ny = {0, 2.22044604925e-16, -1},
    phi0 = q0[6],
    phiMin = -1.57079632679, phiMax = 1.0471975512, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0368414, rCM = {-0, -0.025, 0.0141665},
    i11 = 1.72925e-05, i22 = 3.12694e-06, i33 = 1.63707e-05,
    i21 = -0, i31 = 0, i32 = -0) annotation(Placement(transformation(origin = {-120, -60}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link headPitch(
    r = {-0, -0.05, 0},
    nx = {1, -0, 0}, ny = {0, -1, 0},
    phi0 = q0[7],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.00572, rCM = {-6.533e-09, 0.0116641, -0.0145},
    i11 = 1.09186e-06, i22 = 9.66855e-07, i33 = 5.12111e-07,
    i21 = 0, i31 = 0, i32 = 0) annotation(Placement(transformation(origin = {-75, -60}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link headYaw(
    r = {-0, 0.0186931, -0.0145},
    nx = {-1, 0, 0}, ny = {0, 2.22044604925e-16, 1},
    phi0 = q0[8],
    phiMin = -2.96705972839, phiMax = 2.96705972839, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0486, rCM = {-0.000166657, -0.00500964, 0.0178065},
    i11 = 6.97903e-06, i22 = 8.10293e-06, i33 = 9.57884e-06,
    i21 = 3.21657e-07, i31 = -3.4797e-08, i32 = 3.4295e-07) annotation(Placement(transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link headRoll(
    r = {-0.0179, 0, 0.0145},
    nx = {2.22044604925e-16, -0, 1}, ny = {0, 1, 0},
    phi0 = q0[9],
    phiMin = -0.436332312999, phiMax = 0.436332312999, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = if useMouth then 0.156890099447 else 0.188766, rCM = if useMouth then {0.0078244697816, 0.00155889449113, -0.0274808736393} else {0.00460523, 0.00120047, -0.0324761},
    i11 = if useMouth then 0.000264573570805 else 0.000320811, i22 = if useMouth then 0.00021326414771 else 0.0002541, i33 = if useMouth then 0.000113742336051 else 0.000149882,
    i21 = if useMouth then 1.8293965243e-07 else -6.32293e-07, i31 = if useMouth then 2.28293312329e-05 else 6.57153e-06, i32 = if useMouth then 5.14666282428e-06 else 4.18477e-06) annotation(Placement(transformation(origin = {15, -60}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link rightHipYaw(
    r = {0.006, -0.0175, -0.005},
    nx = {2.22044604925e-16, 1, 0}, ny = {1, 2.22044604925e-16, -0},
    phi0 = q0[10],
    phiMin = -0.523598775598, phiMax = 0.436332312999, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0230406, rCM = {-8.8e-11, 0.00099414, 0.0168917},
    i11 = 4.1964e-06, i22 = 3.41752e-06, i33 = 2.3265e-06,
    i21 = -0, i31 = -0, i32 = -5.0881e-08) annotation(Placement(transformation(origin = {-120, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link rightHipRoll(
    r = {-0, 0.0165, 0.0125},
    nx = {-1, 0, 0}, ny = {0, 2.22044604925e-16, 1},
    phi0 = q0[11],
    phiMin = -0.383972435439, phiMax = 0.383972435439, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.00618934, rCM = {0.014815, 1.6212e-08, -0.00829992},
    i11 = 8.60626e-07, i22 = 1.15753e-06, i33 = 6.51588e-07,
    i21 = 1e-12, i31 = 3.89882e-07, i32 = -1e-12) annotation(Placement(transformation(origin = {-75, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link rightHipPitch(
    r = {0.025, 0, -0.0185},
    nx = {0, -1, 0}, ny = {0, 0, -1},
    phi0 = q0[12],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0482067, rCM = {-0.006766, 0.0228838, 0.0134531},
    i11 = 1.68753e-05, i22 = 1.03691e-05, i33 = 1.98269e-05,
    i21 = 4.41852e-06, i31 = -2.27982e-07, i32 = 3.89298e-07) annotation(Placement(transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link rightKnee(
    r = {-0.022, 0.0357771, -0.004},
    nx = {1, -0, 0}, ny = {0, -1, 0},
    phi0 = q0[13],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0215844, rCM = {-0.0318937, -1.52e-10, -0.00953505},
    i11 = 2.15507e-06, i22 = 5.25056e-06, i33 = 4.33925e-06,
    i21 = 1e-12, i31 = -7.65005e-07, i32 = 0) annotation(Placement(transformation(origin = {15, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link rightAnkle(
    r = {-0.042, 0, -0.026},
    nx = {2.22044604925e-16, 1, 0}, ny = {1, 2.22044604925e-16, -0},
    phi0 = q0[14],
    phiMin = -1.57079632679, phiMax = 1.57079632679, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0300251, rCM = {0.00615539, -0.0137482, -0.0156566},
    i11 = 5.91534e-06, i22 = 1.11534e-05, i33 = 7.74015e-06,
    i21 = 2.04877e-07, i31 = -8.2467e-08, i32 = 5.7017e-08) annotation(Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Link mouth(
    r = {0.003, 0.04, -0.018},
    nx = {-0.342489629494, -3.33788365691e-16, -0.939521608953}, ny = {-0.939521608953, -2.08615704478e-16, 0.342489629494},
    phi0 = mouthHome,
    phiMin = -0.0872664625997, phiMax = 0.523598775598, stopStiffness = stopStiffness, stopDamping = stopDamping,
    backlash = backlash, backlashStiffness = backlashStiffness, backlashDamping = backlashDamping,
    positionGain = positionGain, supplyVoltage = supplyVoltage, useSupplyInput = useBattery, useThermal = useThermal,
    m = 0.0318759005534, rCM = {0.0415766073185, 0, -0.0405636606781},
    i11 = 2.63114971148e-05, i22 = 3.30154020788e-05, i33 = 8.0243747083e-06,
    i21 = 1.097286073e-06, i31 = -7.47079136761e-07, i32 = -9.92784998634e-10) if useMouth "Lower beak, estimated from the meshes" annotation(Placement(transformation(origin = {70, -112}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.Battery battery(
    soc0 = soc0,
    cellCapacity = batteryCapacity,
    cellResistance = batteryResistance / 2) if useBattery "Two-cell NP-F550 lithium-ion battery pack" annotation(Placement(transformation(origin = {-160, 75}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation[8] soleLeft(
    r = {{-0.02644722875, -0.0241078931753, -0.0140688847063}, {-0.02424303875, -0.023134518446, -0.00539417071244}, {-0.00058662875, -0.0227101585687, -0.00161227618086}, {0.01044962125, -0.0229881118133, -0.00408939504161}, {0.01269674125, -0.0241078753517, -0.0140687258619}, {0.00926549125, -0.0252454924953, -0.0242071679833}, {-0.00399096875, -0.0255008810327, -0.0264831902104}, {-0.02407428875, -0.0250691157131, -0.0226352983893}},
    each animation = false) annotation(Placement(transformation(origin = {100, 140}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.PointContact[8] contactLeft(
    each normal = {0, 0, 1},
    each mu = mu, each timeConstant = contactTimeConstant, each dampingRatio = contactDampingRatio,
    each accelerationTerm = contactAcceleration,
    each inverseWeight = if contactAcceleration then 3.5 * footInverseWeight else footInverseWeight) annotation(Placement(transformation(origin = {140, 140}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation[8] soleRight(
    r = {{0.02634170125, -0.0239565409071, -0.0127200335395}, {0.02402618125, -0.0250691157131, -0.0226352983893}, {0.00053852125, -0.0255020523095, -0.0264936286273}, {-0.00931359875, -0.0252454924953, -0.0242071679833}, {-0.01274682875, -0.0241078692701, -0.014068671663}, {-0.01049772875, -0.0229881118133, -0.00408939504161}, {0.00053852125, -0.0227101744198, -0.00161241744524}, {0.02419493125, -0.023134518446, -0.00539417071244}},
    each animation = false) annotation(Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));
  MicroDuck.Components.PointContact[8] contactRight(
    each normal = {0, 0, 1},
    each mu = mu, each timeConstant = contactTimeConstant, each dampingRatio = contactDampingRatio,
    each accelerationTerm = contactAcceleration,
    each inverseWeight = if contactAcceleration then 3.5 * footInverseWeight else footInverseWeight) annotation(Placement(transformation(origin = {140, 40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation siteImu(
    r = {-0.021, 6.64098e-05, -0.0146984},
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = {1, 0, 0}, n_y = {0, 1, 0},
    animation = false) annotation(Placement(transformation(origin = {-100, -125}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation siteLeftFoot(
    r = {-0, -0.0238146, -0.0140852},
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = {-1, 0, 0}, n_y = {0, 2.22044604925e-16, 1},
    animation = false) annotation(Placement(transformation(origin = {100, 165}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation siteHeadCamera(
    r = {0.0155, -9.13778e-05, -0.0733},
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = {2.22044604925e-16, 0, -1}, n_y = {-0, 1, 0},
    animation = false) annotation(Placement(transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation siteTof(
    r = {0.0135, 0.0224086, -0.0733},
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = {2.22044604925e-16, 0, -1}, n_y = {-0, 1, 0},
    animation = false) annotation(Placement(transformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation siteMouthTip(
    r = {-0.00809334, -0, -0.0777383},
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = {-0.00872561527001, 0, -0.999961931094}, n_y = {-0, 1, 0},
    animation = false) annotation(Placement(transformation(origin = {110, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation siteRightFoot(
    r = {0, -0.0238146, -0.0140852},
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.TwoAxesVectors,
    n_x = {1, 0, 0}, n_y = {0, 2.22044604925e-16, -1},
    animation = false) annotation(Placement(transformation(origin = {100, 15}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {0.006, 0.0175, -0.005},
    lengthDirection = {1, 0, 0}, widthDirection = {0, 1, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/right_shell.stl",
    r_shape = {0.004, -0.0005, -0.0175},
    lengthDirection = {2.22044604925e-16, 1, 0}, widthDirection = {-1, 2.22044604925e-16, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 217, 214},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/banana_pcb_locker.stl",
    r_shape = {0.0042, 0, -0.0175215},
    lengthDirection = {2.22044604925e-16, 1, -0}, widthDirection = {-1, 2.22044604925e-16, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {165, 165, 165},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/trunk_base.stl",
    r_shape = {0.004, -0, -0.0175},
    lengthDirection = {2.22044604925e-16, 1, 0}, widthDirection = {-1, 2.22044604925e-16, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual5(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0.006, 0.0175, 0.0115},
    lengthDirection = {2.22044604925e-16, -0, 1}, widthDirection = {0, -1, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual6(
    shapeType = "modelica://MicroDuck/Resources/Meshes/power_support.stl",
    r_shape = {0.004, -0, -0.0175215},
    lengthDirection = {2.22044604925e-16, 1, 0}, widthDirection = {-1, 2.22044604925e-16, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual7(
    shapeType = "modelica://MicroDuck/Resources/Meshes/np_f970.stl",
    r_shape = {-0.0359693, 0, -0.00385049},
    lengthDirection = {0, -1, 0}, widthDirection = {-0.999847695338, 0, 0.0174523960378},
    length = 1, width = 1, height = 1, extra = 0, color = {77, 77, 77},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual8(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0.006, -0.0175, 0.0115},
    lengthDirection = {2.22044604925e-16, -0, 1}, widthDirection = {0, -1, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual9(
    shapeType = "modelica://MicroDuck/Resources/Meshes/left_shell.stl",
    r_shape = {0.004, -0, -0.0175785},
    lengthDirection = {2.22044604925e-16, 1, 0}, widthDirection = {-1, 2.22044604925e-16, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {204, 204, 204},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape trunkVisual10(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {0.006, -0.0175, -0.005},
    lengthDirection = {1, 0, 0}, widthDirection = {0, 1, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipYawVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/yaw2roll.stl",
    r_shape = {-0.0175, -0.002, 0.0125},
    lengthDirection = {1, 0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipYawVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0, -0, 0.0125},
    lengthDirection = {2.22044604925e-16, -1, 0}, widthDirection = {-1, 2.22044604925e-16, -0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipYawVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {-0, 0.0165, 0.0125},
    lengthDirection = {0, 0, -1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipYawVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/bearing_roll.stl",
    r_shape = {-0.0175, -0.002, 0.0125},
    lengthDirection = {1, 0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipRollVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {0.021, 0, -0.0185},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipRollVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/hip_l.stl",
    r_shape = {-0.0175, 0, -0.0185},
    lengthDirection = {1, 0, -0}, widthDirection = {-0, 2.22044604925e-16, -1},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipPitchVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/upper_leg_left.stl",
    r_shape = {-0, -0, -0.0425},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 217, 214},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipPitchVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0.022, 0.0357771, 0.0125},
    lengthDirection = {2.22044604925e-16, 0, 1}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipPitchVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/upper_leg_rigidity_plate.stl",
    r_shape = {0, 0, -0.0425},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipPitchVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {0.022, 0.0357771, -0.004},
    lengthDirection = {-1, 0, 0}, widthDirection = {-0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftHipPitchVisual5(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0, -0, 0.0125},
    lengthDirection = {0, 0, 1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftKneeVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/leg.stl",
    r_shape = {-0.022, 0.0357771, 0.0385},
    lengthDirection = {0, 0, -1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftKneeVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0, 0.042, -0.0115},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftAnkleVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/sole_left.stl",
    r_shape = {-0.022, 0.00622291, -0.0647},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {244, 175, 35},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftAnkleVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/foot_left.stl",
    r_shape = {-0.022, 0.00622291, -0.0647},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftAnkleVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration_default.stl",
    r_shape = {0, 0, -0.0322},
    lengthDirection = {-1, 0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape leftAnkleVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/ankle_left.stl",
    r_shape = {-0.022, 0.00622291, -0.0647},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape neckPitchVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/neck.stl",
    r_shape = {0.022, 0.05, 0.028},
    lengthDirection = {0, 0, -1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape neckPitchVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0, 0, 0.0145},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape neckPitchVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0, -0.05, 0.0145},
    lengthDirection = {0, 0, 1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape neckPitchVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/neck.stl",
    r_shape = {0.022, 0.05, 0.003},
    lengthDirection = {0, 0, -1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headPitchVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/neck_pitch.stl",
    r_shape = {-0, 0.0287931, -0.0145},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {89, 96, 102},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headPitchVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {-0, 0.0207931, -0.0145},
    lengthDirection = {1, 0, 0}, widthDirection = {-0, 2.22044604925e-16, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headYawVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/yaw_roll_motion.stl",
    r_shape = {0, 0, 0.01},
    lengthDirection = {2.22044604925e-16, 1, 0}, widthDirection = {-1, 2.22044604925e-16, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headYawVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0, 0, 0.0145},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headYawVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {-0.016, 0, 0.0145},
    lengthDirection = {2.22044604925e-16, 0, 1}, widthDirection = {0, 1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headYawVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {0.018, 0, 0.0145},
    lengthDirection = {2.22044604925e-16, -0, 1}, widthDirection = {0, 1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/lens.stl",
    r_shape = {0.0155, -9.13778e-05, -0.05548},
    lengthDirection = {0, -1, 0}, widthDirection = {0, 0, -1},
    length = 1, width = 1, height = 1, extra = 0, color = {67, 72, 77},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/face_part.stl",
    r_shape = {-0.0045, -9.13778e-05, -0.0178},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0.003, 0.0255, -0.018},
    lengthDirection = {2.22044604925e-16, -1, 0}, widthDirection = {1, 2.22044604925e-16, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape mouthVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/jaw_soft.stl",
    r_shape = {0.00260254537551, 0.00714050122389, -0.04},
    lengthDirection = {-3.33788365691e-16, -2.08615704478e-16, 1}, widthDirection = {-0.939521608953, 0.342489629494, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 193, 221},
    animation = animation) if useMouth annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/jaw_soft.stl",
    r_shape = {-0.0046, 0, -0.0179996},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 193, 221},
    animation = animation) if not useMouth annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual5(
    shapeType = "modelica://MicroDuck/Resources/Meshes/motor_support.stl",
    r_shape = {-0.0045, -0, -0.018},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual6(
    shapeType = "modelica://MicroDuck/Resources/Meshes/top_head_shell.stl",
    r_shape = {-0.0045, -0, -0.018},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 217, 214},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual7(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {0, 0, 0.0145},
    lengthDirection = {0, 0, 1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual8(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration_default.stl",
    r_shape = {0.0032, -0.04, -0.018},
    lengthDirection = {0, 0, -1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual9(
    shapeType = "modelica://MicroDuck/Resources/Meshes/elec_rpi_robot_hat_pcb.stl",
    r_shape = {0.02694, 0.0289936, -0.0519},
    lengthDirection = {2.22044604925e-16, -1, 0}, widthDirection = {1, 2.22044604925e-16, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {80, 124, 105},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual10(
    shapeType = "modelica://MicroDuck/Resources/Meshes/soft_mouth_top.stl",
    r_shape = {-0.00420071, 1.7776e-08, -0.0180814},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 193, 221},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual11(
    shapeType = "modelica://MicroDuck/Resources/Meshes/noenoeil.stl",
    r_shape = {-0.0045, -9.13778e-05, -0.0178},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {244, 175, 35},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape mouthVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/jaw.stl",
    r_shape = {0.0025686722212, 0.00704641206715, -0.04},
    lengthDirection = {-3.33788365691e-16, -2.08615704478e-16, 1}, widthDirection = {-0.939521608953, 0.342489629494, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) if useMouth annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual12(
    shapeType = "modelica://MicroDuck/Resources/Meshes/jaw.stl",
    r_shape = {-0.0045, -0, -0.018},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) if not useMouth annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual13(
    shapeType = "modelica://MicroDuck/Resources/Meshes/bottom_head_shell.stl",
    r_shape = {-0.0043, -0, -0.018},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual14(
    shapeType = "modelica://MicroDuck/Resources/Meshes/m12_lens_holder.stl",
    r_shape = {0.0155, -9.13778e-05, -0.05558},
    lengthDirection = {0, -1, 0}, widthDirection = {0, 0, -1},
    length = 1, width = 1, height = 1, extra = 0, color = {67, 72, 77},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual15(
    shapeType = "modelica://MicroDuck/Resources/Meshes/pcb__raspberry_pi_zero_2_w.stl",
    r_shape = {0.015435, -1.13778e-05, -0.06001},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, -1},
    length = 1, width = 1, height = 1, extra = 0, color = {127, 127, 127},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape headRollVisual16(
    shapeType = "modelica://MicroDuck/Resources/Meshes/speaker.stl",
    r_shape = {0.01, -0.016786, 0.0260226},
    lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {75, 74, 74},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipYawVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/yaw2roll.stl",
    r_shape = {-0.0175, -0.002, 0.0125},
    lengthDirection = {1, 0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipYawVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0, -0, 0.0125},
    lengthDirection = {2.22044604925e-16, -1, -0}, widthDirection = {-1, 2.22044604925e-16, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipYawVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/bearing_roll.stl",
    r_shape = {-0.0175, -0.002, 0.0125},
    lengthDirection = {1, 0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {33, 39, 33},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipYawVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {-0, 0.0165, 0.0125},
    lengthDirection = {0, 0, -1}, widthDirection = {1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipRollVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/hip_l.stl",
    r_shape = {-0.0175, 0, -0.0185},
    lengthDirection = {1, 0, 0}, widthDirection = {0, 2.22044604925e-16, -1},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipRollVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {0.025, 0, -0.0185},
    lengthDirection = {0, -1, 0}, widthDirection = {0, 0, 1},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipPitchVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0.022, 0.0357771, 0.0125},
    lengthDirection = {2.22044604925e-16, 0, 1}, widthDirection = {0, 1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipPitchVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration__22x16x4.stl",
    r_shape = {-0.022, 0.0357771, 0},
    lengthDirection = {1, -0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipPitchVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/upper_leg_rigidity_plate.stl",
    r_shape = {-0, -0, 0.0435},
    lengthDirection = {0, 0, -1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipPitchVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/upper_leg_right.stl",
    r_shape = {-0, -0, -0.0425},
    lengthDirection = {0, 0, -1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {217, 217, 214},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightHipPitchVisual5(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0, -0, 0.0125},
    lengthDirection = {0, 0, 1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightKneeVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/xl330.stl",
    r_shape = {-0.042, 0, -0.0115},
    lengthDirection = {2.22044604925e-16, 0, 1}, widthDirection = {0, 1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {73, 73, 73},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightKneeVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/leg.stl",
    r_shape = {-0.0357771, -0.022, 0.0385},
    lengthDirection = {2.22044604925e-16, 0, -1}, widthDirection = {0, 1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {172, 163, 154},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightAnkleVisual1(
    shapeType = "modelica://MicroDuck/Resources/Meshes/ankle_right.stl",
    r_shape = {0.022, 0.00622291, -0.0647},
    lengthDirection = {0, 0, -1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightAnkleVisual2(
    shapeType = "modelica://MicroDuck/Resources/Meshes/sole_right.stl",
    r_shape = {0.022, 0.00622291, -0.0647},
    lengthDirection = {0, 0, -1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {244, 175, 35},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightAnkleVisual3(
    shapeType = "modelica://MicroDuck/Resources/Meshes/seeed_bearing__configuration_default.stl",
    r_shape = {0, -0, -0.0292},
    lengthDirection = {1, 0, 0}, widthDirection = {0, -1, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {207, 219, 229},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape rightAnkleVisual4(
    shapeType = "modelica://MicroDuck/Resources/Meshes/foot_right.stl",
    r_shape = {0.022, 0.00622291, -0.0647},
    lengthDirection = {0, 0, -1}, widthDirection = {-1, 0, 0},
    length = 1, width = 1, height = 1, extra = 0, color = {255, 103, 31},
    animation = animation) annotation(Placement(visible = false, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
protected
  Modelica.Blocks.Interfaces.RealInput mouthTarget(unit = "rad") "Mouth angle target in use" annotation(Placement(transformation(origin = {30, -155}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Interfaces.RealOutput iBattery(unit = "A") = batteryCurrent "Total servo supply current" annotation(Placement(transformation(origin = {-185, 75}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Interfaces.RealInput vBus(unit = "V") "Supply bus voltage in use" annotation(Placement(transformation(origin = {-135, 79}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Interfaces.RealInput socBus(unit = "1") "State of charge in use" annotation(Placement(transformation(origin = {-135, 71}, extent = {{-5, -5}, {5, 5}})));
equation
  // Frames
  connect(frame_a, trunk.frame_a) annotation(Line(points = {{-200, -125}, {-160, -125}}, color = {95, 95, 95}));
  connect(trunk.frame_a, leftHipYaw.frame_a) annotation(Line(points = {{-160, -125}, {-160, 140}, {-130, 140}}, color = {95, 95, 95}));
  connect(leftHipYaw.frame_b, leftHipRoll.frame_a) annotation(Line(points = {{-110, 140}, {-85, 140}}, color = {95, 95, 95}));
  connect(leftHipRoll.frame_b, leftHipPitch.frame_a) annotation(Line(points = {{-65, 140}, {-40, 140}}, color = {95, 95, 95}));
  connect(leftHipPitch.frame_b, leftKnee.frame_a) annotation(Line(points = {{-20, 140}, {5, 140}}, color = {95, 95, 95}));
  connect(leftKnee.frame_b, leftAnkle.frame_a) annotation(Line(points = {{25, 140}, {50, 140}}, color = {95, 95, 95}));
  connect(trunk.frame_a, neckPitch.frame_a) annotation(Line(points = {{-160, -125}, {-160, -60}, {-130, -60}}, color = {95, 95, 95}));
  connect(neckPitch.frame_b, headPitch.frame_a) annotation(Line(points = {{-110, -60}, {-85, -60}}, color = {95, 95, 95}));
  connect(headPitch.frame_b, headYaw.frame_a) annotation(Line(points = {{-65, -60}, {-40, -60}}, color = {95, 95, 95}));
  connect(headYaw.frame_b, headRoll.frame_a) annotation(Line(points = {{-20, -60}, {5, -60}}, color = {95, 95, 95}));
  connect(trunk.frame_a, rightHipYaw.frame_a) annotation(Line(points = {{-160, -125}, {-160, 40}, {-130, 40}}, color = {95, 95, 95}));
  connect(rightHipYaw.frame_b, rightHipRoll.frame_a) annotation(Line(points = {{-110, 40}, {-85, 40}}, color = {95, 95, 95}));
  connect(rightHipRoll.frame_b, rightHipPitch.frame_a) annotation(Line(points = {{-65, 40}, {-40, 40}}, color = {95, 95, 95}));
  connect(rightHipPitch.frame_b, rightKnee.frame_a) annotation(Line(points = {{-20, 40}, {5, 40}}, color = {95, 95, 95}));
  connect(rightKnee.frame_b, rightAnkle.frame_a) annotation(Line(points = {{25, 40}, {50, 40}}, color = {95, 95, 95}));
  connect(qRef[1], leftHipYaw.phiRef) annotation(Line(points = {{-200, 170}, {-125, 170}, {-125, 151}}, color = {0, 0, 127}));
  connect(qRef[2], leftHipRoll.phiRef) annotation(Line(points = {{-200, 170}, {-80, 170}, {-80, 151}}, color = {0, 0, 127}));
  connect(qRef[3], leftHipPitch.phiRef) annotation(Line(points = {{-200, 170}, {-35, 170}, {-35, 151}}, color = {0, 0, 127}));
  connect(qRef[4], leftKnee.phiRef) annotation(Line(points = {{-200, 170}, {10, 170}, {10, 151}}, color = {0, 0, 127}));
  connect(qRef[5], leftAnkle.phiRef) annotation(Line(points = {{-200, 170}, {55, 170}, {55, 151}}, color = {0, 0, 127}));
  connect(qRef[6], neckPitch.phiRef) annotation(Line(points = {{-200, 170}, {-200, -5}, {-125, -5}, {-125, -49}}, color = {0, 0, 127}));
  connect(qRef[7], headPitch.phiRef) annotation(Line(points = {{-200, 170}, {-200, -5}, {-80, -5}, {-80, -49}}, color = {0, 0, 127}));
  connect(qRef[8], headYaw.phiRef) annotation(Line(points = {{-200, 170}, {-200, -5}, {-35, -5}, {-35, -49}}, color = {0, 0, 127}));
  connect(qRef[9], headRoll.phiRef) annotation(Line(points = {{-200, 170}, {-200, -5}, {10, -5}, {10, -49}}, color = {0, 0, 127}));
  connect(qRef[10], rightHipYaw.phiRef) annotation(Line(points = {{-200, 170}, {-200, 95}, {-125, 95}, {-125, 51}}, color = {0, 0, 127}));
  connect(qRef[11], rightHipRoll.phiRef) annotation(Line(points = {{-200, 170}, {-200, 95}, {-80, 95}, {-80, 51}}, color = {0, 0, 127}));
  connect(qRef[12], rightHipPitch.phiRef) annotation(Line(points = {{-200, 170}, {-200, 95}, {-35, 95}, {-35, 51}}, color = {0, 0, 127}));
  connect(qRef[13], rightKnee.phiRef) annotation(Line(points = {{-200, 170}, {-200, 95}, {10, 95}, {10, 51}}, color = {0, 0, 127}));
  connect(qRef[14], rightAnkle.phiRef) annotation(Line(points = {{-200, 170}, {-200, 95}, {55, 95}, {55, 51}}, color = {0, 0, 127}));
  // Joint readout and supply
  q[1] = leftHipYaw.phi;
  w[1] = leftHipYaw.w;
  motorCurrent[1] = leftHipYaw.motorCurrent;
  copperLoss[1] = leftHipYaw.copperLoss;
  frictionLoss[1] = leftHipYaw.frictionLoss;
  windingTemperature[1] = leftHipYaw.windingTemperature;
  housingTemperature[1] = leftHipYaw.housingTemperature;
  connect(vBus, leftHipYaw.vSupply);
  q[2] = leftHipRoll.phi;
  w[2] = leftHipRoll.w;
  motorCurrent[2] = leftHipRoll.motorCurrent;
  copperLoss[2] = leftHipRoll.copperLoss;
  frictionLoss[2] = leftHipRoll.frictionLoss;
  windingTemperature[2] = leftHipRoll.windingTemperature;
  housingTemperature[2] = leftHipRoll.housingTemperature;
  connect(vBus, leftHipRoll.vSupply);
  q[3] = leftHipPitch.phi;
  w[3] = leftHipPitch.w;
  motorCurrent[3] = leftHipPitch.motorCurrent;
  copperLoss[3] = leftHipPitch.copperLoss;
  frictionLoss[3] = leftHipPitch.frictionLoss;
  windingTemperature[3] = leftHipPitch.windingTemperature;
  housingTemperature[3] = leftHipPitch.housingTemperature;
  connect(vBus, leftHipPitch.vSupply);
  q[4] = leftKnee.phi;
  w[4] = leftKnee.w;
  motorCurrent[4] = leftKnee.motorCurrent;
  copperLoss[4] = leftKnee.copperLoss;
  frictionLoss[4] = leftKnee.frictionLoss;
  windingTemperature[4] = leftKnee.windingTemperature;
  housingTemperature[4] = leftKnee.housingTemperature;
  connect(vBus, leftKnee.vSupply);
  q[5] = leftAnkle.phi;
  w[5] = leftAnkle.w;
  motorCurrent[5] = leftAnkle.motorCurrent;
  copperLoss[5] = leftAnkle.copperLoss;
  frictionLoss[5] = leftAnkle.frictionLoss;
  windingTemperature[5] = leftAnkle.windingTemperature;
  housingTemperature[5] = leftAnkle.housingTemperature;
  connect(vBus, leftAnkle.vSupply);
  q[6] = neckPitch.phi;
  w[6] = neckPitch.w;
  motorCurrent[6] = neckPitch.motorCurrent;
  copperLoss[6] = neckPitch.copperLoss;
  frictionLoss[6] = neckPitch.frictionLoss;
  windingTemperature[6] = neckPitch.windingTemperature;
  housingTemperature[6] = neckPitch.housingTemperature;
  connect(vBus, neckPitch.vSupply);
  q[7] = headPitch.phi;
  w[7] = headPitch.w;
  motorCurrent[7] = headPitch.motorCurrent;
  copperLoss[7] = headPitch.copperLoss;
  frictionLoss[7] = headPitch.frictionLoss;
  windingTemperature[7] = headPitch.windingTemperature;
  housingTemperature[7] = headPitch.housingTemperature;
  connect(vBus, headPitch.vSupply);
  q[8] = headYaw.phi;
  w[8] = headYaw.w;
  motorCurrent[8] = headYaw.motorCurrent;
  copperLoss[8] = headYaw.copperLoss;
  frictionLoss[8] = headYaw.frictionLoss;
  windingTemperature[8] = headYaw.windingTemperature;
  housingTemperature[8] = headYaw.housingTemperature;
  connect(vBus, headYaw.vSupply);
  q[9] = headRoll.phi;
  w[9] = headRoll.w;
  motorCurrent[9] = headRoll.motorCurrent;
  copperLoss[9] = headRoll.copperLoss;
  frictionLoss[9] = headRoll.frictionLoss;
  windingTemperature[9] = headRoll.windingTemperature;
  housingTemperature[9] = headRoll.housingTemperature;
  connect(vBus, headRoll.vSupply);
  q[10] = rightHipYaw.phi;
  w[10] = rightHipYaw.w;
  motorCurrent[10] = rightHipYaw.motorCurrent;
  copperLoss[10] = rightHipYaw.copperLoss;
  frictionLoss[10] = rightHipYaw.frictionLoss;
  windingTemperature[10] = rightHipYaw.windingTemperature;
  housingTemperature[10] = rightHipYaw.housingTemperature;
  connect(vBus, rightHipYaw.vSupply);
  q[11] = rightHipRoll.phi;
  w[11] = rightHipRoll.w;
  motorCurrent[11] = rightHipRoll.motorCurrent;
  copperLoss[11] = rightHipRoll.copperLoss;
  frictionLoss[11] = rightHipRoll.frictionLoss;
  windingTemperature[11] = rightHipRoll.windingTemperature;
  housingTemperature[11] = rightHipRoll.housingTemperature;
  connect(vBus, rightHipRoll.vSupply);
  q[12] = rightHipPitch.phi;
  w[12] = rightHipPitch.w;
  motorCurrent[12] = rightHipPitch.motorCurrent;
  copperLoss[12] = rightHipPitch.copperLoss;
  frictionLoss[12] = rightHipPitch.frictionLoss;
  windingTemperature[12] = rightHipPitch.windingTemperature;
  housingTemperature[12] = rightHipPitch.housingTemperature;
  connect(vBus, rightHipPitch.vSupply);
  q[13] = rightKnee.phi;
  w[13] = rightKnee.w;
  motorCurrent[13] = rightKnee.motorCurrent;
  copperLoss[13] = rightKnee.copperLoss;
  frictionLoss[13] = rightKnee.frictionLoss;
  windingTemperature[13] = rightKnee.windingTemperature;
  housingTemperature[13] = rightKnee.housingTemperature;
  connect(vBus, rightKnee.vSupply);
  q[14] = rightAnkle.phi;
  w[14] = rightAnkle.w;
  motorCurrent[14] = rightAnkle.motorCurrent;
  copperLoss[14] = rightAnkle.copperLoss;
  frictionLoss[14] = rightAnkle.frictionLoss;
  windingTemperature[14] = rightAnkle.windingTemperature;
  housingTemperature[14] = rightAnkle.housingTemperature;
  connect(vBus, rightAnkle.vSupply);
  connect(vBus, mouth.vSupply);
  batteryCurrent = sum({leftHipYaw.supplyCurrent, leftHipRoll.supplyCurrent, leftHipPitch.supplyCurrent, leftKnee.supplyCurrent, leftAnkle.supplyCurrent, neckPitch.supplyCurrent, headPitch.supplyCurrent, headYaw.supplyCurrent, headRoll.supplyCurrent, rightHipYaw.supplyCurrent, rightHipRoll.supplyCurrent, rightHipPitch.supplyCurrent, rightKnee.supplyCurrent, rightAnkle.supplyCurrent});
  connect(iBattery, battery.iLoad) annotation(Line(points = {{-185, 75}, {-171, 75}}, color = {0, 0, 127}));
  connect(battery.vTerminal, vBus) annotation(Line(points = {{-149, 79}, {-135, 79}}, color = {0, 0, 127}));
  connect(battery.soc, socBus) annotation(Line(points = {{-149, 71}, {-135, 71}}, color = {0, 0, 127}));
  if not useBattery then
    vBus = supplyVoltage;
    socBus = soc0;
  end if;
  terminalVoltage = vBus;
  soc = socBus;
  // Soles and ground contact
  for i in 1:8 loop
    connect(leftAnkle.frame_b, soleLeft[i].frame_a) annotation(Line(points = {{70, 140}, {90, 140}}, color = {95, 95, 95}));
    connect(soleLeft[i].frame_b, contactLeft[i].frame_b) annotation(Line(points = {{110, 140}, {130, 140}}, color = {95, 95, 95}));
    connect(ground.frame_b, contactLeft[i].frame_a) annotation(Line(points = {{160, 95}, {160, 140}, {150, 140}}, color = {95, 95, 95}));
  end for;
  for i in 1:8 loop
    connect(rightAnkle.frame_b, soleRight[i].frame_a) annotation(Line(points = {{70, 40}, {90, 40}}, color = {95, 95, 95}));
    connect(soleRight[i].frame_b, contactRight[i].frame_b) annotation(Line(points = {{110, 40}, {130, 40}}, color = {95, 95, 95}));
    connect(ground.frame_b, contactRight[i].frame_a) annotation(Line(points = {{160, 95}, {160, 40}, {150, 40}}, color = {95, 95, 95}));
  end for;
  // Sites
  connect(trunk.frame_a, siteImu.frame_a) annotation(Line(points = {{-160, -125}, {-110, -125}}, color = {95, 95, 95}));
  connect(siteImu.frame_b, frame_imu) annotation(Line(points = {{-90, -125}, {205, -125}}, color = {95, 95, 95}));
  connect(leftAnkle.frame_b, siteLeftFoot.frame_a) annotation(Line(points = {{70, 140}, {80, 140}, {80, 165}, {90, 165}}, color = {95, 95, 95}));
  connect(siteLeftFoot.frame_b, frame_leftFoot) annotation(Line(points = {{110, 165}, {205, 165}}, color = {95, 95, 95}));
  connect(headRoll.frame_b, siteHeadCamera.frame_a) annotation(Line(points = {{25, -60}, {45, -60}, {45, -30}, {100, -30}}, color = {95, 95, 95}));
  connect(siteHeadCamera.frame_b, frame_headCamera) annotation(Line(points = {{120, -30}, {205, -30}}, color = {95, 95, 95}));
  connect(headRoll.frame_b, siteTof.frame_a) annotation(Line(points = {{25, -60}, {45, -60}, {45, -60}, {100, -60}}, color = {95, 95, 95}));
  connect(siteTof.frame_b, frame_tof) annotation(Line(points = {{120, -60}, {205, -60}}, color = {95, 95, 95}));
  connect(headRoll.frame_b, siteMouthTip.frame_a) annotation(Line(points = {{25, -60}, {45, -60}, {45, -90}, {100, -90}}, color = {95, 95, 95}));
  connect(siteMouthTip.frame_b, frame_mouthTip) annotation(Line(points = {{120, -90}, {205, -90}}, color = {95, 95, 95}));
  connect(rightAnkle.frame_b, siteRightFoot.frame_a) annotation(Line(points = {{70, 40}, {80, 40}, {80, 15}, {90, 15}}, color = {95, 95, 95}));
  connect(siteRightFoot.frame_b, frame_rightFoot) annotation(Line(points = {{110, 15}, {205, 15}}, color = {95, 95, 95}));
  // Mouth, driven by mouthRef or held at mouthHome
  connect(headRoll.frame_b, mouth.frame_a) annotation(Line(points = {{25, -60}, {45, -60}, {45, -112}, {60, -112}}, color = {95, 95, 95}));
  connect(mouthRef, mouthTarget) annotation(Line(points = {{-200, -155}, {30, -155}}, color = {0, 0, 127}));
  connect(mouthTarget, mouth.phiRef) annotation(Line(points = {{30, -155}, {90, -155}, {90, -94}, {65, -94}, {65, -101}}, color = {0, 0, 127}));
  if not useMouthInput then
    mouthTarget = mouthHome;
  end if;
  // Meshes
  connect(trunk.frame_a, trunkVisual1.frame_a);
  connect(trunk.frame_a, trunkVisual2.frame_a);
  connect(trunk.frame_a, trunkVisual3.frame_a);
  connect(trunk.frame_a, trunkVisual4.frame_a);
  connect(trunk.frame_a, trunkVisual5.frame_a);
  connect(trunk.frame_a, trunkVisual6.frame_a);
  connect(trunk.frame_a, trunkVisual7.frame_a);
  connect(trunk.frame_a, trunkVisual8.frame_a);
  connect(trunk.frame_a, trunkVisual9.frame_a);
  connect(trunk.frame_a, trunkVisual10.frame_a);
  connect(leftHipYaw.frame_b, leftHipYawVisual1.frame_a);
  connect(leftHipYaw.frame_b, leftHipYawVisual2.frame_a);
  connect(leftHipYaw.frame_b, leftHipYawVisual3.frame_a);
  connect(leftHipYaw.frame_b, leftHipYawVisual4.frame_a);
  connect(leftHipRoll.frame_b, leftHipRollVisual1.frame_a);
  connect(leftHipRoll.frame_b, leftHipRollVisual2.frame_a);
  connect(leftHipPitch.frame_b, leftHipPitchVisual1.frame_a);
  connect(leftHipPitch.frame_b, leftHipPitchVisual2.frame_a);
  connect(leftHipPitch.frame_b, leftHipPitchVisual3.frame_a);
  connect(leftHipPitch.frame_b, leftHipPitchVisual4.frame_a);
  connect(leftHipPitch.frame_b, leftHipPitchVisual5.frame_a);
  connect(leftKnee.frame_b, leftKneeVisual1.frame_a);
  connect(leftKnee.frame_b, leftKneeVisual2.frame_a);
  connect(leftAnkle.frame_b, leftAnkleVisual1.frame_a);
  connect(leftAnkle.frame_b, leftAnkleVisual2.frame_a);
  connect(leftAnkle.frame_b, leftAnkleVisual3.frame_a);
  connect(leftAnkle.frame_b, leftAnkleVisual4.frame_a);
  connect(neckPitch.frame_b, neckPitchVisual1.frame_a);
  connect(neckPitch.frame_b, neckPitchVisual2.frame_a);
  connect(neckPitch.frame_b, neckPitchVisual3.frame_a);
  connect(neckPitch.frame_b, neckPitchVisual4.frame_a);
  connect(headPitch.frame_b, headPitchVisual1.frame_a);
  connect(headPitch.frame_b, headPitchVisual2.frame_a);
  connect(headYaw.frame_b, headYawVisual1.frame_a);
  connect(headYaw.frame_b, headYawVisual2.frame_a);
  connect(headYaw.frame_b, headYawVisual3.frame_a);
  connect(headYaw.frame_b, headYawVisual4.frame_a);
  connect(headRoll.frame_b, headRollVisual1.frame_a);
  connect(headRoll.frame_b, headRollVisual2.frame_a);
  connect(headRoll.frame_b, headRollVisual3.frame_a);
  connect(mouth.frame_b, mouthVisual1.frame_a);
  connect(headRoll.frame_b, headRollVisual4.frame_a);
  connect(headRoll.frame_b, headRollVisual5.frame_a);
  connect(headRoll.frame_b, headRollVisual6.frame_a);
  connect(headRoll.frame_b, headRollVisual7.frame_a);
  connect(headRoll.frame_b, headRollVisual8.frame_a);
  connect(headRoll.frame_b, headRollVisual9.frame_a);
  connect(headRoll.frame_b, headRollVisual10.frame_a);
  connect(headRoll.frame_b, headRollVisual11.frame_a);
  connect(mouth.frame_b, mouthVisual2.frame_a);
  connect(headRoll.frame_b, headRollVisual12.frame_a);
  connect(headRoll.frame_b, headRollVisual13.frame_a);
  connect(headRoll.frame_b, headRollVisual14.frame_a);
  connect(headRoll.frame_b, headRollVisual15.frame_a);
  connect(headRoll.frame_b, headRollVisual16.frame_a);
  connect(rightHipYaw.frame_b, rightHipYawVisual1.frame_a);
  connect(rightHipYaw.frame_b, rightHipYawVisual2.frame_a);
  connect(rightHipYaw.frame_b, rightHipYawVisual3.frame_a);
  connect(rightHipYaw.frame_b, rightHipYawVisual4.frame_a);
  connect(rightHipRoll.frame_b, rightHipRollVisual1.frame_a);
  connect(rightHipRoll.frame_b, rightHipRollVisual2.frame_a);
  connect(rightHipPitch.frame_b, rightHipPitchVisual1.frame_a);
  connect(rightHipPitch.frame_b, rightHipPitchVisual2.frame_a);
  connect(rightHipPitch.frame_b, rightHipPitchVisual3.frame_a);
  connect(rightHipPitch.frame_b, rightHipPitchVisual4.frame_a);
  connect(rightHipPitch.frame_b, rightHipPitchVisual5.frame_a);
  connect(rightKnee.frame_b, rightKneeVisual1.frame_a);
  connect(rightKnee.frame_b, rightKneeVisual2.frame_a);
  connect(rightAnkle.frame_b, rightAnkleVisual1.frame_a);
  connect(rightAnkle.frame_b, rightAnkleVisual2.frame_a);
  connect(rightAnkle.frame_b, rightAnkleVisual3.frame_a);
  connect(rightAnkle.frame_b, rightAnkleVisual4.frame_a);

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-100, -100}, {100, 100}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid),
      Polygon(points = {{2, -82}, {13, -76}, {5, -56}, {1, -55}, {4, -52}, {16, -31}, {18, -29}, {18, -22}, {16, -20}, {16, -18}, {11, -16}, {10, -13}, {-5, -14}, {-5, -15}, {-10, -16}, {-11, -16}, {-11, -23}, {-20, -28}, {-23, -33}, {-21, -41}, {-17, -47}, {-9, -67}, {-6, -71}, {-6, -73}, {-3, -80}}, lineColor = {64, 64, 64}, fillColor = {140, 140, 140}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-8, -88}, {21, -88}, {26, -85}, {26, -79}, {25, -78}, {15, -76}, {11, -69}, {9, -68}, {4, -68}, {2, -66}, {-2, -65}, {-6, -68}, {-11, -76}, {-13, -78}, {-13, -84}}, lineColor = {64, 64, 64}, fillColor = {204, 82, 25}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-21, -41}, {-19, -41}, {-18, -38}, {-15, -37}, {-14, -15}, {8, -15}, {8, -13}, {10, -13}, {9, -11}, {11, -8}, {12, 3}, {16, 4}, {18, 7}, {18, 9}, {16, 13}, {13, 14}, {-1, 14}, {-21, 15}, {-34, 14}, {-40, 10}, {-41, 2}, {-40, -5}, {-38, -9}, {-38, -37}, {-26, -38}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Polygon(points = {{3, 1}, {8, 1}, {19, 5}, {8, 38}, {8, 48}, {7, 49}, {4, 50}, {6, 50}, {6, 52}, {4, 53}, {10, 54}, {12, 58}, {12, 63}, {10, 67}, {8, 70}, {-14, 70}, {-16, 64}, {-16, 58}, {-14, 54}, {-7, 53}, {-9, 52}, {-9, 50}, {-8, 50}, {-10, 49}, {-11, 43}, {-12, 43}, {-10, 36}, {-6, 33}, {-5, 29}, {-7, 28}, {-6, 24}, {-2, 15}, {-1, 15}, {1, 12}, {-0, 9}}, lineColor = {64, 64, 64}, fillColor = {80, 85, 90}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-22, -82}, {-16, -80}, {-6, -75}, {-10, -67}, {-11, -66}, {-12, -60}, {-20, -44}, {-17, -38}, {-16, -40}, {3, -39}, {3, -31}, {8, -26}, {8, -22}, {6, -18}, {3, -16}, {3, -13}, {-12, -14}, {-13, -15}, {-17, -16}, {-19, -18}, {-23, -19}, {-42, -29}, {-46, -34}, {-45, -40}, {-38, -54}, {-35, -58}, {-28, -56}, {-27, -59}, {-32, -62}}, lineColor = {64, 64, 64}, fillColor = {172, 164, 153}, fillPattern = FillPattern.Solid),
      Polygon(points = {{-27, -88}, {2, -88}, {8, -85}, {8, -78}, {6, -76}, {-3, -75}, {-8, -68}, {-12, -65}, {-14, -65}, {-18, -68}, {-23, -68}, {-25, -70}, {-29, -78}, {-32, -79}, {-32, -85}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Polygon(points = {{15, 49}, {42, 50}, {44, 52}, {45, 58}, {42, 58}, {39, 60}, {35, 60}, {35, 59}, {18, 60}, {17, 59}, {13, 67}, {9, 69}, {6, 68}, {4, 66}, {2, 64}, {3, 59}, {-2, 59}, {-7, 67}, {-12, 69}, {-16, 66}, {-17, 59}, {-34, 60}, {-34, 59}, {-38, 59}, {-38, 58}, {-38, 52}, {-35, 50}}, lineColor = {64, 64, 64}, fillColor = {255, 102, 31}, fillPattern = FillPattern.Solid),
      Polygon(points = {{21, 57}, {45, 58}, {46, 81}, {44, 85}, {39, 88}, {-3, 85}, {-23, 81}, {-29, 78}, {-34, 73}, {-38, 60}, {-38, 58}}, lineColor = {64, 64, 64}, fillColor = {216, 216, 212}, fillPattern = FillPattern.Solid),
      Polygon(points = {{32, 61}, {38, 61}, {40, 65}, {40, 73}, {37, 80}, {31, 80}, {29, 76}, {19, 76}, {19, 66}, {29, 66}}, lineColor = {64, 64, 64}, fillColor = {66, 71, 77}, fillPattern = FillPattern.Solid),
      Text(extent = {{-150, 140}, {150, 100}}, textString = "%name", textColor = {64, 64, 64})}),
    Diagram(coordinateSystem(extent = {{-220, -180}, {220, 190}})),
    Documentation(info = "<html>
<h4>Robot</h4>
<p>Rigid-body model of the Microduck with a servo at each of its 14 joints and contact points under both soles.</p>
<p>Figure 1 shows the robot at the home pose <code>q0</code>. A dot marks each joint axis, where a <a href=\"modelica://MicroDuck.Components.Link\">Link</a> with its <a href=\"modelica://MicroDuck.Components.PositionServo\">PositionServo</a> sits.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://MicroDuck/Resources/Images/Components/Robot/robot.svg\"
     width=\"600\"
     alt=\"Microduck at the home pose with its 14 joint axes and the joint chains of the neck, head and legs\">
<strong>Figure 1:</strong> Robot at the home pose with its joints.</p>
<p>
Built from the training MJCF file (MuJoCo XML model) of Pollen Robotics (Apache-2.0), with eight contact points per sole on the flat contact patch inside the perimeter bevel.
Joint order in <code>q0</code>, <code>q</code>, <code>w</code> and <code>qRef</code>: left leg (hip yaw, roll, pitch, knee, ankle), neck and head (neck pitch, head pitch, yaw, roll), right leg.
<code>frame_a</code> is the trunk frame; the other frames are the MJCF sites. The ground is the plane z = 0 of the world, so gravity must point along negative z.
The 70 meshes in <code>Resources/Meshes</code> animate the robot; <code>animation = false</code> hides them.
</p>
<p>The parameters that decide the behaviour are:</p>
<p>
<code>positionGain</code>, <code>supplyVoltage</code> = Servo firmware gain and fixed supply voltage when <code>useBattery</code> is <code>false</code><br>
<code>useBattery</code>, <code>soc0</code>, <code>batteryCapacity</code>, <code>batteryResistance</code> = Shared 2S NP-F550 <a href=\"modelica://MicroDuck.Components.Battery\">Battery</a> pack (built on <code>Modelica.Electrical.Batteries.BatteryStacks.CellRCStack</code>) with state of charge <code>soc</code>, nonlinear open-circuit voltage and two RC branches feeding <code>terminalVoltage</code> to all servos from their total H-bridge draw <code>batteryCurrent</code><br>
<code>useThermal</code> = Enables the two-node winding and housing thermal network in each servo (<a href=\"modelica://MicroDuck.Components.PositionServo\">PositionServo</a>) with copper and friction heating and temperature-dependent resistance<br>
<code>stopStiffness</code>, <code>stopDamping</code> = Joint limit stops; the ranges come from the MJCF file<br>
<code>backlash</code>, <code>backlashStiffness</code>, <code>backlashDamping</code> = Gear play of every servo (2&deg; as in training) and its tooth contact<br>
<code>mu</code>, <code>contactTimeConstant</code>, <code>contactDampingRatio</code>, <code>contactAcceleration</code> = Friction, MuJoCo solref and inertial term of the 8 contact points per sole; the defaults are MuJoCo's
</p>
<p>The lower beak is rigid in the head, as in the training model. <code>useMouth = true</code> adds it as a fifteenth servo-driven <code>Link</code>, <code>mouth</code> (positive <code>mouth.phi</code> opens, range &minus;5&deg; to +30&deg; as in the robot software). It is not part of <code>q</code>, <code>w</code> and <code>qRef</code>; it holds <code>mouthHome</code>, or follows <code>mouthRef</code> with <code>useMouthInput = true</code>.</p>
<h4>Limitations</h4>
<p>The mouth of <code>useMouth</code> is an estimate, because no training model has it: the beak turns about the output axis of the second XL330 servo in the head, and its 32 g come from its mesh volume at the density of the printed ankle link (1.14 g/cm&sup3;) and are taken off the head. Only the soles collide with the ground.</p>
</html>"));
end Robot;
