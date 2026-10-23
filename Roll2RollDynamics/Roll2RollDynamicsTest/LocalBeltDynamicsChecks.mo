within Roll2RollDynamicsTest;
package LocalBeltDynamicsChecks
  "Independent force-driven checks for span-owned material dynamics"
  extends Modelica.Icons.ExamplesPackage;

  model ForceBoundary
    "Prescribed position and transport force with free tangent direction"
    extends Modelica.Icons.Example;
    //Inputs and Outputs
    input Modelica.Units.SI.Position position[3] = zeros(3)
      "Prescribed tangency position";
    input Modelica.Units.SI.Force force = 150
      "Force entering this boundary through the transport flange";
    //Variables
    Modelica.Units.SI.Angle yaw(start=0) "Free tangent yaw";
    Modelica.Units.SI.Angle skew(start=0) "Free tangent skew";
    //Physical connectors
    Roll2RollDynamics.Utilities.Interfaces.RollPort port
      "Connection to one span end"
      annotation(Placement(visible = true, transformation(origin = {100, 0},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    port.web.s = 0;
  equation
    Connections.root(port.frame.R);
    port.frame.r_0 = position;
    skew = Modelica.Math.atan2(port.spanDirection[2],
      cos(yaw)*port.spanDirection[1] - sin(yaw)*port.spanDirection[3]);
    port.frame.R = Modelica.Mechanics.MultiBody.Frames.axesRotations(
      {2,3,1}, {yaw,skew,0}, {der(yaw),der(skew),0});
    port.web.f = force;
    annotation(Documentation(info="<html><h4>Force boundary</h4><p>Helper boundary that prescribes tangency position and transport force while the attached span determines the tangent angles. It leaves frame force and moment free so the span can deliver its complete support reaction; it is not a standalone test.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-100, -100}, {140, 100}})));
  end ForceBoundary;

  partial model Fixture
    "Integrate external mass, momentum and energy exchange independently"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Modelica.Units.SI.Acceleration gravity = 0 "Gravity magnitude";
    parameter Modelica.Units.SI.Mass initialMass(fixed=false)
      "Initial material inventory";
    parameter Modelica.Units.SI.Momentum initialMomentum[3](each fixed=false)
      "Initial spatial material momentum";
    parameter Modelica.Units.SI.Energy initialEnergy(fixed=false)
      "Initial kinetic and elastic energy";
    //Variables with Binding Equations
    Modelica.Units.SI.Position positionA[3] = zeros(3)
      "Entry boundary position";
    Modelica.Units.SI.Position positionB[3] = {1,0,0}
      "Exit boundary position";
    Modelica.Units.SI.Force entryTension = 150 "Prescribed entry tension";
    Modelica.Units.SI.Force exitTension = 150 "Prescribed exit tension";
    //Variables
    Modelica.Units.SI.Velocity boundaryVelocityA[3] "Entry boundary velocity";
    Modelica.Units.SI.Velocity boundaryVelocityB[3] "Exit boundary velocity";
    Modelica.Units.SI.Velocity materialVelocityA[3] "Physical entry velocity";
    Modelica.Units.SI.Velocity materialVelocityB[3] "Physical exit velocity";
    Modelica.Units.SI.Force forceA[3] "Entry spatial force into the span";
    Modelica.Units.SI.Force forceB[3] "Exit spatial force into the span";
    Modelica.Units.SI.Force supportResultant[3] "Total spatial boundary force";
    Real direction[3](each unit="1") "Span direction in world coordinates";
    Real stretchA(unit="1") "Entry constitutive stretch";
    Real stretchB(unit="1") "Exit constitutive stretch";
    Modelica.Units.SI.MassFlowRate massFlowA "Entry reference-material flux";
    Modelica.Units.SI.MassFlowRate massFlowB "Exit reference-material flux";
    Modelica.Units.SI.SpecificEnergy strainEnergyA "Entry specific elastic energy";
    Modelica.Units.SI.SpecificEnergy strainEnergyB "Exit specific elastic energy";
    Modelica.Units.SI.Power portPower "Power supplied through the actual connectors";
    Modelica.Units.SI.Power gravityWork "Power supplied by gravity";
    Modelica.Units.SI.Power transportedEnergy "Signed incoming energy flux";
    Modelica.Units.SI.Power velocityLumpingDefect "Central-velocity kinetic defect";
    Modelica.Units.SI.Power dissipatedPower "Viscous work from end forces and velocity differences";
    Modelica.Units.SI.Energy storedEnergy "Independently evaluated stored energy";
    Modelica.Units.SI.Mass integratedMass(min=-Modelica.Constants.inf,
      start=0, fixed=true)
      "Accumulated external material exchange";
    Modelica.Units.SI.Momentum integratedMomentum[3](each start=0, each fixed=true)
      "Accumulated external force and momentum flux";
    Modelica.Units.SI.Energy integratedEnergy(start=0, fixed=true)
      "Accumulated boundary energy including the declared kinetic defect";
    Modelica.Units.SI.Energy integratedDefect(start=0, fixed=true)
      "Accumulated approximation error in the energy balance";
    Modelica.Units.SI.Energy energyResidual "Energy accounting residual";
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=gravity)
      "Reference material and gravity"
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt belt(fixedInitialTension=false, dampingTime=0)
      "Span under test"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    ForceBoundary inlet(position=positionA, force=entryTension)
      "Prescribed entry force and position"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    ForceBoundary outlet(position=positionB, force=-exitTension)
      "Prescribed exit force and position"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    initialMass = belt.beltVariables.webMass;
    initialMomentum = belt.beltVariables.webMass*belt.physicalVelocity;
    initialEnergy = storedEnergy;
  equation
    connect(inlet.port, belt.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(belt.frame_b, outlet.port)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    direction = (positionB-positionA)/Modelica.Math.Vectors.length(positionB-positionA);
    boundaryVelocityA = der(positionA);
    boundaryVelocityB = der(positionB);
    materialVelocityA = boundaryVelocityA + direction*der(inlet.port.web.s);
    materialVelocityB = boundaryVelocityB + direction*der(outlet.port.web.s);
    stretchA = belt.frame_a.stretch;
    stretchB = belt.frame_b.stretch;
    massFlowA = world.web.density*world.web.thickness*world.web.width
      *der(inlet.port.web.s)/stretchA;
    massFlowB = world.web.density*world.web.thickness*world.web.width
      *der(outlet.port.web.s)/stretchB;
    strainEnergyA = world.web.EA*(stretchA-1)^2
      /(2*world.web.density*world.web.thickness*world.web.width);
    strainEnergyB = world.web.EA*(stretchB-1)^2
      /(2*world.web.density*world.web.thickness*world.web.width);
    forceA = Modelica.Mechanics.MultiBody.Frames.resolve1(
      belt.frame_a.frame.R, belt.frame_a.frame.f);
    forceB = Modelica.Mechanics.MultiBody.Frames.resolve1(
      belt.frame_b.frame.R, belt.frame_b.frame.f);
    supportResultant = forceA + forceB;
    portPower = forceA*boundaryVelocityA + forceB*boundaryVelocityB
      + belt.frame_a.web.f*der(inlet.port.web.s)
      + belt.frame_b.web.f*der(outlet.port.web.s);
    gravityWork = belt.beltVariables.webMass*{0,0,-gravity}*belt.physicalVelocity;
    transportedEnergy = massFlowA*(materialVelocityA*materialVelocityA/2 + strainEnergyA)
      - massFlowB*(materialVelocityB*materialVelocityB/2 + strainEnergyB);
    velocityLumpingDefect = massFlowB*(materialVelocityB-belt.physicalVelocity)
      *(materialVelocityB-belt.physicalVelocity)/2
      - massFlowA*(materialVelocityA-belt.physicalVelocity)
      *(materialVelocityA-belt.physicalVelocity)/2;
    storedEnergy = belt.beltVariables.webMass*(belt.physicalVelocity*belt.physicalVelocity)/2
      + world.web.EA*belt.beltVariables.freeLength/4
        *((stretchA-1)^2/stretchA + (stretchB-1)^2/stretchB);
    der(integratedMass) = massFlowA-massFlowB;
    der(integratedMomentum) = supportResultant + belt.beltVariables.webMass*{0,0,-gravity}
      + massFlowA*materialVelocityA-massFlowB*materialVelocityB;
    dissipatedPower = (entryTension - world.web.EA*(stretchA - 1))
        * (direction*(belt.physicalVelocity - materialVelocityA))
      + (exitTension - world.web.EA*(stretchB - 1))
        * (direction*(materialVelocityB - belt.physicalVelocity));
    der(integratedEnergy) = portPower + gravityWork + transportedEnergy
      + velocityLumpingDefect - dissipatedPower;
    der(integratedDefect) = velocityLumpingDefect;
    energyResidual = storedEnergy-initialEnergy-integratedEnergy;
    assert(abs(belt.beltVariables.webMass-initialMass-integratedMass) < 1e-6,
      "Span material inventory differs from independently integrated boundary flux");
    assert(Modelica.Math.Vectors.length(belt.beltVariables.webMass*belt.physicalVelocity
      - initialMomentum-integratedMomentum) < 1e-5,
      "Span spatial momentum differs from boundary forces, gravity and momentum flux");
    assert(abs(energyResidual) < 1e-4,
      "Span energy differs from physical port work, transported energy and declared defect");
    assert(abs(belt.energyDefect-velocityLumpingDefect) < 1e-7,
      "Reported energy defect differs from the boundary-velocity defect");
    assert(dissipatedPower >= -1e-8
      and abs(belt.dampingPower - dissipatedPower) < 1e-7,
      "Dashpot loss must equal nonnegative viscous work at the material velocities");
    annotation(Documentation(info="<html><h4>Span balance fixture</h4><p>Partial helper for independently integrating a span's external mass, spatial momentum and energy exchange from actual connector forces and boundary fluxes. Derived cases must satisfy these balances, including gravity work, the declared central-velocity kinetic defect and nonnegative viscous dissipation; this fixture is not a standalone test.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
  end Fixture;

  model ForceDriven
    "Unequal end forces accelerate the material in a fixed free span"
    extends Fixture(exitTension=151);
  equation
    when terminal() then
      assert(belt.physicalVelocity[1] > 0.1,
        "A sustained positive tension difference must accelerate span material");
      assert(belt.aBelt > 0.3 and belt.aBelt < 0.5,
        "Span acceleration must reflect its own material mass");
    end when;
    annotation(experiment(StopTime=0.5, Tolerance=1e-9),
      Documentation(info="<html><h4>Force driven</h4><p>Applies 1 N more exit tension than entry tension to a fixed span containing approximately 2.65 kg of material. The material must accelerate forward according to its own mass while the independent conservation checks remain satisfied.</p></html>"));
  end ForceDriven;

  model InclinedGravity
    "Gravity along a span drives transport rather than vanishing into support load"
    extends Fixture(gravity=9.81, positionB={sqrt(0.5),0,sqrt(0.5)});
  equation
    assert(abs(belt.aBelt + gravity*sqrt(0.5)) < 1e-6,
      "Inclined-span longitudinal acceleration must include projected gravity");
    assert(abs(supportResultant*direction) < 1e-6,
      "Equal tensions must leave no artificial axial support reaction");
    annotation(experiment(StopTime=0.2, Tolerance=1e-9),
      Documentation(info="<html><h4>Inclined gravity</h4><p>Holds equal end tensions on a 45-degree incline so gravity drives material downhill. Axial acceleration must equal the projected gravitational acceleration, and the supports must add no artificial axial force while carrying the normal load.</p></html>"));
  end InclinedGravity;

  model TranslatingSupports
    "All span mass contributes to mount acceleration and gravity reactions"
    extends Fixture(gravity=9.81,
      positionA={0,time^2,0}, positionB={1,time^2,0});
  equation
    assert(Modelica.Math.Vectors.length(supportResultant
      - belt.beltVariables.webMass*{0,2,gravity}) < 1e-6,
      "Span support forces must include its complete transverse inertia and weight");
    assert(abs(belt.aBelt) < 1e-7,
      "Purely transverse mount acceleration must not accelerate material longitudinally");
    annotation(experiment(StopTime=0.5, Tolerance=1e-9),
      Documentation(info="<html><h4>Translating supports</h4><p>Accelerates both endpoints sideways at 2 m/s2 in gravity. The connector reactions must carry the full span inertia and weight without creating longitudinal material acceleration, while the independent balances close.</p></html>"));
  end TranslatingSupports;

  model RotatingSpan
    "A changing tangent direction contributes to projected material momentum"
    extends Fixture(positionB={cos(time),sin(time),0});
  equation
    assert(abs(belt.beltVariables.webVelocity-0.5*time) < 1e-6,
      "Rotating span must retain the tangent-direction derivative in projected momentum");
    assert(abs(belt.aBelt) < 1e-6,
      "A rotating projection must not be mistaken for physical longitudinal acceleration");
    annotation(experiment(StopTime=0.5, Tolerance=1e-9),
      Documentation(info="<html><h4>Rotating span</h4><p>Rotates a unit span about its inlet at 1 rad/s. The absolute projected material velocity must grow as 0.5 times time while physical longitudinal acceleration remains zero, and the independent conservation checks must pass.</p></html>"));
  end RotatingSpan;

  model ChangingInventory
    "Moving boundaries and changing strains exercise the complete energy accounting"
    extends Fixture(positionB={1+0.04*sin(3*time),0,0},
      entryTension=150+5*sin(2*time), exitTension=151+3*cos(2*time));
  equation
    when terminal() then
      assert(abs(integratedMass) > 1e-3,
        "Changing geometry and strain must exercise nonzero material exchange");
      assert(abs(integratedDefect) > 1e-8,
        "This case must exercise the central-velocity energy approximation");
    end when;
    annotation(experiment(StopTime=0.5, Tolerance=1e-9),
      Documentation(info="<html><h4>Changing inventory</h4><p>Combines smooth endpoint motion with independently varying end tensions to exercise nonzero material exchange and a nonzero central-velocity kinetic defect. The integrated mass, momentum and energy checks must close with that defect included explicitly.</p></html>"));
  end ChangingInventory;
  model ReverseTransport "Nonzero reverse transport with externally initialized momentum"
    extends Fixture(belt(fixedInitialMomentum=false));
  initial equation
    belt.momentum = -1.5*belt.beltVariables.webMass;
  equation
    assert(abs(belt.beltVariables.webVelocity+1.5) < 1e-7 and abs(belt.aBelt) < 1e-7,
      "Equal boundary tensions must preserve the initialized reverse material velocity");
    assert(belt.massFlowIn < 0 and belt.massFlowOut < 0,
      "Reverse material velocity must produce signed reverse boundary fluxes");
    annotation(experiment(StopTime=0.5,Tolerance=1e-9));
    annotation(Documentation(info="<html><h4>Reverse transport</h4><p>Initializes material velocity at -1.5 m/s between fixed boundaries with equal tensions. Velocity must remain constant, both boundary mass flows must be negative, and the independent mass, momentum and energy checks must pass.</p></html>"));
  end ReverseTransport;
  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Force-driven isolated span regressions for
    material acceleration, support loads and open-system accounting.</p></html>"));
end LocalBeltDynamicsChecks;
