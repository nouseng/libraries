within Roll2RollDynamicsTest;
package LocalBeltDynamicsChecks
  "Independent force-driven checks for span-owned material dynamics"
  extends Modelica.Icons.ExamplesPackage;

  model ForceBoundary
    "Prescribed position and transport force with free tangent direction"
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
      "Connection to one span end";
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
    annotation(Documentation(info="<html><p>The span determines the two
    tangent angles. This fixture prescribes neither frame force nor moment;
    it therefore receives the span's complete support reaction.</p></html>"));
  end ForceBoundary;

  partial model Fixture
    "Integrate external mass, momentum and energy exchange independently"
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
      "Reference material and gravity";
    Roll2RollDynamics.Components.Belt belt(fixedInitialTension=false, dampingTime=0)
      "Span under test";
    ForceBoundary inlet(position=positionA, force=entryTension)
      "Prescribed entry force and position";
    ForceBoundary outlet(position=positionB, force=-exitTension)
      "Prescribed exit force and position";
  initial equation
    initialMass = belt.beltVariables.webMass;
    initialMomentum = belt.beltVariables.webMass*belt.physicalVelocity;
    initialEnergy = storedEnergy;
  equation
    connect(inlet.port, belt.frame_a);
    connect(belt.frame_b, outlet.port);
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
    annotation(Documentation(info="<html><p>The balances use prescribed end
    tensions and actual connector forces. Energy includes the explicitly
    measured central-velocity approximation; it is not a claim of passivity.
    Gravity is included as body-force work.</p></html>"));
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
      Documentation(info="<html><p>The 1 N excess exit tension accelerates
      only the free span's approximately 2.65 kg of material.</p></html>"));
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
      Documentation(info="<html><p>Equal tensions on a 45-degree incline leave
      material free to accelerate downhill. Supports carry only the normal
      gravity component.</p></html>"));
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
      Documentation(info="<html><p>Both endpoints accelerate sideways at
      2 m/s2. The full material mass supplies the reaction through the span
      connectors, together with its vertical weight.</p></html>"));
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
      Documentation(info="<html><p>A unit span rotates at 1 rad/s about its
      inlet. The midpoint boundary velocity produces a 0.5 m/s2 rate of
      change of the absolute projected material velocity.</p></html>"));
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
      Documentation(info="<html><p>Independent smooth boundary motion and
      tension changes produce varying material inventory and a nonzero kinetic
      energy defect. The integrated energy check includes that defect explicitly.
      </p></html>"));
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
  end ReverseTransport;
  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Force-driven isolated span regressions for
    material acceleration, support loads and open-system accounting.</p></html>"));
end LocalBeltDynamicsChecks;
