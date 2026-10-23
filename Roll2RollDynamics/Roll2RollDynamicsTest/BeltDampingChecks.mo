within Roll2RollDynamicsTest;
package BeltDampingChecks "Kelvin-Voigt span response and conservation checks"
  extends Modelica.Icons.ExamplesPackage;

  model RingDown "Internal damping removes a small axial velocity perturbation"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Modelica.Units.SI.Time dampingTime=0.01 "Material damping time";
    //Variables
    Real lateTensionIntegral(unit="N2.s", start=0, fixed=true)
      "Squared tension difference integrated after the initial transient";
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0,
      webDampingTime=dampingTime)
      "Stationary test environment"
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt belt(fixedInitialMomentum=false)
      "Initially moving material between closed boundaries"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    MassConservationChecks.MovingBoundary inlet "Closed inlet"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    MassConservationChecks.MovingBoundary outlet(position={1,0,0})
      "Closed outlet"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    belt.momentum = 0.01*belt.beltVariables.webMass;
  equation
    connect(inlet.frame_b, belt.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(belt.frame_b, outlet.frame_b)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    der(lateTensionIntegral) = if time > 0.3
      then (belt.beltVariables.tensionOut - belt.beltVariables.tensionIn)^2 else 0;
    when terminal() then
      assert(dampingTime <= 0 or lateTensionIntegral < 1e-4,
        "Internal belt damping must settle the axial tension oscillation");
      assert(dampingTime > 0 or lateTensionIntegral > 0.1,
        "Disabling damping must preserve the purely elastic oscillation");
    end when;
    annotation(experiment(StopTime=0.5, Tolerance=1e-9),
      Documentation(info="<html><h4>Ring down</h4><p>A small initial axial velocity excites the two elastic cells between fixed, closed boundaries. Material damping must settle the late tension oscillation; with damping disabled, that oscillation must persist.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
  end RingDown;

  model ElasticRingDown "Zero damping preserves the elastic baseline"
    extends RingDown(dampingTime=0);
    annotation(Documentation(info="<html><h4>Elastic ring down</h4><p>Repeats the axial velocity disturbance with the dashpots disabled. The late tension oscillation must remain above the elastic-reference threshold.</p></html>"));
  end ElasticRingDown;

  model ElasticClosedLoop "Zero damping retains assembled elastic loop conservation"
    extends MassConservationChecks.ClosedLoop(
      upperSpan(dampingTime=0), lowerSpan(dampingTime=0));
    annotation(Documentation(info="<html><h4>Elastic closed loop</h4><p>Runs the assembled two-roll loop with both free-span dashpots disabled. Independently computed material inventory, including both roller wraps, must remain constant.</p></html>"));
  end ElasticClosedLoop;

  model UniformExtension "Constant extension rate gives the analytic viscous force"
    extends Modelica.Icons.Example;
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0,
      tension=100, web(EA=10000), webDampingTime=0)
      "Elastic line default; this span overrides it with local damping"
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt belt(fixedInitialMomentum=false, dampingTime=0.01)
      "Closed material span under uniform extension"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    MassConservationChecks.MovingBoundary inlet "Closed inlet"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    MassConservationChecks.MovingBoundary outlet(position={1 + 0.001*time,0,0})
      "Outlet moving at one millimetre per second"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  initial equation
    belt.momentum = 0.0005*belt.beltVariables.webMass;
  equation
    connect(inlet.frame_b, belt.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(belt.frame_b, outlet.frame_b)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(belt.tension - (100 + 10.1*time + 0.101)) < 1e-5,
      "Uniform extension must add 0.101 N of viscous force to the elastic force");
    assert(abs(belt.beltVariables.webMass - 2.7/1.01) < 1e-8,
      "Damping must not alter the material inventory of a closed span");
    annotation(experiment(StopTime=1, Tolerance=1e-9),
      Documentation(info="<html><h4>Uniform extension</h4><p>Extends a closed span at 1 mm/s and checks its analytic Kelvin-Voigt tension while material inventory stays fixed. The reference length is 1/1.01 m, so the strain rate of 0.00101/s adds 0.101 N of viscous force to the elastic response.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
  end UniformExtension;

  model Creep "Constant pull approaches the elastic extension exponentially"
    extends Modelica.Icons.Example;
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0,
      tension=100, web(EA=10000)) "Known elastic stiffness"
      annotation(Placement(visible = true, transformation(origin = {-160, 80},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Components.Belt belt(dampingTime=0.01)
      "Kelvin-Voigt material under equal end pulls"
      annotation(Placement(visible = true, transformation(origin = {0, 0},
        extent = {{-10, -10}, {10, 10}})));
    LocalBeltDynamicsChecks.ForceBoundary inlet(force=300) "Entry pull"
      annotation(Placement(visible = true, transformation(origin = {-80, 0},
        extent = {{-10, -10}, {10, 10}})));
    LocalBeltDynamicsChecks.ForceBoundary outlet(position={1,0,0}, force=-300)
      "Exit pull"
      annotation(Placement(visible = true, transformation(origin = {80, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    connect(inlet.port, belt.frame_a)
      annotation(Line(visible = true, points = {{-70, 0}, {-10, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    connect(belt.frame_b, outlet.port)
      annotation(Line(visible = true, points = {{10, 0}, {90, 0}},
        color = {0, 128, 180}, thickness = 0.5, smooth = Smooth.None));
    assert(abs(belt.beltVariables.stretch - (1.03 - 0.02*exp(-time/0.01))) < 1e-7,
      "Kelvin-Voigt creep must approach 3 percent strain with the damping time constant");
    annotation(experiment(StopTime=0.1, Tolerance=1e-9),
      Documentation(info="<html><h4>Creep</h4><p>Applies equal 300 N end pulls to material initially carrying 100 N elastically. Stretch must approach 1.03 exponentially with a 0.01 s time constant as the initial 200 N dashpot force decays.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-200, -100}, {120, 120}})));
  end Creep;

  model DampedEnergy "Open moving span balances energy including viscous dissipation"
    extends LocalBeltDynamicsChecks.ChangingInventory(
      belt(dampingTime=0.01, fixedInitialTension=true));
    annotation(Documentation(info="<html><h4>Damped energy</h4><p>Adds internal damping to the moving-boundary, changing-tension span case. Independent mass, momentum and energy balances must close when connector work, the central-velocity energy defect and nonnegative dashpot dissipation are included.</p></html>"));
  end DampedEnergy;

  model ReverseTransport "Uniform reverse travel must not create material damping"
    extends LocalBeltDynamicsChecks.ReverseTransport(
      belt(dampingTime=0.01, fixedInitialTension=true));
  equation
    assert(abs(belt.dampingPower) < 1e-10,
      "Rigid material transport must not dissipate power in the belt dashpots");
    annotation(Documentation(info="<html><h4>Reverse transport</h4><p>Carries material backward at constant speed with equal end tensions and internal damping enabled. The inherited signed-flow and conservation checks must pass, and rigid transport must produce zero dashpot power.</p></html>"));
  end ReverseTransport;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Independent response checks for damping
    implemented inside the free-span Belt component.</p></html>"));
end BeltDampingChecks;
