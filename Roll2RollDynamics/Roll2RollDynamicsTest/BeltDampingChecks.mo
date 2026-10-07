within Roll2RollDynamicsTest;
package BeltDampingChecks "Kelvin-Voigt span response and conservation checks"
  extends Modelica.Icons.ExamplesPackage;

  model RingDown "Internal damping removes a small axial velocity perturbation"
    //Parameters
    parameter Modelica.Units.SI.Time dampingTime=0.01 "Material damping time";
    //Variables
    Real lateTensionIntegral(unit="N2.s", start=0, fixed=true)
      "Squared tension difference integrated after the initial transient";
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0,
      webDampingTime=dampingTime)
      "Stationary test environment";
    Roll2RollDynamics.Components.Belt belt(fixedInitialMomentum=false)
      "Initially moving material between closed boundaries";
    MassConservationChecks.MovingBoundary inlet "Closed inlet";
    MassConservationChecks.MovingBoundary outlet(position={1,0,0})
      "Closed outlet";
  initial equation
    belt.momentum = 0.01*belt.beltVariables.webMass;
  equation
    connect(inlet.frame_b, belt.frame_a);
    connect(belt.frame_b, outlet.frame_b);
    der(lateTensionIntegral) = if time > 0.3
      then (belt.beltVariables.tensionOut - belt.beltVariables.tensionIn)^2 else 0;
    when terminal() then
      assert(dampingTime <= 0 or lateTensionIntegral < 1e-4,
        "Internal belt damping must settle the axial tension oscillation");
      assert(dampingTime > 0 or lateTensionIntegral > 0.1,
        "Disabling damping must preserve the purely elastic oscillation");
    end when;
    annotation(experiment(StopTime=0.5, Tolerance=1e-9),
      Documentation(info="<html><p>A small initial material velocity excites
      the two elastic cells. Fixed boundaries supply no work or mass.</p></html>"));
  end RingDown;

  model ElasticRingDown "Zero damping preserves the elastic baseline"
    extends RingDown(dampingTime=0);
    annotation(Documentation(info="<html><p>Runs the same velocity disturbance
      with the dashpots disabled to preserve the elastic reference.</p></html>"));
  end ElasticRingDown;

  model ElasticClosedLoop "Zero damping retains assembled elastic loop conservation"
    extends MassConservationChecks.ClosedLoop(
      upperSpan(dampingTime=0), lowerSpan(dampingTime=0));
    annotation(Documentation(info="<html><p>The complete loop exercises
      the zero-damping equations with roller wrap storage and transport.
      Its independently computed total material inventory must stay fixed.</p></html>"));
  end ElasticClosedLoop;

  model UniformExtension "Constant extension rate gives the analytic viscous force"
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0,
      tension=100, web(EA=10000), webDampingTime=0)
      "Elastic line default; this span overrides it with local damping";
    Roll2RollDynamics.Components.Belt belt(fixedInitialMomentum=false, dampingTime=0.01)
      "Closed material span under uniform extension";
    MassConservationChecks.MovingBoundary inlet "Closed inlet";
    MassConservationChecks.MovingBoundary outlet(position={1 + 0.001*time,0,0})
      "Outlet moving at one millimetre per second";
  initial equation
    belt.momentum = 0.0005*belt.beltVariables.webMass;
  equation
    connect(inlet.frame_b, belt.frame_a);
    connect(belt.frame_b, outlet.frame_b);
    assert(abs(belt.tension - (100 + 10.1*time + 0.101)) < 1e-5,
      "Uniform extension must add 0.101 N of viscous force to the elastic force");
    assert(abs(belt.beltVariables.webMass - 2.7/1.01) < 1e-8,
      "Damping must not alter the material inventory of a closed span");
    annotation(experiment(StopTime=1, Tolerance=1e-9),
      Documentation(info="<html><p>Reference length is 1/1.01 m, so strain
      rate is 0.00101/s and EA*tau times that rate is 0.101 N.</p></html>"));
  end UniformExtension;

  model Creep "Constant pull approaches the elastic extension exponentially"
    //Components
    inner Roll2RollDynamics.WebWorld world(enableAnimation=false, g=0,
      tension=100, web(EA=10000)) "Known elastic stiffness";
    Roll2RollDynamics.Components.Belt belt(dampingTime=0.01)
      "Kelvin-Voigt material under equal end pulls";
    LocalBeltDynamicsChecks.ForceBoundary inlet(force=300) "Entry pull";
    LocalBeltDynamicsChecks.ForceBoundary outlet(position={1,0,0}, force=-300)
      "Exit pull";
  equation
    connect(inlet.port, belt.frame_a);
    connect(belt.frame_b, outlet.port);
    assert(abs(belt.beltVariables.stretch - (1.03 - 0.02*exp(-time/0.01))) < 1e-7,
      "Kelvin-Voigt creep must approach 3 percent strain with the damping time constant");
    annotation(experiment(StopTime=0.1, Tolerance=1e-9),
      Documentation(info="<html><p>The prescribed 300 N pull acts on material
      initially carrying 100 N elastically. The dashpot initially carries
      200 N; its force decays as the material stretches.</p></html>"));
  end Creep;

  model DampedEnergy "Open moving span balances energy including viscous dissipation"
    extends LocalBeltDynamicsChecks.ChangingInventory(
      belt(dampingTime=0.01, fixedInitialTension=true));
    annotation(Documentation(info="<html><p>Checks mass, momentum and energy
      against connector work and independently evaluated dashpot work.</p></html>"));
  end DampedEnergy;

  model ReverseTransport "Uniform reverse travel must not create material damping"
    extends LocalBeltDynamicsChecks.ReverseTransport(
      belt(dampingTime=0.01, fixedInitialTension=true));
  equation
    assert(abs(belt.dampingPower) < 1e-10,
      "Rigid material transport must not dissipate power in the belt dashpots");
    annotation(Documentation(info="<html><p>Uniform reverse speed is transport,
      with no material extension and therefore no viscous belt loss.</p></html>"));
  end ReverseTransport;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Independent response checks for damping
    implemented inside the free-span Belt component.</p></html>"));
end BeltDampingChecks;
