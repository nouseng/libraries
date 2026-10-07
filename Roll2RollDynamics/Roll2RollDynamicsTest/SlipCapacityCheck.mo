within Roll2RollDynamicsTest;
model SlipCapacityCheck
  "The component-level sweep must stay inside its dry capacity and cross zero with the command"
  extends Roll2RollDynamics.Examples.SlipAndTractionCapacity(world(enableAnimation = false));
  Modelica.Units.SI.Force expectedNipCapacity = nipTest.rollerVariables.nipLoad
    *nipTest.traction.muAdhesion
    "Dry capacity the static nip load adds";
equation
  assert(abs(nipTest.rollerVariables.nipLoad - nipLoad) < 1e-6,
    "The fixed nip must apply the requested static load");
  assert(wrapTraction*wrapSlip >= -1e-6 and totalTraction*totalSlip >= -1e-6,
    "Friction must dissipate power for either slip direction");
  assert(abs(nipMaster.rollerVariables.slipVelocity) < 0.5e-3,
    "The high-grip master must hold the web speed while the test roller slips");
  when time >= sweepStart + sweepDuration then
    assert(abs(wrapSlip - maximumSlip) < 0.5e-3 and abs(totalSlip - maximumSlip) < 0.5e-3,
      "At the end of the sweep the test rollers must slip by the commanded amount");
    assert(totalTraction > wrapTraction + 0.5*expectedNipCapacity,
      "The nip must raise the sliding traction");
    assert(totalTraction > 0 and totalTraction < totalCapacity,
      "Positive sliding must carry less traction than the adhesion peak");
  end when;
end SlipCapacityCheck;
