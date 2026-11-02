within Roll2RollDynamicsTest;
model NoSlipBeltDriveCheck
  "Toothed-belt drive: world.noSlip must lock both pulley surfaces to the belt"
  extends Roll2RollDynamics.Examples.OpenBeltDrive(world(enableAnimation = false, noSlip = true));
equation
  assert(abs(bigRoll.rollerVariables.slipVelocity) < 1e-6
    and abs(smallRoll.rollerVariables.slipVelocity) < 1e-6,
    "No-slip pulleys must move with the belt");
  annotation(Documentation(info = "<html><h4>No-slip belt drive check</h4>
<p>Run the open belt drive with <code>world.noSlip = true</code>, the toothed-belt idealization, which the example starts in steady running. The assertion requires zero slip on both pulleys throughout.</p></html>"));
end NoSlipBeltDriveCheck;
