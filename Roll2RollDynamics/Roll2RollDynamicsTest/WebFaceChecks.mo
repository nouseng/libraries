within Roll2RollDynamicsTest;
package WebFaceChecks "Web-face selection on the supporting roller and its nip"
  extends Modelica.Icons.ExamplesPackage;

  model InnerFace "Supporting roller touches the inner face and the nip the outer face"
    extends Roll2RollDynamics.Examples.NipLoading(
      world(enableAnimation=false,
        innerTraction(muAdhesion=0.2, muSliding=0.15),
        outerTraction(muAdhesion=0.6, muSliding=0.4)));
    parameter Boolean supportsInnerFace=true "Expected material face on the supporting roller";
    final parameter Real nipMuPeak = if supportsInnerFace then 0.6 else 0.2
      "Expected peak coefficient on the opposite web face";
    final parameter Real nipMuSlide = if supportsInnerFace then 0.4 else 0.15
      "Expected sliding coefficient on the opposite web face";
  equation
    assert(nip.contactFace <> topRoll.contactFace,
      "Nip and supporting roller must contact opposite material faces");
    assert(abs(topRoll.traction.muAdhesion - (if supportsInnerFace then 0.2 else 0.6)) < 1e-12,
      "Supporting roller must use its selected face characteristic");
    assert(abs(nip.nipVariables.nipTraction - nip.nipVariables.nipLoad*
      Roll2RollDynamics.Functions.tripleS(0.001, 0.003, nipMuPeak,
        nipMuSlide, nip.nipVariables.nipSlip, 0)) < 1e-7,
      "Nip traction must follow the opposite face coefficient");
    assert(nip.nipContact.frictionLoss >= -1e-10,
      "Nip traction must dissipate energy for both travel directions");
    assert(abs(massBalanceError) < 1e-6,
      "Face selection must preserve the closed web inventory");
    annotation(experiment(StopTime=8, Tolerance=1e-8),
      Documentation(info = "<html><h4>Inner face</h4>
<p>Run the nip-loading loop with distinct inner and outer web friction coefficients, with the supporting roller on the inner face and the nip on the outer face. Assertions require the selected roller coefficient and opposite-face nip law, nonnegative nip friction loss and conserved web inventory, detecting swapped face selection.</p></html>"));
  end InnerFace;

  model OuterFace "Changing the contacting face exchanges the friction characteristics"
    extends InnerFace(supportsInnerFace=false,
      topRoll(contactFace=Roll2RollDynamics.Utilities.Types.WebFace.Outer));
    annotation(Documentation(info = "<html><h4>Outer face</h4>
<p>Move the supporting roller to the outer web face so the nip automatically contacts the inner face. The inherited checks must exchange the friction characteristics while preserving contact dissipation and closed-loop web mass.</p></html>"));
  end OuterFace;

  model ReverseInner "Travel reversal retains the same material faces"
    extends InnerFace(world(lineSpeed=-2));
    annotation(Documentation(info = "<html><h4>Reverse inner</h4>
<p>Reverse transport to -2 m/s with the supporting roller on the inner face. Velocity and traction signs should reverse without exchanging the material faces, and the inherited friction, dissipation and mass checks must still hold.</p></html>"));
  end ReverseInner;

  model ReverseOuter "Opposite-face threading also supports reverse transport"
    extends OuterFace(world(lineSpeed=-2));
    annotation(Documentation(info = "<html><h4>Reverse outer</h4>
<p>Reverse transport to -2 m/s with the supporting roller on the outer face and the nip on the inner face. The opposite-face friction selection must remain valid in reverse travel while the inherited dissipation and mass checks hold.</p></html>"));
  end ReverseOuter;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Material-face selection and contact
    dissipation checks using the existing nip-loading example.</p></html>"));
end WebFaceChecks;
