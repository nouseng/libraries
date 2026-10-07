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
      Documentation(info="<html><p>Distinct coefficients on the two faces
      detect a swapped face selection.</p></html>"));
  end InnerFace;

  model OuterFace "Changing the contacting face exchanges the friction characteristics"
    extends InnerFace(supportsInnerFace=false,
      topRoll(contactFace=Roll2RollDynamics.Utilities.Types.WebFace.Outer));
    annotation(Documentation(info="<html><p>The outer face on the supporting
      roller makes the inner face contact the nip automatically.</p></html>"));
  end OuterFace;

  model ReverseInner "Travel reversal retains the same material faces"
    extends InnerFace(world(lineSpeed=-2));
    annotation(Documentation(info="<html><p>Reverse transport changes
      velocity and traction signs without exchanging faces.</p></html>"));
  end ReverseInner;

  model ReverseOuter "Opposite-face threading also supports reverse transport"
    extends OuterFace(world(lineSpeed=-2));
    annotation(Documentation(info="<html><p>Checks both contact choices
      independently of travel direction.</p></html>"));
  end ReverseOuter;

  annotation(uses(Modelica(version="4.1.0")),
    Documentation(info="<html><p>Material-face selection and contact
    dissipation checks using the existing nip-loading example.</p></html>"));
end WebFaceChecks;
