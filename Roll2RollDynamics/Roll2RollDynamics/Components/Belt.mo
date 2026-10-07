within Roll2RollDynamics.Components;
model Belt
  "Kelvin-Voigt web span connecting two rolls through multibody frames"
  extends Roll2RollDynamics.Utilities.Icons.Belt;
  //Parameters
  // Geometry
  parameter Roll2RollDynamics.Utilities.Types.WebParameters web = world.web
    "Web thickness, width and axial elasticity"
    annotation(Dialog(group = "Geometry"));

  // Dynamics
  parameter Modelica.Units.SI.Force tensionNominal = world.tension
    "Initial elastic span tension"
    annotation(Dialog(group = "Dynamics"));
  parameter Modelica.Units.SI.Time dampingTime(min=0) = world.webDampingTime
    "Kelvin-Voigt viscosity divided by elastic modulus; zero disables damping"
    annotation(Dialog(group = "Dynamics"));

  // Initialization
  parameter Boolean fixedInitialTension=true
    "Retain both elastic cell states and initialize them to tensionNominal"
    annotation(Dialog(tab="Initialization"));
  parameter Boolean fixedInitialMomentum=true "Initialize absolute projected momentum to zero"
    annotation(Dialog(tab="Initialization"));

  // Animation
  parameter Integer ribbonSegments(min = 1) = 8
    "Number of boxes drawn along the span, which let it twist between its ends"
    annotation(Dialog(tab = "Animation"));
  parameter Real webColor[3] = world.webColor "Ribbon color at the low stress limit"
    annotation(Dialog(tab = "Animation"));
  parameter Real nominalColor[3] = world.nominalColor
    "Ribbon color midway between the stress limits"
    annotation(Dialog(tab = "Animation"));
  parameter Real stressColor[3] = world.stressColor "Ribbon color at the high stress limit"
    annotation(Dialog(tab = "Animation"));
  parameter Modelica.Units.SI.Stress stressLow = world.stressLow
    "Web stress drawn in webColor"
    annotation(Dialog(tab = "Animation"));
  parameter Modelica.Units.SI.Stress stressHigh = world.stressHigh
    "Web stress drawn in stressColor"
    annotation(Dialog(tab = "Animation"));
  parameter Modelica.Units.SI.Stress stressNominal = world.stressNominal
    "Web stress drawn in nominalColor"
    annotation(Dialog(tab = "Animation"));
  parameter Real colorGamma(min = Modelica.Constants.small) = world.stressColorGamma
    "Contrast exponent of the stress color gradient"
    annotation(Dialog(tab = "Animation"));
  parameter Real markerColor[3] = {255, 160, 0} "Transport marker color"
    annotation(Dialog(tab = "Animation"));
  parameter Boolean markerAnimation = true "= true, animate the traveling marker"
    annotation(Dialog(tab = "Animation"));

  //Variables
  // Main results, grouped into one browser node
  record BeltVariables "Span results a line engineer plots"
    Modelica.Units.SI.Force tension "Mean span tension";
    Modelica.Units.SI.Velocity webVelocity "Physical web velocity projected along the span";
    Modelica.Units.SI.Force tensionIn "Total elastic and viscous entry tension";
    Modelica.Units.SI.Force tensionOut "Total elastic and viscous exit tension";
    Modelica.Units.SI.Stress stress "Axial web stress in the span";
    Real stretch(unit = "1") "Current length divided by unstretched length";
    Modelica.Units.SI.Length freeLength "Free length between the two tangency points";
    Modelica.Units.SI.Mass webMass "Material inventory in the span";
    Real spanSkewSin(unit = "1")
      "Sine of the angle the span makes across the machine";
  end BeltVariables;
  BeltVariables beltVariables "Span results displayed in the variable browser";

  Modelica.Units.SI.Velocity vBelt
    "Central material velocity relative to the moving span midpoint"
    annotation(HideResult=true);

  Modelica.Units.SI.Length sTraveled(start = 0, fixed = true) "Web transport distance"
    annotation(HideResult=true);
  //Inputs and Outputs
  // Diagnostics and states hidden from the result file
  output Modelica.Units.SI.Length referenceLength(start=1, fixed=false,
    stateSelect=StateSelect.default)
    "Unstretched material length in the span"
    annotation(HideResult=true);
  // Independently initialized cells must not be recovered from zero flow at rest.
  output Modelica.Units.SI.Force elasticTensionIn(start=tensionNominal,
    stateSelect=if fixedInitialTension then StateSelect.always else StateSelect.prefer)
    "Inlet-cell elastic tension state"
    annotation(HideResult=true);
  output Modelica.Units.SI.Force elasticTensionOut(start=tensionNominal,
    stateSelect=if fixedInitialTension then StateSelect.always else StateSelect.prefer)
    "Outlet-cell elastic tension state"
    annotation(HideResult=true);
  output Modelica.Units.SI.Force dampingTensionIn "Viscous entry tension contribution"
    annotation(HideResult=true);
  output Modelica.Units.SI.Force dampingTensionOut "Viscous exit tension contribution"
    annotation(HideResult=true);
  output Modelica.Units.SI.Power dampingPower "Power dissipated by the two dashpots"
    annotation(HideResult=true);
  output Modelica.Units.SI.Mass massInletCell "Material in the inlet half of the current geometry"
    annotation(HideResult=true);
  output Modelica.Units.SI.Mass massOutletCell "Material in the outlet half of the current geometry"
    annotation(HideResult=true);
  output Modelica.Units.SI.Momentum momentum(start=0,stateSelect=StateSelect.always)
    "Absolute material momentum projected along the span"
    annotation(HideResult=true);
  output Modelica.Units.SI.Velocity physicalVelocity[3] "Central material velocity in world coordinates"
    annotation(HideResult=true);
  output Modelica.Units.SI.Acceleration aBelt "Physical web acceleration projected along the span"
    annotation(HideResult=true);
  output Modelica.Units.SI.Power energyDefect "Signed kinetic-energy error from the central-velocity approximation"
    annotation(HideResult=true);
  output Modelica.Units.SI.MassFlowRate massFlowIn "Signed mass flow entering frame_a"
    annotation(HideResult=true);
  output Modelica.Units.SI.MassFlowRate massFlowOut "Signed mass flow leaving frame_b"
    annotation(HideResult=true);

  //Components
  outer Roll2RollDynamics.WebWorld world "Multibody world and shared line defaults";

protected
  //Parameters
  final parameter Real segmentBlend[ribbonSegments](each unit = "1") =
    {if ribbonSegments > 1 then (i - 1)/(ribbonSegments - 1) else 0.5
      for i in 1:ribbonSegments}
    "Endpoint blend fraction of each ribbon segment";
  final parameter Real referenceDensity(unit="kg/m")=web.density*web.thickness*web.width
    "Unstrained material mass per unit length";

  //Variables
  // Working names of the displayed results; beltVariables carries them outward.
  Modelica.Units.SI.Velocity webVelocity "Physical web velocity projected along the span";
  Modelica.Units.SI.Force tensionIn "Total elastic and viscous entry tension";
  Modelica.Units.SI.Force tensionOut "Total elastic and viscous exit tension";
  Modelica.Units.SI.Stress stress "Axial web stress in the span";
  Real stretch(unit = "1") "Current length divided by unstretched length";
  Modelica.Units.SI.Length freeLength "Free length between the two tangency points";
  Modelica.Units.SI.Mass webMass "Material inventory in the span";
  Real spanSkewSin(unit = "1")
    "Sine of the angle the span makes across the machine";
  Real e[3](each unit = "1") "Unit vector from frame_a to frame_b";
  Real directionRate[3](each unit="1/s") "Rate of the directed span unit vector";
  Real widthAxisA[3](each unit = "1")
    "Upstream local y-axis resolved in world coordinates";
  Real widthAxisB[3](each unit = "1")
    "Downstream local y-axis resolved in world coordinates";
  Real segmentWidth[ribbonSegments, 3](each unit = "1")
    "Blended endpoint width axis of each segment, projected normal to the span";
  Real segmentWidthDirection[ribbonSegments, 3](each unit = "1")
    "Normalized ribbon width direction of each segment in world coordinates";
  Integer markerSegment "Ribbon segment currently under the marker";
  Real markerWidthDirection[3](each unit = "1")
    "Marker width direction, same plate orientation as the segment it rides on";
  Modelica.Units.SI.Velocity vA "Upstream material velocity";
  Modelica.Units.SI.Velocity vB "Downstream material velocity";
  Real stretchA(unit="1") "Inlet-cell stretch";
  Real stretchB(unit="1") "Outlet-cell stretch";
  Real materialStrainRateA(unit="1/s") "Inlet-cell material strain rate";
  Real materialStrainRateB(unit="1/s") "Outlet-cell material strain rate";
  Modelica.Units.SI.Velocity boundaryVelocityA[3] "Entry tangency velocity in world coordinates";
  Modelica.Units.SI.Velocity boundaryVelocityB[3] "Exit tangency velocity in world coordinates";
  Modelica.Units.SI.Velocity centerVelocity[3] "Velocity of the geometric span midpoint";
  Modelica.Units.SI.Velocity materialVelocityA[3] "Physical material velocity at entry";
  Modelica.Units.SI.Velocity materialVelocityB[3] "Physical material velocity at exit";
  Modelica.Units.SI.Acceleration gravity[3] "Gravity evaluated at the span midpoint";
  Modelica.Units.SI.Force reactionResidual[3] "Support balance before transverse projection";
  Modelica.Units.SI.Force transverseReaction[3] "Nonlongitudinal support reaction shared by the two ends";
  Modelica.Units.SI.Force forceA[3] "Total spatial force entering the span at entry";
  Modelica.Units.SI.Force forceB[3] "Total spatial force entering the span at exit";


  Real markerProgress(unit = "1") "Marker progress along the span";
  Real ribbonColor[3] "Span color, cool when slack and hot when tight";

public
  //Physical connectors
  Roll2RollDynamics.Utilities.Interfaces.SpanPort frame_a
    "Upstream tangency geometry and web transport"
    annotation(Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}})));
  Roll2RollDynamics.Utilities.Interfaces.SpanPort frame_b
    "Downstream tangency geometry and web transport"
    annotation(Placement(transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealOutput tension(unit = "N") "Mean span tension"
    annotation(Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));

protected
  //Components
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape ribbon[ribbonSegments](
    each shapeType = "box",
    each length = freeLength/ribbonSegments,
    each r_shape = {0, 0, 0},
    each width = web.width,
    each height = web.thickness,
    each color = ribbonColor,
    each lengthDirection = e,
    widthDirection = {segmentWidthDirection[i, :] for i in 1:ribbonSegments},
    each R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
    r = {frame_a.frame.r_0 + e*(freeLength*(i - 1)/ribbonSegments)
      for i in 1:ribbonSegments}) if world.enableAnimation "Free-span ribbon";
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape marker(
    shapeType = "box",
    length = 0.01,
    r_shape = -0.005*e,
    width = web.width,
    height = 3*web.thickness,
    color = markerColor,
    lengthDirection = e,
    widthDirection = markerWidthDirection,
    R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
    r = frame_a.frame.r_0 + e*(markerProgress*freeLength))
    if markerAnimation and world.enableAnimation
    "Traveling transport marker";

initial equation
  if fixedInitialTension then
    elasticTensionIn = tensionNominal;
    elasticTensionOut = tensionNominal;
  end if;
  if fixedInitialMomentum then
    momentum = 0;
  end if;

equation
  // Displayed results, grouped for the variable browser
  beltVariables.tension = tension;
  beltVariables.webVelocity = webVelocity;
  beltVariables.tensionIn = tensionIn;
  beltVariables.tensionOut = tensionOut;
  beltVariables.stress = stress;
  beltVariables.stretch = stretch;
  beltVariables.freeLength = freeLength;
  beltVariables.webMass = webMass;
  beltVariables.spanSkewSin = spanSkewSin;

  // Span geometry
  freeLength = sqrt(
    (frame_b.frame.r_0 - frame_a.frame.r_0)
    * (frame_b.frame.r_0 - frame_a.frame.r_0));
  e = (frame_b.frame.r_0 - frame_a.frame.r_0)/freeLength;
  frame_a.spanDirection = e;
  frame_b.spanDirection = e;
  frame_a.stretch = stretchA;
  frame_b.stretch = stretchB;
  0 = frame_a.frame.R.T[3, :]*e;
  0 = frame_b.frame.R.T[3, :]*e;

  // Ribbon orientation
  widthAxisA = frame_a.frame.R.T[2, :];
  widthAxisB = frame_b.frame.R.T[2, :];
  for i in 1:ribbonSegments loop
    segmentWidth[i, :] =
      widthAxisA + (widthAxisB - widthAxisA)*segmentBlend[i]
      - e*(e*(widthAxisA + (widthAxisB - widthAxisA)*segmentBlend[i]));
    segmentWidthDirection[i, :] = segmentWidth[i, :]/max(
      sqrt(segmentWidth[i, :]*segmentWidth[i, :]),
      Roll2RollDynamics.Utilities.Types.lengthTol);
  end for;
  // The ribbon is flat plates. A marker blended smoothly between the end
  // axes tilts against the plate it sits on and buries its ends in it, so
  // it takes the plate's own orientation instead.
  markerSegment = min(ribbonSegments,
    1 + integer(markerProgress*ribbonSegments));
  markerWidthDirection = segmentWidthDirection[markerSegment, :];

  // Material inventory and elastic cells
  vA = der(frame_a.web.s);
  vB = der(frame_b.web.s);
  tension = (tensionIn + tensionOut)/2;
  stretchA = 1 + elasticTensionIn/web.EA;
  stretchB = 1 + elasticTensionOut/web.EA;
  stretch = freeLength/referenceLength;
  massInletCell = referenceDensity*freeLength/(2*stretchA);
  massOutletCell = referenceDensity*freeLength/(2*stretchB);
  webMass = massInletCell + massOutletCell;
  referenceLength = webMass/referenceDensity;
  massFlowIn = referenceDensity*vA/stretchA;
  massFlowOut = referenceDensity*vB/stretchB;
  // [1] J. Yan and X. Du, "Web Tension and Speed Control in Roll-to-Roll
  // Systems", IntechOpen, 2020, sec. 2.1, Eqs. (10), (12)-(17).
  // https://www.intechopen.com/chapters/68827 (accessed 2026-09-06).
  // Model-specific two-cell derivation from elasticity and mass conservation:
  // m_i = referenceDensity*freeLength/(2*stretch_i),
  // qMid = 2*referenceDensity*vBelt/(stretchA + stretchB).
  // Differentiate m_i using der(m_a) = massFlowIn - qMid and
  // der(m_b) = qMid - massFlowOut, with T_i = web.EA*(stretch_i - 1).
  // The moving-length term is der(freeLength) = e*(w_b - w_a), where
  // w_a/b are boundaryVelocityA/B. These are not the single-span Eq. (20).
  der(elasticTensionIn) = web.EA*stretchA/freeLength
    * (e*(boundaryVelocityB - boundaryVelocityA)
       - 2*vA + 4*stretchA*vBelt/(stretchA + stretchB));
  der(elasticTensionOut) = web.EA*stretchB/freeLength
    * (e*(boundaryVelocityB - boundaryVelocityA)
       + 2*vB - 4*stretchB*vBelt/(stretchA + stretchB));

  // Kelvin-Voigt material law: T = EA*(lambda - 1) + EA*tau*D(lambda)/Dt.
  // D(lambda)/Dt = lambda*du/dx, evaluated across each current half-span.
  // der(stretchA/B) also contains transport between cells and is not the
  // material strain rate. Constant rigid transport must produce no damping.
  materialStrainRateA = stretchA/freeLength
    * (e*(boundaryVelocityB - boundaryVelocityA) + 2*(vBelt - vA));
  materialStrainRateB = stretchB/freeLength
    * (e*(boundaryVelocityB - boundaryVelocityA) + 2*(vB - vBelt));
  if dampingTime > 0 then
    dampingTensionIn = web.EA*dampingTime*materialStrainRateA;
    dampingTensionOut = web.EA*dampingTime*materialStrainRateB;
  else
    dampingTensionIn = 0;
    dampingTensionOut = 0;
  end if;
  tensionIn = elasticTensionIn + dampingTensionIn;
  tensionOut = elasticTensionOut + dampingTensionOut;
  dampingPower = web.EA*dampingTime*freeLength/2
    * (materialStrainRateA^2/stretchA + materialStrainRateB^2/stretchB);

  // Boundary and material velocities
  // Kinematic definitions: the midpoint is (r_a + r_b)/2.
  boundaryVelocityA = der(frame_a.frame.r_0);
  boundaryVelocityB = der(frame_b.frame.r_0);
  centerVelocity = (boundaryVelocityA + boundaryVelocityB)/2;
  // Differentiate e = (r_b - r_a)/freeLength: directionRate = der(e).
  directionRate = (boundaryVelocityB - boundaryVelocityA
    - e*(e*(boundaryVelocityB - boundaryVelocityA)))/freeLength;
  // [2] A. A. Sonin, "Fundamental Laws of Motion for Particles, Material
  // Volumes, and Control Volumes", MIT, 2003, sec. 3, Eq. (25), p. 13.
  // https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/resources/mit2_25f13_fundam_law-son/
  // Accessed 2026-09-06. Material velocity = boundary + relative velocity.
  materialVelocityA = boundaryVelocityA + e*vA;
  materialVelocityB = boundaryVelocityB + e*vB;
  // Model definition: momentum = webMass*(e*physicalVelocity), with
  // physicalVelocity = centerVelocity + e*vBelt and e*e = 1.
  vBelt = momentum/webMass - e*centerVelocity;
  physicalVelocity = centerVelocity + e*vBelt;
  // Modelica.Mechanics.MultiBody.World.gravityAcceleration, evaluated at
  // the midpoint (this model's lumped gravity approximation).
  gravity = world.gravityAcceleration(
    (frame_a.frame.r_0 + frame_b.frame.r_0)/2);

  // Longitudinal momentum
  // [2], sec. 3.2, Eq. (27A), p. 14: moving-control-volume momentum balance.
  // Here P = webMass*physicalVelocity and momentum = e*P; differentiating
  // the projection adds der(e)*P = webMass*(directionRate*centerVelocity).
  der(momentum) = tensionOut - tensionIn
    + webMass*(e*gravity + directionRate*centerVelocity)
    + massFlowIn*(e*materialVelocityA) - massFlowOut*(e*materialVelocityB);

  // Spatial support loads
  // [2], Eq. (27A), solved for the transverse force with P as defined above.
  // Remove the axial projection after expanding der(P) and boundary fluxes.
  for i in 1:3 loop
    reactionResidual[i] = webMass
      * (der(centerVelocity[i]) + directionRate[i]*vBelt - gravity[i])
      + (massFlowIn - massFlowOut)*centerVelocity[i]
      - massFlowIn*boundaryVelocityA[i] + massFlowOut*boundaryVelocityB[i];
    transverseReaction[i] = reactionResidual[i] - e[i]*(e*reactionResidual);
    // Model-specific closure: share the transverse resultant equally;
    // momentum conservation alone does not determine this load split.
    forceA[i] = -tensionIn*e[i] + transverseReaction[i]/2;
    forceB[i] = tensionOut*e[i] + transverseReaction[i]/2;
  end for;

  // Reported motion and energy approximation
  // Product-rule identity for momentum = webMass*(e*physicalVelocity),
  // using der(webMass) = massFlowIn - massFlowOut ([2], Eq. (26A), p. 13).
  // aBelt is e*der(physicalVelocity), including the changing-axis correction.
  webVelocity = momentum/webMass;
  aBelt = (der(momentum) - (massFlowIn - massFlowOut)*webVelocity)/webMass
    - directionRate*centerVelocity;
  // Local velocity-lumping residual, derived from [2], Eqs. (26A), (27A):
  // for m = webMass, u = physicalVelocity, P = m*u and K = m*(u*u)/2,
  // der(K) = u*der(P) - (u*u)*der(m)/2. Subtract force power and net
  // incoming kinetic-energy flux to obtain the expression below.
  // This signed approximation error is not heat or a published closure law.
  energyDefect = (
    massFlowOut*((materialVelocityB - physicalVelocity)
      * (materialVelocityB - physicalVelocity))
    - massFlowIn*((materialVelocityA - physicalVelocity)
      * (materialVelocityA - physicalVelocity)))/2;

  // Validity checks
  assert(dampingTime >= 0, "Belt dampingTime must be nonnegative");
  assert(min(stretchA, stretchB) > 0, "Belt cell stretches must be positive");
  assert(min(tensionIn, tensionOut) > 0,
    "Belt requires taut entry and exit cells; slack and buckling are not modeled");
  assert(freeLength > Roll2RollDynamics.Utilities.Types.lengthTol,
    "Belt requires distinct tangency positions");
  assert(frame_a.frame.R.T[1, :]*e > 0 and frame_b.frame.R.T[1, :]*e > 0,
    "Belt port x axes must point from entry to exit; check tangency angle guesses");
  assert(web.EA > 0 and web.density > 0 and web.thickness > 0 and web.width > 0,
    "Conservative Belt requires positive stiffness, density and section");
  assert(abs(web.EA/world.web.EA - 1) < 1e-12
    and abs(web.density/world.web.density - 1) < 1e-12
    and abs(web.thickness/world.web.thickness - 1) < 1e-12
    and abs(web.width/world.web.width - 1) < 1e-12,
    "Conservative components must use the same world.web material");

  // Stress and transport animation
  spanSkewSin = e[2];
  stress = tension/(web.thickness*web.width);
  ribbonColor = Roll2RollDynamics.Functions.stressColor(
    stress, stressLow, stressHigh, stressNominal, colorGamma,
    webColor, nominalColor, stressColor);
  der(sTraveled) = vBelt;
  // FMU initialization can evaluate visual results before geometry is solved.
  markerProgress = if world.enableAnimation and markerAnimation then
    noEvent(mod(sTraveled, max(freeLength, Roll2RollDynamics.Utilities.Types.lengthTol)))
      /max(freeLength, Roll2RollDynamics.Utilities.Types.lengthTol) else 0;

  // Physical port forces
  // Each port's x axis is e, so transverse acceleration has no axial component.
  frame_a.frame.f[1] = -tensionIn;
  frame_b.frame.f[1] = tensionOut;
  for i in 2:3 loop
    frame_a.frame.f[i] = frame_a.frame.R.T[i, :]*reactionResidual/2;
    frame_b.frame.f[i] = frame_b.frame.R.T[i, :]*reactionResidual/2;
  end for;
  frame_a.frame.t = zeros(3);
  frame_b.frame.t = zeros(3);
  frame_a.web.f = -tensionIn;
  frame_b.web.f = tensionOut;

  annotation(
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Text(extent = {{-60, 20}, {60, -20}}, textString = "%name", textColor = {64, 64, 64})}),
    Documentation(info = "<html>
<h4>Belt</h4>
<p>This free span carries material between two tangencies using two
Kelvin&ndash;Voigt cells and one momentum state. Rollers carry only their
wrapped material. Elasticity and strain-dependent inventory follow [1];
moving-boundary mass and momentum balances follow [2].</p>
<p>Figure 1 defines the span: L is <code>freeLength</code>; both port x axes
follow e from <code>frame_a</code> to <code>frame_b</code>. T_a and T_b are
<code>tensionIn</code> and <code>tensionOut</code>; lambda_a and lambda_b are
cell stretches; m_a + m_b is <code>webMass</code>. Flows q_a and q_b are
<code>massFlowIn</code> and <code>massFlowOut</code>, negative in reverse.
Each tangency carries tension, half the transverse support reaction and
half the span weight.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Belt/belt-schematic.svg\"
     width=\"900\"
     alt=\"Belt orientation, elastic cells, forces, mass flow and velocities\">
<strong>Figure 1:</strong> Span orientation, elastic cells and material transport.</p>
<p><code>web</code> = Material properties from <code>world.web</code>, shared
throughout the line<br>
<code>tensionNominal</code> = Initial elastic cell tension, from
<code>world.tension</code><br>
<code>dampingTime</code> = From <code>world.webDampingTime</code>, default
0.01 s; zero gives elastic cells</p>
<p>Figure 2 gives the Kelvin&ndash;Voigt law [3], with EA =
<code>web.EA</code> and tau = <code>dampingTime</code>. Physical velocity
differences across each half-span determine strain rate; uniform transport
adds no damping. The default is illustrative, not calibrated.
<code>elasticTensionIn</code>/<code>elasticTensionOut</code> are elastic tension states;
<code>dampingTensionIn</code>/<code>dampingTensionOut</code> add viscous load;
<code>dampingPower</code> is dissipation. Wraps have no viscous law.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Belt/kelvin-voigt.svg\"
     width=\"900\"
     alt=\"Kelvin-Voigt cell tension, material strain rate and dissipation\">
<strong>Figure 2:</strong> Kelvin&ndash;Voigt cell tension and dissipation.</p>
<p>Figure 3 compares 0.01 s and zero damping in a 1 m span of default material
at 150 N, initially moving at 0.01 m/s between fixed, flow-free endpoints
without gravity. Damping removes the tension and velocity oscillations;
elastic cells sustain them.</p>
<p style=\"text-align: center;\">
<img style=\"display: block; margin-left: auto; margin-right: auto;\"
     src=\"modelica://Roll2RollDynamics/Resources/Images/Belt/damping-comparison.png\"
     width=\"600\"
     alt=\"End-tension difference and material velocity decaying at 0.01 s damping and oscillating at zero damping\">
<strong>Figure 3:</strong> Span response with and without material damping.</p>
<h4>Connections and initialization</h4>
<p>Connect both <a href=\"modelica://Roll2RollDynamics.Utilities.Interfaces.SpanPort\">SpanPort</a>
connectors: <code>frame_a</code> to the upstream roll exit and
<code>frame_b</code> to the downstream entry. Tangencies determine length;
the icon arrow marks positive travel, with reverse flow supported.</p>
<p><code>fixedInitialTension</code> and <code>fixedInitialMomentum</code>
default to true, fixing both elastic tensions to <code>tensionNominal</code>
and momentum to zero. Disable <code>fixedInitialTension</code> when
<code>dampingTime = 0</code> and boundaries prescribe tension; disable
<code>fixedInitialMomentum</code> when they prescribe initial momentum.
With positive damping, force boundaries
still require initial elastic strain: retain
<code>fixedInitialTension</code> or supply explicit elastic-state starts.</p>
<p>The model-level <code>tension</code> output feeds controllers; the ribbon
shows web motion and stress.</p>
<h4>Displayed variables</h4>
<p>Main simulation results are grouped in <code>beltVariables</code>:</p>
<ul>
<li><code>tension</code> [N]: Mean of the entry and exit tensions.</li>
<li><code>tensionIn</code>, <code>tensionOut</code> [N]: Total elastic and viscous end tensions.</li>
<li><code>webVelocity</code> [m/s]: Physical web velocity along the span.</li>
<li><code>stress</code> [Pa]: Mean tension over the reference cross-section.</li>
<li><code>stretch</code> [1]: Span length over unstretched material length.</li>
<li><code>freeLength</code> [m]: Distance between the two tangency points.</li>
<li><code>webMass</code> [kg]: Material inventory in the span.</li>
<li><code>spanSkewSin</code> [1]: Sine of the span's cross-machine angle.</li>
</ul>
<p>Cell-state and balance diagnostics stay public with
<code>HideResult = true</code>; implementation variables are protected.</p>
<h4>Limitations</h4>
<p>Mass and linear momentum are conserved, but energy conservation is not
guaranteed: signed <code>energyDefect</code> is approximation error, not heat.
Sag, slack, bending and distributed dynamics are omitted.</p>
<h4>References</h4>
<ul>
<li>[<a href=\"https://www.intechopen.com/chapters/68827\">1</a>]
J. Yan and X. Du, &quot;Web Tension and Speed Control in Roll-to-Roll
Systems&quot;, Control Theory in Engineering, IntechOpen, 2020, sec. 2.1,
eqs. (10), (12)&ndash;(17). DOI: 10.5772/intechopen.88797.
Open-access background on elasticity and strain-dependent material inventory.
This component adds its own two-cell discretization and moving-boundary
momentum balance. Accessed 2026-09-06.</li>
<li>[<a href=\"https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/resources/mit2_25f13_fundam_law-son/\">2</a>]
A. A. Sonin, &quot;Fundamental Laws of Motion for Particles, Material Volumes,
and Control Volumes&quot;, MIT, 2003, secs. 3.1&ndash;3.2,
eqs. (25), (26A), (27A), pp. 13&ndash;14.
Moving-boundary velocity, mass and momentum balances; the two-cell model,
support-load split and energy residual are derived here. Accessed 2026-09-06.</li>
<li>[<a href=\"https://doc.comsol.com/6.3/doc/com.comsol.help.sme/sme_ug_theory.06.029.html\">3</a>]
COMSOL, &quot;Linear Viscoelasticity&quot;, Structural Mechanics Module User's
Guide, version 6.3, &quot;The Kelvin&ndash;Voigt Model&quot;, eq. (3-36).
Spring and dashpot law, viscosity-to-modulus time scale and dissipation;
the moving two-cell discretization is specific to this library.
Accessed 2026-09-06.</li>
</ul>
</html>"));
end Belt;
