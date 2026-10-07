within Roll2RollDynamicsTest;
package VisualGeometryChecks "Animation geometry and component composition regressions"
  extends Modelica.Icons.ExamplesPackage;

  model VisualSpanBoundary
    "Fixed material endpoint with free tangent alignment and prescribed width reference"
    //Parameters
    parameter Modelica.Units.SI.Position position[3]=zeros(3) "Endpoint position";
    parameter Real directionGuess[3]={1,0,0} "Nominal span direction";
    parameter Real widthReference[3]={0,1,0} "Width reference before projection";
    //Variables
    Modelica.Units.SI.Angle yaw(start=0) "Free tangent yaw";
    Modelica.Units.SI.Angle skew(start=0) "Free tangent skew";
    Real localDirection[3](each unit="1") "Span direction in the prescribed reference basis";
    //Physical connectors
    Roll2RollDynamics.Utilities.Interfaces.RollPort port "Closed span endpoint";
  equation
    Connections.root(port.frame.R);
    port.frame.r_0=position;
    localDirection=Modelica.Mechanics.MultiBody.Frames.resolve2(
      Modelica.Mechanics.MultiBody.Frames.from_nxy(directionGuess,widthReference),
      port.spanDirection);
    skew=Modelica.Math.atan2(localDirection[2],
      cos(yaw)*localDirection[1]-sin(yaw)*localDirection[3]);
    port.frame.R=Modelica.Mechanics.MultiBody.Frames.absoluteRotation(
      Modelica.Mechanics.MultiBody.Frames.from_nxy(directionGuess,widthReference),
      Modelica.Mechanics.MultiBody.Frames.axesRotations(
        {2,3,1},{yaw,skew,0},{der(yaw),der(skew),0}));
    port.web.s=0;
  end VisualSpanBoundary;

  model VisualBeltProbe
    "Belt exposing numeric visual geometry for regression checks"
    extends Roll2RollDynamics.Components.Belt;
    output Real visualLengthDirection[3](each unit = "1")
      "Ribbon length direction resolved in world coordinates";
    output Real firstWidthDirection[3](each unit = "1")
      "Width direction of the segment leaving frame_a";
    output Real lastWidthDirection[3](each unit = "1")
      "Width direction of the segment arriving at frame_b";
    output Modelica.Units.SI.Position visualMidpoint[3]
      "Ribbon geometric midpoint in world coordinates";
    output Modelica.Units.SI.Position visualStart[3]
      "Centreline start of the first ribbon segment";
    output Modelica.Units.SI.Position visualEnd[3]
      "Centreline end of the last ribbon segment";
  equation
    visualLengthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(
      ribbon[1].R, ribbon[1].lengthDirection);
    firstWidthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(
      ribbon[1].R, ribbon[1].widthDirection);
    lastWidthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(
      ribbon[ribbonSegments].R, ribbon[ribbonSegments].widthDirection);
    visualStart = ribbon[1].r + Modelica.Mechanics.MultiBody.Frames.resolve1(
      ribbon[1].R, ribbon[1].r_shape);
    visualEnd = ribbon[ribbonSegments].r
      + Modelica.Mechanics.MultiBody.Frames.resolve1(
        ribbon[ribbonSegments].R,
        ribbon[ribbonSegments].r_shape
          + ribbon[ribbonSegments].lengthDirection*ribbon[ribbonSegments].length);
    visualMidpoint = 0.5*(visualStart + visualEnd);
  end VisualBeltProbe;

  model BeltVisualGeometryCheck
    "Centred straight-span visual follows the full endpoint geometry"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared line defaults";
    VisualSpanBoundary anchorA(directionGuess=expectedDirection,
      widthReference=widthAxisA) "Upstream fixed material boundary";
    VisualSpanBoundary anchorB(position={1,0.1,0.5},
      directionGuess=expectedDirection, widthReference=widthAxisB)
      "Downstream fixed material boundary";
    VisualBeltProbe span "Span under test";
    final parameter Real expectedDirection[3](each unit = "1") =
      {1, 0.1, 0.5}/sqrt(1^2 + 0.1^2 + 0.5^2)
      "Expected three-dimensional span direction";
    final parameter Real widthAxisA[3](each unit = "1") = {
      0,
      Modelica.Math.cos(15*Modelica.Constants.pi/180),
      Modelica.Math.sin(15*Modelica.Constants.pi/180)}
      "Known upstream local y-axis in world coordinates";
    final parameter Real widthAxisB[3](each unit = "1") = {
      0,
      Modelica.Math.cos(45*Modelica.Constants.pi/180),
      Modelica.Math.sin(45*Modelica.Constants.pi/180)}
      "Known downstream local y-axis in world coordinates";
    final parameter Real expectedWidthA[3](each unit = "1") =
      Modelica.Math.Vectors.normalize(
        widthAxisA - expectedDirection*(expectedDirection*widthAxisA))
      "Upstream width axis projected normal to the span";
    final parameter Real expectedWidthB[3](each unit = "1") =
      Modelica.Math.Vectors.normalize(
        widthAxisB - expectedDirection*(expectedDirection*widthAxisB))
      "Downstream width axis projected normal to the span";
    final parameter Modelica.Units.SI.Angle spanTwist =
      Modelica.Math.acos(expectedWidthA*expectedWidthB)
      "Total twist the ribbon has to cover between its ends";
  equation
    connect(anchorA.port, span.frame_a);
    connect(anchorB.port, span.frame_b);
    when terminal() then
      assert(Modelica.Math.Vectors.length(
        span.visualLengthDirection - expectedDirection) < 1e-9,
        "Ribbon must follow the full three-dimensional span direction");
      assert(Modelica.Math.Vectors.length(
        span.visualMidpoint - {0.5, 0.05, 0.25}) < 1e-9,
        "Ribbon must be centred on the span centreline");
      assert(Modelica.Math.Vectors.length(span.visualStart) < 1e-9
          and Modelica.Math.Vectors.length(span.visualEnd - {1, 0.1, 0.5}) < 1e-9,
        "Ribbon segments must cover the span from end to end");
      // A single averaged box leaves half the twist unmet at each end, which is
      // what opens a wedge where a span meets a yawed roll. Each end carries its
      // own frame's width axis exactly, not a half-segment short of it.
      assert(Modelica.Math.Vectors.length(
        span.firstWidthDirection - expectedWidthA) < 1e-9,
        "Ribbon must leave frame_a on the upstream width direction");
      assert(Modelica.Math.Vectors.length(
        span.lastWidthDirection - expectedWidthB) < 1e-9,
        "Ribbon must reach frame_b on the downstream width direction");
      assert(abs(Modelica.Math.Vectors.length(span.firstWidthDirection) - 1) < 1e-9
          and abs(Modelica.Math.Vectors.length(span.lastWidthDirection) - 1) < 1e-9,
        "Ribbon width directions must be normalized");
      assert(abs(span.visualLengthDirection*span.firstWidthDirection) < 1e-9
          and abs(span.visualLengthDirection*span.lastWidthDirection) < 1e-9,
        "Ribbon width directions must be perpendicular to its length");
      assert(spanTwist > 0.1,
        "Ribbon twist check must be run on a span that actually twists");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end BeltVisualGeometryCheck;

  model VisualWinderProbe
    "Winder exposing numeric visual geometry for regression checks"
    extends Roll2RollDynamics.Components.Winder;
    output Modelica.Units.SI.Position woundWebCenter[3]
      "Wound-web visual centre in world coordinates";
    output Modelica.Units.SI.Position hubCenter[3]
      "Hub visual centre in world coordinates";
    output Modelica.Units.SI.Length woundWebWidth = winder.length
      "Width drawn for wound web";
  equation
    woundWebCenter = winder.r + Modelica.Mechanics.MultiBody.Frames.resolve1(
      winder.R, winder.r_shape + winder.lengthDirection*winder.length/2);
    hubCenter = hub.r + Modelica.Mechanics.MultiBody.Frames.resolve1(
      hub.R, hub.r_shape + hub.lengthDirection*hub.length/2);
  end VisualWinderProbe;

  model WinderVisualGeometryCheck
    "Centred winder visuals distinguish web width from roll-face width"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(web(width = 0.6)) "Shared line defaults";
    Modelica.Mechanics.Rotational.Components.Fixed defaultDrive
      "Fixed drive for the default-width winder";
    Modelica.Mechanics.Rotational.Components.Fixed overrideDrive
      "Fixed drive for the overridden-width winder";
    VisualWinderProbe defaultWinder(useSupport = true, fixedInitialWebState=true,
      tangencyAngleStart=Modelica.Constants.pi)
      "Winder using the line-level width";
    VisualWinderProbe overrideWinder(useSupport = true, width = 0.72, fixedInitialWebState=true,
      tangencyAngleStart=Modelica.Constants.pi)
      "Winder using a component-level width override";
    Roll2RollDynamics.Components.WebForce boundaryA(direction=1,force={150,0,0});
    Roll2RollDynamics.Components.WebForce boundaryB(direction=1,force={150,0,0});
    final parameter Modelica.Units.SI.Mass expectedMass =
      world.web.density*Modelica.Constants.pi*
      (defaultWinder.radius0^2 - defaultWinder.coreRadius^2)*world.web.width
      "Expected initial wound-web mass";
    final parameter Modelica.Units.SI.Inertia expectedInertia =
      0.5*expectedMass*(defaultWinder.radius0^2 + defaultWinder.coreRadius^2)
      "Expected initial wound-web inertia";
  equation
    connect(boundaryA.frame_b,defaultWinder.frame_t);
    connect(boundaryB.frame_b,overrideWinder.frame_t);
    connect(world.frame_b, defaultWinder.frame_support);
    connect(world.frame_b, overrideWinder.frame_support);
    connect(defaultDrive.flange, defaultWinder.flange_a);
    connect(overrideDrive.flange, overrideWinder.flange_a);
    when terminal() then
      assert(abs(defaultWinder.width - 0.66) < 1e-12,
        "Default roll face must be 10% wider than the web");
      assert(abs(overrideWinder.width - 0.72) < 1e-12,
        "A component width override must take precedence");
      assert(abs(defaultWinder.woundWebWidth - 0.6) < 1e-12,
        "Wound-web visual must use the web width");
      assert(Modelica.Math.Vectors.length(defaultWinder.woundWebCenter) < 1e-9,
        "Wound web must be centred on the winder axis");
      assert(Modelica.Math.Vectors.length(defaultWinder.hubCenter) < 1e-9,
        "Winder hub must be centred on its axis");
      assert(abs(defaultWinder.frame_t.frame.r_0[2]) < 1e-9,
        "Winder tangency must constrain the web centreline");
      assert(abs(defaultWinder.winderVariables.webMass - expectedMass) < 1e-9,
        "Wound mass must use web width, not face width");
      assert(abs(defaultWinder.winderVariables.J - expectedInertia) < 1e-9,
        "Wound inertia must use web width, not face width");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end WinderVisualGeometryCheck;

  model VisualRollBodyProbe
    "Roll body exposing protected visualizer geometry for regression checks"
    extends Roll2RollDynamics.Utilities.Parts.RollBody;
    output Modelica.Units.SI.Position rollCenter[3] = revolute.frame_b.r_0
      "Mechanical roll centre in world coordinates";
    output Modelica.Units.SI.Length drumShapeRadius = drumSurface.width/2
      "Radius the drum surface is actually drawn at";
    output Modelica.Units.SI.Length faceMarkReach = faceMark[1].length
      "Radial reach of the face rotation mark";
    output Modelica.Units.SI.Position drumCenter[3]
      "Drum visual centre in world coordinates";
  equation
    drumCenter = drumSurface.r
      + Modelica.Mechanics.MultiBody.Frames.resolve1(
        drumSurface.R,
        drumSurface.r_shape
          + drumSurface.lengthDirection*drumSurface.length/2);
  end VisualRollBodyProbe;

  model VisualWebWrapProbe
    "Web wrap exposing numeric visual geometry for regression checks"
    extends Roll2RollDynamics.Utilities.Parts.WebWrap;
    output Modelica.Units.SI.Position visualEntryPoint[3]
      "Centreline endpoint of the first wrap box";
    output Modelica.Units.SI.Position visualExitPoint[3]
      "Centreline endpoint of the last wrap box";
    output Modelica.Units.SI.Position entryTangencyPoint[3] =
      tangentA.frame_b.r_0
      "Solved entry tangency point";
    output Modelica.Units.SI.Position exitTangencyPoint[3] =
      tangentB.frame_b.r_0
      "Solved exit tangency point";
    output Modelica.Units.SI.Length wrappedWebWidth = wrapArc[1].width
      "Width drawn for wrapped web";
    output Modelica.Units.SI.Position wrapCenter[3] = tangentA.frame_a.r_0
      "Centre the wrap arc is drawn about";
    output Modelica.Units.SI.Length beltRadius = centrelineRadius "Belt centreline radius";
    output Modelica.Units.SI.Length segmentChord = chord
      "Chord of one drawn wrap segment";
    output Modelica.Units.SI.Angle wrapOverhangAngle = overhangAngle
      "Arc drawn past each tangency";
    output Modelica.Units.SI.Angle chordAngle = wrapAngle/drawingSegments
      "Angle one wrap chord covers, the overhang a straight roll would get";
    output Modelica.Units.SI.Length overhangDepth =
      centrelineRadius*(1 - Modelica.Math.cos(overhangAngle))
      "How far the overhang sinks below the departing span";
    output Modelica.Units.SI.Length wrappedWebInnerRadius =
      arcRadius - beltThickness/2
      "Smallest radius the wrapped-web boxes reach";
  protected
    Real firstAxis[3](each unit = "1")
      "First wrap-box length axis in world coordinates";
    Real lastAxis[3](each unit = "1")
      "Last wrap-box length axis in world coordinates";
    Modelica.Units.SI.Position firstCenter[3]
      "First wrap-box centre";
    Modelica.Units.SI.Position lastCenter[3]
      "Last wrap-box centre";
  equation
    firstAxis = Modelica.Mechanics.MultiBody.Frames.resolve1(
      wrapArc[1].R, wrapArc[1].lengthDirection);
    lastAxis = Modelica.Mechanics.MultiBody.Frames.resolve1(
      wrapArc[drawingSegments].R, wrapArc[drawingSegments].lengthDirection);
    firstCenter = wrapArc[1].r
      + Modelica.Mechanics.MultiBody.Frames.resolve1(
        wrapArc[1].R,
        wrapArc[1].r_shape
          + wrapArc[1].lengthDirection*wrapArc[1].length/2);
    lastCenter = wrapArc[drawingSegments].r
      + Modelica.Mechanics.MultiBody.Frames.resolve1(
        wrapArc[drawingSegments].R,
        wrapArc[drawingSegments].r_shape
          + wrapArc[drawingSegments].lengthDirection*
            wrapArc[drawingSegments].length/2);
    visualEntryPoint = firstCenter - firstAxis*chord/2;
    visualExitPoint = lastCenter + lastAxis*chord/2;
  end VisualWebWrapProbe;

  model RollerWrapVisualGeometryCheck
    "Yawed wrap visual meets both solved tangencies"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(
      web(width = 0.6),
      lateralDynamics = true)
      "Shared line defaults with lateral contact motion";
    Modelica.Blocks.Sources.Constant entryLoad[3](
      k = {-100*cos(0.6), 40, -100*sin(0.6)})
      "Entry load selecting a known tangent";
    Modelica.Blocks.Sources.Constant exitLoad[3](
      k = {100*cos(0.6), 40, -100*sin(0.6)})
      "Exit load selecting a known tangent";
    Modelica.Mechanics.MultiBody.Forces.WorldForce entryForce(animation = false)
      "Entry span force stand-in";
    Modelica.Mechanics.MultiBody.Forces.WorldForce exitForce(animation = false)
      "Exit span force stand-in";
    Modelica.Mechanics.Rotational.Components.Fixed fixedDrive(phi0 = 0.37)
      "Fixed roller drive";
    VisualRollBodyProbe body(
      radius = world.rollRadius,
      width = world.rollWidth,
      beltThickness = world.web.thickness,
      yaw = 0.2,
      useSupport = true,
      useFlange = true,
      bearingDamping = world.bearingDamping,
      normalForce = wrap.normalForce,
      drumDrawnRadius = wrap.drumDrawnRadius)
      "Yawed roll body under test";
    VisualWebWrapProbe wrap(
      spanDirectionA=-entryLoad.k/Modelica.Math.Vectors.length(entryLoad.k),
      spanDirectionB=exitLoad.k/Modelica.Math.Vectors.length(exitLoad.k),
      entryTension=Modelica.Math.Vectors.length(entryLoad.k),
      exitTension=Modelica.Math.Vectors.length(exitLoad.k),
      radius = world.rollRadius,
      webWidth = world.web.width,
      beltThickness = world.web.thickness,
      yaw = 0.2,
      rotationDirection = 1,
      lateral = true,
      stressLow = world.stressLow,
      stressNominal = world.stressNominal,
      stressHigh = world.stressHigh,
      colorGamma = world.stressColorGamma)
      "Yawed web wrap under test";
    Roll2RollDynamics.Utilities.Parts.WebFriction friction(
      fixedInitialWebState=true,
      meanTension = wrap.meanTension,
      wrapAngle = abs(wrap.arcAngle),
      radius = world.rollRadius,
      rotationDirection = 1,
      traction = world.innerTraction,
      lateral = true,
      spanDirectionA = wrap.spanDirectionA,
      spanDirectionB = wrap.spanDirectionB)
      "Contact closure for the web transport coordinates";
  equation
    connect(world.frame_b, body.frame_support)
      annotation(Line(points = {{-80, 80}, {-40, 80}, {-40, 60}, {0, 60}}, color = {95, 95, 95}, thickness = 0.5));
    connect(fixedDrive.flange, body.flange_a)
      annotation(Line(points = {{-80, -80}, {-40, -80}, {-40, -60}, {0, -60}}, color = {0, 0, 0}));
    connect(body.frame_center, wrap.frame_center)
      annotation(Line(points = {{20, 40}, {40, 40}, {40, 20}, {60, 20}}, color = {95, 95, 95}, thickness = 0.5));
    connect(entryLoad.y, entryForce.force)
      annotation(Line(points = {{-80, 30}, {-60, 30}, {-60, 10}, {-40, 10}}, color = {0, 0, 127}));
    connect(exitLoad.y, exitForce.force)
      annotation(Line(points = {{-80, -30}, {-60, -30}, {-60, -10}, {-40, -10}}, color = {0, 0, 127}));
    connect(entryForce.frame_b, wrap.frame_a)
      annotation(Line(points = {{-20, 10}, {20, 10}, {20, 30}, {60, 30}}, color = {95, 95, 95}, thickness = 0.5));
    connect(exitForce.frame_b, wrap.frame_b)
      annotation(Line(points = {{-20, -10}, {20, -10}, {20, 0}, {60, 0}}, color = {95, 95, 95}, thickness = 0.5));
    0 = wrap.frame_a.R.T[3,:]*entryLoad.y;
    0 = wrap.frame_b.R.T[3,:]*exitLoad.y;
    connect(body.frame_drum, friction.frame_drum)
      annotation(Line(points = {{20, -30}, {40, -30}, {40, -70}, {70, -70}}, color = {95, 95, 95}, thickness = 0.5));
    connect(wrap.lateralFlange, friction.lateralFlange)
      annotation(Line(points = {{70, 40}, {100, 40}, {100, -40}, {70, -40}}, color = {0, 127, 0}));
    when terminal() then
      // A band stopping exactly on the tangency leaves a notch there, so it runs
      // past both by the overhang the yaw and the chord angle call for.
      assert(abs(Modelica.Math.Vectors.length(
        wrap.visualEntryPoint - wrap.entryTangencyPoint)
        - 2*wrap.beltRadius*Modelica.Math.sin(wrap.wrapOverhangAngle/2)) < 1e-9,
        "Wrapped-web visual must run past entry by the solved overhang");
      assert(abs(Modelica.Math.Vectors.length(
        wrap.visualExitPoint - wrap.exitTangencyPoint)
        - 2*wrap.beltRadius*Modelica.Math.sin(wrap.wrapOverhangAngle/2)) < 1e-9,
        "Wrapped-web visual must run past exit by the solved overhang");
      assert(abs(Modelica.Math.Vectors.length(
        wrap.visualEntryPoint - wrap.wrapCenter) - wrap.beltRadius) < 1e-9
          and abs(Modelica.Math.Vectors.length(
        wrap.visualExitPoint - wrap.wrapCenter) - wrap.beltRadius) < 1e-9,
        "Wrapped-web visual must stay on the belt circle where it overhangs");
      assert(Modelica.Math.Vectors.length(
        body.drumCenter - body.rollCenter) < 1e-9,
        "Roll face must be centred on its mechanical axis");
      assert(abs(wrap.wrappedWebWidth - world.web.width) < 1e-12,
        "Wrapped-web visual must use web width");
      // Two surfaces drawn at the same radius fight in the depth buffer, which
      // is what stripes the wrap and drops parts of it behind the drum.
      assert(wrap.wrappedWebInnerRadius > body.drumShapeRadius,
        "Wrapped web must be drawn clear of the drum surface it lies on");
      // A yawed roll meets its spans out of the arc's plane, so it has to reach
      // further than the chord angle a straight roll gets -- but never so far
      // that the band drops out from under the span and hangs below it.
      assert(wrap.wrapOverhangAngle > wrap.chordAngle,
        "Yaw must buy the wrapped web more reach than a straight roll gets");
      assert(wrap.wrapOverhangAngle
          <= wrap.chordAngle + abs(wrap.yaw) + 1e-12,
        "Overhang must not exceed the chord angle plus the yaw");
      assert(wrap.overhangDepth <= world.web.thickness,
        "Overhang must stay within the departing span's own thickness");
      assert(body.faceMarkReach < body.radius,
        "Rotation mark must lie on the roll face, not reach past the rim");
      assert(abs(friction.lateralFlange.s) > 1e-9,
        "Lateral roller regression must move the contact line");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end RollerWrapVisualGeometryCheck;

  model VisualRollerFacadeProbe
    "Roller exposing how far each republished value sits from its owning part"
    extends Roll2RollDynamics.Components.Roller;
    output Real wrapDeviation
      "Largest gap between a republished wrap value and webWrap";
    output Real bodyDeviation
      "Largest gap between a republished body value and rollBody";
    output Real contactDeviation
      "Largest gap between a republished contact value and webFriction";
    output Real compositionDeviation
      "Largest gap between a composed facade value and the parts it composes";
    output Modelica.Units.SI.Velocity contactDrift = webFriction.lateralDrift
      "Roll-relative contact-line speed for zero axial slip";
    discrete Modelica.Units.SI.Angle entryAngleAtStart
      "Entry tangency angle at the initial time";
  equation
    when initial() then
      entryAngleAtStart = entryAngle;
    end when;
    // Each republished name is compared against the part that owns it, never
    // against a formula re-typed here. A facade alias pointing at the wrong
    // member, or at nothing, shows up as a non-zero gap.
    wrapDeviation = max({
      abs(tensionIn - webWrap.entryTension),
      abs(tensionOut - webWrap.exitTension),
      abs(Fn - webWrap.normalForce),
      abs(entryAngle - webWrap.entryAngle),
      abs(exitAngle - webWrap.exitAngle),
      abs(arcAngle - webWrap.arcAngle)});
    bodyDeviation = max({
      abs(surfaceVelocity - rollBody.surfaceVelocity),
      abs(angle - rollBody.angle),
      abs(bearingTorque - rollBody.bearingTorque)});
    contactDeviation = max({
      abs(slipVelocity - webFriction.slipVelocity),
      abs(Fcap - webFriction.tractionCapacity),
      abs(Ft - webFriction.longitudinalForce),
      abs(lateralPosition - webFriction.lateralFlange.s),
      abs(lateralSlip - webFriction.lateralSlip),
      abs(lateralForce - webFriction.lateralForce)});
    // The facade supplies the connected directions; the contact owns the slip law.
    compositionDeviation = max(
      Modelica.Math.Vectors.length(webFriction.spanDirectionA - frame_a.spanDirection),
      Modelica.Math.Vectors.length(webFriction.spanDirectionB - frame_b.spanDirection));
  end VisualRollerFacadeProbe;

  model RollerFacadeCompositionCheck
    "Assembled roller republishes exactly what its three parts compute"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(
      web(width = 0.6),
      lateralDynamics = true)
      "Shared line defaults with lateral contact motion";
    Modelica.Blocks.Sources.Constant entryLoad[3](
      k = {-100*cos(0.6), 40, -100*sin(0.6)})
      "Entry load selecting a known tangent";
    Modelica.Blocks.Sources.Constant exitLoad[3](
      k = {100*cos(0.6), 40, -100*sin(0.6)})
      "Exit load selecting a known tangent";
    Roll2RollDynamics.Components.WebForce entryForce(direction=1)
      "Entry span force stand-in";
    Roll2RollDynamics.Components.WebForce exitForce(direction=-1)
      "Exit span force stand-in";
    Modelica.Mechanics.Rotational.Sources.ConstantTorque drive(tau_constant = 20)
      "Drive that keeps the roll turning under the web";
    VisualRollerFacadeProbe roller(
      yaw = 0.2,
      useFlange = true,
      fixedInitialAngle = true,
      fixedInitialSpeed = true,
      fixedInitialWebState = true,
      useDynamicColor = true)
      "Assembled roller under composition test";
  equation
    connect(entryLoad.y, entryForce.force)
      annotation(Line(points = {{-80, 30}, {-60, 30}, {-60, 10}, {-40, 10}}, color = {0, 0, 127}));
    connect(exitLoad.y, exitForce.force)
      annotation(Line(points = {{-80, -30}, {-60, -30}, {-60, -10}, {-40, -10}}, color = {0, 0, 127}));
    connect(entryForce.frame_b, roller.frame_a)
      annotation(Line(points = {{-20, 10}, {20, 10}, {20, 30}, {60, 30}}, color = {95, 95, 95}, thickness = 0.5));
    connect(exitForce.frame_b, roller.frame_b)
      annotation(Line(points = {{-20, -10}, {20, -10}, {20, 0}, {60, 0}}, color = {95, 95, 95}, thickness = 0.5));
    connect(drive.flange, roller.flange_a)
      annotation(Line(points = {{-80, -70}, {-20, -70}, {-20, -40}, {60, -40}}, color = {0, 0, 0}));
    when terminal() then
      // The facade owns no physics; every public value it reports has to be the
      // one its part computed. Comparing against the parts, rather than against
      // a formula written here, is what makes a mis-wired facade fail.
      assert(roller.wrapDeviation < 1e-12,
        "Roller must republish webWrap's values unchanged");
      assert(roller.bodyDeviation < 1e-12,
        "Roller must republish rollBody's values unchanged");
      assert(roller.contactDeviation < 1e-12,
        "Roller must republish webFriction's values unchanged");
      assert(roller.compositionDeviation < 1e-12,
        "Roller must compose contact drift and lateral transport from its parts");
      // The comparisons above are only worth anything on a roll that is actually
      // turning, gripping and steering, so pin that the run reaches that state.
      assert(abs(roller.rollerVariables.angle) > 1e-6,
        "Composition check must run on a roll that turns");
      assert(abs(roller.contactDrift) > 1e-9,
        "Composition check must run with a non-zero cross-machine drift");
      assert(abs(roller.rollerVariables.lateralPosition) > 1e-9,
        "Composition check must move the contact line across the machine");
      assert(abs(roller.Fn) > 1,
        "Composition check must run under a real wrap load");
      // The wrap rides on the non-spinning yawed centre frame. Were it taken from
      // the shaft instead, a constant world load would sweep the tangency round
      // with the drum.
      assert(abs(roller.entryAngle - roller.entryAngleAtStart) < 1e-9,
        "Solved tangency must not turn with the drum under a constant load");
    end when;
    annotation(experiment(StopTime = 0.05, Tolerance = 1e-8));
  end RollerFacadeCompositionCheck;

  model SpanColorGradingCheck
    "Stress color ramp separates the spans a converged line actually shows"
    extends Modelica.Icons.Example;
    final parameter Modelica.Units.SI.Stress stressLow = 40000
      "Low color limit, 20% below nominal";
    final parameter Modelica.Units.SI.Stress stressHigh = 60000
      "High color limit, 20% above nominal";
    final parameter Real gamma = 0.4 "Contrast exponent under test";
    final parameter Real lowColor[3] = {60, 110, 200} "Color at the low limit";
    final parameter Real midColor[3] = {235, 232, 220} "Color at nominal";
    final parameter Real highColor[3] = {205, 50, 35} "Color at the high limit";
    final parameter Real atNominal[3] = Roll2RollDynamics.Functions.stressColor(
      50000, stressLow, stressHigh, 50000, gamma, lowColor, midColor, highColor)
      "Color drawn at the nominal stress";
    final parameter Real slightlyTight[3] = Roll2RollDynamics.Functions.stressColor(
      51000, stressLow, stressHigh, 50000, gamma, lowColor, midColor, highColor)
      "Color drawn two percent above nominal";
    final parameter Real slightlySlack[3] = Roll2RollDynamics.Functions.stressColor(
      49000, stressLow, stressHigh, 50000, gamma, lowColor, midColor, highColor)
      "Color drawn two percent below nominal";
    final parameter Real farAbove[3] = Roll2RollDynamics.Functions.stressColor(
      90000, stressLow, stressHigh, 50000, gamma, lowColor, midColor, highColor)
      "Color drawn far above the high limit";
    final parameter Real farBelow[3] = Roll2RollDynamics.Functions.stressColor(
      0, stressLow, stressHigh, 50000, gamma, lowColor, midColor, highColor)
      "Color drawn far below the low limit";
  equation
    when terminal() then
      assert(Modelica.Math.Vectors.length(atNominal - midColor) < 1e-9,
        "Nominal stress must be drawn in the mid color");
      assert(Modelica.Math.Vectors.length(farAbove - highColor) < 1e-9
          and Modelica.Math.Vectors.length(farBelow - lowColor) < 1e-9,
        "Stresses outside the window must clamp to the limit colors");
      // A converged tension loop holds every span within a percent or two of
      // nominal. A linear ramp would move only a tenth of the way to the limit
      // there and draw the whole line in one color.
      assert(Modelica.Math.Vectors.length(slightlyTight - midColor) > 60,
        "Two percent above nominal must be clearly separated from nominal");
      assert(Modelica.Math.Vectors.length(slightlySlack - midColor) > 60,
        "Two percent below nominal must be clearly separated from nominal");
      assert(Modelica.Math.Vectors.length(slightlyTight - slightlySlack) > 60,
        "The ramp must show the sign of a small deviation");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end SpanColorGradingCheck;

  model WebWorldStressBarCheck
    "WebWorld draws a fixed stress scale matching the web color limits"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(
      tension = 15000,
      stressLow = 4e6,
      stressHigh = 6e6)
      "Line defaults with known stress limits, nominal stress 5e6 Pa";
  equation
    when terminal() then
      assert(abs(world.stressBarMinimum - 4e6) < 1e-9
          and abs(world.stressBarMaximum - 6e6) < 1e-9,
        "Stress bar must report the configured minimum and maximum stresses");
      assert(Modelica.Math.Vectors.length(
        world.stressBar[1].color - world.stressBarLowColor) < 1e-9,
        "Bottom of the stress bar must show the low-stress color");
      assert(Modelica.Math.Vectors.length(
        world.stressBar[world.stressBarSegments].color
          - world.stressBarHighColor) < 1e-9,
        "Top of the stress bar must show the high-stress color");
      assert(world.stressBar[1].shapeType == "cylinder",
        "The stress bar must be drawn as a column rather than a flat plate");
      assert(abs(world.stressBarMinimumCap.r[3]) < 1e-9
          and abs(world.stressBarMaximumCap.r[3] - world.nominalLength) < 1e-9,
        "The two caps must sit at the ends of the drawn scale");
      assert(world.stressBarNominalFraction >= 0
          and world.stressBarNominalFraction <= 1
          and abs(world.stressBarNominalRing.r[3]
            - world.stressBarNominalFraction*world.nominalLength) < 1e-9,
        "The nominal ring must sit where the nominal stress falls on the scale");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end WebWorldStressBarCheck;

  model AnimationDisabledCheck
    "Turning the world animation off leaves a line that still runs"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world(enableAnimation = false)
      "Line defaults with every drawn shape switched off";
    Roll2RollDynamics.Components.Roller driven(
      useFlange = true, fixedInitialAngle = true, rotationDirection = -1,
      fixedInitialWebState = true,
      traction = Roll2RollDynamics.Utilities.Types.TractionFrictionParameters(
        muAdhesion = 0.2, muSliding = 0.15))
      "Roll carrying its own contact parameters rather than the line's";
    Modelica.Mechanics.Rotational.Sources.ConstantSpeed drive(
      w_fixed = -world.lineSpeed/world.rollRadius) "Speed master";
    Roll2RollDynamics.Components.WebForce entryLoad(
      direction=1,
      force = {-150*cos(0.5), 0, 150*sin(0.5)})
      "Nonzero incoming web tension with matching transport effort";
    Roll2RollDynamics.Components.WebForce exitLoad(
      direction=-1,
      force = {150*cos(0.5), 0, 150*sin(0.5)})
      "Nonzero outgoing web tension with matching transport effort";
  equation
    connect(drive.flange, driven.flange_a);
    connect(entryLoad.frame_b, driven.frame_a);
    connect(exitLoad.frame_b, driven.frame_b);
    when terminal() then
      assert(driven.webMass > 0,
        "A loaded wrap must carry its actual material mass");
      assert(abs(driven.traction.muAdhesion - 0.2) < 1e-12,
        "A roll must keep the traction parameters it was given");
      assert(driven.rollerVariables.Fcap > 1,
        "The animation-disabled test must exercise a loaded wrap");
      assert(abs(driven.rollerVariables.Fcap - (driven.rollerVariables.tensionIn + driven.rollerVariables.tensionOut - 2*driven.centrifugalTension)*
          tanh(0.2*abs(driven.rollerVariables.arcAngle)/2)) < 1e-9,
        "The roll's own adhesion coefficient must reach the contact law");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end AnimationDisabledCheck;

  model DegenerateSpanWidthCheck
    "A cross-machine span retains a finite transverse width"
    extends Modelica.Icons.Example;
    inner Roll2RollDynamics.WebWorld world "Shared line defaults";
    VisualSpanBoundary anchorA(directionGuess={0,1,0},widthReference={0,0,1})
      "Upstream fixed anchor with a transverse width reference";
    VisualSpanBoundary anchorB(position={0,1,0},directionGuess={0,1,0},
      widthReference={0,0,1}) "Downstream fixed anchor across the machine";
    VisualBeltProbe span "Span running along the world y-axis";
  equation
    connect(anchorA.port, span.frame_a);
    connect(anchorB.port, span.frame_b);
    when terminal() then
      assert(Modelica.Math.Vectors.length(span.firstWidthDirection-{0,0,1}) < 1e-6
          and Modelica.Math.Vectors.length(span.lastWidthDirection-{0,0,1}) < 1e-6,
        "A cross-machine span must preserve a normalized transverse width");
      assert(Modelica.Math.Vectors.length(
        span.visualEnd - span.visualStart - {0, 1, 0}) < 1e-6,
        "A cross-machine span must still be drawn end to end");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end DegenerateSpanWidthCheck;

  model WrapColorGradingCheck
    "Drawn wrap ramps between its two tangency colours instead of holding one mean colour"
    extends Modelica.Icons.Example;
    final parameter Modelica.Units.SI.Length thickness = 0.005 "Web thickness";
    final parameter Modelica.Units.SI.Length width = 0.6 "Web width";
    final parameter Modelica.Units.SI.Stress stressLow = 0 "Low colour limit";
    final parameter Modelica.Units.SI.Stress stressHigh = 100000 "High colour limit";
    final parameter Modelica.Units.SI.Stress stressNominal = 50000
      "Stress drawn in the nominal colour";
    final parameter Real lowColor[3] = {60, 110, 200} "Colour at the low limit";
    final parameter Real midColor[3] = {245, 210, 70} "Colour at nominal";
    final parameter Real highColor[3] = {205, 50, 35} "Colour at the high limit";
    inner Modelica.Mechanics.MultiBody.World world(g = 0, enableAnimation = false)
      "Inertial reference";
    Modelica.Mechanics.MultiBody.Forces.WorldForce inlet(animation = false)
      "Entry tension boundary";
    Modelica.Mechanics.MultiBody.Forces.WorldForce outlet(animation = false)
      "Exit tension boundary";
    Real entryLoad[3](each unit = "N") = {-200, 0, -15.707963267948966}
      "Applied entry force in roll coordinates";
    Real exitLoad[3](each unit = "N") = {100, 0, -15.707963267948966}
      "Applied exit force in roll coordinates";
    Roll2RollDynamics.Utilities.Parts.WebWrap wrap(
      spanDirectionA = -entryLoad/sqrt(entryLoad*entryLoad),
      spanDirectionB = exitLoad/sqrt(exitLoad*exitLoad),
      radius = 0.15, webWidth = width, beltThickness = thickness,
      entryTension = sqrt(entryLoad*entryLoad),
      exitTension = sqrt(exitLoad*exitLoad),
      stressLow = stressLow, stressHigh = stressHigh, stressNominal = stressNominal,
      colorGamma = 1, animation = false,
      webColor = lowColor, nominalColor = midColor, stressColor = highColor)
      "Wrap under test";
    Real meanColor[3]
      "Single colour a mean-tension wrap would hold across the whole band";
  equation
    inlet.force = entryLoad;
    outlet.force = exitLoad;
    0 = wrap.frame_a.R.T[3,:]*entryLoad;
    0 = wrap.frame_b.R.T[3,:]*exitLoad;
    //Derived from the component's solved state rather than rebound as a
    //parameter, which cannot be initialized across the component boundary.
    meanColor = Roll2RollDynamics.Functions.stressColor(
      wrap.meanTension/(thickness*width), stressLow, stressHigh, stressNominal, 1,
      lowColor, midColor, highColor);
    connect(world.frame_b, wrap.frame_center);
    connect(inlet.frame_b, wrap.frame_a);
    connect(outlet.frame_b, wrap.frame_b);
    when terminal() then
      //With only the two ends compared, a flat mean colour sits close enough to
      //both to pass, so the midpoint segment is what actually pins the ramp.
      assert(Modelica.Math.Vectors.length(
        wrap.arcSegmentColor[div(wrap.drawingSegments, 2),:] - meanColor) > 2,
        "A middle wrap segment must differ from the mean-tension colour, so the band ramps");
      assert(Modelica.Math.Vectors.length(
        wrap.arcSegmentColor[1,:] - wrap.arcSegmentColor[wrap.drawingSegments,:]) > 2,
        "The wrap colour must differ between its two tangencies");
    end when;
    annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  end WrapColorGradingCheck;

  annotation(Documentation(info = "<html><h4>Visual geometry checks</h4>
<p>Fixtures and scenarios for rendered web geometry, color grading and animation switches. Use the corresponding reference runner for simulation settings and expected diagnostics.</p>
</html>"));
end VisualGeometryChecks;
