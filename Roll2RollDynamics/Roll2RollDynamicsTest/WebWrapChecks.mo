within Roll2RollDynamicsTest;
package WebWrapChecks "Directed wrap geometry and load regression checks"
  extends Modelica.Icons.ExamplesPackage;

  model LoadedWrap "Prescribed spatial tensions on a fixed wrap centre"
    extends Modelica.Icons.Example;
    //Parameters
    parameter Boolean axialEntry=false
      "Prescribe the otherwise undefined entry angle for an invalid axial load";
    //Variables with Binding Equations
    Real entryLoad[3](each unit="N") = {-80, 30, -60}
      "Applied entry force in roll coordinates";
    Real exitLoad[3](each unit="N") = {80, 40, -60}
      "Applied exit force in roll coordinates";
    Real entryAlignment[3] = entryLoad
      "Tangent direction, prescribed separately only for invalid-load diagnostics";
    //Components
    inner Modelica.Mechanics.MultiBody.World world(g=0, enableAnimation=false)
      "Inertial reference"
      annotation(Placement(visible = true, transformation(origin = {-120, 120},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce inlet(
      animation=false)
      "Entry tension boundary"
      annotation(Placement(visible = true, transformation(origin = {-40, 40},
        extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce outlet(
      animation=false)
      "Exit tension boundary"
      annotation(Placement(visible = true, transformation(origin = {-40, -40},
        extent = {{-10, -10}, {10, 10}})));
    Roll2RollDynamics.Utilities.Parts.WebWrap wrap(
      spanDirectionA=-entryAlignment/sqrt(entryAlignment*entryAlignment),
      spanDirectionB=exitLoad/sqrt(exitLoad*exitLoad),
      radius=0.15, webWidth=0.6, beltThickness=0.005,
      entryTension=sqrt(entryLoad*entryLoad),
      exitTension=sqrt(exitLoad*exitLoad),
      stressLow=0, stressHigh=100000, stressNominal=50000, colorGamma=1, animation=false)
      "Wrap under test"
      annotation(Placement(visible = true, transformation(origin = {40, 0},
        extent = {{-10, -10}, {10, 10}})));
  equation
    inlet.force = entryLoad;
    outlet.force = exitLoad;
    if axialEntry then
      wrap.entryAngle = 0;
    else
      0 = wrap.frame_a.R.T[3,:]*entryAlignment;
    end if;
    0 = wrap.frame_b.R.T[3,:]*exitLoad;
    connect(world.frame_b, wrap.frame_center)
      annotation(Line(visible = true, points = {{-110, 120}, {31, -10}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(inlet.frame_b, wrap.frame_a)
      annotation(Line(visible = true, points = {{-30, 40}, {30, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    connect(outlet.frame_b, wrap.frame_b)
      annotation(Line(visible = true, points = {{-30, -40}, {50, 0}},
        color = {95, 95, 95}, thickness = 0.5, smooth = Smooth.None));
    annotation(experiment(StopTime=0.01, Tolerance=1e-9),
      Documentation(info = "<html><h4>Loaded wrap fixture</h4><p>Helper fixture that applies independently specified spatial entry and exit loads to a grounded wrap and aligns the tangencies with their projected directions. Derived tests select the geometry, visual options or invalid inputs and provide the acceptance assertions.</p></html>"));
    annotation(
      Diagram(coordinateSystem(extent = {{-160, -100}, {100, 160}})));
  end LoadedWrap;

  model SpatialResultant "Skewed loads retain their lateral resultant"
    extends LoadedWrap;
  equation
    assert(abs(wrap.normalForce - sqrt(19300)) < 1e-9,
      "The applied forces sum to {0,70,-120} N; normalForce must include the lateral load");
    assert(abs(wrap.arcAngle - 1.2870022175865687) < 1e-9,
      "Lateral tension must not change the projected tangency angles");
    annotation(Documentation(info = "<html><h4>Spatial resultant</h4><p>Applies spatial loads with resultant <code>{0,70,-120}</code> N. The reported resultant magnitude must be <code>sqrt(19300)</code> N and the projected wrap angle must remain 1.2870022175865687 rad, both within 1e-9.</p></html>"));
  end SpatialResultant;

  model CoarseReverseWrap "Coarse drawing of a reverse wrap remains outside the drum"
    extends LoadedWrap(entryLoad={-50,0,-86.60254037844386},
      exitLoad={50,0,-86.60254037844386},
      wrap(rotationDirection=-1, arcSegments=1, animation=true));
  equation
    assert(wrap.chord > 0 and wrap.drumDrawnRadius > 0,
      "Coarse wrap drawing must retain positive box and drum dimensions");
    assert(wrap.arcRadius - 0.0025 > wrap.drumDrawnRadius,
      "The web boxes must clear the drum");
    assert(abs(wrap.arcAngle + 4.1887902047863905) < 1e-9,
      "A reverse wrap must have the opposite mechanical angle");
    annotation(Documentation(info = "<html><h4>Coarse reverse wrap</h4><p>Requests one drawing segment for a reverse wrap to exercise safe drawing resolution without changing mechanical geometry. Drawn dimensions must remain positive, the web boxes must clear the drum, and the mechanical arc angle must be -4.1887902047863905 rad within 1e-9 rad.</p></html>"));
  end CoarseReverseWrap;

  model MovingTangency "Tangency speeds remain finite across the angle branch cut"
    extends LoadedWrap(
      entryLoad={-100*cos(4.6+0.2*time), 0, 100*sin(4.6+0.2*time)},
      exitLoad={100*cos(5.8+0.2*time), 0, -100*sin(5.8+0.2*time)});
  equation
    assert(abs(wrap.arcAngle - 1.2) < 1e-9,
      "Crossing the absolute angle branch cut must preserve the directed wrap");
    assert(abs(wrap.boundaryVelocityA - 0.0305) < 1e-9
      and abs(wrap.boundaryVelocityB - 0.0305) < 1e-9,
      "Both tangencies must migrate at radius times 0.2 rad/s");
    assert(abs(wrap.frame_a.r_0[1] - 0.1525*sin(4.6+0.2*time)) < 1e-9
      and abs(wrap.frame_a.r_0[3] - 0.1525*cos(4.6+0.2*time)) < 1e-9,
      "The entry point must stay on its prescribed continuous circular trajectory");
    annotation(experiment(StopTime=2, Tolerance=1e-9),
      Documentation(info = "<html><h4>Moving tangency</h4><p>Rotates both load directions at 0.2 rad/s through the absolute-angle branch cut. The directed wrap must remain 1.2 rad, both tangency speeds must remain 0.0305 m/s, and the entry point must follow its continuous circular trajectory within 1e-9.</p></html>"));
  end MovingTangency;

  model ZeroResultant "Opposing forces can have a zero resultant at nonzero tension"
    extends LoadedWrap(entryLoad={-100,0,0}, exitLoad={100,0,0});
  equation
    assert(abs(wrap.normalForce) < 1e-12 and abs(wrap.arcLength) < 1e-12,
      "A straight-through taut web must have zero wrap and zero resultant");
    annotation(Documentation(info = "<html><h4>Zero resultant</h4><p>Applies equal opposing nonzero tensions to a straight-through web. Both resultant force and wrap length must vanish within 1e-12; zero wrap is valid for this isolated geometry, although the material contact model requires positive wrap length.</p></html>"));
  end ZeroResultant;

  model SpanGradient "Wrap colour meets the adjacent span at both tangencies"
    extends LoadedWrap(
      entryLoad={-100,0,-15.707963267948966},
      exitLoad={200,0,-15.707963267948966},
      wrap(animation=true, colorGamma=1,
        stressLow=0, stressHigh=100000, stressNominal=50000));
    //The wrap carries a lower tension at entry than at exit, so the band must
    //ramp between their colours instead of holding one mean colour across the
    //whole wrap. The expectations are derived from the segment tensions the
    //component actually solved, so the check tests the mapping rather than a
    //hand-computed load.
    Real expectedFirst[3] = Roll2RollDynamics.Functions.stressColor(
      wrap.arcSegmentTension[1]/(0.005*0.6), 0, 100000, 50000, 1,
      {60,110,200}, {245,210,70}, {205,50,35})
      "Colour the adjacent span shows at the first segment's tension";
    Real expectedMid[3] = Roll2RollDynamics.Functions.stressColor(
      wrap.arcSegmentTension[div(wrap.drawingSegments, 2)]/(0.005*0.6),
      0, 100000, 50000, 1,
      {60,110,200}, {245,210,70}, {205,50,35})
      "Colour the adjacent span shows at the middle segment's tension";
    Real expectedLast[3] = Roll2RollDynamics.Functions.stressColor(
      wrap.arcSegmentTension[wrap.drawingSegments]/(0.005*0.6),
      0, 100000, 50000, 1,
      {60,110,200}, {245,210,70}, {205,50,35})
      "Colour the adjacent span shows at the last segment's tension";
  equation
    assert(max(abs(wrap.arcSegmentColor[1,:] - expectedFirst)) < 1e-9,
      "The first drawn segment must take the colour of its own tension so the wrap meets its entry span");
    //The middle segment is the load-bearing part of this check: with only the
    //two ends compared, a single flat colour at the mean would sit close enough
    //to both to pass. Requiring the midpoint hue as well pins the ramp.
    assert(max(abs(wrap.arcSegmentColor[div(wrap.drawingSegments, 2),:] - expectedMid)) < 1e-9,
      "A middle segment must take the colour of its own tension, so the band ramps instead of holding one mean colour");
    assert(max(abs(wrap.arcSegmentColor[wrap.drawingSegments,:] - expectedLast)) < 1e-9,
      "The last drawn segment must take the colour of its own tension so the wrap meets its exit span");
    annotation(Documentation(info = "<html><h4>Span gradient</h4><p>Applies unequal boundary tensions and compares the first, middle and last drawn wrap colours with <code>stressColor</code> evaluated at their own segment tensions. Every RGB component must agree within 1e-9 so the wrap follows a tension-dependent ramp rather than one mean colour.</p></html>"));
  end SpanGradient;

  model InvalidDirection "An invalid winding direction must be diagnosed"
    extends LoadedWrap(wrap(rotationDirection=0));
    annotation(Documentation(info = "<html><h4>Invalid wrap direction</h4><p>Sets the wrap direction to zero. Initialization is expected to fail with the diagnostic that <code>rotationDirection</code> must be -1 or +1.</p></html>"));
  end InvalidDirection;

  model InvalidRadius "A zero mechanical radius must be diagnosed"
    extends LoadedWrap(wrap(radius=0));
    annotation(Documentation(info = "<html><h4>Invalid wrap radius</h4><p>Sets the mechanical wrap radius to zero. Initialization is expected to fail with the positive-radius, web-width and thickness diagnostic.</p></html>"));
  end InvalidRadius;

  model ZeroTension "A slack boundary must be diagnosed"
    extends LoadedWrap(entryLoad={0,0,0},entryAlignment={-1,0,0});
    annotation(Documentation(info = "<html><h4>Zero tension</h4><p>Applies zero entry tension while retaining a separate finite alignment direction. Initialization is expected to fail because the entry and exit spans must be taut.</p></html>"));
  end ZeroTension;

  model AxialOnlyForce "Nonzero tension along the roll axis does not define tangency"
    extends LoadedWrap(entryLoad={0,100,0},axialEntry=true);
    annotation(Documentation(info = "<html><h4>Axial-only force</h4><p>Applies a nonzero entry force entirely along the roller axis and prescribes the otherwise undefined entry angle. Initialization is expected to fail because the entry tension has no projection into the roll plane.</p></html>"));
  end AxialOnlyForce;
  annotation(Documentation(info = "<html>
<h4>Web wrap checks</h4>
<p>Exercise spatial force routing, directed wrap geometry, moving tangencies
and the stress-colour transition between free spans and the wrapped web.
Separate invalid-input cases must reject zero radius, invalid winding
direction and boundary loads that cannot define a tangency.</p>
</html>"));
end WebWrapChecks;
